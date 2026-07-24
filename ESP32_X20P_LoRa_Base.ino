#include <Arduino.h>
#include <HardwareSerial.h>

/*
 * Production base bridge
 *
 * ZED-X20P RTCM3 UART -> ESP32 -> RYLR896 LoRa
 *
 * Features:
 * - Preserves complete RTCM3 frames.
 * - Validates RTCM CRC-24Q before transmission.
 * - Base64 encodes and fragments frames for the RYLR896 AT interface.
 * - Uses a complete-frame ACK from the rover.
 * - Retries the same frame when the ACK is missing.
 * - Buffers incoming RTCM while LoRa transmission is in progress.
 * - Replaces older queued frames of the same RTCM type with fresher ones.
 *
 * Matching rover code:
 * STM32_F9P_LoRa_Rover/main.c
 */

HardwareSerial X20P(1);
HardwareSerial LoRa(2);

/* ----------------------------- USER SETTINGS ----------------------------- */

/* ESP32 UART pins. */
static constexpr int X20P_RX_PIN = 26;   // Connect to ZED-X20P UART TX
static constexpr int X20P_TX_PIN = -1;   // Not used
static constexpr int LORA_RX_PIN = 16;   // Connect to RYLR896 TXD
static constexpr int LORA_TX_PIN = 17;   // Connect to RYLR896 RXD

static constexpr uint32_t USB_DEBUG_BAUD = 115200U;
static constexpr uint32_t GNSS_BAUD = 115200U;
static constexpr uint32_t LORA_TARGET_BAUD = 115200U;

/* Must match the STM32 rover project. */
static constexpr uint16_t BASE_ADDRESS = 2U;
static constexpr uint16_t ROVER_ADDRESS = 1U;
static constexpr uint8_t NETWORK_ID = 5U;

/*
 * Highest-rate laboratory setting supported by this module command set:
 * SF7, BW500 kHz, CR4/5, preamble 4.
 *
 * For more range / robustness, use:
 * "AT+PARAMETER=7,7,1,4"   // BW125 kHz
 */
static constexpr const char *LORA_RF_PARAMETER = "AT+PARAMETER=7,9,1,4";
static constexpr const char *LORA_FREQUENCY = "AT+BAND=866000000";
static constexpr const char *LORA_RF_POWER = "AT+CRFOP=15";

/* Frame-level reliability. Set false for one-way RTCM streaming. */
static constexpr bool REQUIRE_ROVER_ACK = false;
static constexpr uint8_t MAX_FRAME_ATTEMPTS = 3U;
static constexpr uint8_t MAX_AT_COMMAND_ATTEMPTS = 2U;
static constexpr uint32_t COMMAND_OK_TIMEOUT_MS = 3000U;
static constexpr uint32_t FRAME_ACK_TIMEOUT_MS = 2500U;
static constexpr uint32_t RETRY_GAP_MS = 40U;

/* Diagnostics. */
static constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 5000U;

/* --------------------------- PROTOCOL CONSTANTS --------------------------- */

/* RTCM payload length is 10 bits. Maximum complete frame = 1023 + 6. */
static constexpr size_t RTCM_MAX_FRAME = 1029U;

/*
 * 171 raw bytes -> 228 Base64 characters.
 * 9-character header + 228 = 237 bytes, below the 240-byte RYLR896 limit.
 *
 * Fragment header:
 *   R SSSS FF TT
 *   R     = RTCM fragment
 *   SSSS  = 16-bit frame sequence in hex
 *   FF    = zero-based fragment index in hex
 *   TT    = total fragment count in hex
 *
 * ACK payload from rover:
 *   A SSSS
 */
static constexpr size_t RAW_FRAGMENT_SIZE = 171U;
static constexpr size_t BASE64_FRAGMENT_MAX = 228U;
static constexpr size_t LORA_FRAGMENT_HEADER_SIZE = 9U;
static constexpr size_t LORA_PAYLOAD_MAX = 240U;
static constexpr uint8_t RTCM_MAX_FRAGMENTS = 8U;

/* Complete RTCM queue. ESP32 has sufficient RAM for this depth. */
static constexpr uint8_t RTCM_QUEUE_DEPTH = 16U;

struct RtcmQueueItem
{
  uint16_t length;
  uint16_t messageType;
  uint8_t data[RTCM_MAX_FRAME];
};

static RtcmQueueItem rtcmQueue[RTCM_QUEUE_DEPTH];
/* Separate transmit copy prevents newly arriving RTCM from overwriting the frame in flight. */
static RtcmQueueItem activeTransmitItem;
static uint8_t queueReadIndex = 0U;
static uint8_t queueWriteIndex = 0U;
static uint8_t queueCount = 0U;

/* RTCM parser state. */
static uint8_t rtcmParserFrame[RTCM_MAX_FRAME];
static size_t rtcmParserPosition = 0U;
static size_t rtcmParserExpectedLength = 0U;

/* LoRa line parser state. */
static constexpr size_t LORA_LINE_MAX = 320U;
static char loraLineBuffer[LORA_LINE_MAX];
static size_t loraLineIndex = 0U;
static bool commandOkSeen = false;
static bool commandErrorSeen = false;
static bool pendingAckValid = false;
static uint16_t pendingAckSequence = 0U;

static uint16_t nextFrameSequence = 0U;
static uint32_t lastStatusPrintMs = 0U;

/* Statistics. */
static uint32_t statRtcmBytesRead = 0U;
static uint32_t statValidRtcmFrames = 0U;
static uint32_t statInvalidRtcmFrames = 0U;
static uint32_t statQueuedFrames = 0U;
static uint32_t statReplacedQueuedFrames = 0U;
static uint32_t statDroppedQueuedFrames = 0U;
static uint32_t statSentFrames = 0U;
static uint32_t statAcknowledgedFrames = 0U;
static uint32_t statRetriedFrames = 0U;
static uint32_t statFailedFrames = 0U;
static uint32_t statSentFragments = 0U;
static uint32_t statAtCommandErrors = 0U;

static const char BASE64_TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/* ------------------------------- RTCM CRC -------------------------------- */

static uint32_t Crc24Q(const uint8_t *data, size_t length)
{
  uint32_t crc = 0U;

  for (size_t i = 0U; i < length; i++)
  {
    crc ^= (uint32_t)data[i] << 16U;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc <<= 1U;

      if ((crc & 0x1000000UL) != 0U)
      {
        crc ^= 0x1864CFBUL;
      }
    }
  }

  return crc & 0xFFFFFFUL;
}

static bool IsValidRtcmFrame(const uint8_t *frame, size_t length)
{
  if ((frame == nullptr) || (length < 6U) || (frame[0] != 0xD3U))
  {
    return false;
  }

  if ((frame[1] & 0xFCU) != 0U)
  {
    return false;
  }

  const size_t payloadLength =
      ((size_t)(frame[1] & 0x03U) << 8U) |
      (size_t)frame[2];

  if ((payloadLength + 6U) != length)
  {
    return false;
  }

  const uint32_t calculated = Crc24Q(frame, length - 3U);
  const uint32_t received =
      ((uint32_t)frame[length - 3U] << 16U) |
      ((uint32_t)frame[length - 2U] << 8U) |
      (uint32_t)frame[length - 1U];

  return calculated == received;
}

static uint16_t GetRtcmMessageType(const uint8_t *frame, size_t length)
{
  if ((frame == nullptr) || (length < 5U))
  {
    return 0U;
  }

  return (uint16_t)(((uint16_t)frame[3] << 4U) |
                    ((uint16_t)frame[4] >> 4U));
}

/* ------------------------------ BASE64 ----------------------------------- */

static size_t Base64Encode(const uint8_t *input,
                           size_t inputLength,
                           char *output,
                           size_t outputCapacity)
{
  const size_t requiredLength = 4U * ((inputLength + 2U) / 3U);

  if ((input == nullptr) ||
      (output == nullptr) ||
      (outputCapacity < (requiredLength + 1U)))
  {
    return 0U;
  }

  size_t inputIndex = 0U;
  size_t outputIndex = 0U;

  while ((inputIndex + 3U) <= inputLength)
  {
    const uint32_t value =
        ((uint32_t)input[inputIndex] << 16U) |
        ((uint32_t)input[inputIndex + 1U] << 8U) |
        (uint32_t)input[inputIndex + 2U];

    output[outputIndex++] = BASE64_TABLE[(value >> 18U) & 0x3FU];
    output[outputIndex++] = BASE64_TABLE[(value >> 12U) & 0x3FU];
    output[outputIndex++] = BASE64_TABLE[(value >> 6U) & 0x3FU];
    output[outputIndex++] = BASE64_TABLE[value & 0x3FU];

    inputIndex += 3U;
  }

  const size_t remaining = inputLength - inputIndex;

  if (remaining == 1U)
  {
    const uint32_t value = (uint32_t)input[inputIndex] << 16U;

    output[outputIndex++] = BASE64_TABLE[(value >> 18U) & 0x3FU];
    output[outputIndex++] = BASE64_TABLE[(value >> 12U) & 0x3FU];
    output[outputIndex++] = '=';
    output[outputIndex++] = '=';
  }
  else if (remaining == 2U)
  {
    const uint32_t value =
        ((uint32_t)input[inputIndex] << 16U) |
        ((uint32_t)input[inputIndex + 1U] << 8U);

    output[outputIndex++] = BASE64_TABLE[(value >> 18U) & 0x3FU];
    output[outputIndex++] = BASE64_TABLE[(value >> 12U) & 0x3FU];
    output[outputIndex++] = BASE64_TABLE[(value >> 6U) & 0x3FU];
    output[outputIndex++] = '=';
  }

  output[outputIndex] = '\0';
  return outputIndex;
}

/* --------------------------- RTCM INPUT QUEUE ----------------------------- */

static void RemoveOldestQueuedFrame(void)
{
  if (queueCount == 0U)
  {
    return;
  }

  queueReadIndex = (uint8_t)((queueReadIndex + 1U) % RTCM_QUEUE_DEPTH);
  queueCount--;
  statDroppedQueuedFrames++;
}

static bool ReplaceQueuedFrameOfSameType(const uint8_t *frame,
                                         uint16_t frameLength,
                                         uint16_t messageType)
{
  for (uint8_t offset = 0U; offset < queueCount; offset++)
  {
    const uint8_t index =
        (uint8_t)((queueReadIndex + offset) % RTCM_QUEUE_DEPTH);

    if (rtcmQueue[index].messageType == messageType)
    {
      rtcmQueue[index].length = frameLength;
      memcpy(rtcmQueue[index].data, frame, frameLength);
      statReplacedQueuedFrames++;
      return true;
    }
  }

  return false;
}

static void QueueCompleteRtcmFrame(const uint8_t *frame, size_t length)
{
  if (!IsValidRtcmFrame(frame, length))
  {
    statInvalidRtcmFrames++;
    return;
  }

  statValidRtcmFrames++;

  const uint16_t messageType = GetRtcmMessageType(frame, length);
  const uint16_t frameLength = (uint16_t)length;

  /* Fresh correction data is preferable to a stale backlog. */
  if (ReplaceQueuedFrameOfSameType(frame, frameLength, messageType))
  {
    return;
  }

  if (queueCount >= RTCM_QUEUE_DEPTH)
  {
    RemoveOldestQueuedFrame();
  }

  RtcmQueueItem &item = rtcmQueue[queueWriteIndex];
  item.length = frameLength;
  item.messageType = messageType;
  memcpy(item.data, frame, frameLength);

  queueWriteIndex = (uint8_t)((queueWriteIndex + 1U) % RTCM_QUEUE_DEPTH);
  queueCount++;
  statQueuedFrames++;
}

static bool PeekQueuedFrame(RtcmQueueItem **item)
{
  if ((item == nullptr) || (queueCount == 0U))
  {
    return false;
  }

  *item = &rtcmQueue[queueReadIndex];
  return true;
}

static void PopQueuedFrame(void)
{
  if (queueCount == 0U)
  {
    return;
  }

  queueReadIndex = (uint8_t)((queueReadIndex + 1U) % RTCM_QUEUE_DEPTH);
  queueCount--;
}

static void ResetRtcmParser(void)
{
  rtcmParserPosition = 0U;
  rtcmParserExpectedLength = 0U;
}

static void FeedRtcmByte(uint8_t value)
{
  statRtcmBytesRead++;

  if (rtcmParserPosition == 0U)
  {
    if (value == 0xD3U)
    {
      rtcmParserFrame[rtcmParserPosition++] = value;
    }

    return;
  }

  if (rtcmParserPosition >= RTCM_MAX_FRAME)
  {
    ResetRtcmParser();

    if (value == 0xD3U)
    {
      rtcmParserFrame[rtcmParserPosition++] = value;
    }

    return;
  }

  rtcmParserFrame[rtcmParserPosition++] = value;

  if (rtcmParserPosition == 2U)
  {
    if ((rtcmParserFrame[1] & 0xFCU) != 0U)
    {
      ResetRtcmParser();
    }

    return;
  }

  if (rtcmParserPosition == 3U)
  {
    const size_t payloadLength =
        ((size_t)(rtcmParserFrame[1] & 0x03U) << 8U) |
        (size_t)rtcmParserFrame[2];

    rtcmParserExpectedLength = payloadLength + 6U;

    if ((rtcmParserExpectedLength < 6U) ||
        (rtcmParserExpectedLength > RTCM_MAX_FRAME))
    {
      ResetRtcmParser();
    }

    return;
  }

  if ((rtcmParserExpectedLength > 0U) &&
      (rtcmParserPosition == rtcmParserExpectedLength))
  {
    QueueCompleteRtcmFrame(rtcmParserFrame, rtcmParserExpectedLength);
    ResetRtcmParser();
  }
}

static void ServiceX20PInput(size_t maximumBytes = 0U)
{
  size_t processed = 0U;

  while (X20P.available() > 0)
  {
    FeedRtcmByte((uint8_t)X20P.read());
    processed++;

    if ((maximumBytes > 0U) && (processed >= maximumBytes))
    {
      break;
    }
  }
}

/* -------------------------- LORA LINE HANDLING ---------------------------- */

static int HexNibble(char value)
{
  if ((value >= '0') && (value <= '9')) return value - '0';
  if ((value >= 'A') && (value <= 'F')) return value - 'A' + 10;
  if ((value >= 'a') && (value <= 'f')) return value - 'a' + 10;
  return -1;
}

static bool ParseHex16(const char *text, uint16_t *value)
{
  if ((text == nullptr) || (value == nullptr))
  {
    return false;
  }

  uint16_t result = 0U;

  for (uint8_t i = 0U; i < 4U; i++)
  {
    const int nibble = HexNibble(text[i]);

    if (nibble < 0)
    {
      return false;
    }

    result = (uint16_t)((result << 4U) | (uint16_t)nibble);
  }

  *value = result;
  return true;
}

static void ProcessBaseLoRaLine(char *line)
{
  if ((line == nullptr) || (line[0] == '\0'))
  {
    return;
  }

  if (strcmp(line, "+OK") == 0)
  {
    commandOkSeen = true;
    return;
  }

  if (strncmp(line, "+ERR=", 5U) == 0)
  {
    commandErrorSeen = true;
    statAtCommandErrors++;
    Serial.print("RYLR896 error: ");
    Serial.println(line);
    return;
  }

  if (strncmp(line, "+RCV=", 5U) != 0)
  {
    return;
  }

  char *addressEnd = strchr(&line[5], ',');
  if (addressEnd == nullptr)
  {
    return;
  }

  *addressEnd = '\0';
  const uint16_t sourceAddress =
      (uint16_t)strtoul(&line[5], nullptr, 10);

  if (sourceAddress != ROVER_ADDRESS)
  {
    return;
  }

  char *lengthEnd = strchr(addressEnd + 1, ',');
  if (lengthEnd == nullptr)
  {
    return;
  }

  *lengthEnd = '\0';
  const uint16_t declaredLength =
      (uint16_t)strtoul(addressEnd + 1, nullptr, 10);

  char *payloadStart = lengthEnd + 1;

  if ((declaredLength != 5U) ||
      (strlen(payloadStart) < declaredLength))
  {
    return;
  }

  char ackPayload[6];
  memcpy(ackPayload, payloadStart, 5U);
  ackPayload[5] = '\0';

  if (ackPayload[0] != 'A')
  {
    return;
  }

  uint16_t sequence = 0U;

  if (!ParseHex16(&ackPayload[1], &sequence))
  {
    return;
  }

  pendingAckSequence = sequence;
  pendingAckValid = true;
}

static void PumpLoRaInput(void)
{
  while (LoRa.available() > 0)
  {
    const char character = (char)LoRa.read();

    if (character == '\r')
    {
      continue;
    }

    if (character == '\n')
    {
      loraLineBuffer[loraLineIndex] = '\0';

      if (loraLineIndex > 0U)
      {
        ProcessBaseLoRaLine(loraLineBuffer);
      }

      loraLineIndex = 0U;
    }
    else if (loraLineIndex < (LORA_LINE_MAX - 1U))
    {
      loraLineBuffer[loraLineIndex++] = character;
    }
    else
    {
      loraLineIndex = 0U;
    }
  }
}

static void ClearLoRaSerialAtCurrentBaud(void)
{
  while (LoRa.available() > 0)
  {
    (void)LoRa.read();
  }

  loraLineIndex = 0U;
}

static bool WaitForCommandOK(uint32_t timeoutMs)
{
  const uint32_t startMs = millis();

  while ((millis() - startMs) < timeoutMs)
  {
    PumpLoRaInput();
    ServiceX20PInput(256U);

    if (commandOkSeen)
    {
      return true;
    }

    if (commandErrorSeen)
    {
      return false;
    }

    delay(1U);
  }

  return false;
}

static bool SendATCommand(const char *command,
                          uint32_t timeoutMs = COMMAND_OK_TIMEOUT_MS)
{
  if (command == nullptr)
  {
    return false;
  }

  PumpLoRaInput();
  commandOkSeen = false;
  commandErrorSeen = false;

  LoRa.print(command);
  LoRa.print("\r\n");

  return WaitForCommandOK(timeoutMs);
}

static bool ProbeLoRaBaud(uint32_t baudRate)
{
  LoRa.end();
  delay(20U);

  LoRa.begin(baudRate, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(80U);

  ClearLoRaSerialAtCurrentBaud();
  return SendATCommand("AT", 600U);
}

static bool ConfigureLoRa(void)
{
  static const uint32_t baudCandidates[] =
  {
    115200U,
    57600U,
    38400U,
    19200U,
    9600U
  };

  uint32_t detectedBaud = 0U;

  for (size_t i = 0U;
       i < (sizeof(baudCandidates) / sizeof(baudCandidates[0]));
       i++)
  {
    if (ProbeLoRaBaud(baudCandidates[i]))
    {
      detectedBaud = baudCandidates[i];
      break;
    }
  }

  if (detectedBaud == 0U)
  {
    Serial.println("ERROR: RYLR896 did not respond at supported UART rates.");
    return false;
  }

  Serial.printf("RYLR896 detected at %lu baud\n",
                (unsigned long)detectedBaud);

  if (detectedBaud != LORA_TARGET_BAUD)
  {
    if (!SendATCommand("AT+IPR=115200"))
    {
      Serial.println("ERROR: Failed to change RYLR896 UART rate.");
      return false;
    }

    LoRa.end();
    delay(80U);
    LoRa.begin(LORA_TARGET_BAUD,
               SERIAL_8N1,
               LORA_RX_PIN,
               LORA_TX_PIN);
    delay(120U);
    ClearLoRaSerialAtCurrentBaud();

    if (!SendATCommand("AT"))
    {
      Serial.println("ERROR: RYLR896 did not respond at 115200 baud.");
      return false;
    }
  }

  char command[48];

  snprintf(command, sizeof(command),
           "AT+ADDRESS=%u", (unsigned int)BASE_ADDRESS);
  if (!SendATCommand(command)) return false;

  snprintf(command, sizeof(command),
           "AT+NETWORKID=%u", (unsigned int)NETWORK_ID);
  if (!SendATCommand(command)) return false;

  if (!SendATCommand(LORA_FREQUENCY)) return false;
  if (!SendATCommand(LORA_RF_PARAMETER)) return false;
  if (!SendATCommand(LORA_RF_POWER)) return false;

  return true;
}

static bool WaitForFrameAck(uint16_t expectedSequence,
                            uint32_t timeoutMs)
{
  if (!REQUIRE_ROVER_ACK)
  {
    return true;
  }

  if (pendingAckValid && (pendingAckSequence == expectedSequence))
  {
    pendingAckValid = false;
    return true;
  }

  const uint32_t startMs = millis();

  while ((millis() - startMs) < timeoutMs)
  {
    PumpLoRaInput();
    ServiceX20PInput(512U);

    if (pendingAckValid)
    {
      const uint16_t receivedSequence = pendingAckSequence;
      pendingAckValid = false;

      if (receivedSequence == expectedSequence)
      {
        return true;
      }
    }

    delay(1U);
  }

  return false;
}

/* --------------------------- LORA TRANSMISSION ---------------------------- */

static bool SendLoRaPayload(const char *payload, size_t payloadLength)
{
  if ((payload == nullptr) ||
      (payloadLength == 0U) ||
      (payloadLength > LORA_PAYLOAD_MAX))
  {
    return false;
  }

  char command[288];

  const int commandLength = snprintf(command,
                                     sizeof(command),
                                     "AT+SEND=%u,%u,%s",
                                     (unsigned int)ROVER_ADDRESS,
                                     (unsigned int)payloadLength,
                                     payload);

  if ((commandLength <= 0) ||
      ((size_t)commandLength >= sizeof(command)))
  {
    return false;
  }

  for (uint8_t attempt = 0U;
       attempt < MAX_AT_COMMAND_ATTEMPTS;
       attempt++)
  {
    if (SendATCommand(command))
    {
      statSentFragments++;
      return true;
    }

    delay(RETRY_GAP_MS);
  }

  return false;
}

static bool SendAllFragments(const uint8_t *frame,
                             size_t frameLength,
                             uint16_t sequence)
{
  const uint8_t totalFragments = (uint8_t)(
      (frameLength + RAW_FRAGMENT_SIZE - 1U) /
      RAW_FRAGMENT_SIZE);

  if ((totalFragments == 0U) ||
      (totalFragments > RTCM_MAX_FRAGMENTS))
  {
    return false;
  }

  for (uint8_t fragmentIndex = 0U;
       fragmentIndex < totalFragments;
       fragmentIndex++)
  {
    const size_t offset =
        (size_t)fragmentIndex * RAW_FRAGMENT_SIZE;

    const size_t rawLength =
        min(RAW_FRAGMENT_SIZE, frameLength - offset);

    char encoded[BASE64_FRAGMENT_MAX + 1U];

    const size_t encodedLength = Base64Encode(frame + offset,
                                              rawLength,
                                              encoded,
                                              sizeof(encoded));

    if (encodedLength == 0U)
    {
      return false;
    }

    char payload[LORA_PAYLOAD_MAX + 1U];

    const int headerLength = snprintf(payload,
                                      sizeof(payload),
                                      "R%04X%02X%02X",
                                      (unsigned int)sequence,
                                      (unsigned int)fragmentIndex,
                                      (unsigned int)totalFragments);

    if (headerLength != (int)LORA_FRAGMENT_HEADER_SIZE)
    {
      return false;
    }

    if (((size_t)headerLength + encodedLength) > LORA_PAYLOAD_MAX)
    {
      return false;
    }

    memcpy(payload + headerLength, encoded, encodedLength + 1U);

    const size_t payloadLength =
        (size_t)headerLength + encodedLength;

    if (!SendLoRaPayload(payload, payloadLength))
    {
      return false;
    }

    /* Keep draining the GNSS UART between radio fragments. */
    ServiceX20PInput(1024U);
  }

  return true;
}

static bool TransmitRtcmFrameReliably(const uint8_t *frame,
                                      size_t frameLength,
                                      uint16_t messageType)
{
  if ((frame == nullptr) ||
      (frameLength < 6U) ||
      (frameLength > RTCM_MAX_FRAME))
  {
    return false;
  }

  const uint16_t sequence = nextFrameSequence++;

  for (uint8_t attempt = 0U;
       attempt < MAX_FRAME_ATTEMPTS;
       attempt++)
  {
    if (attempt > 0U)
    {
      statRetriedFrames++;
      Serial.printf("Retry RTCM %u seq=%04X attempt=%u/%u\n",
                    (unsigned int)messageType,
                    (unsigned int)sequence,
                    (unsigned int)(attempt + 1U),
                    (unsigned int)MAX_FRAME_ATTEMPTS);
    }

    /* Ignore a stale ACK with the same sequence before this attempt. */
    if (pendingAckValid && (pendingAckSequence == sequence))
    {
      pendingAckValid = false;
    }

    if (!SendAllFragments(frame, frameLength, sequence))
    {
      delay(RETRY_GAP_MS);
      continue;
    }

    statSentFrames++;

    if (!REQUIRE_ROVER_ACK)
    {
      Serial.printf("RTCM %u sent seq=%04X len=%u queue=%u\n",
                    (unsigned int)messageType,
                    (unsigned int)sequence,
                    (unsigned int)frameLength,
                    (unsigned int)queueCount);
      return true;
    }

    if (WaitForFrameAck(sequence, FRAME_ACK_TIMEOUT_MS))
    {
      statAcknowledgedFrames++;
      Serial.printf("RTCM %u ACK seq=%04X len=%u queue=%u\n",
                    (unsigned int)messageType,
                    (unsigned int)sequence,
                    (unsigned int)frameLength,
                    (unsigned int)queueCount);
      return true;
    }

    Serial.printf("ACK timeout RTCM %u seq=%04X\n",
                  (unsigned int)messageType,
                  (unsigned int)sequence);

    delay(RETRY_GAP_MS);
  }

  statFailedFrames++;

  Serial.printf("DROP RTCM %u seq=%04X after %u attempts\n",
                (unsigned int)messageType,
                (unsigned int)sequence,
                (unsigned int)MAX_FRAME_ATTEMPTS);

  return false;
}

/* ------------------------------ DIAGNOSTICS ------------------------------- */

static void PrintStatusIfDue(void)
{
  const uint32_t nowMs = millis();

  if ((nowMs - lastStatusPrintMs) < STATUS_PRINT_INTERVAL_MS)
  {
    return;
  }

  lastStatusPrintMs = nowMs;

  Serial.printf(
      "STATUS bytes=%lu valid=%lu invalid=%lu queue=%u queued=%lu replaced=%lu "
      "dropped=%lu sent=%lu ack=%lu retries=%lu failed=%lu fragments=%lu atErr=%lu\n",
      (unsigned long)statRtcmBytesRead,
      (unsigned long)statValidRtcmFrames,
      (unsigned long)statInvalidRtcmFrames,
      (unsigned int)queueCount,
      (unsigned long)statQueuedFrames,
      (unsigned long)statReplacedQueuedFrames,
      (unsigned long)statDroppedQueuedFrames,
      (unsigned long)statSentFrames,
      (unsigned long)statAcknowledgedFrames,
      (unsigned long)statRetriedFrames,
      (unsigned long)statFailedFrames,
      (unsigned long)statSentFragments,
      (unsigned long)statAtCommandErrors);
}

/* -------------------------------- SETUP ---------------------------------- */

void setup()
{
  Serial.begin(USB_DEBUG_BAUD);
  delay(300U);

  /* Large hardware RX buffer protects against LoRa blocking time. */
  X20P.setRxBufferSize(32768U);
  X20P.begin(GNSS_BAUD,
             SERIAL_8N1,
             X20P_RX_PIN,
             X20P_TX_PIN);

  if (!ConfigureLoRa())
  {
    Serial.println("FATAL: LoRa base configuration failed.");

    while (true)
    {
      delay(1000U);
    }
  }

  Serial.println();
  Serial.println("ESP32 X20P -> RYLR896 RTCM base bridge READY");
  Serial.printf("X20P UART=%lu, LoRa UART=%lu, base=%u, rover=%u, network=%u\n",
                (unsigned long)GNSS_BAUD,
                (unsigned long)LORA_TARGET_BAUD,
                (unsigned int)BASE_ADDRESS,
                (unsigned int)ROVER_ADDRESS,
                (unsigned int)NETWORK_ID);
  Serial.printf("RF: %s | %s | %s\n",
                LORA_FREQUENCY,
                LORA_RF_PARAMETER,
                LORA_RF_POWER);
  Serial.printf("Frame ACK=%s, max attempts=%u\n\n",
                REQUIRE_ROVER_ACK ? "ON" : "OFF",
                (unsigned int)MAX_FRAME_ATTEMPTS);

  lastStatusPrintMs = millis();
}

void loop()
{
  PumpLoRaInput();
  ServiceX20PInput();

  RtcmQueueItem *item = nullptr;

  if (PeekQueuedFrame(&item))
  {
    /*
     * Copy and remove the queue head before transmitting. While LoRa blocks,
     * newer RTCM of the same type can safely replace queued data without
     * changing the frame currently being fragmented.
     */
    activeTransmitItem.length = item->length;
    activeTransmitItem.messageType = item->messageType;
    memcpy(activeTransmitItem.data, item->data, item->length);
    PopQueuedFrame();

    (void)TransmitRtcmFrameReliably(activeTransmitItem.data,
                                     activeTransmitItem.length,
                                     activeTransmitItem.messageType);
  }

  PrintStatusIfDue();
}
