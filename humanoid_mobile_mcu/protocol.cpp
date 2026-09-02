#include "protocol.h"

// =============================================================================
// CRC-16-CCITT (Polynomial 0x1021, Initial value 0xFFFF)
// =============================================================================
uint16_t calcCRC16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// =============================================================================
// Packet Parser State Machine
// =============================================================================
PacketParser::PacketParser()
    : _state(STATE_HEADER_1), _msg_id(0), _length(0), _payload_index(0),
      _received_crc(0), _callback(nullptr) {}

void PacketParser::setCallback(PacketHandlerCallback callback) {
  _callback = callback;
}

void PacketParser::parseByte(uint8_t byte) {
  switch (_state) {
  case STATE_HEADER_1:
    if (byte == PROTOCOL_HEADER_1) {
      _state = STATE_HEADER_2;
    }
    break;

  case STATE_HEADER_2:
    if (byte == PROTOCOL_HEADER_2) {
      _state = STATE_MSG_ID;
    } else if (byte == PROTOCOL_HEADER_1) {
      _state = STATE_HEADER_2; // consecutive 0xAA
    } else {
      _state = STATE_HEADER_1;
    }
    break;

  case STATE_MSG_ID:
    _msg_id = byte;
    _state = STATE_LENGTH;
    break;

  case STATE_LENGTH:
    _length = byte;
    if (_length > PROTOCOL_MAX_PAYLOAD_SIZE) {
      // Invalid length, reset parser (if this byte is HEADER_1, transition to
      // HEADER_2)
      if (byte == PROTOCOL_HEADER_1) {
        _state = STATE_HEADER_2;
      } else {
        _state = STATE_HEADER_1;
      }
    } else if (_length == 0) {
      // Zero-length payload, jump straight to CRC
      _state = STATE_CRC_L;
    } else {
      _payload_index = 0;
      _state = STATE_PAYLOAD;
    }
    break;

  case STATE_PAYLOAD:
    _payload[_payload_index++] = byte;
    if (_payload_index >= _length) {
      _state = STATE_CRC_L;
    }
    break;

  case STATE_CRC_L:
    _received_crc = byte;
    _state = STATE_CRC_H;
    break;

  case STATE_CRC_H: {
    _received_crc |= ((uint16_t)byte << 8);

    // Verify CRC over Header(2) + MsgId(1) + Len(1) + Payload(N)
    uint8_t check_buf[4 + PROTOCOL_MAX_PAYLOAD_SIZE];
    check_buf[0] = PROTOCOL_HEADER_1;
    check_buf[1] = PROTOCOL_HEADER_2;
    check_buf[2] = _msg_id;
    check_buf[3] = _length;
    if (_length > 0) {
      memcpy(&check_buf[4], _payload, _length);
    }

    uint16_t computed_crc = calcCRC16(check_buf, 4 + _length);
    if (computed_crc == _received_crc) {
      if (_callback != nullptr) {
        _callback(_msg_id, _payload, _length);
      }
    }

    // Reset parser to seek next header
    _state = STATE_HEADER_1;
    break;
  }

  default:
    _state = STATE_HEADER_1;
    break;
  }
}

// =============================================================================
// Packet Transmission Function
// =============================================================================
void sendPacket(HardwareSerial &serial, uint8_t msg_id, const void *payload,
                uint8_t length) {
  if (length > PROTOCOL_MAX_PAYLOAD_SIZE) {
    length = PROTOCOL_MAX_PAYLOAD_SIZE;
  }

  uint8_t tx_buf[4 + PROTOCOL_MAX_PAYLOAD_SIZE + 2];
  tx_buf[0] = PROTOCOL_HEADER_1;
  tx_buf[1] = PROTOCOL_HEADER_2;
  tx_buf[2] = msg_id;
  tx_buf[3] = length;

  if (length > 0 && payload != nullptr) {
    memcpy(&tx_buf[4], payload, length);
  }

  uint16_t crc = calcCRC16(tx_buf, 4 + length);
  tx_buf[4 + length] = (uint8_t)(crc & 0xFF);            // CRC Low byte
  tx_buf[4 + length + 1] = (uint8_t)((crc >> 8) & 0xFF); // CRC High byte

  serial.write(tx_buf, 4 + length + 2);
}
