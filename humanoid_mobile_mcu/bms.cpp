#include "bms.h"

BMSManager::BMSManager()
    : _voltage(0.0f), _current(0.0f), _soc(0.0f), _soh(0.0f), _temp(0.0f),
      _is_valid(false), _rx_index(0), _waiting_response(false),
      _last_query_time(0), _last_valid_time(0) {}

void BMSManager::init() {
  pinMode(PIN_BMS_SW, OUTPUT);
  digitalWrite(PIN_BMS_SW, HIGH);

  Serial3.begin(BMS_SERIAL_BAUD_RATE);
  reset();
}

void BMSManager::reset() {
  // Pulse sequence to wake up / reset BMS
  digitalWrite(PIN_BMS_SW, HIGH);
  delay(100);
  digitalWrite(PIN_BMS_SW, LOW);
  delay(500);
  digitalWrite(PIN_BMS_SW, HIGH);
  delay(500);
}

void BMSManager::sendQueryPacket() {
  uint8_t cmd[11];
  long chkSum = 0;

  cmd[0] = 0xAF;
  cmd[1] = 0xFA;
  cmd[2] = 0x60; // address
  cmd[3] = 0x05; // length
  cmd[4] = 0x01; // command
  cmd[5] = 0x60; // order
  cmd[6] = 0x47; // kind1 (voltage, current, SOC, temp)
  cmd[7] = 0x01; // kind2 (SOH)

  for (int i = 2; i < 8; i++) {
    chkSum += cmd[i];
  }
  cmd[8] = (uint8_t)(chkSum & 0xFF);
  cmd[9] = 0xAF;
  cmd[10] = 0xA0;

  Serial3.write(cmd, 11);
  _waiting_response = true;
  _rx_index = 0;
}

bool BMSManager::parsePacket(const uint8_t *buffer, uint8_t length) {
  if (length < 18) {
    return false;
  }

  // Calculate and verify checksum
  // Protocol: packet from index 2 to (checksum_index - 1)
  int checksum_index = -1;
  for (int i = 0; i < length - 1; i++) {
    if (buffer[i] == 0xAF && buffer[i + 1] == 0xA0) {
      checksum_index = i - 1;
      break;
    }
  }

  if (checksum_index <= 2) {
    return false;
  }

  long chkSum = 0;
  for (int i = 2; i < checksum_index; i++) {
    chkSum += buffer[i];
  }

  if ((uint8_t)(chkSum & 0xFF) != buffer[checksum_index]) {
    return false;
  }

  int raw_v = (int)buffer[6] | ((int)buffer[7] << 8);
  int raw_c = (int)buffer[8] | ((int)buffer[9] << 8);
  int raw_soc = (int)buffer[10] | ((int)buffer[11] << 8);
  int raw_temp = (int)buffer[12] | ((int)buffer[13] << 8);
  int raw_soh = (int)buffer[14] | ((int)buffer[15] << 8);

  _voltage = raw_v * 0.01f;
  _current = raw_c * 0.01f;
  _soc = (float)raw_soc;
  _temp = raw_temp * 0.1f;
  _soh = (float)raw_soh;
  _is_valid = true;
  _last_valid_time = millis();

  return true;
}

void BMSManager::poll() {
  unsigned long now = millis();

  // Send query packet every 1000ms
  if (!_waiting_response && (now - _last_query_time >= INTERVAL_BMS_MS)) {
    sendQueryPacket();
    _last_query_time = now;
  }

  // Read available bytes from Serial3
  while (Serial3.available() > 0) {
    uint8_t b = Serial3.read();
    if (_rx_index < sizeof(_rx_buffer)) {
      _rx_buffer[_rx_index++] = b;
    } else {
      // Overflow, reset buffer
      _rx_index = 0;
    }

    // Check for end of packet: 0xAF 0xA0
    if (_rx_index >= 2 && _rx_buffer[_rx_index - 2] == 0xAF &&
        _rx_buffer[_rx_index - 1] == 0xA0) {
      parsePacket(_rx_buffer, _rx_index);
      _rx_index = 0;
      _waiting_response = false;
      break;
    }
  }

  // Timeout if no response within 500ms
  if (_waiting_response && (now - _last_query_time > 500)) {
    _waiting_response = false;
    _rx_index = 0;
  }

  // Invalidate after 5 seconds of no valid response
  if (_is_valid && (now - _last_valid_time > 5000)) {
    _is_valid = false;
  }
}

void BMSManager::update() { poll(); }

void BMSManager::populateTelemetry(BmsTelemetryPayload &payload) const {
  payload.voltage = _voltage;
  payload.current = _current;
  payload.soc = _soc;
  payload.soh = _soh;
  payload.temp = _temp;
  payload.is_valid = _is_valid ? 1 : 0;
}
