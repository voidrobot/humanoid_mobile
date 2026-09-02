#ifndef BMS_H_
#define BMS_H_

#include "config.h"
#include "protocol.h"
#include <Arduino.h>

class BMSManager {
public:
  BMSManager();
  void init();
  void reset();
  void poll();
  void update();

  float getVoltage() const { return _voltage; }
  float getCurrent() const { return _current; }
  float getSoC() const { return _soc; }
  float getSoH() const { return _soh; }
  float getTemp() const { return _temp; }
  bool isValid() const { return _is_valid; }

  void populateTelemetry(BmsTelemetryPayload &payload) const;

private:
  void sendQueryPacket();
  bool parsePacket(const uint8_t *buffer, uint8_t length);

  float _voltage;
  float _current;
  float _soc;
  float _soh;
  float _temp;
  bool _is_valid;

  uint8_t _rx_buffer[64];
  uint8_t _rx_index;
  bool _waiting_response;
  unsigned long _last_query_time;
  unsigned long _last_valid_time;
};

#endif // BMS_H_
