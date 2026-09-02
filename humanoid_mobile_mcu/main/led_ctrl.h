#ifndef LED_CTRL_H_
#define LED_CTRL_H_

#include "config.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

class LedCtrl {
public:
  LedCtrl();
  void init();
  void setMode(uint8_t mode);
  void setCustomColor(uint8_t r, uint8_t g, uint8_t b,
                      uint8_t brightness = 255);
  void setEmergency(bool is_emergency);
  void setCharging(bool is_charging, float soc = 0.0f);
  void update();

  uint8_t getActiveMode() const { return _active_mode; }

private:
  void renderFrame();
  void clearAll();
  void setAllColor(uint32_t color);

  // Animation routines
  void updateBlink(uint32_t color, uint16_t period_frames = 10);
  void updateWipe(uint32_t color);
  void updateFade(uint8_t r, uint8_t g, uint8_t b);
  void updateIndiBlink(uint32_t color);
  void updateIndiRight(uint32_t color);
  void updateIndiLeft(uint32_t color);
  void updateChargingGauge(float soc);

  Adafruit_NeoPixel _strip;

  uint8_t _requested_mode;
  uint8_t _active_mode;
  bool _is_emergency;
  bool _is_charging;
  float _charging_soc;

  uint8_t _custom_r;
  uint8_t _custom_g;
  uint8_t _custom_b;
  uint8_t _brightness;

  uint16_t _anim_step;
  uint16_t _blink_counter;
  bool _blink_state;
  int16_t _fade_brightness;
  int8_t _fade_direction;
  unsigned long _last_update_time;
};

#endif // LED_CTRL_H_
