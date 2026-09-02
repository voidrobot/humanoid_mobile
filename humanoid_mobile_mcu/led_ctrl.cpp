#include "led_ctrl.h"

// Standard NeoPixel Colors
static inline uint32_t colorRGB(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static const uint32_t C_RED = 0xFF0000;
static const uint32_t C_GREEN = 0x00FF00;
static const uint32_t C_BLUE = 0x0000FF;
static const uint32_t C_WHITE = 0xFFFFFF;
static const uint32_t C_YELLOW = 0xFFFF00;
static const uint32_t C_ORANGE = 0xFF5500;
static const uint32_t C_INDIGO = 0x4B0082;
static const uint32_t C_OFF = 0x000000;

LedCtrl::LedCtrl()
    : _strip(LED_INDI_NUM, PIN_LED_INDI, NEO_GRB + NEO_KHZ800),
      _requested_mode(LED_INDI_DEFAULT_MODE),
      _active_mode(LED_INDI_DEFAULT_MODE), _is_emergency(false),
      _is_charging(false), _charging_soc(0.0f), _custom_r(255), _custom_g(255),
      _custom_b(255), _brightness(LED_BRIGHT_DEFAULT), _anim_step(0),
      _blink_counter(0), _blink_state(true), _fade_brightness(0),
      _fade_direction(5), _last_update_time(0) {}

void LedCtrl::init() {
  _strip.begin();
  _strip.setBrightness(_brightness);
  clearAll();
  _strip.show();
}

void LedCtrl::setMode(uint8_t mode) {
  if (_requested_mode != mode) {
    _requested_mode = mode;
    _anim_step = 0;
    _blink_counter = 0;
    _blink_state = true;
    _fade_brightness = 0;
    _fade_direction = 5;
  }
}

void LedCtrl::setCustomColor(uint8_t r, uint8_t g, uint8_t b,
                             uint8_t brightness) {
  _custom_r = r;
  _custom_g = g;
  _custom_b = b;
  _brightness = brightness;
  _strip.setBrightness(_brightness);
}

void LedCtrl::setEmergency(bool is_emergency) { _is_emergency = is_emergency; }

void LedCtrl::setCharging(bool is_charging, float soc) {
  _is_charging = is_charging;
  _charging_soc = soc;
}

void LedCtrl::clearAll() {
  for (uint16_t i = 0; i < LED_INDI_NUM; i++) {
    _strip.setPixelColor(i, C_OFF);
  }
}

void LedCtrl::setAllColor(uint32_t color) {
  for (uint16_t i = 0; i < LED_INDI_NUM; i++) {
    _strip.setPixelColor(i, color);
  }
}

void LedCtrl::updateBlink(uint32_t color, uint16_t period_frames) {
  _blink_counter++;
  if (_blink_counter >= period_frames) {
    _blink_counter = 0;
    _blink_state = !_blink_state;
  }
  setAllColor(_blink_state ? color : C_OFF);
}

void LedCtrl::updateWipe(uint32_t color) {
  _anim_step++;
  if (_anim_step >= LED_INDI_NUM) {
    _anim_step = 0;
    clearAll();
  }
  _strip.setPixelColor(_anim_step, color);
}

void LedCtrl::updateFade(uint8_t r, uint8_t g, uint8_t b) {
  _fade_brightness += _fade_direction;
  if (_fade_brightness >= 255) {
    _fade_brightness = 255;
    _fade_direction = -5;
  } else if (_fade_brightness <= 0) {
    _fade_brightness = 0;
    _fade_direction = 5;
  }

  uint8_t cur_r = (uint16_t)r * _fade_brightness / 255;
  uint8_t cur_g = (uint16_t)g * _fade_brightness / 255;
  uint8_t cur_b = (uint16_t)b * _fade_brightness / 255;
  setAllColor(colorRGB(cur_r, cur_g, cur_b));
}

void LedCtrl::updateIndiBlink(uint32_t color) {
  _blink_counter++;
  if (_blink_counter >= 6) {
    _blink_counter = 0;
    _blink_state = !_blink_state;
  }
  setAllColor(_blink_state ? color : C_OFF);
}

void LedCtrl::updateIndiRight(uint32_t color) {
  _blink_counter++;
  if (_blink_counter >= 5) {
    _blink_counter = 0;
    _blink_state = !_blink_state;
  }
  clearAll();
  if (_blink_state) {
    // Right section LEDs (pixels 24~72)
    for (uint16_t i = 24; i < 72 && i < LED_INDI_NUM; i++) {
      _strip.setPixelColor(i, color);
    }
  }
}

void LedCtrl::updateIndiLeft(uint32_t color) {
  _blink_counter++;
  if (_blink_counter >= 5) {
    _blink_counter = 0;
    _blink_state = !_blink_state;
  }
  clearAll();
  if (_blink_state) {
    // Left section LEDs (pixels 0~24 and 72~96)
    for (uint16_t i = 0; i < 24 && i < LED_INDI_NUM; i++) {
      _strip.setPixelColor(i, color);
    }
    for (uint16_t i = 72; i < 96 && i < LED_INDI_NUM; i++) {
      _strip.setPixelColor(i, color);
    }
  }
}

void LedCtrl::updateChargingGauge(float soc) {
  clearAll();
  uint16_t fill_count = (uint16_t)((soc / 100.0f) * LED_INDI_NUM);
  if (fill_count > LED_INDI_NUM)
    fill_count = LED_INDI_NUM;

  for (uint16_t i = 0; i < fill_count; i++) {
    _strip.setPixelColor(i, C_GREEN);
  }

  // Blinking top segment to show active charging
  _blink_counter++;
  if (_blink_counter >= 8) {
    _blink_counter = 0;
    _blink_state = !_blink_state;
  }
  if (_blink_state && fill_count < LED_INDI_NUM) {
    _strip.setPixelColor(fill_count, C_YELLOW);
  }
}

void LedCtrl::renderFrame() {
  // 1. Emergency mode has highest priority
  if (_is_emergency) {
    _active_mode = INDI_FULL_RED;
    setAllColor(C_RED);
    _strip.show();
    return;
  }

  // 2. Charging mode
  if (_is_charging) {
    _active_mode = BLINK_CHG_PWR_OFF;
    updateChargingGauge(_charging_soc);
    _strip.show();
    return;
  }

  // 3. Normal operating modes
  _active_mode = _requested_mode;

  switch (_active_mode) {
  case LED_OFF:
    clearAll();
    break;

  // Full Solid Colors
  case FULL_WHITE:
  case INDI_FULL_WHITE:
    setAllColor(C_WHITE);
    break;
  case FULL_RED:
  case INDI_FULL_RED:
    setAllColor(C_RED);
    break;
  case FULL_GREEN:
  case INDI_FULL_GREEN:
    setAllColor(C_GREEN);
    break;
  case FULL_BLUE:
  case INDI_FULL_BLUE:
    setAllColor(C_BLUE);
    break;
  case FULL_ORANGE:
  case INDI_FULL_ORANGE:
    setAllColor(C_ORANGE);
    break;
  case FULL_YELLOW:
  case INDI_FULL_YELLOW:
    setAllColor(C_YELLOW);
    break;
  case FULL_INDIGO:
    setAllColor(C_INDIGO);
    break;

  // Color Wipes
  case RED_WIPE:
    updateWipe(C_RED);
    break;
  case GREEN_WIPE:
    updateWipe(C_GREEN);
    break;
  case BLUE_WIPE:
    updateWipe(C_BLUE);
    break;
  case WHITE_WIPE:
    updateWipe(C_WHITE);
    break;
  case ORANGE_WIPE:
    updateWipe(C_ORANGE);
    break;
  case YELLOW_WIPE:
    updateWipe(C_YELLOW);
    break;
  case INDIGO_WIPE:
    updateWipe(C_INDIGO);
    break;

  // Fades
  case FADE_WHITE:
    updateFade(255, 255, 255);
    break;
  case FADE_RED:
    updateFade(255, 0, 0);
    break;
  case FADE_GREEN:
    updateFade(0, 255, 0);
    break;
  case FADE_BLUE:
    updateFade(0, 0, 255);
    break;
  case FADE_YELLOW:
    updateFade(255, 255, 0);
    break;
  case FADE_ORANGE:
    updateFade(255, 85, 0);
    break;
  case FADE_INDIGO:
    updateFade(75, 0, 130);
    break;

  // Standard Blinks
  case BLINK_RED:
    updateBlink(C_RED);
    break;
  case BLINK_GREEN:
    updateBlink(C_GREEN);
    break;
  case BLINK_BLUE:
    updateBlink(C_BLUE);
    break;
  case BLINK_WHITE:
    updateBlink(C_WHITE);
    break;
  case BLINK_YELLOW:
    updateBlink(C_YELLOW);
    break;
  case BLINK_ORANGE:
    updateBlink(C_ORANGE);
    break;
  case BLINK_INDIGO:
    updateBlink(C_INDIGO);
    break;

  // Indicator Blinks (Bottom/Side)
  case INDI_BLINK_RED:
    updateIndiBlink(C_RED);
    break;
  case INDI_BLINK_GREEN:
    updateIndiBlink(C_GREEN);
    break;
  case INDI_BLINK_BLUE:
    updateIndiBlink(C_BLUE);
    break;
  case INDI_BLINK_WHITE:
    updateIndiBlink(C_WHITE);
    break;
  case INDI_BLINK_YELLOW:
    updateIndiBlink(C_YELLOW);
    break;
  case INDI_BLINK_ORANGE:
    updateIndiBlink(C_ORANGE);
    break;

  // Directional Blinks
  case INDI_RIGHT_BLINK_RED:
    updateIndiRight(C_RED);
    break;
  case INDI_RIGHT_BLINK_GREEN:
    updateIndiRight(C_GREEN);
    break;
  case INDI_RIGHT_BLINK_BLUE:
    updateIndiRight(C_BLUE);
    break;
  case INDI_RIGHT_BLINK_WHITE:
    updateIndiRight(C_WHITE);
    break;
  case INDI_RIGHT_BLINK_YELLOW:
    updateIndiRight(C_YELLOW);
    break;
  case INDI_RIGHT_BLINK_ORANGE:
    updateIndiRight(C_ORANGE);
    break;

  case INDI_LEFT_BLINK_RED:
    updateIndiLeft(C_RED);
    break;
  case INDI_LEFT_BLINK_GREEN:
    updateIndiLeft(C_GREEN);
    break;
  case INDI_LEFT_BLINK_BLUE:
    updateIndiLeft(C_BLUE);
    break;
  case INDI_LEFT_BLINK_WHITE:
    updateIndiLeft(C_WHITE);
    break;
  case INDI_LEFT_BLINK_YELLOW:
    updateIndiLeft(C_YELLOW);
    break;
  case INDI_LEFT_BLINK_ORANGE:
    updateIndiLeft(C_ORANGE);
    break;

  default:
    setAllColor(C_OFF);
    break;
  }

  _strip.show();
}

void LedCtrl::update() {
  unsigned long now = millis();
  if (now - _last_update_time >= INTERVAL_LED_UPDATE_MS) {
    _last_update_time = now;
    renderFrame();
  }
}
