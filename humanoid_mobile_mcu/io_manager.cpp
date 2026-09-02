#include "io_manager.h"

IOManager::IOManager()
    : _pwr_btn_raw(false), _start_btn(false), _stop_btn(false), _manu_sw(false),
      _nav_pc_live(false), _humanoid_pc_live(false), _ai_pc_live(false),
      _safety_ems(false), _manu_chg_dock(false), _chg_current_adc(0.0f),
      _is_charging(false), _pwr_hold(false), _pwr_btn_lamp(false),
      _start_btn_lamp(false), _stop_btn_lamp(false), _volt24v_1(true),
      _volt24v_2(true), _chg_on_enable(false), _tower_lamp_red(false),
      _tower_lamp_grn(false), _tower_lamp_yel(false), _buzzer(false),
      _tm_remote(false), _live_led(false), _nav_pc_pulse_end(0),
      _humanoid_pc_pulse_end(0), _ai_pc_pulse_end(0), _pwr_btn_press_start(0),
      _pwr_btn_held(false), _shutdown_notified(false),
      _force_off_triggered(false), _last_button_poll_time(0),
      _last_adc_poll_time(0), _last_heartbeat_time(0) {}

void IOManager::init() {
  // --- Input Pins ---
  pinMode(PIN_PWR_BTN_IN, INPUT);
  pinMode(PIN_MANU_SW_IN, INPUT);
  pinMode(PIN_START_BTN_IN, INPUT);
  pinMode(PIN_STOP_BTN_IN, INPUT);
  pinMode(PIN_PC_PWR_LIVE, INPUT);
  pinMode(PIN_HUMANOID_PC_LIVE, INPUT);
  pinMode(PIN_AI_PC_LIVE, INPUT);
  pinMode(PIN_SPARE_IN_3, INPUT);
  pinMode(PIN_SPARE_IN_4, INPUT);
  pinMode(PIN_SAFETY_EMS, INPUT);
  pinMode(PIN_MANU_CHG_DET, INPUT);
  pinMode(PIN_CHG_DET_ADC, INPUT);

  // --- Output Pins ---
  pinMode(PIN_PWR_BTN_LAMP, OUTPUT);
  pinMode(PIN_SW1_OUT2, OUTPUT);
  pinMode(PIN_START_BTN_LAMP, OUTPUT);
  pinMode(PIN_STOP_BTN_LAMP, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_PC_PWR_SW, OUTPUT);
  pinMode(PIN_TM_RMT, OUTPUT);
  pinMode(PIN_HUMANOID_PC_ON, OUTPUT);
  pinMode(PIN_AI_PC_ON, OUTPUT);
  pinMode(PIN_SPARE_OUT_3, OUTPUT);
  pinMode(PIN_SPARE_OUT_4, OUTPUT);
  pinMode(PIN_LAMP_RED, OUTPUT);
  pinMode(PIN_LAMP_GRN, OUTPUT);
  pinMode(PIN_LAMP_YEL, OUTPUT);
  pinMode(PIN_CHG_ON_ENABLE, OUTPUT);
  pinMode(PIN_PWR_HOLD, OUTPUT);
  pinMode(PIN_VOLT24V_1, OUTPUT);
  pinMode(PIN_VOLT24V_2, OUTPUT);
  pinMode(PIN_LIVE_LED, OUTPUT);

  // Initial Output States
  digitalWrite(PIN_PC_PWR_SW, LOW);
  digitalWrite(PIN_HUMANOID_PC_ON, LOW);
  digitalWrite(PIN_AI_PC_ON, LOW);
  digitalWrite(PIN_SW1_OUT2, LOW);
  digitalWrite(PIN_SPARE_OUT_3, LOW);
  digitalWrite(PIN_SPARE_OUT_4, LOW);

  digitalWrite(PIN_PWR_BTN_LAMP, LOW);
  digitalWrite(PIN_START_BTN_LAMP, LOW);
  digitalWrite(PIN_STOP_BTN_LAMP, HIGH); // Default Stop lamp ON
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_TM_RMT, LOW);

  digitalWrite(PIN_LAMP_RED, LOW);
  digitalWrite(PIN_LAMP_GRN, LOW);
  digitalWrite(PIN_LAMP_YEL, LOW);

  digitalWrite(PIN_CHG_ON_ENABLE, LOW);
  digitalWrite(PIN_PWR_HOLD, LOW);
  digitalWrite(PIN_VOLT24V_1, HIGH); // 24V Conv 1 Active
  digitalWrite(PIN_VOLT24V_2, HIGH); // 24V Conv 2 Active
  digitalWrite(PIN_LIVE_LED, LOW);

  pollInputs();
}

void IOManager::pollInputs() {
  _pwr_btn_raw = digitalRead(PIN_PWR_BTN_IN);
  _start_btn = digitalRead(PIN_START_BTN_IN);
  _stop_btn = digitalRead(PIN_STOP_BTN_IN);
  _manu_sw = digitalRead(PIN_MANU_SW_IN);

  _nav_pc_live = digitalRead(PIN_PC_PWR_LIVE);
  _humanoid_pc_live = digitalRead(PIN_HUMANOID_PC_LIVE);
  _ai_pc_live = digitalRead(PIN_AI_PC_LIVE);

  // Safety PLC EMS: PB7 (D13) is active low (when Emergency is triggered)
  _safety_ems = !digitalRead(PIN_SAFETY_EMS);
  _manu_chg_dock = digitalRead(PIN_MANU_CHG_DET);
}

void IOManager::pollADC() {
  int raw_adc = analogRead(PIN_CHG_DET_ADC);
  // Scale raw 0~1023 to voltage/current (5V reference)
  _chg_current_adc = (float)raw_adc * (5.0f / 1023.0f);

  // Charging active threshold detection
  _is_charging = (_chg_current_adc > 0.5f) || _manu_chg_dock;
}

void IOManager::pulseNavPC() {
  digitalWrite(PIN_PC_PWR_SW, HIGH);
  _nav_pc_pulse_end = millis() + PC_PULSE_DURATION_MS;
}

void IOManager::pulseHumanoidPC() {
  digitalWrite(PIN_HUMANOID_PC_ON, HIGH);
  _humanoid_pc_pulse_end = millis() + PC_PULSE_DURATION_MS;
}

void IOManager::pulseAiPC() {
  digitalWrite(PIN_AI_PC_ON, HIGH);
  _ai_pc_pulse_end = millis() + PC_PULSE_DURATION_MS;
}

void IOManager::handlePowerButtonSequence() {
  unsigned long now = millis();

  // Pulse end handling
  if (_nav_pc_pulse_end > 0 && now >= _nav_pc_pulse_end) {
    digitalWrite(PIN_PC_PWR_SW, LOW);
    _nav_pc_pulse_end = 0;
  }
  if (_humanoid_pc_pulse_end > 0 && now >= _humanoid_pc_pulse_end) {
    digitalWrite(PIN_HUMANOID_PC_ON, LOW);
    _humanoid_pc_pulse_end = 0;
  }
  if (_ai_pc_pulse_end > 0 && now >= _ai_pc_pulse_end) {
    digitalWrite(PIN_AI_PC_ON, LOW);
    _ai_pc_pulse_end = 0;
  }

  // Power Button state machine
  if (_pwr_btn_raw) {
    if (!_pwr_btn_held) {
      _pwr_btn_held = true;
      _pwr_btn_press_start = now;
      _shutdown_notified = false;
      _force_off_triggered = false;
    }

    unsigned long hold_duration = now - _pwr_btn_press_start;

    // Case 1: PC is currently OFF -> Turn ON immediately upon button press
    // (500ms pulse)
    if (!_nav_pc_live && hold_duration < 1000 && _nav_pc_pulse_end == 0 &&
        !_shutdown_notified) {
      pulseNavPC();
      _shutdown_notified = true; // prevent repeated pulse while holding
    }

    // Case 2: PC is ON and button held > 2.0s -> Normal Shutdown request
    if (_nav_pc_live && hold_duration >= POWER_BTN_PRESS_SHUTDOWN_MS &&
        !_shutdown_notified) {
      _shutdown_notified = true;
      // Power button lamp will blink to indicate shutdown initiated
      _pwr_btn_lamp = false;
    }

    // Case 3: Button held > 7.0s -> Forced Hard Power OFF
    if (hold_duration >= POWER_BTN_PRESS_FORCE_MS && !_force_off_triggered) {
      _force_off_triggered = true;
      digitalWrite(PIN_PC_PWR_SW, HIGH); // Assert power switch
      digitalWrite(PIN_PWR_HOLD, LOW);   // Release power hold
      _pwr_hold = false;
    }
  } else {
    // Button released
    if (_pwr_btn_held) {
      _pwr_btn_held = false;
      if (_force_off_triggered) {
        digitalWrite(PIN_PC_PWR_SW, LOW);
      }
    }
  }

  // Latch Power Hold and Power Button Lamp with Nav PC Boot State
  if (_nav_pc_live) {
    _pwr_hold = true;
    _pwr_btn_lamp = true;
    digitalWrite(PIN_PWR_HOLD, HIGH);
    digitalWrite(PIN_PWR_BTN_LAMP, HIGH);
  } else {
    if (!_pwr_btn_held) {
      _pwr_hold = false;
      _pwr_btn_lamp = false;
      digitalWrite(PIN_PWR_HOLD, LOW);
      digitalWrite(PIN_PWR_BTN_LAMP, LOW);
    }
  }
}

void IOManager::handleHeartbeat() {
  unsigned long now = millis();
  if (now - _last_heartbeat_time >= INTERVAL_HEARTBEAT_MS) {
    _last_heartbeat_time = now;
    _live_led = !_live_led;
    digitalWrite(PIN_LIVE_LED, _live_led ? HIGH : LOW);
  }
}

void IOManager::updateLampsAndRelays() {
  digitalWrite(PIN_START_BTN_LAMP, _start_btn_lamp ? HIGH : LOW);
  digitalWrite(PIN_STOP_BTN_LAMP, _stop_btn_lamp ? HIGH : LOW);

  // Emergency overrides buzzer & stop lamp
  if (_safety_ems) {
    digitalWrite(PIN_BUZZER, HIGH);
    digitalWrite(PIN_STOP_BTN_LAMP, HIGH);
    digitalWrite(PIN_LAMP_RED, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, _buzzer ? HIGH : LOW);
    digitalWrite(PIN_LAMP_RED, _tower_lamp_red ? HIGH : LOW);
  }

  digitalWrite(PIN_LAMP_GRN, _tower_lamp_grn ? HIGH : LOW);
  digitalWrite(PIN_LAMP_YEL, _tower_lamp_yel ? HIGH : LOW);
  digitalWrite(PIN_TM_RMT, _tm_remote ? HIGH : LOW);
  digitalWrite(PIN_VOLT24V_1, _volt24v_1 ? HIGH : LOW);
  digitalWrite(PIN_VOLT24V_2, _volt24v_2 ? HIGH : LOW);
  digitalWrite(PIN_CHG_ON_ENABLE, _chg_on_enable ? HIGH : LOW);
}

void IOManager::update() {
  unsigned long now = millis();

  if (now - _last_button_poll_time >= INTERVAL_BUTTON_POLL_MS) {
    _last_button_poll_time = now;
    pollInputs();
    handlePowerButtonSequence();
  }

  if (now - _last_adc_poll_time >= INTERVAL_ADC_POLL_MS) {
    _last_adc_poll_time = now;
    pollADC();
  }

  handleHeartbeat();
  updateLampsAndRelays();
}

bool IOManager::handlePowerCtrl(const CmdPowerCtrlPayload &cmd) {
  // Target: 1=Humanoid PC, 2=AI PC, 3=24V Conv 1, 4=24V Conv 2, 5=Main Nav PC
  switch (cmd.target_device) {
  case 1: // Humanoid PC
    if (cmd.action == 2) {
      pulseHumanoidPC();
    } else {
      digitalWrite(PIN_HUMANOID_PC_ON, cmd.action == 1 ? HIGH : LOW);
    }
    return true;

  case 2: // AI PC
    if (cmd.action == 2) {
      pulseAiPC();
    } else {
      digitalWrite(PIN_AI_PC_ON, cmd.action == 1 ? HIGH : LOW);
    }
    return true;

  case 3: // 24V Converter 1
    _volt24v_1 = (cmd.action == 1);
    digitalWrite(PIN_VOLT24V_1, _volt24v_1 ? HIGH : LOW);
    return true;

  case 4: // 24V Converter 2
    _volt24v_2 = (cmd.action == 1);
    digitalWrite(PIN_VOLT24V_2, _volt24v_2 ? HIGH : LOW);
    return true;

  case 5: // Main Nav PC
    if (cmd.action == 2) {
      pulseNavPC();
    } else if (cmd.action == 0) {
      // Soft power down / release hold
      _pwr_hold = false;
      digitalWrite(PIN_PWR_HOLD, LOW);
    }
    return true;

  default:
    return false;
  }
}

bool IOManager::handleSignalCtrl(const CmdSignalCtrlPayload &cmd) {
  if (cmd.lamp_red != 0xFF)
    _tower_lamp_red = (cmd.lamp_red == 1);
  if (cmd.lamp_grn != 0xFF)
    _tower_lamp_grn = (cmd.lamp_grn == 1);
  if (cmd.lamp_yel != 0xFF)
    _tower_lamp_yel = (cmd.lamp_yel == 1);
  if (cmd.buzzer != 0xFF)
    _buzzer = (cmd.buzzer == 1);
  if (cmd.tm_remote != 0xFF)
    _tm_remote = (cmd.tm_remote == 1);
  return true;
}

bool IOManager::handleBmsCtrl(const CmdBmsCtrlPayload &cmd) {
  _chg_on_enable = (cmd.chg_enable == 1);
  digitalWrite(PIN_CHG_ON_ENABLE, _chg_on_enable ? HIGH : LOW);
  return true;
}

void IOManager::populateTelemetry(RobotTelemetryPayload &payload) const {
  // 1. Button Status Bitfield
  payload.btn_status = 0;
  if (_pwr_btn_raw)
    payload.btn_status |= (1 << 0);
  if (_start_btn)
    payload.btn_status |= (1 << 1);
  if (_stop_btn)
    payload.btn_status |= (1 << 2);
  if (_manu_sw)
    payload.btn_status |= (1 << 3);

  // 2. PC Live States Bitfield
  payload.pc_live_status = 0;
  if (_nav_pc_live)
    payload.pc_live_status |= (1 << 0);
  if (_humanoid_pc_live)
    payload.pc_live_status |= (1 << 1);
  if (_ai_pc_live)
    payload.pc_live_status |= (1 << 2);

  // 3. Safety & Charging Bitfield
  payload.safety_status = 0;
  if (_safety_ems)
    payload.safety_status |= (1 << 0);
  if (_manu_chg_dock)
    payload.safety_status |= (1 << 1);
  if (_is_charging)
    payload.safety_status |= (1 << 2);

  // 4. Output Status Bitfield
  payload.output_status = 0;
  if (_pwr_hold)
    payload.output_status |= (1 << 0);
  if (_volt24v_1)
    payload.output_status |= (1 << 1);
  if (_volt24v_2)
    payload.output_status |= (1 << 2);
  if (_chg_on_enable)
    payload.output_status |= (1 << 3);
  if (_tower_lamp_red)
    payload.output_status |= (1 << 4);
  if (_tower_lamp_grn)
    payload.output_status |= (1 << 5);
  if (_tower_lamp_yel)
    payload.output_status |= (1 << 6);
  if (_buzzer)
    payload.output_status |= (1 << 7);

  // 5. Analog Measurements & Uptime
  payload.chg_current_adc = _chg_current_adc;
  payload.uptime_sec = millis() / 1000;
}
