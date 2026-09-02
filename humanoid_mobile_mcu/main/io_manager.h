#ifndef IO_MANAGER_H_
#define IO_MANAGER_H_

#include "config.h"
#include "protocol.h"
#include <Arduino.h>

class IOManager {
public:
  IOManager();
  void init();
  void update();

  // Command handlers
  bool handlePowerCtrl(const CmdPowerCtrlPayload &cmd);
  bool handleSignalCtrl(const CmdSignalCtrlPayload &cmd);
  bool handleBmsCtrl(const CmdBmsCtrlPayload &cmd);

  // Status accessors
  bool isEmergency() const { return _safety_ems; }
  bool isCharging() const { return _is_charging; }
  bool isNavPCLive() const { return _nav_pc_live; }
  bool isHumanoidPCLive() const { return _humanoid_pc_live; }
  bool isAiPCLive() const { return _ai_pc_live; }
  float getChargeCurrent() const { return _chg_current_adc; }

  // Telemetry builder
  void populateTelemetry(RobotTelemetryPayload &payload) const;

  // Manual pulse triggers
  void pulseNavPC();
  void pulseHumanoidPC();
  void pulseAiPC();

private:
  void pollInputs();
  void pollADC();
  void handlePowerButtonSequence();
  void handleHeartbeat();
  void updateLampsAndRelays();

  // Input states
  bool _pwr_btn_raw;
  bool _start_btn;
  bool _stop_btn;
  bool _manu_sw;

  bool _nav_pc_live;
  bool _humanoid_pc_live;
  bool _ai_pc_live;
  bool _safety_ems;
  bool _manu_chg_dock;

  float _chg_current_adc;
  bool _is_charging;

  // Output states
  bool _pwr_hold;
  bool _pwr_btn_lamp;
  bool _start_btn_lamp;
  bool _stop_btn_lamp;
  bool _volt24v_1;
  bool _volt24v_2;
  bool _chg_on_enable;
  bool _tower_lamp_red;
  bool _tower_lamp_grn;
  bool _tower_lamp_yel;
  bool _buzzer;
  bool _tm_remote;
  bool _live_led;

  // Pulse timers
  unsigned long _nav_pc_pulse_end;
  unsigned long _humanoid_pc_pulse_end;
  unsigned long _ai_pc_pulse_end;

  // Button timing for Power Button
  unsigned long _pwr_btn_press_start;
  bool _pwr_btn_held;
  bool _shutdown_notified;
  bool _force_off_triggered;

  // Scheduling timers
  unsigned long _last_button_poll_time;
  unsigned long _last_adc_poll_time;
  unsigned long _last_heartbeat_time;
};

#endif // IO_MANAGER_H_
