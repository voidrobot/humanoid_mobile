/* ================================================================================
 * Mobile Humanoid MCU Firmware v1.0
 * Based on Mobile Humanoid Hardware I/O List ver 1.0 (ATmega2560)
 * High-Reliability Custom Binary Serial Protocol (57600 bps)
 * ================================================================================ */

#include "config.h"
#include "protocol.h"
#include "bms.h"
#include "led_ctrl.h"
#include "io_manager.h"

// =============================================================================
// Global Module Instances
// =============================================================================
static PacketParser g_parser;
static BMSManager   g_bms;
static LedCtrl      g_leds;
static IOManager    g_io;

// Timers for periodic telemetry
static unsigned long g_last_robot_telemetry_time = 0;
static unsigned long g_last_bms_telemetry_time   = 0;

// =============================================================================
// Command & Packet Dispatch Callback
// =============================================================================
void sendAck(uint8_t target_msg_id, uint8_t status_code) {
  AckNackPayload ack;
  ack.target_msg_id = target_msg_id;
  ack.status_code   = status_code;
  sendPacket(Serial, MSG_ID_ACK_NACK, &ack, sizeof(ack));
}

void onPacketReceived(uint8_t msg_id, const uint8_t* payload, uint8_t length) {
  switch (msg_id) {
    case MSG_ID_CMD_POWER_CTRL: {
      if (length < sizeof(CmdPowerCtrlPayload)) {
        sendAck(msg_id, ACK_STATUS_INVALID_LEN);
        return;
      }
      CmdPowerCtrlPayload cmd;
      memcpy(&cmd, payload, sizeof(cmd));
      bool ok = g_io.handlePowerCtrl(cmd);
      sendAck(msg_id, ok ? ACK_STATUS_OK : ACK_STATUS_EXEC_FAILED);
      break;
    }

    case MSG_ID_CMD_LED_CTRL: {
      if (length < sizeof(CmdLedCtrlPayload)) {
        sendAck(msg_id, ACK_STATUS_INVALID_LEN);
        return;
      }
      CmdLedCtrlPayload cmd;
      memcpy(&cmd, payload, sizeof(cmd));
      g_leds.setCustomColor(cmd.r, cmd.g, cmd.b, cmd.brightness);
      g_leds.setMode(cmd.mode);
      sendAck(msg_id, ACK_STATUS_OK);
      break;
    }

    case MSG_ID_CMD_SIGNAL_CTRL: {
      if (length < sizeof(CmdSignalCtrlPayload)) {
        sendAck(msg_id, ACK_STATUS_INVALID_LEN);
        return;
      }
      CmdSignalCtrlPayload cmd;
      memcpy(&cmd, payload, sizeof(cmd));
      bool ok = g_io.handleSignalCtrl(cmd);
      sendAck(msg_id, ok ? ACK_STATUS_OK : ACK_STATUS_EXEC_FAILED);
      break;
    }

    case MSG_ID_CMD_BMS_CTRL: {
      if (length < sizeof(CmdBmsCtrlPayload)) {
        sendAck(msg_id, ACK_STATUS_INVALID_LEN);
        return;
      }
      CmdBmsCtrlPayload cmd;
      memcpy(&cmd, payload, sizeof(cmd));
      if (cmd.bms_reset_pulse == 1) {
        g_bms.reset();
      }
      g_io.handleBmsCtrl(cmd);
      sendAck(msg_id, ACK_STATUS_OK);
      break;
    }

    case MSG_ID_CMD_REQUEST_INFO: {
      SystemInfoPayload info;
      memset(&info, 0, sizeof(info));
      strncpy(info.sw_ver, SW_VER, sizeof(info.sw_ver) - 1);
      strncpy(info.hw_ver, HW_VER, sizeof(info.hw_ver) - 1);
      sendPacket(Serial, MSG_ID_SYSTEM_INFO, &info, sizeof(info));
      break;
    }

    default:
      sendAck(msg_id, ACK_STATUS_INVALID_CMD);
      break;
  }
}

// =============================================================================
// Arduino Setup
// =============================================================================
void setup() {
  // 1. Initialize PC binary serial interface
  Serial.begin(PC_SERIAL_BAUD_RATE);

  // 2. Initialize hardware modules
  g_io.init();
  g_leds.init();
  g_bms.init();

  // 3. Register protocol callback
  g_parser.setCallback(onPacketReceived);

  // 4. Send initial system info packet upon startup
  SystemInfoPayload info;
  memset(&info, 0, sizeof(info));
  strncpy(info.sw_ver, SW_VER, sizeof(info.sw_ver) - 1);
  strncpy(info.hw_ver, HW_VER, sizeof(info.hw_ver) - 1);
  sendPacket(Serial, MSG_ID_SYSTEM_INFO, &info, sizeof(info));
}

// =============================================================================
// Arduino Main Loop (Non-blocking)
// =============================================================================
void loop() {
  // 1. Process incoming serial bytes from PC
  while (Serial.available() > 0) {
    g_parser.parseByte((uint8_t)Serial.read());
  }

  // 2. Update hardware IO manager (buttons, power hold, debouncing, alarms)
  g_io.update();

  // 3. Update BMS communication (1Hz polling on Serial3)
  g_bms.update();

  // 4. Update LED controller with safety/charging states
  g_leds.setEmergency(g_io.isEmergency());
  g_leds.setCharging(g_io.isCharging(), g_bms.getSoC());
  g_leds.update();

  // 5. Periodic Robot Telemetry Transmission (20Hz / 50ms)
  unsigned long now = millis();
  if (now - g_last_robot_telemetry_time >= INTERVAL_TELEMETRY_MS) {
    g_last_robot_telemetry_time = now;

    RobotTelemetryPayload telem;
    g_io.populateTelemetry(telem);
    telem.led_current_mode = g_leds.getActiveMode();

    sendPacket(Serial, MSG_ID_ROBOT_TELEMETRY, &telem, sizeof(telem));
  }

  // 6. Periodic BMS Telemetry Transmission (1Hz / 1000ms)
  if (now - g_last_bms_telemetry_time >= INTERVAL_BMS_MS) {
    g_last_bms_telemetry_time = now;

    BmsTelemetryPayload bms_telem;
    g_bms.populateTelemetry(bms_telem);

    sendPacket(Serial, MSG_ID_BMS_TELEMETRY, &bms_telem, sizeof(bms_telem));
  }
}
