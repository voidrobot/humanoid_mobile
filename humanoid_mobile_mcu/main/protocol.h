#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <Arduino.h>

// =============================================================================
// Protocol Framing Constants
// =============================================================================
#define PROTOCOL_HEADER_1 0xAA
#define PROTOCOL_HEADER_2 0x55
#define PROTOCOL_MAX_PAYLOAD_SIZE 64
#define PROTOCOL_MAX_PACKET_SIZE                                               \
  (4 + PROTOCOL_MAX_PAYLOAD_SIZE +                                             \
   2) // Header(2) + MsgId(1) + Len(1) + Payload(64) + CRC(2)

// =============================================================================
// Message IDs
// =============================================================================
// MCU -> PC Telemetry & Info
#define MSG_ID_ROBOT_TELEMETRY 0x01
#define MSG_ID_BMS_TELEMETRY 0x02
#define MSG_ID_SYSTEM_INFO 0x03
#define MSG_ID_ACK_NACK 0x04

// PC -> MCU Commands
#define MSG_ID_CMD_POWER_CTRL 0x10
#define MSG_ID_CMD_LED_CTRL 0x11
#define MSG_ID_CMD_SIGNAL_CTRL 0x12
#define MSG_ID_CMD_BMS_CTRL 0x13
#define MSG_ID_CMD_REQUEST_INFO 0x14

// =============================================================================
// ACK/NACK Status Codes
// =============================================================================
#define ACK_STATUS_OK 0x00
#define ACK_STATUS_CRC_ERROR 0x01
#define ACK_STATUS_INVALID_LEN 0x02
#define ACK_STATUS_INVALID_CMD 0x03
#define ACK_STATUS_EXEC_FAILED 0x04

// =============================================================================
// Packed Struct Definitions (1-byte alignment)
// =============================================================================
#pragma pack(push, 1)

// --- 0x01: Robot Telemetry Payload (MCU -> PC) ---
struct RobotTelemetryPayload {
  // Bitfield 0: Buttons [bit0: PWR_BTN, bit1: START_BTN, bit2: STOP_BTN, bit3:
  // MANU_SW]
  uint8_t btn_status;
  // Bitfield 1: PC Live States [bit0: NAV_PC_LIVE, bit1: HUMANOID_PC_LIVE,
  // bit2: AI_PC_LIVE]
  uint8_t pc_live_status;
  // Bitfield 2: Safety & Charging [bit0: SAFETY_EMS (1=EMG active), bit1:
  // MANU_CHG_DOCK, bit2: IS_CHARGING_ACTIVE]
  uint8_t safety_status;
  // Bitfield 3: Outputs [bit0: PWR_HOLD, bit1: VOLT24V_1, bit2: VOLT24V_2,
  // bit3: CHG_ON_ENABLE,
  //                     bit4: LAMP_RED, bit5: LAMP_GRN, bit6: LAMP_YEL, bit7:
  //                     BUZZER]
  uint8_t output_status;
  // Analog Charge Current ADC reading (Amperes / raw scaled float)
  float chg_current_adc;
  // Current active LED mode code
  uint8_t led_current_mode;
  // Timestamp / Uptime in seconds (MCU millis() / 1000)
  uint32_t uptime_sec;
};

// --- 0x02: BMS Telemetry Payload (MCU -> PC) ---
struct BmsTelemetryPayload {
  float voltage;    // Battery Voltage (V)
  float current;    // Battery Current (A)
  float soc;        // State of Charge (%)
  float soh;        // State of Health (%)
  float temp;       // Battery Temperature (deg C)
  uint8_t is_valid; // 1 if BMS packet received & valid, 0 otherwise
};

// --- 0x03: System Info Payload (MCU -> PC) ---
struct SystemInfoPayload {
  char sw_ver[24]; // Firmware Version String
  char hw_ver[24]; // Hardware Version String
};

// --- 0x04: ACK / NACK Payload (MCU -> PC) ---
struct AckNackPayload {
  uint8_t target_msg_id;
  uint8_t status_code;
};

// --- 0x10: Power Control Command (PC -> MCU) ---
struct CmdPowerCtrlPayload {
  // Target: 1=Humanoid PC, 2=AI PC, 3=24V Converter 1, 4=24V Converter 2,
  // 5=Main Nav PC
  uint8_t target_device;
  // Action: 0=OFF, 1=ON, 2=PULSE (e.g. 500ms power switch pulse)
  uint8_t action;
};

// --- 0x11: LED Control Command (PC -> MCU) ---
struct CmdLedCtrlPayload {
  uint8_t mode;       // Mode enum (LED_CTRL)
  uint8_t r;          // Red (0~255)
  uint8_t g;          // Green (0~255)
  uint8_t b;          // Blue (0~255)
  uint8_t brightness; // Master brightness (0~255)
};

// --- 0x12: Signal & Alarm Control Command (PC -> MCU) ---
struct CmdSignalCtrlPayload {
  uint8_t lamp_red;  // 0=OFF, 1=ON, 0xFF=NO_CHANGE
  uint8_t lamp_grn;  // 0=OFF, 1=ON, 0xFF=NO_CHANGE
  uint8_t lamp_yel;  // 0=OFF, 1=ON, 0xFF=NO_CHANGE
  uint8_t buzzer;    // 0=OFF, 1=ON, 0xFF=NO_CHANGE
  uint8_t tm_remote; // 0=OFF, 1=ON, 0xFF=NO_CHANGE
};

// --- 0x13: BMS & Charging Command (PC -> MCU) ---
struct CmdBmsCtrlPayload {
  uint8_t bms_reset_pulse; // 1 to trigger BMS reset pulse sequence
  uint8_t chg_enable;      // 0=Disable auto charging, 1=Enable auto charging
};

#pragma pack(pop)

// =============================================================================
// Helper Functions & Parser Class
// =============================================================================

// Calculate 16-bit CRC (CCITT polynomial: 0x1021, init: 0xFFFF)
uint16_t calcCRC16(const uint8_t *data, uint16_t length);

typedef void (*PacketHandlerCallback)(uint8_t msg_id, const uint8_t *payload,
                                      uint8_t length);

class PacketParser {
public:
  enum ParserState {
    STATE_HEADER_1 = 0,
    STATE_HEADER_2,
    STATE_MSG_ID,
    STATE_LENGTH,
    STATE_PAYLOAD,
    STATE_CRC_L,
    STATE_CRC_H
  };

  PacketParser();
  void setCallback(PacketHandlerCallback callback);
  void parseByte(uint8_t byte);

private:
  ParserState _state;
  uint8_t _msg_id;
  uint8_t _length;
  uint8_t _payload_index;
  uint8_t _payload[PROTOCOL_MAX_PAYLOAD_SIZE];
  uint16_t _received_crc;
  PacketHandlerCallback _callback;
};

// Packet Transmission Helper
void sendPacket(HardwareSerial &serial, uint8_t msg_id, const void *payload,
                uint8_t length);

#endif // PROTOCOL_H_
