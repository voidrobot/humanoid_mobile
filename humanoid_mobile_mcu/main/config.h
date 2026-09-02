#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>

// =============================================================================
// Software & Hardware Version
// =============================================================================
#define SW_VER "MOBILE_HUMANOID_MCU_v1.0"
#define HW_VER "Mobile_Humanoid_IO_v1.0"

// =============================================================================
// Serial Communication Baud Rates
// =============================================================================
// PC Serial: Binary protocol interface on UART0 (Serial)
#define PC_SERIAL_BAUD_RATE 57600
// BMS Serial: Battery Management System interface on UART3 (Serial3)
#define BMS_SERIAL_BAUD_RATE 19200

// =============================================================================
// GPIO Pin Mapping (Mobile Humanoid IO List ver 1.0)
// =============================================================================

// --- CON1: Battery (BMS) & Manual Charging ---
#define PIN_BMS_SW 43      // PL6: BMS Power Switch Relay Output (A-contact)
#define PIN_MANU_CHG_DET 3 // PE5: Manual Charge Dock State Input (NPN Input)

// --- CON2: LED Board (Bottom/Side SW2814 24V Strip) ---
#define PIN_LED_INDI 11   // PB5: 5V PWM for Bottom/Side LED Strip
#define PIN_LED_STATUS 10 // PB4: 5V PWM for Reserved/Top LED Strip

// --- CON3: Switch #1 & Power Control ---
#define PIN_PWR_BTN_IN 6    // PH3: Main Power Button Input (NPN Input)
#define PIN_PWR_BTN_LAMP 47 // PL2: Main Power Button Lamp Output (NPN Output)
#define PIN_MANU_SW_IN 7    // PH4: Manual Mode Check Switch Input (NPN Input)
#define PIN_SW1_OUT2 46     // PL3: Switch #1 OUT #2 Output (NPN Output)

// --- CON4: Switch #2 & Buttons / Buzzer ---
#define PIN_START_BTN_IN 8    // PH5: Start Button Input (NPN Input)
#define PIN_START_BTN_LAMP 45 // PL4: Start Button Lamp Output (NPN Output)
#define PIN_STOP_BTN_IN 9     // PH6: Stop Button Input (NPN Input)
#define PIN_STOP_BTN_LAMP 44  // PL5: Stop Button Lamp Output (NPN Output)
#define PIN_BUZZER 69         // PK7: Buzzer Output (PNP Relay Output)

// --- CON5: Main Navigation PC (NUC) Power ---
#define PIN_PC_PWR_SW                                                          \
  31 // PC6: Main PC Power On/Off Pulse Output (Relay A-contact)
#define PIN_PC_PWR_LIVE                                                        \
  42 // PL7: Main PC Boot State Input (12V/3.3V Optocoupler)

// --- CON7: Driver & Remote Control ---
#define PIN_TM_RMT 30 // PC7: TM Remote ON/OFF Output (Relay A-contact)

// --- CON9: Multi-PC Power & Status (Spare I/O & Power) ---
#define PIN_HUMANOID_PC_LIVE                                                   \
  22                      // PA0: Humanoid Control PC State Input (NPN Input)
#define PIN_AI_PC_LIVE 23 // PA1: AI Humanoid PC State Input (NPN Input)
#define PIN_SPARE_IN_3 24 // PA2: Spare NPN Input 3
#define PIN_SPARE_IN_4 25 // PA3: Spare NPN Input 4
#define PIN_HUMANOID_PC_ON                                                     \
  26                    // PA4: Humanoid Control PC ON Pulse Output (NPN Output)
#define PIN_AI_PC_ON 27 // PA5: AI Humanoid PC ON Pulse Output (NPN Output)
#define PIN_SPARE_OUT_3 28 // PA6: Spare NPN Output 3
#define PIN_SPARE_OUT_4 29 // PA7: Spare NPN Output 4

// --- CON10: External Signal Tower Lamps ---
#define PIN_LAMP_RED 66 // PK4: Signal Tower RED Lamp Output (PNP Output)
#define PIN_LAMP_GRN 67 // PK5: Signal Tower GREEN Lamp Output (PNP Output)
#define PIN_LAMP_YEL 68 // PK6: Signal Tower YELLOW Lamp Output (PNP Output)

// --- CON11: Safety PLC Interface ---
#define PIN_SAFETY_EMS                                                         \
  13 // PB7: Safety PLC Emergency Signal Input (Safety PNP -> MCU NPN)

// --- CON15: Power Board Interface ---
#define PIN_CHG_DET_ADC A1   // PF1: Charge Current ADC Input
#define PIN_CHG_ON_ENABLE A0 // PF0: Auto Charge Power Enable Output
#define PIN_PWR_HOLD A2      // PF2: Power Hold Output
#define PIN_VOLT24V_1 A9     // PK1 (D63): 24V Converter #1 Remote Output
#define PIN_VOLT24V_2 A10    // PK2 (D64): 24V Converter #2 Remote Output

// --- Onboard Status ---
#define PIN_LIVE_LED 39 // Arduino Heartbeat Status LED

// =============================================================================
// LED Configuration & Modes
// =============================================================================
#define LED_INDI_NUM 128  // Number of pixels on bottom/side LED strip
#define LED_STATUS_NUM 60 // Number of pixels on reserved top LED
#define LED_BRIGHT_DEFAULT 200
#define LED_BRIGHT_FADE 255

enum LED_CTRL {
  LED_OFF = 0,
  RED_WIPE,
  GREEN_WIPE,
  WHITE_WIPE,
  BLUE_WIPE,
  FULL_WHITE, // 5
  FULL_RED,
  FULL_GREEN,
  FULL_BLUE,
  FULL_ORANGE,
  FULL_YELLOW, // 10
  FULL_INDIGO,
  ORANGE_WIPE,
  YELLOW_WIPE,
  INDIGO_WIPE,
  FADE_WHITE, // 15
  FADE_RED,
  FADE_BLUE,
  FADE_GREEN,
  FADE_YELLOW,
  FADE_INDIGO,             // 20
  SWING_RED,               // 21
  SWING_BLUE,              // 22
  SWING_GREEN,             // 23
  SWING_WHITE,             // 24
  BLINK_RED,               // 25
  BLINK_GREEN,             // 26
  BLINK_BLUE,              // 27
  BLINK_WHITE,             // 28
  BLINK_YELLOW,            // 29
  BLINK_ORANGE,            // 30
  FADE_ORANGE,             // 31
  BLINK_INDIGO,            // 32
  INDI_RIGHT_BLINK_RED,    // 33
  INDI_RIGHT_BLINK_GREEN,  // 34
  INDI_RIGHT_BLINK_BLUE,   // 35
  INDI_RIGHT_BLINK_WHITE,  // 36
  INDI_RIGHT_BLINK_YELLOW, // 37
  INDI_RIGHT_BLINK_ORANGE, // 38
  INDI_LEFT_BLINK_RED,     // 39
  INDI_LEFT_BLINK_GREEN,   // 40
  INDI_LEFT_BLINK_BLUE,    // 41
  INDI_LEFT_BLINK_WHITE,   // 42
  INDI_LEFT_BLINK_YELLOW,  // 43
  INDI_LEFT_BLINK_ORANGE,  // 44
  INDI_BLINK_RED,          // 45
  INDI_BLINK_GREEN,        // 46
  INDI_BLINK_BLUE,         // 47
  INDI_BLINK_WHITE,        // 48
  INDI_BLINK_YELLOW,       // 49
  INDI_BLINK_ORANGE,       // 50
  INDI_FULL_RED,           // 51
  INDI_FULL_GREEN,
  INDI_FULL_BLUE,
  INDI_FULL_WHITE,
  INDI_FULL_YELLOW, // 55
  INDI_FULL_ORANGE,
  BLINK_CHG_PWR_OFF = 100,
  BLINK_NOR_PWR_OFF = 101
};

#define LED_DEFAULT_MODE BLINK_YELLOW
#define LED_INDI_DEFAULT_MODE INDI_BLINK_ORANGE

// =============================================================================
// Update Intervals & Timing Constants (Milliseconds)
// =============================================================================
#define INTERVAL_TELEMETRY_MS 50   // 20Hz Robot Telemetry transmission
#define INTERVAL_BMS_MS 1000       // 1Hz BMS query & telemetry
#define INTERVAL_LED_UPDATE_MS 50  // 20Hz LED animation frame rate
#define INTERVAL_BUTTON_POLL_MS 20 // 50Hz Button poll & debounce
#define INTERVAL_HEARTBEAT_MS 1000 // 1Hz Live LED toggle
#define INTERVAL_ADC_POLL_MS 100   // 10Hz Charge ADC sampling

#define POWER_BTN_PRESS_SHUTDOWN_MS                                            \
  2000 // 2.0s press -> Trigger normal PC shutdown
#define POWER_BTN_PRESS_FORCE_MS                                               \
  7000                           // 7.0s press -> Trigger forced hardware off
#define PC_PULSE_DURATION_MS 500 // 500ms power switch pulse duration

#endif // CONFIG_H_
