# Mobile Humanoid MCU 펌웨어 및 전용 시리얼 프로토콜 개발 계획서 (개정본 v2)

## 1. 개요 (Overview)
본 프로젝트는 **모바일 휴머노이드 하드웨어 I/O 명세서([`Mobile Humanoid_IO_List_ver1.0.xlsx`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/raw_data/mobile_mcu/Mobile%20Humanoid_IO_List_ver1.0.xlsx))**를 기반으로 기존 머신텐딩용 아두이노 펌웨어([`mcu/mt_cnc.ino`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/mt_cnc.ino))를 전면 리팩토링하여 [`mcu/main.ino`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/main.ino) 및 관련 모듈로 새롭게 구성하고, ATmega2560의 8KB SRAM 한계를 극복하기 위해 `rosserial`을 제거하고 **고신뢰성 경량 바이너리 시리얼 프로토콜**을 구현합니다.

---

## 2. 사용자 피드백 반영 사항 (User Feedback Incorporated)

> [!IMPORTANT]
> 1. **소스 파일 저장 경로 및 아두이노 빌드 환경 준수**:
>    - 모든 MCU 소스 파일은 **`mcu/`** 디렉터리 내에 평탄(Flat)하게 배치됩니다.
>    - **아두이노 IDE / Arduino CLI 표준 빌드 규칙**을 완벽히 준수하여, 아두이노 IDE에서 `mcu/main.ino`를 열었을 때 같은 폴더의 `.h` 및 `.cpp` 탭들이 자동으로 인식되고 원클릭으로 컴파일 및 업로드가 가능하도록 설계합니다.
>    - 외부 라이브러리 의존성은 아두이노 표준 라이브러리인 **`Adafruit_NeoPixel`** 하나만 남기고 나머지는 순수 표준 C/C++로 구현합니다.
> 2. **메인 파일명**: `mcu/main.ino`
> 3. **통신 속도(Baud Rate)**: 기본 **57,600 bps** 유지 및 `config.h`에서 매크로로 간편 변경 지원.

---

## 3. 세부 설계 사양 (Detailed Specifications)

### 3.1 바이너리 패킷 프로토콜 명세

#### 프레임 포맷 (Frame Format)
```text
┌───────────┬───────────┬────────┬─────────────┬─────────────────┬───────────────┬───────────────┐
│ HEADER 1  │ HEADER 2  │ MSG_ID │ LENGTH (N)  │  PAYLOAD (N B)  │  CRC16 (Low)  │ CRC16 (High)  │
│   0xAA    │   0x55    │ (1 B)  │    (1 B)    │    (0 ~ 64 B)   │     (1 B)     │     (1 B)     │
└───────────┴───────────┴────────┴─────────────┴─────────────────┴───────────────┴───────────────┘
```
- **Header**: `0xAA`, `0x55` (2바이트 고정 매직 넘버)
- **Msg ID**: 메시지 식별자 (1바이트)
- **Length**: Payload의 바이트 수 (1바이트, 최대 64B)
- **Payload**: 구조체 데이터 (Little-Endian)
- **CRC16**: CCITT-16 (Poly: 0x1021, Init: 0xFFFF)로 Header부터 Payload까지의 무결성 검증

#### 메시지 ID 및 데이터 구조 정의

| Msg ID | 이름 | 방향 | 주기 / 조건 | 주요 페이로드 내용 |
| :---: | :--- | :---: | :---: | :--- |
| **`0x01`** | `MSG_ROBOT_TELEMETRY` | MCU $\rightarrow$ PC | 주기 20Hz (50ms) | - 버튼 상태 비트필드 (Power, Start, Stop, Manual)<br/>- 3대 PC 부팅 상태 (Nav PC, Humanoid PC, AI PC)<br/>- Safety PLC EMS 수신 상태, 충전 감지 상태<br/>- 충전 전류 ADC (Float 4B)<br/>- 출력 릴레이/램프 현재 상태 비트필드 |
| **`0x02`** | `MSG_BMS_TELEMETRY` | MCU $\rightarrow$ PC | 주기 1Hz (1000ms) | - 전압 (Float 4B), 전류 (Float 4B)<br/>- SoC (Float 4B), SoH (Float 4B), Temp (Float 4B) |
| **`0x03`** | `MSG_SYSTEM_INFO` | MCU $\rightarrow$ PC | 부팅 시 / 요청 시 | - 펌웨어 버전 (예: `"MOBILE_HUMANOID_MCU_v1.0"`) |
| **`0x04`** | `MSG_ACK_NACK` | MCU $\rightarrow$ PC | 커맨드 수신 즉시 | - 수신 Msg ID (1B), 결과 코드 (0: OK, 1: CRC_ERR, 2: INVALID_CMD) |
| **`0x10`** | `MSG_CMD_POWER_CTRL` | PC $\rightarrow$ MCU | 이벤트 | - 제어 대상 (1: Humanoid PC, 2: AI PC, 3: 24V Conv, 4: Main PC 셧다운)<br/>- 액션 (0: OFF, 1: ON, 2: PULSE) |
| **`0x11`** | `MSG_CMD_LED_CTRL` | PC $\rightarrow$ MCU | 이벤트 | - 모드 코드 (0~55), 컬러 (R, G, B), 밝기 (0~255) |
| **`0x12`** | `MSG_CMD_SIGNAL_CTRL` | PC $\rightarrow$ MCU | 이벤트 | - 시그널 타워 (Red/Green/Yellow 비트마스크)<br/>- 부저 ON/OFF, TM Remote ON/OFF |
| **`0x13`** | `MSG_CMD_BMS_CTRL` | PC $\rightarrow$ MCU | 이벤트 | - BMS 리셋 펄스 트리거, 자동 충전 인에이블 ON/OFF |

---

### 3.2 하드웨어 핀 맵 매핑 ([`Mobile Humanoid_IO_List_ver1.0.xlsx`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/raw_data/mobile_mcu/Mobile%20Humanoid_IO_List_ver1.0.xlsx) 기준)

```cpp
// ==================== Switch #1 & Power (CON3 / CON5) ====================
#define PIN_PWR_BTN_IN        6   // PH3: Main Power Button Input
#define PIN_PWR_BTN_LAMP      47  // PL2: Main Power Button Lamp
#define PIN_MANU_SW_IN        7   // PH4: Manual Mode Check Input (신규)
#define PIN_SW1_OUT2          46  // PL3: Switch#1 OUT#2 Spare
#define PIN_PC_PWR_SW         31  // PC6: Main PC Power On/Off Pulse Output
#define PIN_PC_PWR_LIVE       42  // PL7: Main PC Boot Live Input
#define PIN_PWR_HOLD          A2  // PF2: Power Hold Output

// ==================== Switch #2 & Alarms (CON4 / CON10) ==================
#define PIN_START_BTN_IN      8   // PH5: Start Button Input (신규)
#define PIN_START_BTN_LAMP    45  // PL4: Start Button Lamp Output (신규)
#define PIN_STOP_BTN_IN       9   // PH6: Stop Button Input
#define PIN_STOP_BTN_LAMP     44  // PL5: Stop Button Lamp Output
#define PIN_BUZZER            69  // PK7: Buzzer Output
#define PIN_LAMP_RED          66  // PK4: Signal Tower RED (신규)
#define PIN_LAMP_GRN          67  // PK5: Signal Tower GREEN (신규)
#define PIN_LAMP_YEL          68  // PK6: Signal Tower YELLOW (신규)

// ==================== Multi-PC & Remote (CON9 / CON7) ====================
#define PIN_HUMANOID_PC_LIVE  22  // PA0: Humanoid Control PC Live Input (신규)
#define PIN_AI_PC_LIVE        23  // PA1: AI Humanoid PC Live Input (신규)
#define PIN_HUMANOID_PC_ON    26  // PA4: Humanoid Control PC ON Pulse Output (신규)
#define PIN_AI_PC_ON          27  // PA5: AI Humanoid PC ON Pulse Output (신규)
#define PIN_TM_RMT            30  // PC7: TM Remote ON/OFF Output (신규)

// ==================== Safety PLC & Battery/Power (CON11/CON1/CON15) =====
#define PIN_SAFETY_EMS        13  // PB7: Safety PLC Emergency Signal Input
#define PIN_BMS_SW            43  // PL6: BMS Power Switch Relay Output
#define PIN_MANU_CHG_DET      3   // PE5: Manual Charge Dock Detect Input
#define PIN_CHG_DET_ADC       A1  // PF1: Charge Current ADC Input
#define PIN_CHG_ON_ENABLE     A0  // PF0: Charge Enable Output
#define PIN_VOLT24V_1         A9  // PK1 (D63): 24V Converter #1 Remote
#define PIN_VOLT24V_2         A10 // PK2 (D64): 24V Converter #2 Remote
#define PIN_LIVE_LED          39  // MCU Heartbeat LED

// ==================== LED Strip (CON2) ==================================
#define PIN_LED_INDI          11  // PB5: SW2814 LED Strip PWM (Bottom/Side)
#define PIN_LED_STATUS        10  // PB4: SW2814 LED Strip PWM (Reserved)
```

---

## 4. 제안된 변경 사항 (Proposed Changes)

모든 아두이노 소스 코드는 **`mcu/`** 디렉터리 내에 위치하며, 아두이노 빌드 시스템 표준을 따릅니다.

### [Component] MCU Firmware (`mcu/`)

#### [NEW] [mcu/config.h](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/config.h)
- 핀 매핑 정의, `PC_SERIAL_BAUD_RATE (57600)`, `BMS_SERIAL_BAUD_RATE (19200)`, 타이머 및 버퍼 크기 상수 정의.

#### [NEW] [mcu/protocol.h](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/protocol.h) / [mcu/protocol.cpp](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/protocol.cpp)
- 바이너리 패킷 프레임 정의, 메시지 구조체(Telemetry, BMS, Command 등), CRC-CCITT 계산 함수, 링버퍼 기반 FSM 패킷 파서 구현.

#### [NEW] [mcu/bms.h](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/bms.h) / [mcu/bms.cpp](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/bms.cpp)
- `Serial3` 기반 BMS 전용 통신 모듈. 주기적 폴링 및 응답 패킷 디코딩, 리셋 시퀀스 담당.

#### [NEW] [mcu/led_ctrl.h](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/led_ctrl.h) / [mcu/led_ctrl.cpp](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/led_ctrl.cpp)
- `Adafruit_NeoPixel` 기반 비동기 LED 애니메이션 엔진 (Blink, Wipe, Fade, Battery gauge, Directional Indicator).

#### [NEW] [mcu/io_manager.h](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/io_manager.h) / [mcu/io_manager.cpp](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/io_manager.cpp)
- 디지털 입출력 디바운스, 전원 버튼 및 3개 PC On/Off 시퀀스 제어, Safety PLC 비상정지 모니터링, 충전 ADC 필터링, 타워 램프/부저/TM Remote 제어.

#### [NEW] [mcu/main.ino](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/main.ino)
- `ros.h` 완전 제거 및 모듈 통합 메인 스케치. 비차단(Non-blocking) 타이머 스케줄러 기반 메인 루프 구현.

#### [DELETE] [mcu/mt_cnc.ino](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/mt_cnc.ino)
- 레거시 단일 파일 제거 (신규 모듈화 코드로 대체).

---

### [Component] Documentation & Host Tools

#### [NEW] [docs/serial_protocol_spec.md](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/docs/serial_protocol_spec.md)
- ROS 2 브릿지 노드 개발자가 바로 참고할 수 있는 상세 바이너리 프로토콜 명세서 (패킷 구조, 바이트 오더, 비트맵 필드 상세 설명).

#### [NEW] [tools/mcu_serial_tester.py](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/tools/mcu_serial_tester.py)
- PC에서 MCU와 바이너리 패킷을 송수신하며 텔레메트리 파싱 및 제어 명령(LED, PC 전원, 타워 램프, 부저 등)을 테스트할 수 있는 Python CLI GUI 도구.

---

## 5. 검증 계획 (Verification Plan)

### 5.1 문법 검증 및 빌드 확인
- Arduino CLI 및 컴파일러를 통해 문법 오류, 메모리 크기(Flash/SRAM 사용량) 정적 분석.
- 8KB SRAM 기준 잔여 메모리가 6KB 이상 확보되는지 확인.

### 5.2 프로토콜 및 패킷 무결성 검증
- [`tools/mcu_serial_tester.py`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/tools/mcu_serial_tester.py)를 사용하여:
  1. 텔레메트리 수신 주기(50ms) 및 CRC 검증 확인
  2. 제어 명령 송신 시 MCU의 `ACK` 응답 및 실제 GPIO 출력 변경 확인
  3. 비정상 패킷 / 노이즈 주입 시 FSM 파서의 동기화 복구 테스트
