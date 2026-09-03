# 모바일 휴머노이드 MCU 및 ROS 2 브릿지 노드 통합 테스트 매뉴얼

본 문서는 **모바일 휴머노이드 로봇**의 전원, 안전 및 하드웨어 I/O를 제어하는 아두이노 Mega 2560 기반 펌웨어(**`humanoid_mobile_mcu`**)와 ROS 2 상위 제어 노드(**`humanoid_mobile_mcu_ros`**) 간의 연동 상태 및 정상 동작 여부를 검증하기 위한 실무용 통합 테스트 매뉴얼입니다.

---

## 1. 개요 및 사전 준비

### 1.1 하드웨어 및 통신 명세
* **메인 MCU**: ATmega2560 (Arduino Mega 2560)
* **PC 시리얼 통신 포트**: `/dev/ttyUSB0` (상황에 따라 다를 수 있음)
* **보드레이트**: `57,600 bps` (데이터 8비트, 패리티 없음, 정지 비트 1개)
* **프로토콜**: 고신뢰성 경량 바이너리 패킷 프로토콜 (0xAA 0x55 매직 넘버 및 CRC-16-CCITT 무결성 검증)

### 1.2 ROS 2 환경 준비
테스트 진행 전, ROS 2 작업 공간에서 패키지를 빌드하고 런치 파일을 구동합니다.

```bash
# 1. ROS 2 작업 공간으로 이동 및 빌드
cd /home/void/workspace/humanoid_tmp_ws
colcon build --packages-select humanoid_mobile_mcu_ros
source install/setup.bash

# 2. MCU 시리얼 브릿지 노드 구동
ros2 launch humanoid_mobile_mcu_ros mcu_bridge.launch.py port:=/dev/ttyUSB0
```

---

## 2. PC ➔ MCU 전송 (입력 토픽 제어 테스트)

상위 PC의 터미널에서 제어 명령 토픽을 발행하여 MCU가 이를 수신하고, 실제 릴레이, 핀 제어 및 하드웨어 물리 상태를 올바르게 출력하는지 검증합니다.

### 2.1 전원 제어 테스트 (`/mcu/cmd_power`)
이 토픽은 3대의 온보드 PC 및 24V 전원 컨버터의 전력을 제어합니다. 명령이 성공적으로 처리되면 MCU는 수신 확인용 ACK 패킷을 보냅니다.

```bash
# 토픽 구조: humanoid_mobile_mcu_ros/msg/PowerCmd
# 필드:
#   uint8 target_device (1: Humanoid PC, 2: AI PC, 3: 24V Conv 1, 4: 24V Conv 2, 5: Main Nav PC)
#   uint8 action (0: OFF, 1: ON, 2: PULSE)
```

| 테스트 케이스 ID | 테스트 대상 디바이스 | ROS 2 토픽 발행 명령어 | 물리적 확인 방법 및 기대 동작 (MCU 출력) |
| :--- | :--- | :--- | :--- |
| **TC-PWR-01** | **휴머노이드 제어 PC<br>(Humanoid PC)** | `ros2 topic pub --once /mcu/cmd_power humanoid_mobile_mcu_ros/msg/PowerCmd "{target_device: 1, action: 2}"` | - **PIN_HUMANOID_PC_ON (D26, NPN)** 포트에서 **500ms 동안 HIGH(5V)** 펄스가 인가된 후 다시 LOW로 내려가는 것을 오실로스코프 또는 멀티미터로 확인.<br>- 실제 휴머노이드 PC가 켜지고 부팅이 완료되면 **PIN_HUMANOID_PC_LIVE (D22, NPN Input)**의 전압 상태가 바뀌어 텔레메트리(`humanoid_pc_live`)가 `True`로 전환되는지 확인. |
| **TC-PWR-02** | **AI 휴머노이드 PC<br>(AI PC)** | `ros2 topic pub --once /mcu/cmd_power humanoid_mobile_mcu_ros/msg/PowerCmd "{target_device: 2, action: 2}"` | - **PIN_AI_PC_ON (D27, NPN)** 포트에서 **500ms 동안 HIGH(5V)** 펄스가 인가됨.<br>- 실제 AI PC 전원 버튼이 릴레이 접점에 의해 눌린 효과가 발생하여 PC가 켜짐.<br>- 부팅 완료 시 **PIN_AI_PC_LIVE (D23)** 입력에 따라 텔레메트리(`ai_pc_live`)가 `True`로 리포트되는지 감시. |
| **TC-PWR-03** | **24V 컨버터 #1** | `ros2 topic pub --once /mcu/cmd_power humanoid_mobile_mcu_ros/msg/PowerCmd "{target_device: 3, action: 0}"` | - **PIN_VOLT24V_1 (A9/D63, PNP Output)** 포트가 **LOW(0V)**가 됨.<br>- 전원 분배 보드의 24V 컨버터 #1 라인이 차단되며 관련 장비 전원이 즉시 꺼지는 것을 확인. |
| **TC-PWR-04** | **24V 컨버터 #1** | `ros2 topic pub --once /mcu/cmd_power humanoid_mobile_mcu_ros/msg/PowerCmd "{target_device: 3, action: 1}"` | - **PIN_VOLT24V_1 (A9/D63)** 포트가 다시 **HIGH(5V)**가 되어 컨버터 #1 출력이 활성화되는지 확인. |
| **TC-PWR-05** | **24V 컨버터 #2** | `ros2 topic pub --once /mcu/cmd_power humanoid_mobile_mcu_ros/msg/PowerCmd "{target_device: 4, action: 0}"` | - **PIN_VOLT24V_2 (A10/D64, PNP Output)** 포트가 **LOW(0V)**가 되며 24V 컨버터 #2 전원이 꺼짐을 확인. |
| **TC-PWR-06** | **24V 컨버터 #2** | `ros2 topic pub --once /mcu/cmd_power humanoid_mobile_mcu_ros/msg/PowerCmd "{target_device: 4, action: 1}"` | - **PIN_VOLT24V_2 (A10/D64)** 포트가 **HIGH(5V)**가 되며 컨버터 #2 출력이 재개됨을 확인. |
| **TC-PWR-07** | **메인 내비게이션 PC<br>(Nav PC)** | `ros2 topic pub --once /mcu/cmd_power humanoid_mobile_mcu_ros/msg/PowerCmd "{target_device: 5, action: 2}"` | - **PIN_PC_PWR_SW (D31, A-접점 Relay)** 제어 핀이 **500ms 동안 HIGH**가 되어 릴레이 자화음(딸깍 소리)이 발생하는지 청각적으로 확인.<br>- 실제 메인 PC의 전원 전압 감지 입력인 **PIN_PC_PWR_LIVE (D42)**의 입력에 의해 전원 보류 신호인 **PIN_PWR_HOLD (A2)** 핀이 자동으로 HIGH 래치되는지 멀티미터로 체크. |

---

### 2.2 LED 스트립 제어 테스트 (`/mcu/cmd_led`)
로봇 하부 및 측면의 SW2814 24V LED 스트립(128개 LED) 동작 상태와 전역 밝기를 변경하는 테스트입니다.

```bash
# 토픽 구조: humanoid_mobile_mcu_ros/msg/LedCmd
# 필드:
#   uint8 mode (0: OFF, 5: FULL_WHITE, 6: FULL_RED, 7: FULL_GREEN, 8: FULL_BLUE, 25: BLINK_RED, 34: INDI_RIGHT_BLINK_GREEN, 40: INDI_LEFT_BLINK_GREEN, 51: INDI_FULL_RED 등)
#   uint8 r (0~255), uint8 g (0~255), uint8 b (0~255) -> 사용자 커스텀 컬러 지정용
#   uint8 brightness (0~255, 스트립 마스터 밝기, 기본 200)
```

| 테스트 케이스 ID | 제어 모드 및 색상 | ROS 2 토픽 발행 명령어 | 물리적 확인 방법 및 기대 동작 (LED 스트립) |
| :--- | :--- | :--- | :--- |
| **TC-LED-01** | **전체 끄기** | `ros2 topic pub --once /mcu/cmd_led humanoid_mobile_mcu_ros/msg/LedCmd "{mode: 0, r: 0, g: 0, b: 0, brightness: 0}"` | - 하부 및 측면 LED 스트립(**PIN_LED_INDI, D11**) 전체가 소등되는지 확인. |
| **TC-LED-02** | **단색 녹색 점등 (Full Green)** | `ros2 topic pub --once /mcu/cmd_led humanoid_mobile_mcu_ros/msg/LedCmd "{mode: 7, r: 0, g: 255, b: 0, brightness: 200}"` | - LED 스트립 128개 전역이 밝기 200 수준의 선명한 **단색 녹색**으로 일정하게 켜지는지 육안 확인. |
| **TC-LED-03** | **경보 적색 점멸 (Blink Red)** | `ros2 topic pub --once /mcu/cmd_led humanoid_mobile_mcu_ros/msg/LedCmd "{mode: 25, r: 255, g: 0, b: 0, brightness: 255}"` | - 전체 LED 스트립이 최대 밝기(255) 수준에서 일정 주기(20Hz 비차단 프레임 업데이트 주기 기반)로 **적색 점멸(Blinking)**을 반복하는지 확인. |
| **TC-LED-04** | **오른쪽 방향지시등 녹색 깜빡이<br>(Indicator Right Blink)** | `ros2 topic pub --once /mcu/cmd_led humanoid_mobile_mcu_ros/msg/LedCmd "{mode: 34, r: 0, g: 255, b: 0, brightness: 200}"` | - 전체 스트립 중 우측 영역인 **LED 24번부터 71번 픽셀**만 녹색 깜빡이로 동작하고, 나머지 픽셀은 꺼져 있는지 정밀 확인. |
| **TC-LED-05** | **왼쪽 방향지시등 녹색 깜빡이<br>(Indicator Left Blink)** | `ros2 topic pub --once /mcu/cmd_led humanoid_mobile_mcu_ros/msg/LedCmd "{mode: 40, r: 0, g: 255, b: 0, brightness: 200}"` | - 전체 스트립 중 좌측 영역인 **LED 0~23번 및 72~95번 픽셀**만 녹색 깜빡이로 동작하고, 우측 영역(24~71번)은 소등 상태를 유지하는지 확인. |

---

### 2.3 시그널 타워 / 부저 / TM Remote 제어 테스트 (`/mcu/cmd_signal`)
로봇 외부 시그널 타워 램프 3색, 멜로디/경보 부저, 그리고 협동로봇용 TM Remote 출력의 릴레이 상태를 원격 제어합니다.

```bash
# 토픽 구조: humanoid_mobile_mcu_ros/msg/SignalCmd
# 필드 및 값 범위: (0: OFF, 1: ON, 255: 현재 상태 유지)
#   uint8 lamp_red, uint8 lamp_grn, uint8 lamp_yel, uint8 buzzer, uint8 tm_remote
```

| 테스트 케이스 ID | 제어 대상 및 동작 | ROS 2 토픽 발행 명령어 | 물리적 확인 방법 및 기대 동작 (MCU 출력) |
| :--- | :--- | :--- | :--- |
| **TC-SIG-01** | **경보 부저 ON** | `ros2 topic pub --once /mcu/cmd_signal humanoid_mobile_mcu_ros/msg/SignalCmd "{lamp_red: 255, lamp_grn: 255, lamp_yel: 255, buzzer: 1, tm_remote: 255}"` | - **PIN_BUZZER (D69, PNP Relay Output)** 제어 핀이 즉시 **HIGH**가 되며, 물리 경보 부저에서 삐- 하는 경보음이 지속해서 발생하는지 청각 확인. |
| **TC-SIG-02** | **경보 부저 OFF** | `ros2 topic pub --once /mcu/cmd_signal humanoid_mobile_mcu_ros/msg/SignalCmd "{lamp_red: 255, lamp_grn: 255, lamp_yel: 255, buzzer: 0, tm_remote: 255}"` | - **PIN_BUZZER (D69)** 핀이 **LOW**가 되며 경보음이 즉시 멈추는지 확인. |
| **TC-SIG-03** | **시그널 타워 황색등 ON** | `ros2 topic pub --once /mcu/cmd_signal humanoid_mobile_mcu_ros/msg/SignalCmd "{lamp_red: 255, lamp_grn: 255, lamp_yel: 1, buzzer: 255, tm_remote: 255}"` | - **PIN_LAMP_YEL (D68, PNP Output)** 제어 핀이 **HIGH**로 올라가며, 시그널 타워의 **황색(Yellow) 램프**가 점등되는지 확인 (적색, 녹색 등은 이전 상태 유지). |
| **TC-SIG-04** | **시그널 타워 녹색등 ON & 황색등 OFF** | `ros2 topic pub --once /mcu/cmd_signal humanoid_mobile_mcu_ros/msg/SignalCmd "{lamp_red: 255, lamp_grn: 1, lamp_yel: 0, buzzer: 255, tm_remote: 255}"` | - **PIN_LAMP_GRN (D67, PNP)**가 **HIGH**가 되어 녹색등이 켜지는 동시에 **PIN_LAMP_YEL (D68)**은 **LOW**가 되어 황색등이 꺼지는지 복합 동작 검증. |
| **TC-SIG-05** | **협동로봇 TM Remote 릴레이 ON** | `ros2 topic pub --once /mcu/cmd_signal humanoid_mobile_mcu_ros/msg/SignalCmd "{lamp_red: 255, lamp_grn: 255, lamp_yel: 255, buzzer: 255, tm_remote: 1}"` | - **PIN_TM_RMT (D30, A-접점 Relay)** 제어 핀이 **HIGH**가 되어 내부 릴레이 접점이 붙는 소리가 나는지 청각 및 멀티미터 도통 테스트로 검증. |

---

### 2.4 BMS 및 자동 충전 제어 테스트 (`/mcu/cmd_bms`)
배터리 관리 시스템(BMS)의 하드웨어 전원을 리셋하거나, 자동 충전 시스템의 파워 접점 인에이블 신호를 On/Off 제어합니다.

```bash
# 토픽 구조: humanoid_mobile_mcu_ros/msg/BmsCmd
# 필드:
#   uint8 bms_reset_pulse (1: BMS 전원 리셋 펄스 실행, 0: 대기)
#   uint8 chg_enable (1: 자동 충전 단자 접점 연결, 0: 차단)
```

| 테스트 케이스 ID | 제어 동작 | ROS 2 토픽 발행 명령어 | 물리적 확인 방법 및 기대 동작 (MCU 출력) |
| :--- | :--- | :--- | :--- |
| **TC-BMSCMD-01** | **BMS 리셋 펄스 트리거** | `ros2 topic pub --once /mcu/cmd_bms humanoid_mobile_mcu_ros/msg/BmsCmd "{bms_reset_pulse: 1, chg_enable: 255}"` | - **PIN_BMS_SW (D43, PL6, A-접점 릴레이)** 핀에서 릴레이를 재시작하기 위한 하드웨어 리셋 펄스 파형이 인가되는지 감시.<br>- 시퀀스 동작: HIGH(100ms) ➔ LOW(500ms) ➔ HIGH 유지 상태로 전환되며 릴레이 딸깍임 발생. |
| **TC-BMSCMD-02** | **자동 충점 접점 인에이블 (CHG_ON)** | `ros2 topic pub --once /mcu/cmd_bms humanoid_mobile_mcu_ros/msg/BmsCmd "{bms_reset_pulse: 0, chg_enable: 1}"` | - **PIN_CHG_ON_ENABLE (A0/PF0, Digital Output)** 핀이 즉시 **HIGH(5V)**가 되는지 측정.<br>- 전원 보드의 자동 충전 접점 보호용 파워 릴레이가 활성화되어 자동 충전 준비 상태가 되는 것을 확인. |
| **TC-BMSCMD-03** | **자동 충전 접점 차단 (CHG_OFF)** | `ros2 topic pub --once /mcu/cmd_bms humanoid_mobile_mcu_ros/msg/BmsCmd "{bms_reset_pulse: 0, chg_enable: 0}"` | - **PIN_CHG_ON_ENABLE (A0/PF0)** 핀이 **LOW(0V)**로 차단되어 충전 라인이 안전하게 분리되는지 확인. |

---

## 3. MCU ➔ PC 수신 (출력 토픽 상태 모니터링 테스트)

로봇 외부에 장착된 물리 스위치나 센서 값을 직접 하드웨어 수준에서 조작하고, 상위 PC의 터미널 환경에서 ROS 2 토픽을 모니터링하여 필드 데이터가 정상적으로 갱신되는지 실시간 검증합니다.

```bash
# 모니터링 시작 명령어 (수신되는 텔레메트리 값을 지속해서 보여줍니다)
ros2 topic echo /mcu/robot_telemetry
```

### 3.1 로봇 통합 텔레메트리 감시 테스트 (`/mcu/robot_telemetry`)
아래 시나리오를 바탕으로 물리적인 외부 자극을 가한 뒤 터미널의 출력값이 아래 변화 상태와 일치하는지 모니터링합니다.

| 테스트 케이스 ID | 하드웨어 조작 가이드 (물리 입력) | 확인 대상 필드 명칭 | 정상 변경 전 상태 값 | 정상 변경 후 상태 값 (기대값) |
| :--- | :--- | :--- | :--- | :--- |
| **TC-TEL-01** | **로봇 메인 전원 스위치(Power Button)**<br>스위치 하우징 내부를 손가락으로 가볍게 누른 채 유지합니다. (콘포트 3의 D6 연결 장치) | `pwr_btn` | `False` | **`True`**<br>- 버튼에서 손을 떼면 즉시 다시 `False`로 내려갑니다. |
| **TC-TEL-02** | **미션 시작 버튼(Start Button)**<br>로봇 상부 우측의 Start 버튼(D8 연결)을 누릅니다. | `start_btn` | `False` | **`True`**<br>- 이와 동시에 **D45 (Start Button Lamp)** 출력이 켜지거나 점멸 제어가 연계 동작하는지 함께 감시합니다. |
| **TC-TEL-03** | **미션 정지 버튼(Stop Button)**<br>로봇 상부 좌측의 Stop 버튼(D9 연결)을 누릅니다. | `stop_btn` | `False` | **`True`** |
| **TC-TEL-04** | **수동 스위치(Manual Mode Switch)**<br>사용자 조작 패널의 Auto/Manual 선택 스위치(D7 연결)를 Manual(수동) 상태로 토글합니다. | `manu_sw` | `False` | **`True`** |
| **TC-TEL-05** | **Safety PLC 비상정지 스위치 트리거**<br>전/후방 비상정지 버섯머리 버튼(EMS)을 물리적으로 강하게 눌러 비상정지를 활성화합니다.<br>(Safety PLC DO08 ➔ MCU D13 수신 연결) | `safety_ems` | `False` | **`True`**<br>- **(추가 전역 연쇄 동작)** 비상정지 활성화 즉시 **`lamp_red`** 및 **`buzzer`** 필드가 강제로 `True`로 잠금 전환되고, 하부 LED 스트립이 **INDI_FULL_RED** 고정 빨간색으로 강제 발광하는지 삼중 확인합니다. |
| **TC-TEL-06** | **수동 충전 단자 감지 및 도킹**<br>로봇 충전 가이드 단자를 충전 패드 도크 접점에 손으로 가볍게 갖다 대거나 접지 감지 점퍼를 쇼트시킵니다. (PE5/D3 NPN 접점 입력) | `manu_chg_dock` | `False` | **`True`** |
| **TC-TEL-07** | **충전 전류 인가 (Analog Charge Sensor)**<br>실제 충전기에 충전 단자가 결합하여 충전 전압 및 전류가 0.5A 이상 통과되도록 부하를 줍니다. (PF1/A1 아날로그 전류 전압입력) | `is_charging` | `False` | **`True`**<br>- `chg_current_adc` 센싱 전압값이 흐르는 아날로그 전류 크기에 정비례하여 **0.5 (Amperes/ADC) 이상**의 정밀 실수값(Float)으로 차오르는지 확인.<br>- 이와 연계하여 LED 모드가 배터리 게이지 표현 모드로 변해 스트립 전면에 녹색 바가 충전량 비례로 차오르는지 확인. |
| **TC-TEL-08** | **출력 릴레이 및 자가 유지 감지**<br>내비게이션 PC 부팅 핀(`PC_PWR_LIVE`, D42)에 12V 혹은 HIGH를 인가하여 부팅 완료를 감지시킵니다. | `pwr_hold` | `False` | **`True`**<br>- 이와 동시에 **PIN_PWR_BTN_LAMP (D47)**가 HIGH(전원 램프 점등)가 되는지 함께 확인합니다. |

---

### 3.2 배터리 상세 데이터 모니터링 테스트 (`/mcu/bms_telemetry`)
리튬이온 배터리의 상세 BMS 관리 패킷이 시리얼(UART3, 19200bps)을 거쳐 상위 ROS 2 단으로 정확히 패킹되어 오르는지 감시합니다.

```bash
ros2 topic echo /mcu/bms_telemetry
```

| 검사 필드명 | 타입 | 기대값 범위 및 유효 여부 판별법 |
| :--- | :--- | :--- |
| `voltage` | `float32` | 배터리 사양에 따른 정상 전압 범위 (예: 42.0V ~ 54.6V)가 실시간 부동소수점으로 정확히 오는지 모니터링. |
| `current` | `float32` | - 충전 중일 때는 **양수(+)**의 전류값 출력.<br>- 주행 및 동작 등 소모 상황일 때는 **음수(-)**의 전류값 출력 여부 판별. |
| `soc` | `float32` | 현재 배터리 상태 잔량(State of Charge) `0.0% ~ 100.0%` 확인. |
| `soh` | `float32` | 배터리 건강 상태 수명(State of Health) `0.0% ~ 100.0%` 확인. |
| `temp` | `float32` | 배터리 내부 온도 모니터링 (실온 15.0°C ~ 40.0°C 정상 범위 내인지 확인). |
| `is_valid` | `bool` | - **`True`**: BMS 제어 보드와 MCU(UART3)가 정상적으로 초당 1회씩 통신을 성공하고 있음.<br>- **`False`**: 하드웨어 단선 또는 통신 오류 발생을 감지하는 안전 장치. |

---

### 3.3 표준 배터리 및 비상 정지 간편 토픽 검증 (`/mcu/battery_state` & `/mcu/emergency_status`)
브릿지 노드 내부에서 상위 플래너 및 내비게이션 표준 호환을 위해 추가 변환 발행하는 두 개 토픽도 함께 모니터링하여 검증합니다.

```bash
# 1. 표준 배터리 메시지 수신 확인
ros2 topic echo /mcu/battery_state

# 2. 안전 비상정지 전용 Bool 토픽 감시
ros2 topic echo /mcu/emergency_status
```

* **`/mcu/battery_state` 검증**:
  * `voltage`와 `current` 값이 BMS 텔레메트리와 완벽히 일치하는지 검토합니다.
  * `percentage` 값은 BMS SoC의 `%` 값을 표준 `0.0` (방전) ~ `1.0` (만충) 단위 범위로 변환하여 출력하는지 확인합니다. (예: SoC가 `88.0%` 일 때, `0.88`으로 들어와야 정상)
* **`/mcu/emergency_status` 검증**:
  * 비상정지 버튼을 누르거나 풀 때 전송되는 이벤트 혹은 20Hz 주기 신호가 상위 내비게이션 노드의 안전 긴급 회피 제어기로 `std_msgs/msg/Bool` 형태로 정확히 전달되는지 판별합니다. (`False` ➔ `True`)

---

## 4. 통신 예외 상황 및 안정성 복구 검증 (Fault Tolerance)

실제 로봇 주행 및 운용 도중 진동 등에 의한 USB 케이블 이탈 혹은 순간적인 전기적 노이즈가 유입되었을 때, 노드가 안전하게 복구되는지 검증하기 위한 물리적 시나리오입니다.

### 4.1 USB 통신 케이블 강제 이탈 시나리오
1. ROS 2 시리얼 브릿지 노드가 정상적으로 구동 중인 상태에서, MCU 보드와 내비게이션 PC 간의 **시리얼 전원/데이터 USB 케이블을 손으로 가볍게 잡아 뽑아 분리**합니다.
2. **기대 브릿지 로거 반응 (PC 터미널 창)**:
   * 분리 즉시 노드가 오류를 가볍게 트랩(Trap)하고 아래 형태의 Warning 경고 문구를 주기적으로 터미널에 뿜어내야 합니다.
   ```text
   [WARN] [mcu_bridge_node]: Serial error on port '/dev/ttyUSB0': device reports readiness to read but returned no data. Reconnecting in 2.0s...
   ```
3. 약 5~10초 동안 대기한 뒤, **USB 시리얼 포트를 다시 메인 PC 포트에 재결합**합니다.
4. **기대 브릿지 로거 반응 (PC 터미널 창)**:
   * 사용자의 노드 재실행 명령 없이도 백그라운드 재연결 스레드에 의해 다시 아래 연결 성공 문구가 출력되며 20Hz 텔레메트리가 자동으로 발행 재개되는지 모니터링합니다.
   ```text
   [INFO] [mcu_bridge_node]: Connecting to serial port '/dev/ttyUSB0'...
   [INFO] [mcu_bridge_node]: Successfully connected to '/dev/ttyUSB0'.
   ```

### 4.2 프로토콜 노이즈 복구 (FSM Resync) 시나리오
1. 시리얼 버스 통신 중간에 프레임 헤더 오인 바이트, 노이즈 바이트, 혹은 쓰레기 데이터(`\xFF\x00\xAA\x12\x34\xFE\xDD\xAA\x00\x55`) 등이 유입되더라도 수신 FSM 상태 머신 파서가 무작정 대기하지 않고 다음 정상 패킷 헤더(`0xAA 0x55`)를 찾으면 즉시 복구되는 기능입니다.
2. 실제 펌웨어 및 ROS 노드의 동기화 파서는 이 기능이 내장되어 있어, 중간에 잡음이 있더라도 뒤이어 오는 패킷을 문제없이 수신합니다. 터미널 상에서 `ros2 topic echo /mcu/robot_telemetry`를 계속 관측할 때 데이터 유실로 인한 끊김 현상이 발생하지 않고 정시 수신을 복구하는지 확인하는 것으로 본 무결성 검증을 완료합니다.
