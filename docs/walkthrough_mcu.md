# Mobile Humanoid MCU 펌웨어 & 바이너리 프로토콜 구현 결과 보고서

## 1. 개요 및 구현 요약

하드웨어 명세서([`Mobile Humanoid_IO_List_ver1.0.xlsx`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/raw_data/mobile_mcu/Mobile%20Humanoid_IO_List_ver1.0.xlsx))를 기반으로 기존 머신텐딩용 아두이노 펌웨어(`mt_cnc.ino`)를 전면 개편하고, 8비트 ATmega2560 MCU의 메모리(8KB SRAM) 한계를 극복하기 위해 `rosserial` 의존성을 제거한 뒤 **고신뢰성 경량 바이너리 시리얼 프로토콜**을 적용한 모듈화 펌웨어로 교체했습니다.

---

## 2. 주요 변경 및 생성 파일 목록

### 2.1 MCU 펌웨어 ([`mcu/`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/))
| 파일명 | 역할 및 주요 내용 |
| :--- | :--- |
| [`mcu/config.h`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/config.h) | 핀 맵 정의(신규 멀티 PC, 시작/정지/수동 버튼, 타워 램프, 24V Conv 등), 통신 속도(`57600 bps`), 주기 상수 |
| [`mcu/protocol.h`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/protocol.h) / [`protocol.cpp`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/protocol.cpp) | 바이너리 패킷 프레임(`0xAA 0x55`), 메시지 구조체(Telemetry, Command 등), CRC16-CCITT, FSM 패킷 파서 |
| [`mcu/io_manager.h`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/io_manager.h) / [`io_manager.cpp`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/io_manager.cpp) | 디지털 I/O 디바운스, 3대 PC 전원 온/오프 펄스 시퀀스, 전원 유지 릴레이, Safety PLC 비상 감지, ADC 샘플링 |
| [`mcu/bms.h`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/bms.h) / [`bms.cpp`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/bms.cpp) | `Serial3`(19200 bps) 기반 BMS 주기적 폴링 및 전압/전류/SoC/SoH/온도 패킷 디코딩, 리셋 시퀀스 |
| [`mcu/led_ctrl.h`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/led_ctrl.h) / [`led_ctrl.cpp`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/led_ctrl.cpp) | `Adafruit_NeoPixel` 기반 비동기(Non-blocking) LED 애니메이션 엔진 (Blink, Wipe, Fade, 충전 게이지, 방향지시) |
| [`mcu/main.ino`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/main.ino) | 아두이노 IDE 표준 메인 스케치 (초기화, 패킷 수신 디스패치, 20Hz 텔레메트리 스케줄러) |

### 2.2 문서 및 호스트 도구
| 파일명 | 역할 및 주요 내용 |
| :--- | :--- |
| [`docs/serial_protocol_spec.md`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/docs/serial_protocol_spec.md) | ROS 2 Bridge 노드 개발자를 위한 상세 바이너리 프로토콜 명세서 (패킷 포맷, 메시지 ID, 오프셋별 비트필드) |
| [`tools/mcu_serial_tester.py`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/tools/mcu_serial_tester.py) | PC 측 실시간 텔레메트리 모니터링 CLI 툴 및 패킷 인코딩/디코딩/노이즈 복구 자체 검증기 |

---

## 3. 검증 결과 (Verification Results)

### 3.1 C++ 펌웨어 정적 구조 및 컴파일 검증
- 모든 구조체의 바이트 패킹(`pragma pack(1)`) 일치 여부, CRC16 계산, FSM 상태머신 파서, 모듈 인스턴스 초기화 동작을 C++ 네이티브 빌드로 검증 완료:
  - `sizeof(RobotTelemetryPayload)`: 13 Bytes
  - `sizeof(BmsTelemetryPayload)`: 21 Bytes
  - `sizeof(SystemInfoPayload)`: 48 Bytes
  - `sizeof(AckNackPayload)`: 2 Bytes
  - `sizeof(CmdPowerCtrlPayload)`: 2 Bytes
  - `sizeof(CmdLedCtrlPayload)`: 5 Bytes
  - `sizeof(CmdSignalCtrlPayload)`: 5 Bytes
  - `sizeof(CmdBmsCtrlPayload)`: 2 Bytes
  - **결과**: `All C++ module tests compiled and passed with ZERO errors! ✅`

### 3.2 Python 프로토콜 인코딩/디코딩 및 노이즈 복구 검증
- `tools/mcu_serial_tester.py --test` 실행 결과:
  1. `Robot Telemetry Pack/Unpack`: **PASS ✅**
  2. `LED Command Pack/Unpack`: **PASS ✅**
  3. `Noise Immunity & Re-sync`: **PASS ✅**

---

## 4. 빌드 및 사용 방법

### 아두이노 IDE에서 빌드 및 업로드
1. 아두이노 IDE에서 [`mcu/main.ino`](file:///Users/zzang/Documents/VoidDocs/works/workspace/projects_wonik/20260811_w_humanoid/workspace/mobile/mcu/main.ino) 파일을 엽니다.
2. 보드 설정: **Tools $\rightarrow$ Board $\rightarrow$ Arduino Mega or Mega 2560** 선택.
3. 라이브러리: `Adafruit NeoPixel` 설치 확인.
4. **컴파일 및 업로드** 버튼을 누르면 즉시 빌드 및 펌웨어 다운로드가 완료됩니다.

### PC에서 프로토콜 테스트 도구 실행
```bash
# 자체 인코딩/디코딩 검증 실행
.venv/bin/python tools/mcu_serial_tester.py --test

# 실제 로봇 연결 시 (포트 /dev/ttyUSB0, 57600 baud)
.venv/bin/python tools/mcu_serial_tester.py -p /dev/ttyUSB0 -b 57600
```
