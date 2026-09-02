# PowerCmd.msg
# PC 및 전원 컨버터 제어 명령

# 제어 대상 디바이스 상수
uint8 TARGET_HUMANOID_PC = 1    # 휴머노이드 제어 PC
uint8 TARGET_AI_PC        = 2    # AI 휴머노이드 PC
uint8 TARGET_24V_CONV_1   = 3    # 24V 컨버터 1
uint8 TARGET_24V_CONV_2   = 4    # 24V 컨버터 2
uint8 TARGET_MAIN_PC      = 5    # 메인 내비게이션 PC

# 동작 모드 상수
uint8 ACTION_OFF   = 0          # 전원 끄기 / 셧다운
uint8 ACTION_ON    = 1          # 전원 켜기
uint8 ACTION_PULSE = 2          # 500ms 전원 스위치 ON 펄스 (PC 켜기용)

uint8 target_device             # 제어 대상 (TARGET_* 상수 사용)
uint8 action                    # 제어 동작 (ACTION_* 상수 사용)
