# SignalCmd.msg
# 시그널 타워 램프, 부저 및 TM Remote 제어 명령

# 제어 상태 상수 (각 필드에 적용)
uint8 STATE_OFF       = 0       # OFF (끄기)
uint8 STATE_ON        = 1       # ON (켜기)
uint8 STATE_NO_CHANGE = 255     # 현재 상태 유지 (0xFF)

uint8 lamp_red      # 시그널 타워 RED 램프 (0: OFF, 1: ON, 255: 유지)
uint8 lamp_grn      # 시그널 타워 GREEN 램프 (0: OFF, 1: ON, 255: 유지)
uint8 lamp_yel      # 시그널 타워 YELLOW 램프 (0: OFF, 1: ON, 255: 유지)
uint8 buzzer        # 부저 출력 (0: OFF, 1: ON, 255: 유지)
uint8 tm_remote     # TM Remote 출력 (0: OFF, 1: ON, 255: 유지)
