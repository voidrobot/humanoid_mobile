# BmsCmd.msg
# BMS 리셋 및 충전 제어 명령

uint8 bms_reset_pulse   # 1: BMS 리셋 펄스 시퀀스 실행, 0: 동작 안함
uint8 chg_enable        # 1: 자동 충전 접점 연결 (ON), 0: 접점 차단 (OFF)
