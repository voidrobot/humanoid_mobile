# BmsTelemetry.msg
# Battery Management System (BMS) 텔레메트리 (1Hz 주기)

std_msgs/Header header

float32 voltage     # 배터리 총 전압 (V)
float32 current     # 배터리 충/방전 전류 (A, 충전: +, 방전: -)
float32 soc         # 잔여 용량 State of Charge (%)
float32 soh         # 배터리 수명 State of Health (%)
float32 temp        # 배터리 온도 (°C)
bool is_valid       # BMS 통신 유효 여부 (True: 정상 수신 중, False: 통신 실패/타임아웃)
