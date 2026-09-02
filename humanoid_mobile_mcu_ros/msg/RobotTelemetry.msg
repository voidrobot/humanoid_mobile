# RobotTelemetry.msg
# Mobile Humanoid MCU 종합 텔레메트리 (20Hz 주기)

std_msgs/Header header

# 버튼 및 모드 스위치 상태
bool pwr_btn            # 전원 버튼 눌림 상태
bool start_btn          # 시작 버튼 눌림 상태
bool stop_btn           # 정지 버튼 눌림 상태
bool manu_sw            # 수동 모드 스위치 상태

# PC 부팅 및 동작 상태
bool nav_pc_live        # 메인 내비게이션 PC (NUC) 동작 중
bool humanoid_pc_live   # 휴머노이드 제어 PC 동작 중
bool ai_pc_live         # AI 휴머노이드 PC 동작 중

# 안전 및 충전 감지
bool safety_ems         # Safety PLC 비상정지 활성화 여부 (True: 비상정지 상태)
bool manu_chg_dock      # 수동 충전 단자 감지
bool is_charging        # 충전 중 상태 (전류 > 0.5A 또는 독 감지)

# 출력 릴레이 및 램프 상태
bool pwr_hold           # Main Power Hold 릴레이 상태
bool volt24v_1          # 24V Converter #1 Remote 상태
bool volt24v_2          # 24V Converter #2 Remote 상태
bool chg_on_enable      # 자동 충전 접점 인에이블 상태
bool lamp_red           # 시그널 타워 RED 램프 상태
bool lamp_grn           # 시그널 타워 GREEN 램프 상태
bool lamp_yel           # 시그널 타워 YELLOW 램프 상태
bool buzzer             # 부저 출력 상태

# 아날로그 측정 및 상태
float32 chg_current_adc # 충전 전류 측정값 (Amperes / ADC Voltage)
uint8 led_current_mode  # 현재 LED 동작 모드 번호 (0~55)
uint32 uptime_sec       # MCU 부팅 후 가동 시간 (초)
