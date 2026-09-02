/* ================================================================================
* MCU software for the WONIK QnC CNC 5th Machine tending
* Based on MT_MCT MCU
* Create date : 2026.3.17
* Modification
=================================================================================== */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <Adafruit_NeoPixel.h>
#include <avr/wdt.h>

#define SW_VER  "MT_CNC MCU 1.01"
#define ROS_SERIAL_BAUD_RATE 57600 
#define MAX_BMS_PACKET_SIZE 11

// =================== GPIO ========================
#define BMS_SW            43 // output
#define Manu_Charging      3 // input

#define LED_INDI          11 // output pwm LED2(bottom)
#define LED_STATUS        10 // output pwm LED1(top)

#define PWR_BTN_LED       47 // output
#define PWR_BTN            6 // input
#define PWR_QR_TRIGGER    46 // output

#define BUZZER            69 // output
#define STOP_BTN_LED      44 // output
#define STOP_BTN           9 // input
#define LIVE_LED          39 // output 아두이노 상태 LED

#define PC_PWR_SW         31 // output
#define PC_PWR_LIVE       42 // input

#define QUARTZ_SENSOR     22 // input
#define AMR_QUARTZ_SENSOR 23 // input
#define PWR_5V            26 // output
#define PWR_GRIPPER       27 // output
#define PWR_MD            28 // output
#define PWR_MANIPULATOR   29 // output

#define Emergency         13 // input 

#define CHG_DET           A1 // input
#define CHG_ON_ENABLE     A0 // output
#define PWR_HOLD          A2 // output
#define Volt24v_1         A9 // output 
#define Volt24v_2        A10 // output 
// =================================================

enum LED_CTRL {
  LED_OFF,      //0
  RED_WIPE,
  GREEN_WIPE,
  WHITE_WIPE,
  BLUE_WIPE,
  FULL_WHITE,   //5
  FULL_RED,
  FULL_GREEN,
  FULL_BLUE,
  FULL_ORGNE,
  FULL_YELLOW,    //10
  FULL_INDIGO,
  ORGNE_WIPE,
  YELLOW_WIPE,
  INDIGO_WIPE,
  FADE_WHITE,     //15
  FADE_RED,
  FADE_BLUE,
  FADE_GREEN,
  FADE_YELLOW,
  FADE_INDIGO,   //20
  SWING_RED,     //21
  SWING_BLUE,    //22
  SWING_GREEN,   //23
  SWING_WHITE,   //24
  BLINK_RED,     //25
  BLINK_GREEN,   //26
  BLINK_BLUE,    //27
  BLINK_WHITE,   //28
  BLINK_YELLOW,  //29
  BLINK_ORGNE,   //30
  FADE_ORGNE,    //31
  BLINK_INDIGO,  //32
  INDI_RIGHT_BLINK_RED,     //33
  INDI_RIGHT_BLINK_GREEN,   //34
  INDI_RIGHT_BLINK_BLUE,    //35
  INDI_RIGHT_BLINK_WHITE,   //36
  INDI_RIGHT_BLINK_YELLOW,  //37
  INDI_RIGHT_BLINK_ORGNE,   //38
  INDI_LEFT_BLINK_RED,      //39
  INDI_LEFT_BLINK_GREEN,    //40
  INDI_LEFT_BLINK_BLUE,     //41
  INDI_LEFT_BLINK_WHITE,    //42
  INDI_LEFT_BLINK_YELLOW,   //43
  INDI_LEFT_BLINK_ORGNE,    //44
  INDI_BLINK_RED,           //45
  INDI_BLINK_GREEN,         //46
  INDI_BLINK_BLUE,          //47
  INDI_BLINK_WHITE,         //48
  INDI_BLINK_YELLOW,        //49
  INDI_BLINK_ORGNE,         //50
  INDI_FULL_RED,            //51
  INDI_FULL_GREEN,
  INDI_FULL_BLUE,
  INDI_FULL_WHITE,     
  INDI_FULL_YELLOW,         //55
  INDI_FULL_ORGNE,
};
#define BLINK_CHG_PWR_OFF 100
#define BLINK_NOR_PWR_OFF 101

#define LED_DEFAULT  BLINK_YELLOW //GREEN_WIPE 
#define LED2_DEFAULT INDI_BLINK_ORGNE 
#define LED_STATUS_NUM 60
#define LED_INDI_NUM  128
#define BAT_CHG_THRES   0

#define DOOR_OPEN    0x0
#define DOOR_CLOSE   0x1
#define DOOR_UNKNOWN 0x2

//===================== UPDATE RATE ===================

#define ROBOT_MODEL_UPDATE_RATE 5000
#define LED_UPDATE_RATE         50
#define CHARGING_PUBLISH_RATE   1000
#define BAT_DET_UPDATE_RATE     5000
#define NUC_STATUS_UPDATE_RATE  2000

//====================== LED ==========================

#define LED_BRIGHT_DEFAULT 200
#define LED_BRIGHT_FADE    255

typedef unsigned char BYTE;
typedef unsigned int  WORD;
typedef struct {
  BYTE byLow;
  BYTE byHigh;
} IByte;

bool ReceivePacketOK = true;

#define SIG_OFF              0
#define SIG_EMO_STATUS       1
#define SIG_MIS_WAITING      2
#define SIG_NO_EMO_STATUS    3
#define SIG_DOCKER_MOVING    4
#define SIG_CHG_ST_MOVING    5
#define SIG_CALL_MOVING      6
#define SIG_BAT_CHARGING     7
#define SIG_NAVI_FAIL        8
#define SIG_NAVI_OK          9
#define SIG_NUC_OFF          10
#define SIG_BAT_CHARGING_OFF 11
#define SIG_DOOR_OPEN        12

#ifdef __AVR__
  #include <avr/power.h>
#endif
Adafruit_NeoPixel led_status = Adafruit_NeoPixel(LED_STATUS_NUM, LED_STATUS, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led_indi   = Adafruit_NeoPixel(LED_INDI_NUM,   LED_INDI,   NEO_GRB + NEO_KHZ800);

//======================= LED ====================================
uint8_t led_function_num = LED_DEFAULT;
uint8_t led2_function_num = LED2_DEFAULT;
int16_t currentBright = 0;
uint8_t led_function_num_old = 0;
uint8_t led2_function_num_old = 0;
uint8_t led1_status = 0;
uint8_t led2_status = 0;
uint16_t blink_count = 0 ;
uint16_t blink_count2 = 0 ;
uint16_t blink_count_indi = 0;
uint16_t currentPixel = 0;
bool blink_led = true;
bool blink_on = true;
bool blink_on_indi = true;
bool publishing_led1 = false;
bool publishing_led2 = false;
uint8_t led2_count = 0;

//======================= Timer period  ===========================
unsigned long last_led_updated = 0;
unsigned long last_led2_updated = 0;
unsigned long last_charging_published = 0;
unsigned long last_robot_model_level_published = 0;
unsigned long last_button_updated = 0;
unsigned long last_live_status_updated = 0;
unsigned long last_chg_det_updated = 0;
unsigned long last_pc_pwr_status_updated = 0;
unsigned long last_power_push_time = 1;
unsigned long last_bms_updated = 0;
unsigned long last_poweroff_updated = 0;
unsigned long last_quartz_status_updated = 0;

//========================== BUTTON =============================
bool pwr_button;
bool manual_charging;

bool emerg_button_old = true;
bool emerg_button_first = true;
bool is_emerg_mode = false;
bool is_poweroff_pwrkey = true ;
bool first_push = true;
bool is_action_btn_pressed = false;
uint8_t pwr_btn_count = 0;

//======================== poweroff ===============================
bool poweroff_pub = false;
bool button_release_wait = false;
bool force_pwroff_release_wait = false;

//========================== ETC =================================
bool ros_connect_status;
char strTemp[100];
String strTemp1;
byte buffer1[100];
int buffer_count = -1;
bool live_led_status_flag = false;
int pc_pwr_live = 0;
bool is_poweroff = false;
bool is_poweroff_old = false;
bool ext_emerg_en = false;
float chg_current_adc = 0;
bool is_chg_mode = false;
bool buzzer_en = false;
bool power_off_started = false;
bool qr_trigger_en = false;

bool bright_increase = true;
bool updown_status = false;
boolean reverse = false;
int led_array = 0;
int led_indi_array = 0;

#define DELAY 5 

float voltage, current, temp;
int soc, batt_status, soh;
bool bms_read_ok = false;

unsigned long previous_pc_liveMillis = 0;

//============================ Publisher =========================
std_msgs::Bool charging_msg; 
std_msgs::Bool emerg_mode_msg;
std_msgs::Bool power_btn_msg; 
std_msgs::Bool Manual_charging_msg;
std_msgs::String sw_version_msg;
std_msgs::UInt8 led1_status_msg;
std_msgs::UInt8 led2_status_msg;
std_msgs::Float32 current_adc_msg;
std_msgs::Float32 batt_voltage_msg;
std_msgs::Float32 batt_current_msg;
std_msgs::UInt8 batt_soc_msg;
std_msgs::Float32 batt_temp_msg;
std_msgs::UInt8 batt_soh_msg;
std_msgs::Bool stop_btn_msg; 
std_msgs::Bool quartz_sensor_msg;
std_msgs::Bool amr_quartz_sensor_msg;

ros::NodeHandle nh;
ros::Publisher charging_pub("charging_status", &charging_msg);
ros::Publisher current_adc_pub("current_adc", &current_adc_msg);
ros::Publisher emerg_mode_pub("emerg_mode2", &emerg_mode_msg);
ros::Publisher power_btn_pub("pwr_btn", &power_btn_msg);
ros::Publisher manual_charging_pub("manual_charging", &Manual_charging_msg);
ros::Publisher sw_version_pub("mcu_sw_version", &sw_version_msg);
ros::Publisher led1_status_pub("led_status", &led1_status_msg);
ros::Publisher led2_status_pub("led2_status", &led2_status_msg);
ros::Publisher batt_vol_pub("batt_voltage_bms", &batt_voltage_msg);
ros::Publisher batt_current_pub("batt_current_bms", &batt_current_msg);
ros::Publisher batt_soc_pub("batt_soc_bms", &batt_soc_msg);
ros::Publisher batt_temp_pub("batt_temp_bms", &batt_temp_msg);
ros::Publisher batt_soh_pub("batt_soh_bms", &batt_soh_msg);
ros::Publisher stop_btn_pub("stop_btn",   &stop_btn_msg);
ros::Publisher quartz_sensor_pub("quartz_sensor", &quartz_sensor_msg);
ros::Publisher amr_quartz_sensor_pub("amr_quartz_sensor", &amr_quartz_sensor_msg);

void publishIsCharging() {
  charging_msg.data = is_chg_mode;
  charging_pub.publish(&charging_msg);
}

void publishCurrent_ADC(float adc) {
  current_adc_msg.data = adc;
  current_adc_pub.publish(&current_adc_msg );
}

void publishEmergBTN() {
  emerg_mode_msg.data = is_emerg_mode;
  emerg_mode_pub.publish(&emerg_mode_msg );
}

void publishPowerBTN(bool data) {
  power_btn_msg.data = data;
  power_btn_pub.publish(&power_btn_msg );
}

void publishManualCharging(bool data) {
  Manual_charging_msg.data = data;
  manual_charging_pub.publish(&Manual_charging_msg );
}

void publish_SW_Version(char* data) {
  sw_version_msg.data = data;
  sw_version_pub.publish(&sw_version_msg );
}

void publish_led1_status(uint8_t data) {
  publishing_led1 = true;
  led1_status_msg.data = data;
  led1_status_pub.publish(&led1_status_msg );
  publishing_led1 = false;
}

void publish_led2_status(uint8_t data) {
  publishing_led2 = true;
  led2_status_msg.data = data;
  led2_status_pub.publish(&led2_status_msg );
  publishing_led2 = false;
}

void publish_bms() {
  if((voltage <= 0)||(soc < 10)||(soc > 100)||(soh < 10)||(soh > 100)) return;
  batt_voltage_msg.data = voltage;
  batt_current_msg.data = roundf(current*100.0)/100.0;
  batt_soc_msg.data = soc;
  batt_temp_msg.data = roundf(temp*100.0)/100.0;
  if(batt_temp_msg.data < 0) return;
  batt_soh_msg.data = soh;
  batt_vol_pub.publish(&batt_voltage_msg );
  batt_current_pub.publish(&batt_current_msg );
  batt_soc_pub.publish(&batt_soc_msg );
  batt_temp_pub.publish(&batt_temp_msg );
  batt_soh_pub.publish(&batt_soh_msg );
}

void publishMissionStopBTN(bool data) {
  stop_btn_msg.data = data;
  stop_btn_pub.publish(&stop_btn_msg );
}

void publish_quartz_sensor(bool data) {
  quartz_sensor_msg.data = data;
  quartz_sensor_pub.publish(&quartz_sensor_msg );
}

void publish_amr_quartz_sensor(bool data) {
  amr_quartz_sensor_msg.data = data;
  amr_quartz_sensor_pub.publish(&amr_quartz_sensor_msg );
}

//=========================== Subscriber callbody ===========================
ros::Subscriber<std_msgs::Bool> chg_on_enable_sub("chg_on_enable", chgOnEnableCallback);
// CHG_ON_ENABLE 상태 변경 함수
void chgOnEnableCallback(const std_msgs::Bool& msg) {
    if (msg.data) {
        digitalWrite(CHG_ON_ENABLE, HIGH); // 활성화
    } else {
        digitalWrite(CHG_ON_ENABLE, LOW); // 비활성화
    }
}

ros::Subscriber<std_msgs::UInt16> led_cmd("cmd_led", led_cb);
void led_cb( const std_msgs::UInt16& cmd_msg){
     if(led_function_num == (uint8_t)(cmd_msg.data)) 
     {
         sprintf(strTemp, "LED num is same : %d", led_function_num);
         nh.loginfo(strTemp);
     }
     else
     {
         led_function_num =(uint8_t)(cmd_msg.data);
         sprintf(strTemp, "MCU Received LED : %d", led_function_num);
         nh.loginfo(strTemp);
         publish_led1_status(led_function_num);
         led1_status = led_function_num;
     }
}
ros::Subscriber<std_msgs::UInt16> led2_cmd("cmd_led2", led2_cb);
void led2_cb( const std_msgs::UInt16& cmd_msg){
     if(led2_function_num == (uint8_t)(cmd_msg.data)) 
     {
         sprintf(strTemp, "LED2 num is same : %d", led2_function_num);
         nh.loginfo(strTemp);
     }
     else
     {
         led2_function_num =(uint8_t)(cmd_msg.data);
         sprintf(strTemp, "MCU Received LED2 : %d", led2_function_num);
         nh.loginfo(strTemp);
         publish_led2_status(led2_function_num);
         led2_status = led2_function_num;
     }
}
ros::Subscriber<std_msgs::Bool> ext_emerg_cmd("cmd_ext_emerg", ext_emerg_cb);
void ext_emerg_cb( const std_msgs::Bool& cmd_msg){
     sprintf(strTemp, "MCU EXT EMERG : %d", ext_emerg_en);
     nh.loginfo(strTemp);
}
ros::Subscriber<std_msgs::Bool> bat_bms_pwr_enable_cmd("cmd_bat_bms_pwr_enable", bat_bms_pwr_enable_cb);
void bat_bms_pwr_enable_cb(const std_msgs::Bool& cmd_msg){
     bms_reset();
     sprintf(strTemp, "BAT BMS RESET");
     nh.loginfo(strTemp);
}

ros::Subscriber<std_msgs::Bool> buzzer_cmd("cmd_buzzer", buzzer_cb);
void buzzer_cb( const std_msgs::Bool& cmd_msg){
     buzzer_en = (bool)(cmd_msg.data);
     sprintf(strTemp, "MCU BUZZER : %d", buzzer_en);
     nh.loginfo(strTemp);
}

ros::Subscriber<std_msgs::Bool> qr_trigger_cmd("qr_cmd", qr_trigger_cmd_cb);
void qr_trigger_cmd_cb( const std_msgs::Bool& cmd_msg){
  qr_trigger_en = (bool)(cmd_msg.data);
  sprintf(strTemp, "QR TRIGGER : %d", qr_trigger_en);
  nh.loginfo(strTemp);
  if (qr_trigger_en) {
    digitalWrite(PWR_QR_TRIGGER,HIGH);
  } else {
    digitalWrite(PWR_QR_TRIGGER,LOW);
  }
}

//===========================================================================
void led_init(){
   led_status.begin();
   led_status.setBrightness(LED_BRIGHT_DEFAULT);
   led_status.show(); 
}
void led_indi_init(){
   led_indi.begin();
   led_indi.setBrightness(LED_BRIGHT_DEFAULT);
   led_indi.show(); 
}

void colorBlink_chg_batt(uint32_t c) {

    led_status.setBrightness(LED_BRIGHT_DEFAULT);

    if(blink_on)
    { 
      for(uint16_t i=4; i<6; i++){
            led_status.setPixelColor(i,c);
      }
    }
    else
    {
      for(uint16_t i=4; i<6; i++){
            led_status.setPixelColor(i,0);
      }
    }

   led_status.show();

   blink_count++;
   if(blink_count> 10) { blink_on = !blink_on; blink_count = 0;}
}
void colorBlink(uint32_t c) {
    led_status.setBrightness(LED_BRIGHT_DEFAULT);

    if(blink_on)
    { 
      for(uint16_t i=0; i<7; i++){
          led_array = i*10;
          for(uint16_t j=led_array; j<led_array+10; j++){
            led_status.setPixelColor(j, c );
          }
          delay(DELAY);
       }
    }
    else
    {
      for(uint16_t i=0; i<7; i++){
          led_array = i*10;
          for(uint16_t j=led_array; j<led_array+10; j++){
            led_status.setPixelColor(j, 0 );
          }
          delay(DELAY);
       }
   }

   led_status.show();

   blink_count++;
   if(blink_count> 10) { blink_on = !blink_on; blink_count = 0;}
}
void colorBlink_indi(uint32_t c) {
    led_indi.setBrightness(LED_BRIGHT_DEFAULT);

    if(blink_on_indi)
    { 
       for(uint16_t i=0; i<9; i++){
          led_indi_array = i*12;
          for(uint16_t j=led_indi_array; j<led_indi_array+12; j++){
            led_indi.setPixelColor(j, c );
          }
          delay(DELAY);
       }
   }
   else
   {
       for(uint16_t i=0; i<9; i++){
          led_indi_array = i*12;
          for(uint16_t j=led_indi_array; j<led_indi_array+12; j++){
            led_indi.setPixelColor(j, 0 );
          }
          delay(DELAY);
       }
    }

   led_indi.show();

   blink_count_indi++;
   if(blink_count_indi> 6) { blink_on_indi = !blink_on_indi; blink_count_indi = 0;}
}
void colorBlink_indi_right(uint32_t c) {
    led_indi.setBrightness(LED_BRIGHT_DEFAULT);

    if(blink_on_indi)
    { 
      for(uint16_t i=24; i<36; i++) {
        led_indi.setPixelColor(i, c );
      }
      delay(DELAY);
      for(uint16_t i=36; i<48; i++) {
        led_indi.setPixelColor(i, c );
      }
      delay(DELAY);
      for(uint16_t i=48; i<60; i++) {
        led_indi.setPixelColor(i, c );
      }
      delay(DELAY);
      for(uint16_t i=60; i<72; i++) {
        led_indi.setPixelColor(i, c );
      }
      delay(DELAY);
   }
    else
    {
      for(uint16_t i=24; i<36; i++) {
        led_indi.setPixelColor(i, 0 );
      }
      delay(DELAY);
      for(uint16_t i=36; i<48; i++) {
        led_indi.setPixelColor(i, 0 );
      }
      delay(DELAY);
      for(uint16_t i=48; i<60; i++) {
        led_indi.setPixelColor(i, 0 );
      }
      delay(DELAY);
      for(uint16_t i=60; i<72; i++) {
        led_indi.setPixelColor(i, 0 );
      }
      delay(DELAY);
    }

   led_indi.show();

   blink_count_indi++;
   if(blink_count_indi> 4) { blink_on_indi = !blink_on_indi; blink_count_indi = 0;}
}
void colorBlink_indi_left(uint32_t c) {
    led_indi.setBrightness(LED_BRIGHT_DEFAULT);

    if(blink_on_indi)
    { 
      for(uint16_t i=0; i<12; i++) {
        led_indi.setPixelColor(i, c );
        }
      delay(DELAY);
      for(uint16_t i=12; i<24; i++) {
        led_indi.setPixelColor(i, c );
        }
      delay(DELAY);
      for(uint16_t i=72; i<84; i++) {
        led_indi.setPixelColor(i, c );
       }
      delay(DELAY);
        for(uint16_t i=84; i<96; i++) {
        led_indi.setPixelColor(i, c );
       }
      delay(DELAY);
    }
    else
    {
        for(uint16_t i=0; i<12; i++) {
        led_indi.setPixelColor(i, 0 );
        }
      delay(DELAY);
      for(uint16_t i=12; i<24; i++) {
        led_indi.setPixelColor(i, 0 );
        }
      delay(DELAY);
      for(uint16_t i=72; i<84; i++) {
        led_indi.setPixelColor(i, 0 );
       }
      delay(DELAY);
        for(uint16_t i=84; i<96; i++) {
        led_indi.setPixelColor(i, 0 );
       }
      delay(DELAY);
   }

   led_indi.show();

   blink_count_indi++;
   if(blink_count_indi> 4) { blink_on_indi = !blink_on_indi; blink_count_indi = 0;}
}
#if 1
void colorWipe(uint32_t c) {

   #if 1
     led_status.setBrightness(LED_BRIGHT_DEFAULT);
    uint8_t led_display_num = (led_status.numPixels()-20)/2;
    
    led_status.setPixelColor(currentPixel+9, c);
    if((currentPixel == 0)||(currentPixel == 1)||(currentPixel == 2)||(currentPixel == 3)||(currentPixel == 4))
    {
       led_status.setPixelColor(30-(5-currentPixel), 0);
    }else
    {
       led_status.setPixelColor(currentPixel+9-5, 0);
    }

    #if 1
    led_status.setPixelColor(led_status.numPixels() - currentPixel, c);
     
    if((currentPixel == 0)||(currentPixel == 1)||(currentPixel == 2)||(currentPixel == 3)||(currentPixel == 4))
    {
       led_status.setPixelColor((led_status.numPixels()- 1 - led_display_num) + (5-currentPixel), 0);
    }else
    {
       led_status.setPixelColor(led_status.numPixels()- 1 - (currentPixel-5), 0);
    }
    #endif

    blink_count2++;
    if(blink_count2 > 8 ) {
      blink_count2 = 0; 
      blink_led = !blink_led;
    }

    if(blink_led)
    {
     for(uint16_t i=0 ; i<10; i++) {
        led_status.setPixelColor(i, c );
      }  
     for(uint16_t i=(led_status.numPixels()/2) ; i<(led_status.numPixels()-20); i++) {
        led_status.setPixelColor(i, c );
      }
    }else
    {
      for(uint16_t i=0 ; i<10; i++) {
        led_status.setPixelColor(i, 0);
      }
      for(uint16_t i=(led_status.numPixels()/2) ; i<(led_status.numPixels()-20); i++) {
        led_status.setPixelColor(i, 0);
      }
    }
    led_status.show();
    currentPixel++;
    if(currentPixel == led_display_num+1) currentPixel = 0;

    #endif
}
#endif
void fullColor(uint32_t c) {

    led_status.setBrightness(LED_BRIGHT_DEFAULT);
     for(uint16_t i=0; i<7; i++){
          led_array = i*10;
          for(uint16_t j=led_array; j<led_array+10; j++){
            led_status.setPixelColor(j, c );
          }
           delay(2);
       }
     led_status.show();
}
void fullColor_indi(uint32_t c) {

    led_indi.setBrightness(LED_BRIGHT_DEFAULT);

     for(uint16_t i=0; i<9; i++){
          led_indi_array = i*12;
          for(uint16_t j=led_indi_array; j<led_indi_array+12; j++){
            led_indi.setPixelColor(j, c );
          }
           delay(2);
     }
    led_indi.show();
}
void fullColor_fade(uint32_t c) {

    if(bright_increase) currentBright = currentBright + 20;
     else currentBright = currentBright - 20;

    if(currentBright > LED_BRIGHT_FADE) bright_increase = false;
    if(currentBright < 0)  bright_increase = true;

   #if 1
    for(uint16_t i=0; i<7; i++){
          led_array = i*10;
          for(uint16_t j=led_array; j<led_array+10; j++){
             if(currentBright>LED_BRIGHT_FADE){
                led_status.setPixelColor(j, c);
                led_status.setBrightness(LED_BRIGHT_FADE);
             }
             else if(currentBright<0){
                led_status.setPixelColor(j, c);
                led_status.setBrightness(0);
            }
            else
            {
              led_status.setPixelColor(j, c);
              led_status.setBrightness(currentBright);
             }
          }
          delay(2);
       }
    #endif
   led_status.show();
}

void fullWhite() {

    for(uint16_t i=0; i<led_status.numPixels(); i++) {
        led_status.setPixelColor(i, led_status.Color(0,0,0, 255 ) );
     }
      led_status.show();
      led_status.show();
}

void front_led_colorWipe_forward_backward(uint32_t c) {
    led_status.setBrightness(LED_BRIGHT_DEFAULT);
    led_status.setPixelColor(currentPixel, c);
    if(!reverse){
      if((currentPixel == 0))
      {
           led_status.setPixelColor(led_status.numPixels()-(6-currentPixel), 0);
       }else
      {
            led_status.setPixelColor(currentPixel-6, 0);
       }
       led_status.show();
       currentPixel++;
       if(currentPixel == led_status.numPixels()-1) reverse = true;
    }
    else
    {
       if(currentPixel == led_status.numPixels())
      {
           led_status.setPixelColor(0, 0);
       }else
      {
            led_status.setPixelColor(currentPixel+6, 0);
       }
       led_status.show();
       currentPixel--;
       if(currentPixel == 0) reverse = false;
    }
 }

uint8_t red(uint32_t c) {
  return (c >> 8);
}
uint8_t green(uint32_t c) {
  return (c >> 16);
}
uint8_t blue(uint32_t c) {
  return (c);
}
void led_function(LED_CTRL num)
{
  switch(num)
  {
    case 0 :
      fullColor(led_status.Color(0, 0, 0)); // off
    break;
    case 1 :
      colorWipe(led_status.Color(255, 0, 0)); // Red
    break;
    case 2 :
      colorWipe(led_status.Color(0, 255, 0)); // Green
    break;
    case 4 :
      colorWipe(led_status.Color(0, 0, 255)); // Blue
    break;
    case 3 :
     colorWipe(led_status.Color(255, 255, 255)); // White
    break;
    case 5 :
     fullColor(led_status.Color(255, 255, 255));  // Full White
    break;
     case 8 :
     fullColor(led_status.Color(0, 0, 255));   // Full Blue
    break;
     case 7 :
     fullColor(led_status.Color(0, 255, 0)); // Full Green
    break;
     case 6 :
     fullColor(led_status.Color(255, 0, 0)); // Full RED
    break;
     case 9 :
      fullColor(led_status.Color(255, 127, 0)); // Full oragne
    break;
    case 10 :
      fullColor(led_status.Color(255, 255, 0)); // Full yellow
    break;
      case 11 :
      fullColor(led_status.Color(75, 0, 130)); // Full indigo
    break;
   case 12 :
      colorWipe(led_status.Color(255, 127, 0)); // oragne
    break;
    case 13 :
      colorWipe(led_status.Color(255, 255, 0)); // yellow
    break;
    case 14 :
      colorWipe(led_status.Color(75, 0, 130)); // indigo
    break;
    case 15 :
       fullColor_fade(led_status.Color(255, 255, 255)); // fade white
    break;
     case 16 :
      fullColor_fade(led_status.Color(255, 0, 0)); // fade red
    break;
     case 17 :
      fullColor_fade(led_status.Color(0, 0, 255)); // fade blue
    break;
     case 18 :
      fullColor_fade(led_status.Color(0, 255, 0)); // fade green
    break;
    case 19 :
      fullColor_fade(led_status.Color(255, 255, 0)); // fade yellow
    break;
    case 20 :
      fullColor_fade(led_status.Color(75, 0, 130)); // fade indigo
    break;
    case 21 :
      front_led_colorWipe_forward_backward(led_status.Color(255, 0, 0)); // Red
    break;
    case 23 :
      front_led_colorWipe_forward_backward(led_status.Color(0, 255, 0)); // Green
    break;
    case 22 :
      front_led_colorWipe_forward_backward(led_status.Color(0, 0, 255)); // Blue
     break;
    case 24 :
     front_led_colorWipe_forward_backward(led_status.Color(255, 255, 255)); // White
    break;
    case 25 :
      colorBlink(led_status.Color(255, 0, 0)); // blink Red
    break;
     case 26 :
      colorBlink(led_status.Color(0, 255, 0));// blink Green
    break;
     case 27 :
      colorBlink(led_status.Color(0, 0, 255)); // blink Blue
    break;
     case 28 :
      colorBlink(led_status.Color(255, 255, 255)); // blink White
    break;
    case 29 :
     colorBlink(led_status.Color(255, 255, 0)); // blink yellow
    break;
    case 30 :
     colorBlink(led_status.Color(255, 127, 0)); // blink oragne
    break;
    case 31 :
     fullColor_fade(led_status.Color(255, 127, 0)); // fade oragne
    break;
    case 32 :
     colorBlink(led_status.Color(75, 0, 130)); // blink indigo
    break;
    case 100 :
      colorBlink_chg_batt(led_status.Color(255, 0, 0)); // blink Red
     break;
    case 101 :
      colorBlink_chg_batt(led_status.Color(0, 255, 0)); // blink Green
    break;
   }
}
void led2_function(LED_CTRL num)
{
   switch(num)
  { 
    case 0 :
      fullColor_indi(led_indi.Color(0, 0, 0)); // off
    break;
    case 33 :
      colorBlink_indi_right(led_indi.Color(255, 0, 0)); // blink Red
    break;
    case 34 :
      colorBlink_indi_right(led_indi.Color(0, 255, 0));// blink Green
    break;
    case 35 :
      colorBlink_indi_right(led_indi.Color(0, 0, 255)); // blink Blue
    break;
     case 36 :
      colorBlink_indi_right(led_indi.Color(255, 255, 255)); // blink White
    break;
    case 37 :
     colorBlink_indi_right(led_indi.Color(255, 255, 0)); // blink yellow
    break;
    case 38 :
     colorBlink_indi_right(led_indi.Color(255, 127, 0)); // blink oragne
    break;
    case 39 :
      colorBlink_indi_left(led_indi.Color(255, 0, 0)); // blink Red
    break;
    case 40 :
      colorBlink_indi_left(led_indi.Color(0, 255, 0));// blink Green
    break;
    case 41 :
      colorBlink_indi_left(led_indi.Color(0, 0, 255)); // blink Blue
    break;
    case 42 :
      colorBlink_indi_left(led_indi.Color(255, 255, 255)); // blink White
    break;
    case 43 :
     colorBlink_indi_left(led_indi.Color(255, 255, 0)); // blink yellow
    break;
    case 44 :
     colorBlink_indi_left(led_indi.Color(255, 127, 0)); // blink oragne
    break;
     case 45 :
      colorBlink_indi(led_indi.Color(255, 0, 0)); // blink Red
    break;
    case 46 :
      colorBlink_indi(led_indi.Color(0, 255, 0));// blink Green
    break;
    case 47 :
      colorBlink_indi(led_indi.Color(0, 0, 255)); // blink Blue
    break;
    case 48 :
      colorBlink_indi(led_indi.Color(255, 255, 255)); // blink White
    break;
    case 49 :
     colorBlink_indi(led_indi.Color(255, 255, 0)); // blink yellow
    break;
    case 50 :
     colorBlink_indi(led_indi.Color(255, 127, 0)); // blink oragne
    break;
    case 51 :
     fullColor_indi(led_indi.Color(255, 0, 0)); // Full RED
    break;
    case 52 :
     fullColor_indi(led_indi.Color(0, 255, 0)); // Full Green
    break;
    case 53 :
     fullColor_indi(led_indi.Color(0, 0, 255));   // Full Blue
    break;
    case 54 :
     fullColor_indi(led_indi.Color(255, 255, 255));  // Full White
    break;
     case 55 :
      fullColor_indi(led_indi.Color(255, 255, 0)); // Full yellow
    break;
    case 56 :
      fullColor_indi(led_indi.Color(255, 127, 0)); // Full oragne
    break;

  }
}

int Byte2Int(BYTE byLow, BYTE byHigh)
{
  return (byLow | (int)byHigh<<8);
}

long Byte2Long(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
  return((long)byData1 | (long)byData2<<8 | (long)byData3<<16 | 
    (long)byData4<<24);
}

IByte Int2Byte(int nIn)
{
  IByte Ret;

  Ret.byLow = nIn & 0xff;
  Ret.byHigh = nIn>>8 & 0xff;
  return Ret;
}

int ReceivePacketBMS(BYTE byInData[],int leng)
{
  long byChkSum = 0;
  int checksum_index;
  for(int i = 0 ; i < leng+1; i++) 
  {
    if(byInData[i]==0xAF && byInData[i+1]==0xA0) {
      checksum_index = i-1;
      byChkSum = byChkSum - byInData[i-1];
      break;
    }
    if(i>1) byChkSum += byInData[i];

    strTemp1 =  strTemp1 + String(byInData[i], HEX);

    #if 0
    Serial.print(i);  
    Serial.print("-");
    Serial.print(byInData[i],HEX);
    Serial.print(" ");
    #endif
  }

  #if 0
  if(ros_connect_status)
  {
    sprintf(strTemp, "BMS : %s", strTemp1);
    nh.loginfo(strTemp);
  }
  #endif

  if((byte(byChkSum) != byInData[checksum_index]))  return 1;

  voltage = Byte2Int(byInData[7],byInData[6])*0.01;
  current = Byte2Int(byInData[9],byInData[8])*0.01;
  soc     = Byte2Int(byInData[11],byInData[10]);
  temp    = Byte2Int(byInData[13],byInData[12])*0.1;
  soh     = Byte2Int(byInData[15],byInData[14]);

  #if 1
  Serial.print("voltage:");   
  Serial.print(voltage);
  Serial.print(" current:");
  Serial.print(current);
  Serial.print(" soc:");
  Serial.print(soc);

  Serial.print(" temp:");
  Serial.print(temp);
  Serial.print(" soh:");
  Serial.print(soh);
  Serial.println(" ");
  #endif
  return 1;
}

int PutBMSCmd()
{
  BYTE byD[MAX_BMS_PACKET_SIZE];
  long byChkSum = 0;

  byD[0]  = 0xAF;
  byD[1]  = 0xFA;
  byD[2]  = 0x60; //adderss
  byD[3]  = 0x05; //length
  byD[4]  = 0x01; // command
  byD[5]  = 0x60; // order
  byD[6]  = 0x47; //kind1   7:x, 6:temp, 5:dischaging time, 4:charging time, 3:battery status, 2:SOC, 1:current, 0:voltage
  byD[7]  = 0x01; //kind2   2: remaining energy, 1: residual capacity, 0:SOH
  for(int i = 2 ; i < 8; i++)  byChkSum += byD[i];
  byD[8]  = (byte(byChkSum));
  byD[9]  = 0xAF;
  byD[10] = 0xA0;
  byD[11] = '\0';

  Serial3.write(byD,11); // Created for user firmware

//  for(int i = 0 ; i < 11; i++) {
//    Serial.print(" ");
//    Serial.print(byD[i],HEX);
//  }
//  Serial.println("");

  return 1;
}

void check_sw_status()
{
  bool manual_charging = digitalRead(Manu_Charging);
  bool pwr_button = digitalRead(PWR_BTN);

  publishManualCharging(manual_charging);
//   Serial.print("manual_charging: ");
//   Serial.println(manual_charging);
}

void quartz_status()
{
  bool quartz_sensor = digitalRead(QUARTZ_SENSOR);
  bool amr_quartz_sensor = digitalRead(AMR_QUARTZ_SENSOR);

//  Serial.print("quartz_sensor: ");
//  Serial.println(quartz_sensor);
//  Serial.print("amr_quartz_sensor: ");
//  Serial.println(amr_quartz_sensor);

  publish_quartz_sensor(quartz_sensor);
  publish_amr_quartz_sensor(amr_quartz_sensor);
}
//================================= SETUP ==================================
void setup() {

  pinMode(PWR_BTN,     INPUT);
  pinMode(PWR_BTN_LED, OUTPUT);
  pinMode(PC_PWR_SW,   OUTPUT);
  pinMode(PC_PWR_LIVE, INPUT);
  pinMode(CHG_DET,     INPUT);
  pinMode(BUZZER,      OUTPUT);

  pinMode(BMS_SW,        OUTPUT);
  pinMode(PWR_HOLD,      OUTPUT);
  pinMode(CHG_ON_ENABLE, OUTPUT);

  pinMode(LED_INDI,      OUTPUT);
  pinMode(Manu_Charging, INPUT);
  pinMode(LED_STATUS,    OUTPUT);

  pinMode(Volt24v_1, OUTPUT);
  pinMode(Volt24v_2, OUTPUT);
  pinMode(LIVE_LED,  OUTPUT);

  pinMode(STOP_BTN,          INPUT);  // 9
  pinMode(STOP_BTN_LED,      OUTPUT); // D44 stop button
  pinMode(PWR_5V,            OUTPUT); // D26 5V power
  pinMode(PWR_GRIPPER,       OUTPUT); // D27 Gripper power
  pinMode(PWR_MD,            OUTPUT); // D28 Motor Driver power
  pinMode(PWR_MANIPULATOR,   OUTPUT); // D29 Manipulator power
  pinMode(QUARTZ_SENSOR,     INPUT);  // D22 Quartz sensor
  pinMode(AMR_QUARTZ_SENSOR, INPUT);  // D23 AMR Quartz sensor
  pinMode(PWR_QR_TRIGGER,    OUTPUT); // D46 QR reader trigger power

  Serial.begin(57600);   // PC
  Serial3.begin(19200);  // BMS

  digitalWrite(LED_INDI,   HIGH);
  digitalWrite(LED_STATUS, HIGH);

  digitalWrite(Volt24v_1,HIGH);
  digitalWrite(Volt24v_2,HIGH);
  digitalWrite(LIVE_LED, HIGH);

  digitalWrite(BUZZER,LOW);

  digitalWrite(PWR_QR_TRIGGER,HIGH);
  digitalWrite(STOP_BTN_LED,  HIGH);
  digitalWrite(CHG_ON_ENABLE, LOW);
  digitalWrite(PWR_5V,        HIGH);
  delay(500);
  digitalWrite(PWR_MD,HIGH);
  delay(500);
  digitalWrite(PWR_MANIPULATOR,HIGH);
  delay(500);
  digitalWrite(PWR_GRIPPER,HIGH);
  digitalWrite(PWR_QR_TRIGGER,LOW);

  bms_reset();

  led_init();
  led_indi_init();
  bool manual_charging = digitalRead(Manu_Charging);
  bool quartz_sensor = digitalRead(QUARTZ_SENSOR);
  bool amr_quartz_sensor = digitalRead(AMR_QUARTZ_SENSOR);

  bool pwr_button = digitalRead(PWR_BTN);
  led_function(LED_OFF); 
  led2_function(LED_OFF); 
  Serial.println("MCU1 START");

  nh.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  nh.initNode();

  nh.subscribe(led_cmd);  
  nh.subscribe(led2_cmd);
  nh.subscribe(ext_emerg_cmd);
  nh.subscribe(bat_bms_pwr_enable_cmd);
  nh.subscribe(buzzer_cmd);
  nh.subscribe(qr_trigger_cmd);

  nh.advertise(charging_pub);
  nh.advertise(current_adc_pub);
  nh.advertise(emerg_mode_pub);
  nh.advertise(manual_charging_pub);

  nh.advertise(power_btn_pub);
  nh.advertise(sw_version_pub);
  nh.advertise(led1_status_pub);
  nh.advertise(led2_status_pub);
  nh.advertise(batt_vol_pub); 
  nh.advertise(batt_current_pub);
  nh.advertise(batt_soc_pub);
  nh.advertise(batt_temp_pub);
  nh.advertise(batt_soh_pub);
  nh.advertise(stop_btn_pub);
  nh.advertise(quartz_sensor_pub);
  nh.advertise(amr_quartz_sensor_pub);
  nh.subscribe(chg_on_enable_sub);

  is_poweroff_pwrkey = !check_pc_pwr_live();
  is_poweroff = !is_poweroff_pwrkey; 

  chg_current_adc = analogRead(CHG_DET);
  // Serial.print("chg_current_adc : ");
  // Serial.println(chg_current_adc);

//  if(chg_current_adc > BAT_CHG_THRES) {
//    if(nh.connected()) nh.loginfo("MCU : BAT CHARGING");
//    else Serial.println("MCU : BAT CHARGING");
//    is_chg_mode =true;
//  }
//  else {
//    if(nh.connected()) nh.loginfo("MCU : BAT NOT CHARGING");
//     else Serial.println("MCU : BAT NOT CHARGING");
//    is_chg_mode =false;
//  }

  last_led_updated = millis();
  last_led2_updated = millis();
  last_live_status_updated = millis();
  delay(500);
}
//============================= LOOP =====================================
void loop()
{
  if(!nh.connected()){
    if(ros_connect_status) {
      nh.loginfo("MCU : Disonnected");
      ros_connect_status = false;
      // led_function(LED_OFF);
    }
  }
  else
  {
    if(!ros_connect_status) {
      nh.loginfo("MCU : Connected");
      //led_function(LED_OFF);
      ros_connect_status = true;
//      publishIsCharging();
    }
  }

// ============================ CHECK BUTTON ============================
  if (millis() - last_button_updated > 100){
    check_pwr_button_status();
    check_emerg_button_status();
    check_sw_status();
    check_mission_button_status();
    last_button_updated = millis();
  }

// ======================== CHECK QUARTZ SENSOR =========================
  if (millis() - last_quartz_status_updated > 500){
    quartz_status();
    last_quartz_status_updated = millis();
  }
// ============================ BOTTOM LED ===============================
  if (millis() - last_led_updated > LED_UPDATE_RATE) {
    if(led_function_num != led_function_num_old){
      led_function(LED_OFF);
      currentPixel = 0;
      if(led_function_num_old >= BLINK_RED && led_function_num_old <= BLINK_ORGNE || led_function_num_old == BLINK_INDIGO) {
        blink_count = 0;
        blink_on = true;
      }
    }

    if(is_emerg_mode) {
      led_function(FULL_RED);
      led1_status = FULL_RED;
    }
    else if(ext_emerg_en) {
      led_function(FULL_WHITE);
      led1_status = FULL_WHITE;
    }
    else if(!is_poweroff) {  //NUC power off
      if(is_poweroff_old!=is_poweroff)
      {
        led_function(LED_OFF);
      }
      if(is_chg_mode) //charging
      {
        led_function(BLINK_RED);
        led1_status = BLINK_RED;
      }
      else //no charging
      {
        led_function(BLINK_RED);
        led1_status = BLINK_RED;
        led_function_num = LED_DEFAULT;
      }
    }
    else { //NUC power ON
      led_function(led_function_num);
      led1_status = led_function_num;
    } 
    led_function_num_old = led_function_num;
    last_led_updated = millis();
  }

// ============================ MODEL ==============================
  if (millis() - last_robot_model_level_published > ROBOT_MODEL_UPDATE_RATE)
  {
    publish_SW_Version(SW_VER);
    last_robot_model_level_published = millis();
  }

// ============================ INDI LED ===========================
  if (millis() - last_led2_updated > LED_UPDATE_RATE) {
    if(led2_function_num != led2_function_num_old){
      led2_function(LED_OFF);
      blink_on_indi = false;
      //blink_count_indi = 0;
    }
    if(is_emerg_mode) {
      led2_function(INDI_FULL_RED);
      led2_status = INDI_FULL_RED;
    }
    else {
      led2_function(led2_function_num); 
      led2_status = led2_function_num;
    }
       
    led2_function_num_old = led2_function_num;
    last_led2_updated = millis();
  }

// ============================ LIVE LED ============================
  if (millis() - last_live_status_updated > 2000){
    live_led_status();
    is_poweroff_old = is_poweroff;
    is_poweroff = check_pc_pwr_live(); 
    if(!publishing_led1) publish_led1_status(led1_status);
    if(!publishing_led2) publish_led2_status(led2_status);
    last_live_status_updated = millis();
  }

// ============================ REQUEST BMS DATA =========================
  if (!bms_read_ok && (millis() - last_bms_updated > 1000))
  {
    PutBMSCmd();
//    Serial.print("PUTBMS!!!!!!!!!!!!!!!!!!!!!!!!!");
    bms_read_ok = true ;
    last_bms_updated = millis();
  }

// ============================ READ BMS DATA ============================
  if (bms_read_ok)
  { 
    if(Serial3.available()>0){
//    Serial.print("IMOKAY@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
      byte buffer2 = Serial3.read();
//    Serial.print("buffer2");
//    Serial.println(buffer2,HEX);
      buffer1[buffer_count] = buffer2;
    
      if((buffer_count!=0)&&buffer1[buffer_count-1]==0xAF && buffer1[buffer_count]==0xA0) {
        ReceivePacketBMS(buffer1,buffer_count);
        publish_bms();
        buffer_count = 0 ; 
        bms_read_ok = false ;
      }
      else 
        buffer_count++;
    }
    else
    {
//    Serial.println("NOT!!!!!!!!!!!!!!!!!!!!!!!!!");
      if(buffer_count!=0){ 
        buffer_count = 0 ; 
      }
      bms_read_ok = false ;
    }
  }

// ============================ CHARGING ============================
  if (millis() - last_chg_det_updated > CHARGING_PUBLISH_RATE)
  {
//    chg_current_adc = analogRead(CHG_DET);
    //chg_current_adc = roundf(current*100.0)/100.0;
    if (abs(current) <= 20) {
      if(current > BAT_CHG_THRES) {
        if(!is_chg_mode){
          if(ros_connect_status) 
          {
            is_chg_mode = true;
            if(ros_connect_status) nh.loginfo("MCU : BAT CHARGING");
              publishIsCharging();
          }
          buzzer_en = false;
        }
        is_chg_mode = true;
      }
      else 
      {
        if(is_chg_mode){
          if(ros_connect_status)
          {
            is_chg_mode = false;
            if(ros_connect_status) nh.loginfo("MCU : BAT NOT CHARGING");
              publishIsCharging();
          }
        }
        is_chg_mode = false;
      }
      publishIsCharging();
      publishCurrent_ADC(chg_current_adc);
    }
    last_chg_det_updated = millis();
  } 

// ============================ PC POWER STATUS ============================
  bool pc_pwr_live = digitalRead(PC_PWR_LIVE);
  unsigned long currentMillis = millis();

  if (currentMillis - previous_pc_liveMillis >= 1000) {  // 1초(1000ms) 경과 시
    previous_pc_liveMillis = currentMillis;  // 이전 시간 갱신

    if (pc_pwr_live) {
      //Serial.print("IMOKAY@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
      digitalWrite(PWR_HOLD, HIGH);
      if (!power_off_started) {
        digitalWrite(PWR_BTN_LED, HIGH);
      }
    } else {
      //Serial.print("IMOKAY##############################################");
      digitalWrite(PWR_HOLD, LOW);
      digitalWrite(PWR_BTN_LED, LOW);
    }
  }
  nh.spinOnce();
  delay(5);
}

void bms_reset()
{
  digitalWrite(BMS_SW,HIGH);
  digitalWrite(BMS_SW,LOW);
  delay(500);
  digitalWrite(BMS_SW,HIGH);
  delay(500);
}

bool check_pc_pwr_live()
{
  bool pc_pwr_live_st;
  bool pc_pwr_live;
  pc_pwr_live = digitalRead(PC_PWR_LIVE);

  if(pc_pwr_live) {
    pc_pwr_live_st = true;
  } 
  else 
  {
    pc_pwr_live_st = false;
    Serial.println("PC NO!!!");
  }

  if (!pc_pwr_live_st) ext_emerg_en = false; 
  return pc_pwr_live_st;
}

void live_led_status()
{
   live_led_status_flag =!live_led_status_flag;
   digitalWrite(LIVE_LED, live_led_status_flag);
}

void check_pwr_button_status()
{
  bool pwr_button = digitalRead(PWR_BTN);
  if(!ros_connect_status){
    // Serial.print("pwr_button status:");
    // Serial.println(pwr_button);
  }
  if(pwr_button) {
    if(first_push) {
      last_power_push_time = millis();
      first_push = false;
    }

    if(millis()-last_power_push_time > 2000) //3sec-->  1500--100
    { 
      #if 0
      Serial.print(millis()-last_power_push_time);
      Serial.print("  ");
      // Serial.println("IS_POWEROFF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      // Serial.print(is_poweroff);  
      Serial.println("  ");
      #endif

      bool is_poweroff_pwrkey = !check_pc_pwr_live();

      if(!is_poweroff_pwrkey && !button_release_wait)
      {
        if(!poweroff_pub) // NUC power-on mode + push pwrkey status
        {
          poweroff_pub = true; //power on -> off
          publishPowerBTN(is_poweroff);
          if(ros_connect_status) nh.loginfo("MCU : SYSTEM POWER OFF !!!!");
          else Serial.println("SYSTEM POWER OFF !!!!");
          button_release_wait = true;
          power_off_started = true;
          digitalWrite(PWR_BTN_LED, LOW);
        }
      }

      if(is_poweroff_pwrkey && !button_release_wait) //NUC power-off mode + push pwrkey status
      {
        digitalWrite(PC_PWR_SW,HIGH); // button push 
        delay(500);
        digitalWrite(PC_PWR_SW,LOW);
        first_push = true;
        button_release_wait = true; 
      }
    }

    if(millis()-last_power_push_time > 7000 && !force_pwroff_release_wait)
    {  
      if(!is_poweroff_pwrkey)
      {
        if(ros_connect_status) nh.loginfo("MCU : FORCED POWER OFF !!!!");
        digitalWrite(PC_PWR_SW,HIGH); // button push
      }
      else
      {
        digitalWrite(PC_PWR_SW,HIGH);
        force_pwroff_release_wait = true;
      }
    }
  }// not push pwr key
  else{
    digitalWrite(PC_PWR_SW,LOW);
    first_push = true;
    poweroff_pub = false;
    button_release_wait = false;
    force_pwroff_release_wait = false;
  }
  //is_poweroff_old = is_poweroff;
}

void check_emerg_button_status()
{
  bool emerg_button1 = digitalRead(Emergency);

  bool emerg_button = !emerg_button1 || ext_emerg_en ;

  if(!ros_connect_status){
  // Serial.print("  emerg_button:");
  // Serial.println(emerg_button);
  }
  if(emerg_button) {
    is_emerg_mode = true;
    publishEmergBTN();
    if((emerg_button != emerg_button_old)||emerg_button_first) 
    {
      if (!power_off_started) {
        digitalWrite(BUZZER,HIGH);
      }
      if(ros_connect_status) nh.loginfo("MCU : EMERGENCY !!!!");
    }
  } else {
    is_emerg_mode = false;
      
    if((emerg_button != emerg_button_old)||emerg_button_first) { 
      publishEmergBTN();
      digitalWrite(BUZZER,LOW);
      led_function(LED_OFF);
      led2_function(LED_OFF);
      if(ros_connect_status) nh.loginfo("MCU : NO EMERGENCY !!!!");
    }
  }

  emerg_button_first = false;
  emerg_button_old = emerg_button; 
}

void check_mission_button_status()
{
  bool mission_stop_button  = digitalRead(STOP_BTN);

  if (mission_stop_button)
    Serial.println("Mission Stop button");

  publishMissionStopBTN(mission_stop_button);
}
