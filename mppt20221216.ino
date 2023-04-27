
//====================== ARDUINO LIBRARIES (ESP32 Compatible Libraries) ============================//
// You will have to download and install the following libraries below in order to program the MPPT //
// unit. Visit TechBuilder's YouTube channel for the "MPPT" tutorial.                               //
//============================================================================================= ====//
#include <Arduino.h>
#include <U8g2lib.h>
#include <EEPROM.h>  //SYSTEM PARAMETER  - EEPROM Library (By: Arduino)
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <WiFi.h>  //SYSTEM PARAMETER  - WiFi Library (By: Arduino)
#include <DNSServer.h>
DNSServer dnsserver;
#include <ESPAsyncWebServer.h>
AsyncWebServer server(80);
//#include <WiFiClient.h>             //SYSTEM PARAMETER  - WiFi Library (By: Arduino)
#include <Adafruit_ADS1X15.h>  //SYSTEM PARAMETER  - ADS1115/ADS1015 ADC Library (By: Adafruit)
#include <PubSubClient.h>
#include <ArduinoOTA.h>
//以下为1306程序
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/22, /* data=*/21, /* reset=*/U8X8_PIN_NONE);  // ESP32 Thing, pure SW emulated I2C
TaskHandle_t Core2;                                                                                        //SYSTEM PARAMETER  - Used for the ESP32 dual core operation
Adafruit_ADS1015 ads;
#include <time.h>
#include <ArduinoHA.h>
#define timezone 8
#define ADDR 0b0100011    // 0x23


//====================================== 位图数组 ===================================================//
//下面为电压电流等图标//
//==================================================================================================//
#define tbv_x 16
#define tbv_y 16
static const unsigned char tbv[] U8X8_PROGMEM = { 0xE0, 0x07, 0xF0, 0x0F, 0xF8, 0x1F, 0xFC, 0x3F, 0xCE, 0x73, 0xCF, 0xF3, 0xCF, 0xF3, 0xCF, 0xF3, 0xCF, 0xF3, 0xCF, 0xF3, 0x9F, 0xF9, 0x3E, 0x7C, 0x7C, 0x3E, 0xF8, 0x1F, 0xF0, 0x0F, 0xE0, 0x07 };  //"V"
#define tbw_x 16
#define tbw_y 16
static const unsigned char tbw[] U8X8_PROGMEM = { 0xE0, 0x07, 0xF0, 0x0F, 0xF8, 0x1F, 0xFC, 0x3F, 0xE6, 0x67, 0xE7, 0xE7, 0x67, 0xE6, 0x67, 0xE6, 0x67, 0xE6, 0x67, 0xE6, 0x27, 0xE4, 0x86, 0x61, 0xEC, 0x37, 0xF8, 0x1F, 0xF0, 0x0F, 0xE0, 0x07 };  //"W"
#define tbi_x 16
#define tbi_y 16
static const unsigned char tbi[] U8X8_PROGMEM = { 0xE0, 0x07, 0xF0, 0x0F, 0x18, 0x18, 0x1C, 0x38, 0x7E, 0x7E, 0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x7E, 0x7E, 0x1C, 0x38, 0x18, 0x18, 0xF0, 0x0F, 0xE0, 0x07 };  //  "I"
#define tbd_x 16
#define tbd_y 16
static const unsigned char tbd[] U8X8_PROGMEM = { 0xE0, 0x07, 0xF0, 0x0F, 0xF8, 0x1F, 0x1C, 0x3E, 0x9E, 0x7D, 0x9F, 0xFB, 0x9F, 0xF7, 0x9F, 0xF7, 0x9F, 0xF7, 0x9F, 0xF7, 0x9F, 0xFB, 0x9E, 0x7D, 0x1C, 0x3E, 0xF8, 0x1F, 0xF0, 0x0F, 0xE0, 0x07 };  //  "D"
#define tbwd_x 16
#define tbwd_y 16
static const unsigned char tbwd[] U8X8_PROGMEM = { 0x80, 0x01, 0x60, 0x76, 0x20, 0x04, 0xA0, 0x75, 0xA0, 0x05, 0xA0, 0x75, 0xA0, 0x05, 0xA0, 0x05, 0xA0, 0x05, 0xA0, 0x05, 0x90, 0x09, 0xC8, 0x13, 0xC8, 0x13, 0x88, 0x11, 0x10, 0x08, 0xE0, 0x07 };  //  "WD"
#define tbdc_x 16
#define tbdc_y 16
static const unsigned char tbdc[] U8X8_PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0xFE, 0x3F, 0xC2, 0x20, 0xE2, 0xE0, 0xF2, 0xE7, 0x82, 0xE3, 0x82, 0xE1, 0x82, 0x20, 0xFE, 0x3F, 0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };  //  "dc"
#define tbdca_x 16
#define tbdca_y 16
static const unsigned char tbdca[] U8X8_PROGMEM = { 0x00, 0x00, 0x80, 0x00, 0xC0, 0x01, 0xE0, 0x03, 0x36, 0x36, 0x1A, 0x2C, 0x1A, 0xEC, 0xFA, 0xEF, 0xFA, 0xEF, 0x1A, 0xEC, 0x1A, 0x2C, 0xDE, 0x3D, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //  "dca"
#define tbdcrl_x 16
#define tbdcrl_y 16
static const unsigned char tbdcrl[] U8X8_PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x3F, 0xBA, 0x23, 0xBA, 0xE3, 0xBA, 0xE3, 0xBA, 0xE3, 0xBA, 0xE3, 0xBA, 0x23, 0xFE, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //  "dcrl"
#define tbdcw_x 16
#define tbdcw_y 16
static const unsigned char tbdcw[] U8X8_PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x3F, 0x12, 0x24, 0x92, 0xE4, 0x92, 0xE4, 0x92, 0xE4, 0x62, 0xE3, 0x02, 0x20, 0xFE, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //  "dcW"
#define tbdcv_x 16
#define tbdcv_y 16
static const unsigned char tbdcv[] U8X8_PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x3F, 0x22, 0x22, 0x22, 0xE2, 0x22, 0xE2, 0x22, 0xE2, 0xC2, 0xE1, 0x82, 0x20, 0xFE, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //  "dcV"
#define tbxl_x 16
#define tbxl_y 16
static const unsigned char tbxl[] U8X8_PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x38, 0x04, 0x30, 0x84, 0x28, 0x44, 0x05, 0x24, 0x02, 0x14, 0x00, 0x04, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0x00, 0x00 };  //  "xl"

//====================================== USER PARAMETERS ===========================================//
//下面的参数是没有MPPT充电器设置时使用的默认参数 //
//通过 LCD 菜单界面或手机 WiFi 应用程序设置或保存。这里的一些参数//
//将允许您覆盖或解锁高级用户的功能（不在 LCD 菜单上的设置）//
//==================================================================================================//
#define backflow_MOSFET 27  //SYSTEM PARAMETER - Backflow MOSFET
#define Out_MOSFET      26       //SYSTEM PARAMETER - 负载输出 MOSFET
#define buck_IN         33          //SYSTEM PARAMETER - Buck MOSFET Driver PWM Pin
#define buck_EN         32          //SYSTEM PARAMETER - Buck MOSFET Driver Enable Pin
#define LED             2               //SYSTEM PARAMETER - LED Indicator GPIO Pin
#define FAN             16              //SYSTEM PARAMETER - Fan GPIO Pin
#define ADC_ALERT       34        //SYSTEM PARAMETER - Fan GPIO Pin
#define TempSensor      35       //SYSTEM PARAMETER - Temperature Sensor GPIO Pin
#define buttonLeft      17       //SYSTEM PARAMETER -
#define buttonRight     18      //SYSTEM PARAMETER -
#define buttonBack      19       //SYSTEM PARAMETER -
#define buttonSelect    23     //SYSTEM PARAMETER -

#define eeprom_size 4096

#define MPPT_Mode_add             2448           // charging mode setting
#define voltageBatteryMax_add     2449   // Max Battery Voltage (whole)
#define voltageBatteryMax2_add    2450  // Max Battery Voltage (decimal)
#define voltageBatteryMin_add     2451   // Min Battery Voltage (whole)
#define voltageBatteryMin2_add    2452  // Min Battery Voltage (decimal)
#define currentCharging_add       2453     // Charging Current (whole)
#define currentCharging2_add      2454    // Charging Current (decimal)
#define enableFan_add             2455           // Fan Enable (Bool)
#define temperatureFan_add        2456      // Fan Temp (Integer)
#define temperatureMax_add        2457      // Shutdown Temp (Integer)
#define enableWiFi_add            2458          // Enable WiFi (Boolean)
#define flashMemLoad_add          2459        // Enable autoload (on by default)
#define output_Mode_add           2460         // Charger/PSU Mode Selection (1 = Charger Mode)
#define backlightSleepMode_add    2461  // 液晶背光睡眠定时器(默认值:0 =没有)
#define enableMos_add             2462           // OUTMOS Enable (Bool)新增

//========================================= 以下为MQTT、dns、AP模式程序==============================================//

//=====================================================================================================//

int count = 0;            //时间计数
bool WIFI_Status = true;  //WIFI状态标志位
long lastMsg = 0;         // 记录上一次发送信息的时长

// Xiaomi_1303
const char* ssid = "hailuohai";
const char* password = "12345678";
const char* mqtt_server = "175.178.105.64";
const char* mqtt_user = "admin";
const char* mqtt_pass = "test112211";
const char* manufacturer = "ArduinoHA.h";
const char* device_name = "MPPT";

// mqtt home assistant 初始化
WiFiClient client;
HADevice device(device_name);
HAMqtt mqtt(client, device);
unsigned long lastUpdateAt = 0;

// "myAnalogInput" is unique ID of the sensor. You should define your own ID.
HABinarySensor sensor_buckEnable("sensor_buckEnable");
HASensorNumber analogSensor_powerInput("analogSensor_powerInput");
HASensorNumber analogSensor_powerOutput("analogSensor_powerOutput");
HASensorNumber analogSensor_PWM("analogSensor_PWM");
HASensorNumber analogSensor_voltageInput("analogSensor_voltageInput");
HASensorNumber analogSensor_voltageOutput("analogSensor_voltageOutput");
HASensorNumber analogSensor_currentInput("analogSensor_currentInput");
HASensorNumber analogSensor_currentOutput("analogSensor_currentOutput");


//====================================== USER PARAMETERS ==========================================//
//下面的参数是没有MPPT充电器设置时使用的默认参数 //
//通过 LCD 菜单界面或手机 WiFi 应用程序设置或保存。这里的一些参数//
//将允许您覆盖或解锁高级用户的功能（不在 LCD 菜单上的设置）//
//=================================================================================================//
bool
  MPPT_Mode              = 1,             //   USER PARAMETER - 启用 MPPT 算法，当禁用充电器时使用 CC-CV 算法
  output_Mode            = 1,           //   USER PARAMETER - 0 = PSU 模式, 1 = 充电器模式
  disableFlashAutoLoad   = 0,  //   USER PARAMETER - 强制 MPPT 不使用闪存保存的设置，启用此“1”默认为已编程的固件设置
  enablePPWM             = 1,            //   USER PARAMETER - 启用预测 PWM，这加快了调节速度（仅适用于电池充电应用）
  enableWiFi             = 1,            //   USER PARAMETER - 启用 WiFi 连接
  enableFan              = 1,             //   USER PARAMETER - 启用冷却风扇
  enableBluetooth        = 1,       //   USER PARAMETER - 启用蓝牙连
  enableLCD              = 1,             //   USER PARAMETER - 启用 接LCD 显示
  enableLCDBacklight     = 1,    //   USER PARAMETER - 启用 LCD 显示器的背光
  overrideFan            = 0,           //   USER PARAMETER - 风扇始终开启
  enableDynamicCooling   = 0;  //   USER PARAMETER - 启用 PWM 冷却控制
int
  serialTelemMode        = 2,            //  USER PARAMETER - 选择串行遥测数据馈送（0 - 禁用串行，1 - 显示所有数据，2 - 显示基本，3 - 仅数字）
  enableMos              = 0,                  //  USER PARAMETER - 启用输出MOS模式(0 -关闭， 1 -开启， 2 -自动)
  pwmResolution          = 11,             //  USER PARAMETER - PWM 位分辨率
  pwmFrequency           = 39000,           //  USER PARAMETER - PWM 开关频率 - Hz（用于降压）
  temperatureFan         = 60,            //  USER PARAMETER - 风扇开启的温度阈值
  temperatureMax         = 90,            //  USER PARAMETER - 过热，超过时系统关闭（摄氏度）
  telemCounterReset      = 0,          //  USER PARAMETER - 每隔一次重置 Telem 数据（0 = 从不，1 = 日，2 = 周，3 = 月，4 = 年）
  errorTimeLimit         = 1000,          //  USER PARAMETER - 重置错误计数器的时间间隔（毫秒）
  errorCountLimit        = 5,            //  USER PARAMETER - 最大错误数
  millisRoutineInterval  = 250,    //  USER PARAMETER - 例程函数的时间间隔刷新率 (ms)
  millisSerialInterval   = 1,       //  USER PARAMETER - USB 串行数据馈送的时间间隔刷新率 (ms)
  millisLCDInterval      = 1000,       //  USER PARAMETER - LCD 显示器的时间间隔刷新率 (ms)
  millisWiFiInterval     = 2000,      //  USER PARAMETER - WiFi 遥测的时间间隔刷新率 (ms)
  millisLCDBackLInterval = 2000,  //  USER PARAMETER - 用户参数 - WiFi 遥测的时间间隔刷新率 (ms)
  backlightSleepMode     = 0,         //  USER PARAMETER - - 0 = 从不, 1 = 10 秒, 2 = 5 分钟, 3 = 1 小时, 4 = 6 小时, 5 = 12 小时, 6 = 1 天, 7 = 3 天, 8 = 1 周, 9 = 1个月
  baudRate               = 500000;              //  用户参数 - USB 串行波特率 (bps)

float
  voltageBatteryMax      = 12.6000,  //   USER PARAMETER - 最大电池充电电压（输出 V）
  voltageBatteryMin      = 9.6000,   //   USER PARAMETER - 最小电池充电电压（输出 V）
  currentCharging        = 30.0000,    //   USER PARAMETER - 最大充电电流（A - 输出）
  currentChargingPrer    = 0.0000,
  electricalPrice        = 0.6000;  //   USER PARAMETER - 每千瓦时的输入电价

//================================== CALIBRATION PARAMETERS =======================================//
//可以调整以下参数以设计您自己的 MPPT 充电控制器。只修改 //
//如果你知道你在做什么，下面的值。以下值已针对 // 进行了预校准
// TechBuilder (Angelo S. Casimiro) 设计的 MPPT 充电控制器 //                        //
//=================================================================================================//
const bool
  ADS1015_Mode          = 1;  //  CALIB PARAMETER - Use 1 for ADS1015 ADC model use 0 for ADS1115 ADC model
const int
  ADC_GainSelect        = 2,  //  校准参数 - ADC 增益选择 (0→±6.144V 3mV/bit, 1→±4.096V 2mV/bit, 2→±2.048V 1mV/bit)
  avgCountVS            = 3,      //  校准参数 - 电压传感器平均采样计数（推荐：3）
  avgCountCS            = 4,      //  校准参数 - 电流传感器平均采样计数（推荐：4）
  avgCountTS            = 500;    //  校准参数 - 温度传感器平均采样计数
float
  inVoltageDivRatio     = 40.2156,    //  校准参数 - 输入分压器传感器比率（更改此值以校准电压传感器）
  outVoltageDivRatio    = 24.5000,   //  校准参数 - 输出分压器传感器比率（更改此值以校准电压传感器）
  vOutSystemMax         = 50.0000,        //  校准参数 - 最大输入电压
  cOutSystemMax         = 50.0000,        //  校准参数 - 最大输出电压
  ntcResistance         = 10000.00,       //  校准参数 - NTC 温度传感器的电阻。如果您使用 10k NTC，请更改为 10000.00
  voltageDropout        = 1.0000,        //  校准参数 - 降压稳压器的压降电压（由于最大占空比限制而存在 DOV）
  voltageBatteryThresh  = 1.5000,  //  校准参数 - 达到此电压时断电（输出 V）
  currentInAbsolute     = 31.0000,    //  校准参数 - 系统可以处理的最大输入电流（A - 输入）
  currentOutAbsolute    = 50.0000,   //  校准参数 - 系统可以处理的最大输出电流（A - 输入）
  PPWM_margin           = 99.5000,          //  校准参数 - 预测 PWM 的最小工作占空比 (%)
  PWM_MaxDC             = 97.0000,            //  校准参数 - 最大工作占空比 (%) 90%-97% 是好的
  efficiencyRate        = 1.0000,        //  校准参数 - 理论降压效率（十进制百分比）
  currentMidPoint       = 2.5250,       //  校准参数 - 电流传感器中点 (V) 2.5250
  currentSens           = 0.0000,           //  校准参数 - 电流传感器灵敏度 (V/A)
  currentSensV          = 0.0660,          //  校准参数 - 电流传感器灵敏度 (mV/A)  0.0330
  vInSystemMin          = 10.000;          //  校准参数 - 系统识别最低电压

//===================================== SYSTEM PARAMETERS =========================================//
//不要更改本节中的参数值。下面的值是系统使用的变量 //
//进程。更改值可能会损坏 MPPT 硬件。请保持原样！然而， //
//您可以访问这些变量来获取您的模组所需的数据。//
//=================================================================================================//
bool
  buckEnable            = 1,           // SYSTEM PARAMETER - Buck Enable Status
  enableMosen           = 0,          //  USER PARAMETER - 启用输出MOS状态(0 -关闭， 1 -开启)
  fanStatus             = 0,            // SYSTEM PARAMETER - Fan activity status (1 = On, 0 = Off)
  bypassEnable          = 0,         // SYSTEM PARAMETER -
  chargingPause         = 0,        // SYSTEM PARAMETER -
  lowPowerMode          = 0,         // SYSTEM PARAMETER -
  buttonRightStatus     = 0,    // SYSTEM PARAMETER -
  buttonLeftStatus      = 0,     // SYSTEM PARAMETER -
  buttonBackStatus      = 0,     // SYSTEM PARAMETER -
  buttonSelectStatus    = 0,   // SYSTEM PARAMETER -
  buttonRightCommand    = 0,   // SYSTEM PARAMETER -
  buttonLeftCommand     = 0,    // SYSTEM PARAMETER -
  buttonBackCommand     = 0,    // SYSTEM PARAMETER -
  buttonSelectCommand   = 0,  // SYSTEM PARAMETER -
  settingMode           = 0,          // SYSTEM PARAMETER -
  setMenuPage           = 0,          // SYSTEM PARAMETER -
  boolTemp              = 0,             // SYSTEM PARAMETER -
  flashMemLoad          = 0,         // SYSTEM PARAMETER -
  confirmationMenu      = 0,     // SYSTEM PARAMETER -
  WIFI                  = 0,                 // SYSTEM PARAMETER -
  BNC                   = 0,                  // SYSTEM PARAMETER -
  REC                   = 0,                  // SYSTEM PARAMETER -
  FLV                   = 0,                  // SYSTEM PARAMETER -
  IUV                   = 0,                  // SYSTEM PARAMETER -
  IOV                   = 0,                  // SYSTEM PARAMETER -
  IOC                   = 0,                  // SYSTEM PARAMETER -
  OUV                   = 0,                  // SYSTEM PARAMETER -
  OOV                   = 0,                  // SYSTEM PARAMETER -
  OOC                   = 0,                  // SYSTEM PARAMETER -
  OTE                   = 0;                  // SYSTEM PARAMETER -
int
  inputSource           = 0,     // SYSTEM PARAMETER - 0 = MPPT 没有电源，1 = MPPT 使用太阳能作为电源，2 = MPPT 使用电池作为电源
  avgStoreTS            = 0,      // SYSTEM PARAMETER - 温度传感器使用非侵入式平均，这是用于平均平均的累加器
  temperature           = 0,     // SYSTEM PARAMETER -
  sampleStoreTS         = 0,   // SYSTEM PARAMETER - TS AVG 第 n 个样本
  pwmMax                = 0,          // SYSTEM PARAMETER -
  pwmMaxLimited         = 0,   // SYSTEM PARAMETER -
  PWM                   = 0,             // SYSTEM PARAMETER -
  PPWM                  = 0,            // SYSTEM PARAMETER -
  pwmChannel            = 0,      // SYSTEM PARAMETER -
  batteryPercent        = 0,  // SYSTEM PARAMETER -
  buckEfficiency        = 0,  // SYSTEM PARAMETER - 测量降压转换器功率转换效率（仅适用于我的双电流传感器版本）
  errorCount            = 0,      // SYSTEM PARAMETER -
  menuPage              = 0,        // SYSTEM PARAMETER -
  subMenuPage           = 0,     // SYSTEM PARAMETER -
  ERR                   = 0,             // SYSTEM PARAMETER -
  conv1                 = 0,           // SYSTEM PARAMETER -
  conv2                 = 0,           // SYSTEM PARAMETER -
  intTemp               = 0;         // SYSTEM PARAMETER -
float
  VSI                   = 0.0000,                // SYSTEM PARAMETER - 原始输入电压传感器 ADC 电压
  VSO                   = 0.0000,                // SYSTEM PARAMETER - 原始输出电压传感器 ADC 电压
  CSI                   = 0.0000,                // SYSTEM PARAMETER - 原始电流传感器 ADC 电压A2
  CSIL                  = 0.0000,               // SYSTEM PARAMETER - 原始电流传感器 ADC 电压A0///////
  CSO                   = 0.0000,                // SYSTEM PARAMETER - Raw current sensor ADC voltage
  CSI_converted         = 0.0000,      // SYSTEM PARAMETER - 实际电流传感器 ADC 电压A2
  CSIL_converted        = 0.0000,     // SYSTEM PARAMETER - 实际电流传感器 ADC 电压A0///////
  CSO_converted         = 0.0000,      // SYSTEM PARAMETER - Actual current sensor ADC voltage
  TS                    = 0.0000,                 // SYSTEM PARAMETER - 原始温度传感器 ADC 值
  powerInput            = 0.0000,         // SYSTEM PARAMETER - 输入功率（太阳能）以瓦特为单位
  powerInputPrev        = 0.0000,     // SYSTEM PARAMETER - 先前存储的 MPPT 算法的输入功率变量（瓦特）
  powerOutput           = 0.0000,        // SYSTEM PARAMETER - 输出功率（电池或充电功率，以瓦特为单位）
  energySavings         = 0.0000,      // SYSTEM PARAMETER - 法定货币（比索、美元、欧元等）的能源节约
  voltageInput          = 0.0000,       // SYSTEM PARAMETER - 太阳能电压
  voltageInputPrev      = 0.0000,   // SYSTEM PARAMETER - 先前存储的 MPPT 算法的输入电压变量
  voltageOutput         = 0.0000,      // SYSTEM PARAMETER - 电池电压
  currentInput          = 0.0000,       // SYSTEM PARAMETER - 输入电流（光伏板输入电流，以安培为单位）
  currentOutput         = 0.0000,      // SYSTEM PARAMETER - 输出电流（电池或充电电流，以安培为单位）
  loadcurrentOutput     = 0.0000,  // 负载输出电流
  TSlog                 = 0.0000,              // SYSTEM PARAMETER -  NTC 热敏电阻热感应代码的一部分
  ADC_BitReso           = 0.0000,        // SYSTEM PARAMETER - 系统检测 ADS1015/ADS1115 ADC 的适当位分辨率因子
  daysRunning           = 0.0000,        // SYSTEM PARAMETER - 存储 MPPT 设备自上次通电以来运行的总天数
  Wh                    = 0.0000,                 // SYSTEM PARAMETER - 存储收集到的累积能量（瓦特小时）
  kWh                   = 0.0000,                // SYSTEM PARAMETER - 存储收集到的累积能量（千瓦时）
  MWh                   = 0.0000,                // SYSTEM PARAMETER - 存储收集到的累积能量（兆瓦时）
  loopTime              = 0.0000,           // SYSTEM PARAMETER -
  outputDeviation       = 0.0000,    // SYSTEM PARAMETER - 输出电压偏差 (%)
  floatTemp             = 0.0000,
  vOutSystemMin         = 0.0000;  //  CALIB PARAMETER -

unsigned long
  currentErrorMillis    = 0,     //SYSTEM PARAMETER -
  currentButtonMillis   = 0,    //SYSTEM PARAMETER -
  currentSerialMillis   = 0,    //SYSTEM PARAMETER -
  currentRoutineMillis  = 0,   //SYSTEM PARAMETER -
  currentLCDMillis      = 0,       //SYSTEM PARAMETER -
  currentLCDBackLMillis = 0,  //SYSTEM PARAMETER -
  currentWiFiMillis     = 0,      //SYSTEM PARAMETER -
  currentMenuSetMillis  = 0,   //SYSTEM PARAMETER -
  prevButtonMillis      = 0,       //SYSTEM PARAMETER -
  prevSerialMillis      = 0,       //SYSTEM PARAMETER -
  prevRoutineMillis     = 0,      //SYSTEM PARAMETER -
  prevErrorMillis       = 0,        //SYSTEM PARAMETER -
  prevWiFiMillis        = 0,         //SYSTEM PARAMETER -
  prevLCDMillis         = 0,          //SYSTEM PARAMETER -
  prevLCDBackLMillis    = 0,     //SYSTEM PARAMETER -
  timeOn                = 0,                 //SYSTEM PARAMETER -
  loopTimeStart         = 0,          //SYSTEM PARAMETER - 用于循环循环秒表，记录循环开始时间
  loopTimeEnd           = 0,            //SYSTEM PARAMETER - 用于循环循环秒表，记录循环结束时间
  secondsElapsed        = 0;         //SYSTEM PARAMETER -

 String MPPT_Data(void) {
  const char* fanStatus1 = "";
  const char* enableMos1 = "";
  if (fanStatus == 1) {
    fanStatus1 = "开";
  } else {
    fanStatus1 = "关";
  }

  if (enableMos == 1) {
    enableMos1 = "开";
  } else if (enableMos == 0) {
    enableMos1 = "关";
  } else if (enableMos == 2) {
    enableMos1 = "自动";
  }

  // 数据打包为一个HTML显示代码
  String dataBuffer = "<p>";
  dataBuffer += "<h2>运行数据</h2>";
  dataBuffer += "<b>输入电压: </b>";
  dataBuffer += String(voltageInput);
  dataBuffer += "<b>V</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>输入电流: </b>";
  dataBuffer += String(currentInput);
  dataBuffer += "<b>A</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>电池电压: </b>";
  dataBuffer += String(voltageOutput);
  dataBuffer += "<b>V</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>电池容量: </b>";
  dataBuffer += String(batteryPercent);
  dataBuffer += "<b>%</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>充电电流: </b>";
  dataBuffer += String(currentOutput);
  dataBuffer += "<b>A</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>负载输出电流: </b>";
  dataBuffer += String(loadcurrentOutput);
  dataBuffer += "<b>A</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>BUCK效率: </b>";
  dataBuffer += String(buckEfficiency);
  dataBuffer += "<b>%</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>控制器温度: </b>";
  dataBuffer += String(temperature);
  dataBuffer += "<b>℃</b>";
  dataBuffer += "<br/>";
  dataBuffer += "<b>风扇状态: </b>";
  dataBuffer += String(fanStatus1);
  dataBuffer += "<br/>";
  dataBuffer += "<b>负载输出状态: </b>";
  dataBuffer += String(enableMos1);
  dataBuffer += "<br /></p>";
  return dataBuffer;  // 最后要将数组返回出去
}

void Out_Mosfet_sw(AsyncWebServerRequest* request) {
  String state = request->getParam("outmos")->value();
  if (state == "on") {
    Out_MOSFET_Enable();
  } else if (state == "off") {
    Out_MOSFET_Disable();
  } else if (state == "auto") {
    Out_MOSFET_zt();
  }
  request->send(200, "text/plain", "OK");  // 发送接收成功标志符
  }

//==============================================以下为HTML代码========================================//
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
<meta charset="utf-8">
<title>ESP32-MPPT控制器管理系统V1.20</title>
</head>
<body>
	<h1>ESP32-MPPT控制器V1.20</h1>
	<!-- 创建一个ID位mppt的盒子用于显示获取到的数据 -->
	<div id="mppt"></div>
	<button onclick="sw()"> 打开 </button>
  <button onclick="sw1()"> 关闭 </button>
  <button onclick="sw2()"> 自动 </button>
  <h3>wifi 密码配置</h3>
  <div>
        <label for="name">wifi名称</label>
        <input type="text" id="wifi" name="car_name" placeholder="ssid">
        <br>
        <label for="type">密&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp码</label>
        <input type="text" id="code" name="car_type" placeholder="password">
        <br>
        <button id ="send_WIFI" type="button" onclick="send_wifi()">提交</button>
  </div>
</body>
<script>
function setpath() {
    var default_path = document.getElementById("newfile").files[0].name;
    document.getElementById("filepath").value = default_path;
}

function send_wifi() {
    var input_ssid = document.getElementById("wifi").value;
    var input_code = document.getElementById("code").value;
    var pw = new XMLHttpRequest();
        pw.open("POST", "/wifi_data", true);
        pw.onreadystatechange = function() {
            if (pw.readyState == 4) {
                if (pw.status == 200) {
                    console.log(pw.responseText);
                } else if (pw.status == 0) {
                    alert("Server closed the connection abruptly!");
                    location.reload()
                } else {
                    alert(pw.status + " Error!\n" + pw.responseText);
                    location.reload()
                }
            }
        };
    var data = {
        "wifi_name":input_ssid,
        "wifi_code":input_code
    }
        pw.send(JSON.stringify(data));
}

	// 按下按钮会运行这个JS函数
	function sw() {
    var payload = "on"; // 需要发送的内容
		// 通过get请求给 /sw
		var xhr = new XMLHttpRequest();
		xhr.open("GET", "/sw?outmos=" + payload, true);
		xhr.send();
	}
  	function sw1() {
    var payload = "off"; // 需要发送的内容
		// 通过get请求给 /sw
		var xhr = new XMLHttpRequest();
		xhr.open("GET", "/sw?outmos=" + payload, true);
		xhr.send();
	}
    	function sw2() {
    var payload = "auto"; // 需要发送的内容
		// 通过get请求给 /sw
		var xhr = new XMLHttpRequest();
		xhr.open("GET", "/sw?outmos=" + payload, true);
		xhr.send();
	}
	// 设置一个定时任务, 1000ms执行一次
	setInterval(function () {
		var xhttp = new XMLHttpRequest();
		xhttp.onreadystatechange = function () {
			if (this.readyState == 4 && this.status == 200) {
				// 此代码会搜索ID为mppt的组件，然后使用返回内容替换组件内容
				document.getElementById("mppt").innerHTML = this.responseText;
			}
		};
		// 使用GET的方式请求 /mppt
		xhttp.open("GET", "/mppt", true);
		xhttp.send();
	}, 1000)
</script>)rawliteral";


//===============================主程序============================================================//
//下面的代码包含所有翻译的系统进程MPPT固件。他们中的大多数被称为//
//从8 .ino选项卡。代码太长,Arduino标签在组织他们帮了我很多。//
//固件上运行的两个核心Arduino ESP32看到在两个独立的双空白//
//设置和循环。xTaskCreatePinnedToCore () freeRTOS函数允许您访问//
//未使用通过Arduino ESP32核心。是的它同时多核处理! //
//=================================================================================================//

//================= CORE0: SETUP (DUAL CORE MODE) =====================//
void coreTwo(void* pvParameters) {
    connect_wifi(); //TAB#7 - WiFi Initialization
while (1) {
    Wireless_Telemetry();                                   //TAB#7 - Wireless telemetry (WiFi & Bluetooth)
  }
}
//================== CORE1: SETUP (DUAL CORE MODE) ====================//
void setup() {
  //SERIAL INITIALIZATION
  Serial.begin(baudRate);  //Set serial baud rate
  //client.setServer(mqtt_server, mqtt_port);
  //GPIO PIN INITIALIZATION
  pinMode(backflow_MOSFET, OUTPUT);
  pinMode(Out_MOSFET, OUTPUT);
  pinMode(buck_EN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(TS, INPUT);
  pinMode(ADC_ALERT, INPUT);
  pinMode(buttonLeft, INPUT);
  pinMode(buttonRight, INPUT);
  pinMode(buttonBack, INPUT);
  pinMode(buttonSelect, INPUT);
  //PWM INITIALIZATION
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);  //Set PWM Parameters
  ledcAttachPin(buck_IN, pwmChannel);                  //Set pin as PWM
  ledcWrite(pwmChannel, PWM);                          //Write PWM value at startup (duty = 0)
  pwmMax = pow(2, pwmResolution) - 1;                  //Get PWM Max Bit Ceiling
  pwmMaxLimited = (PWM_MaxDC * pwmMax) / 100.000;      //Get maximum PWM Duty Cycle (pwm limiting protection)
  //ADC INITIALIZATION
  ADC_SetGain();  //Sets ADC Gain & Range
  ads.begin();    //Initialize ADC
  //GPIO INITIALIZATION
  buck_Disable();
  //ENABLE DUAL CORE MULTITASKING
  xTaskCreatePinnedToCore(coreTwo, "coreTwo", 10000, NULL, 0, &Core2, 0);
  //INITIALIZE AND LIOAD FLASH MEMORY DATA
  EEPROM.begin(eeprom_size);
  initializeFlashAutoload();  //Load stored settings from flash memory
  //以下为1306初始化程序
  if (enableLCD == 1) {
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_unifont_t_chinese2);  // 设置字体
    u8g2.setFontDirection(0);       //设置屏方向
    u8g2.clearBuffer();
    u8g2.setCursor(18, 16);
    u8g2.print("MPPT-controler");
    u8g2.setCursor(40, 38);
    u8g2.print("V1.20");
    u8g2.setCursor(26, 60);
    u8g2.print("thanks!");
    u8g2.sendBuffer();
    delay(2000);
  }
  //以下为进度条程序
  if (enableLCD == 1) {
    u8g2.firstPage();
    int i;
    for (i = 0; i < 25; i++) {
      do {
        u8g2.drawFrame(9, 25, 103, 15);
        u8g2.drawBox(12, 27, i * 4, 11);
        u8g2.setCursor(24, 15);
        u8g2.print("system booting");
      } while (u8g2.nextPage());
    }
  }

  mqtt_setup();
}
//================== CORE1: LOOP (DUAL CORE MODE) ======================//
void loop() {
  Read_Sensors();        //TAB#2 - Sensor data measurement and computation
  // Device_Protection();   //TAB#3 - Fault detection algorithm
  System_Processes();    //TAB#4 - Routine system processes
  Charging_Algorithm();  //TAB#5 - Battery Charging Algorithm
  Onboard_Telemetry();   //TAB#6 - Onboard telemetry (USB & Serial Telemetry)
  LCD_Menu();            //TAB#8 - Low Power Algorithm
  Out_Mosfet();          //TAB#9 - 输出MOS控制
  dnsserver.processNextRequest();
  mqtt_loop();
}
