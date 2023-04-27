
//====================== ARDUINO LIBRARIES (ESP32 Compatible Libraries) ============================//
// You will have to download and install the following libraries below in order to program the MPPT //
// unit. Visit TechBuilder's YouTube channel for the "MPPT" tutorial.                               //
//============================================================================================= ====//
#include <Arduino.h>
#include <U8g2lib.h>
#include <EEPROM.h>  //SYSTEM PARAMETER  - EEPROM Library (By: Arduino)
//#include <SPI.h>
#include <Wire.h>

#include <WiFi.h>  //SYSTEM PARAMETER  - WiFi Library (By: Arduino)

#include <Adafruit_ADS1X15.h>  //SYSTEM PARAMETER  - ADS1115/ADS1015 ADC Library (By: Adafruit)
Adafruit_ADS1015 ads;

#include <time.h>


#define timezone 8
#define ADDR 0b0100011    // 0x23

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


//===============================主程序============================================================//
//下面的代码包含所有翻译的系统进程MPPT固件。他们中的大多数被称为//
//从8 .ino选项卡。代码太长,Arduino标签在组织他们帮了我很多。//
//固件上运行的两个核心Arduino ESP32看到在两个独立的双空白//
//设置和循环。xTaskCreatePinnedToCore () freeRTOS函数允许您访问//
//未使用通过Arduino ESP32核心。是的它同时多核处理! //
//=================================================================================================//


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
  
  
  //ADC INITIALIZATION
  ADC_SetGain();  //Sets ADC Gain & Range
  ads.begin();    //Initialize ADC
  Serial.println("Initialize ADC");

}
//================== CORE1: LOOP (DUAL CORE MODE) ======================//
void loop() {
  Read_Sensors();        //TAB#2 - Sensor data measurement and computation

}
