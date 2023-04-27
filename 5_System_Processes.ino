void System_Processes(){
  ///////////////// FAN COOLING /////////////////
  if(enableFan==true){
    if(enableDynamicCooling==false){                                   //STATIC PWM COOLING MODE (2-PIN FAN - no need for hysteresis, temp data only refreshes after 'avgCountTS' or every 500 loop cycles)                       
      if(overrideFan==true){fanStatus=true;}                           //Force on fan
      else if(temperature>=temperatureFan){fanStatus=1;}               //Turn on fan when set fan temp reached
      else if(temperature<temperatureFan){fanStatus=0;}                //Turn off fan when set fan temp reached
      digitalWrite(FAN,fanStatus);                                     //Send a digital signal to the fan MOSFET
    }
    else{}                                                             //DYNAMIC PWM COOLING MODE (3-PIN FAN - coming soon)
  }
  else{digitalWrite(FAN,LOW);}                                         //Fan Disabled
  
  //////////// LOOP TIME STOPWATCH ////////////
  loopTimeStart = micros();                                            //Record Start Time
  loopTime = (loopTimeStart-loopTimeEnd)/1000.000;                     //Compute Loop Cycle Speed (mS)
  loopTimeEnd = micros();                                              //Record End Time

  ///////////// AUTO DATA RESET /////////////
  if(telemCounterReset==0){}                                           //Never Reset
  else if(telemCounterReset==1 && daysRunning>1)  {resetVariables();}  //Daily Reset
  else if(telemCounterReset==2 && daysRunning>7)  {resetVariables();}  //Weekly Reset
  else if(telemCounterReset==3 && daysRunning>30) {resetVariables();}  //Monthly Reset
  else if(telemCounterReset==4 && daysRunning>365){resetVariables();}  //Yearly Reset 

  ///////////// LOW POWER MODE /////////////
  if(lowPowerMode==1){}   
  else{}      
}
void factoryReset() {
  EEPROM.begin(eeprom_size);
  EEPROM.write(MPPT_Mode_add, 1); //STORE: Charging Algorithm (1 = MPPT Mode)
  EEPROM.write(voltageBatteryMax_add, 12); //STORE: Max Battery Voltage (whole)
  EEPROM.write(voltageBatteryMax2_add, 0); //STORE: Max Battery Voltage (decimal)
  EEPROM.write(voltageBatteryMin_add, 9); //STORE: Min Battery Voltage (whole)
  EEPROM.write(voltageBatteryMin2_add, 0); //STORE: Min Battery Voltage (decimal)
  EEPROM.write(currentCharging_add, 1); //STORE: Charging Current (whole)
  EEPROM.write(currentCharging2_add, 0); //STORE: Charging Current (decimal)
  EEPROM.write(enableFan_add, 1); //STORE: Fan Enable (Bool)
  EEPROM.write(enableMos_add, 1); //STORE: OUTMOS Enable (Bool)新增
  EEPROM.write(temperatureFan_add, 60); //STORE: Fan Temp (Integer)
  EEPROM.write(temperatureMax_add, 90); //STORE: Shutdown Temp (Integer)
  EEPROM.write(enableWiFi_add, 1); //STORE: Enable WiFi (Boolean)
  EEPROM.write(flashMemLoad_add, 1); //STORE: Enable autoload (on by default)
  EEPROM.write(output_Mode_add, 1); //STORE: Charger/PSU Mode Selection (1 = Charger Mode)
  EEPROM.write(backlightSleepMode_add, 0); //STORE: LCD backlight sleep timer (default: 0 = never)
  EEPROM.commit();
  loadSettings();
  EEPROM.end();
}

void loadSettings() {
  MPPT_Mode          = EEPROM.read(MPPT_Mode_add);                       // Load saved charging mode setting
  voltageBatteryMax  = EEPROM.read(voltageBatteryMax_add) + (EEPROM.read(voltageBatteryMax2_add) * .01); // Load saved maximum battery voltage setting
  voltageBatteryMin  = EEPROM.read(voltageBatteryMin_add) + (EEPROM.read(voltageBatteryMin2_add) * .01); // Load saved minimum battery voltage setting
  currentCharging    = EEPROM.read(currentCharging_add) + (EEPROM.read(currentCharging2_add) * .01); // Load saved charging current setting
  enableFan          = EEPROM.read(enableFan_add);                       // Load saved fan enable settings
  enableMos          = EEPROM.read(enableMos_add);                       // Load saved OUTMOS enable settings新增
  temperatureFan     = EEPROM.read(temperatureFan_add);                       // Load saved fan temperature settings
  temperatureMax     = EEPROM.read(temperatureMax_add);                       // Load saved shutdown temperature settings
  enableWiFi         = EEPROM.read(enableWiFi_add);                      // Load saved WiFi enable settings
  flashMemLoad       = EEPROM.read(flashMemLoad_add);                      // Load saved flash memory autoload feature
  output_Mode        = EEPROM.read(output_Mode_add);                      // Load saved charging mode setting
  backlightSleepMode = EEPROM.read(backlightSleepMode_add);                      // Load saved lcd backlight sleep timer
}

void saveSettings() {
  EEPROM.begin(eeprom_size);
  EEPROM.write(MPPT_Mode_add, MPPT_Mode);          //STORE: Algorithm
  conv1 = voltageBatteryMax * 100;     //STORE: Maximum Battery Voltage (gets whole number)
  conv2 = conv1 % 100;                 //STORE: Maximum Battery Voltage (gets decimal number and converts to a whole number)
  EEPROM.write(voltageBatteryMax_add, voltageBatteryMax);
  EEPROM.write(voltageBatteryMax2_add, conv2);
  conv1 = voltageBatteryMin * 100;     //STORE: Minimum Battery Voltage (gets whole number)
  conv2 = conv1 % 100;                 //STORE: Minimum Battery Voltage (gets decimal number and converts to a whole number)
  EEPROM.write(voltageBatteryMin_add, voltageBatteryMin);
  EEPROM.write(voltageBatteryMin2_add, conv2);
  conv1 = currentCharging * 100;       //STORE: Charging Current
  conv2 = conv1 % 100;
  EEPROM.write(currentCharging_add, currentCharging);
  EEPROM.write(currentCharging2_add, conv2);
  EEPROM.write(enableFan_add, enableFan);          //STORE: Fan Enable
  EEPROM.write(enableMos_add, enableMos);          //STORE: OUTMos Enable新增
  EEPROM.write(temperatureFan_add, temperatureFan);     //STORE: Fan Temp
  EEPROM.write(temperatureMax_add, temperatureMax);     //STORE: Shutdown Temp
  EEPROM.write(enableWiFi_add, enableWiFi);        //STORE: Enable WiFi
  //EEPROM.write(flashMemLoad_add, flashMemLoad);      //STORE: Enable autoload (must be excluded from bulk save, uncomment under discretion)
  EEPROM.write(output_Mode_add, output_Mode);       //STORE: Charge/PSU Mode Selection
  EEPROM.write(backlightSleepMode_add, backlightSleepMode); //STORE: LCD backlight sleep timer
  EEPROM.commit();                     //Saves setting changes to flash memory
  EEPROM.end();
  
}
void saveAutoloadSettings() {
  EEPROM.begin(eeprom_size);
  EEPROM.write(flashMemLoad_add, flashMemLoad);      //STORE: Enable autoload
  EEPROM.commit();                     //Saves setting changes to flash memory
  EEPROM.end();
  Serial.println("Saves setting changes");
}
void initializeFlashAutoload() {
  if (disableFlashAutoLoad == 0) {
    flashMemLoad = EEPROM.read(flashMemLoad_add);       //Load saved autoload (must be excluded from bulk save, uncomment under discretion)
    if (flashMemLoad == 1) {
      loadSettings(); //Load stored settings from flash memory
    }
    EEPROM.end();
  }
}
