void lcdBacklight_Wake() {
  prevLCDBackLMillis = millis();
  u8g2.setPowerSave(0);
}
void lcdBacklight() {
  unsigned long backLightInterval;
  if (backlightSleepMode == 0) { prevLCDBackLMillis = millis(); }  //Set backlight var to sleep never
  else if (backlightSleepMode == 1) {backLightInterval = 10000;}   //Set backlight var to sleep after 10 seconds without keypress
  else if (backlightSleepMode == 2) { backLightInterval = 300000; }//Set backlight var to sleep after 5 minutes without keypress
  else if (backlightSleepMode == 3) {backLightInterval = 3600000;} //Set backlight var to sleep after 1 hour without keypress
  else if (backlightSleepMode == 4) { backLightInterval = 21600000; }  //Set backlight var to sleep after 6 hours without keypress
  else if (backlightSleepMode == 5) {backLightInterval = 43200000;}   //Set backlight var to sleep after 12 hours without keypress
  else if (backlightSleepMode == 6) { backLightInterval = 86400000; }  //Set backlight var to sleep after 1 day without keypress
  else if (backlightSleepMode == 7) {backLightInterval = 259200000;}   //Set backlight var to sleep after 3 days without keypress
  else if (backlightSleepMode == 8) { backLightInterval = 604800000; }  //Set backlight var to sleep after 1 week without keypress
  else if (backlightSleepMode == 9) {backLightInterval = 2419200000;}  //Set backlight var to sleep after 1 month without keypress
  if (backlightSleepMode > 0 && settingMode == 0) {
    currentLCDBackLMillis = millis();
    if (currentLCDBackLMillis - prevLCDBackLMillis >= backLightInterval) {  //Run routine every millisRoutineInterval (ms)
      prevLCDBackLMillis = currentLCDBackLMillis;                           //Store previous time
      u8g2.setPowerSave(1);
    }
  }
}
//以下为菜单1显示
void displayConfig1() {
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setFontDirection(0);       //设置显示方向
  u8g2.clearBuffer();             //清除当前Buffer内的内容
  //u8g2.drawXBM(0,0,tbdc_x,tbdc_y,tbdc);//logo图标
  u8g2.drawXBM(80, 0, tbwd_x, tbwd_y, tbwd);        //温度图标
  u8g2.drawXBM(0, 19, tbv_x, tbv_y, tbv);           //输入电压图标
  u8g2.drawXBM(64, 19, tbi_x, tbi_y, tbi);          //输入电流图标
  u8g2.drawXBM(0, 35, tbdcv_x, tbdcv_y, tbdcv);     //输出电压图标
  u8g2.drawXBM(64, 35, tbdca_x, tbdca_y, tbdca);    //输出电流图标
  u8g2.drawXBM(0, 50, tbdcrl_x, tbdcrl_y, tbdcrl);  //电池容量图标
  u8g2.drawXBM(64, 49, tbxl_x, tbxl_y, tbxl);       //总电量图标
  //以下为画横线
  u8g2.drawLine(0, 16, 128, 16);
  u8g2.drawLine(0, 35, 128, 35);
  u8g2.drawLine(0, 51, 128, 51);
  //以下为画竖线
  u8g2.drawLine(80, 0, 80, 16);
  u8g2.drawLine(62, 16, 62, 64);
  u8g2.setCursor(8, 13);
  u8g2.print("Operating");  //标题
  u8g2.setCursor(96, 13);u8g2.print(temperature);u8g2.print("℃");//内部温度
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setCursor(16, 31);u8g2.print(voltageInput, 1);u8g2.print("V");//输入电压
  u8g2.setCursor(80, 31);u8g2.print(currentInput, 2);u8g2.print("A");//输入电流
  if (BNC == 0) {u8g2.setCursor(16, 47);u8g2.print(voltageOutput, 1);u8g2.print("V");
  } else {u8g2.setCursor(16, 47);u8g2.print("nrll");} //输出电压
  u8g2.setCursor(80, 47);u8g2.print(currentOutput, 2);u8g2.print("A");//输出电流
  u8g2.setCursor(16, 63);u8g2.print(batteryPercent);u8g2.print("%");//电池容量
  u8g2.setCursor(80, 63);u8g2.print(buckEfficiency);u8g2.print("%");//工作效率
  u8g2.sendBuffer();           //显示当前Buffer内的内容
}

//一下为菜单2显示
void displayConfig2() {
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setFontDirection(0);       //设置显示方向
  u8g2.clearBuffer();             //清除当前Buffer内的内容
  //u8g2.drawXBM(0,0,tbdc_x,tbdc_y,tbdc);//logo图标
  u8g2.drawXBM(80, 0, tbwd_x, tbwd_y, tbwd);  //温度图标
  u8g2.drawXBM(0, 19, tbv_x, tbv_y, tbv);     //输入电压图标
  u8g2.drawXBM(64, 19, tbi_x, tbi_y, tbi);    //输入电流图标
  u8g2.drawXBM(0, 35, tbw_x, tbw_y, tbw);     //输入功率图标
  //u8g2.drawXBM(64,35,tbdca_x,tbdca_y,tbdca);//输出电流图标
  u8g2.drawXBM(0, 51, tbdcrl_x, tbdcrl_y, tbdcrl);  //电池容量图标
  u8g2.drawXBM(48, 51, tbdc_x, tbdc_y, tbdc);       //总电量图标
  //以下为画横线
  u8g2.drawLine(0, 16, 128, 16);
  u8g2.drawLine(0, 35, 128, 35);
  u8g2.drawLine(0, 51, 128, 51);
  //以下为画竖线
  u8g2.drawLine(80, 0, 80, 16);
  u8g2.drawLine(62, 16, 62, 51);
  u8g2.drawLine(45, 51, 45, 64);
  u8g2.setCursor(8, 13);u8g2.print("Input");  //标题
  u8g2.setCursor(96, 13);u8g2.print(temperature);u8g2.print("℃");//内部温度
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setCursor(16, 31);u8g2.print(voltageInput, 1);u8g2.print("V");//输入电压
  u8g2.setCursor(80, 31);u8g2.print(currentInput, 2);u8g2.print("A");//输入电流
  u8g2.setCursor(16, 47); u8g2.print(powerInput, 1); u8g2.print("W");//输入功率
  u8g2.setCursor(16, 63);u8g2.print(batteryPercent);u8g2.print("%");///电池容量
  u8g2.setCursor(64, 63);
  if (Wh < 10) {u8g2.print(Wh, 3);u8g2.print("Wh ");}  //9.999Wh_
  else if (Wh < 100) {u8g2.print(Wh, 2);u8g2.print("Wh ");} //99.99Wh_
  else if (Wh < 1000) {u8g2.print(Wh, 1);u8g2.print("Wh ");} //999.9Wh_
  else if (Wh < 10000) {u8g2.print(kWh, 2);u8g2.print("kWh ");} //9.99kWh_
  else if (Wh < 100000) {u8g2.print(kWh, 1);u8g2.print("kWh ");} //99.9kWh_
  else if (Wh < 1000000) {u8g2.print(kWh, 0);u8g2.print("kWh  ");} //999kWh__
  else if (Wh < 10000000) {u8g2.print(MWh, 2);u8g2.print("MWh ");} //9.99MWh_
  else if (Wh < 100000000) {u8g2.print(MWh, 1);u8g2.print("MWh ");} //99.9MWh_
  else if (Wh < 1000000000) {u8g2.print(MWh, 0);u8g2.print("MWh  ");} //999MWh__
  u8g2.sendBuffer();  //显示当前Buffer内的内容
}
//一下为菜单3显示
void displayConfig3() {
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setFontDirection(0);       //设置显示方向
  u8g2.clearBuffer();             //清除当前Buffer内的内容
  //u8g2.drawXBM(0,0,tbdc_x,tbdc_y,tbdc);//logo图标
  u8g2.drawXBM(80, 0, tbwd_x, tbwd_y, tbwd);        //温度图标
  u8g2.drawXBM(0, 19, tbdcv_x, tbdcv_y, tbdcv);     //输出电压图标
  u8g2.drawXBM(64, 19, tbdca_x, tbdca_y, tbdca);    //输出电流图标
  u8g2.drawXBM(0, 35, tbdcw_x, tbdcw_y, tbdcw);     //输出功率图标
  u8g2.drawXBM(0, 51, tbdcrl_x, tbdcrl_y, tbdcrl);  //电池容量图标
  u8g2.drawXBM(48, 51, tbdc_x, tbdc_y, tbdc);       //总电量图标
  //以下为画横线
  u8g2.drawLine(0, 16, 128, 16);
  u8g2.drawLine(0, 35, 128, 35);
  u8g2.drawLine(0, 51, 128, 51);
  //以下为画竖线
  u8g2.drawLine(80, 0, 80, 16);
  u8g2.drawLine(62, 16, 62, 51);
  u8g2.drawLine(45, 51, 45, 64);
  u8g2.setCursor(8, 13);
  u8g2.print("charging");  //标题
  u8g2.setCursor(96, 13);u8g2.print(temperature);u8g2.print("℃");//内部温度
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setCursor(16, 31);u8g2.print(voltageOutput, 1);u8g2.print("V");//输出电压
  u8g2.setCursor(80, 31);u8g2.print(currentOutput, 2);u8g2.print("A");//输出电流
  u8g2.setCursor(16, 47);u8g2.print(powerOutput, 1);u8g2.print("W"); //输出功率
  u8g2.setCursor(16, 63);u8g2.print(batteryPercent);u8g2.print("%");//电池容量
  u8g2.setCursor(64, 63);
  if (Wh < 10) {u8g2.print(Wh, 3);u8g2.print("Wh ");}  //9.999Wh_
  else if (Wh < 100) {u8g2.print(Wh, 2);u8g2.print("Wh ");} //99.99Wh_
  else if (Wh < 1000) {u8g2.print(Wh, 1);u8g2.print("Wh ");} //999.9Wh_
  else if (Wh < 10000) {u8g2.print(kWh, 2);u8g2.print("kWh ");} //9.99kWh_
  else if (Wh < 100000) {u8g2.print(kWh, 1);u8g2.print("kWh ");} //99.9kWh_
  else if (Wh < 1000000) {u8g2.print(kWh, 0);u8g2.print("kWh  ");} //999kWh__
  else if (Wh < 10000000) {u8g2.print(MWh, 2);u8g2.print("MWh ");} //9.99MWh_
  else if (Wh < 100000000) {u8g2.print(MWh, 1);u8g2.print("MWh ");} //99.9MWh_
  else if (Wh < 1000000000) {u8g2.print(MWh, 0);u8g2.print("MWh  ");} //999MWh__
  u8g2.sendBuffer();  //显示当前Buffer内的内容
}
//一下为菜单4显示
void displayConfig4() {
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setFontDirection(0);       //设置显示方向
  u8g2.clearBuffer();             //清除当前Buffer内的内容
  //u8g2.drawXBM(0,0,tbdc_x,tbdc_y,tbdc);//logo图标
  u8g2.drawXBM(80, 0, tbwd_x, tbwd_y, tbwd);        //温度图标
  u8g2.drawXBM(0, 19, tbdcv_x, tbdcv_y, tbdcv);     //输出电压图标
  u8g2.drawXBM(64, 19, tbdca_x, tbdca_y, tbdca);    //输出电流图标
  //以下为画横线
  u8g2.drawLine(0, 16, 128, 16);
  u8g2.drawLine(0, 35, 128, 35);
  //以下为画竖线
  u8g2.drawLine(80, 0, 80, 16);
  u8g2.drawLine(62, 16, 62, 35);
  u8g2.setCursor(8, 13);
  u8g2.print("output");  //标题
  u8g2.setCursor(96, 13);u8g2.print(temperature);u8g2.print("℃");//内部温度
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setCursor(16, 31);u8g2.print(voltageOutput, 1);u8g2.print("V");//输出电压
  u8g2.setCursor(80, 31);u8g2.print(loadcurrentOutput, 2);u8g2.print("A");//输出电流
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setCursor(8, 56);u8g2.print("LoadState:");
  const char* enableMos2 = "";
  if (enableMosen == 1) {enableMos2 = "OPEN";}
  else if (enableMosen == 0) {enableMos2 = "CLOSE";} 
  u8g2.setCursor(90, 56);u8g2.print(enableMos2);
  u8g2.sendBuffer();  //显示当前Buffer内的内容
}
//一下为菜单5显示
void displayConfig5() {
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.clearBuffer();
  u8g2.setCursor(8, 16);u8g2.print("==device status==");
  u8g2.setCursor(0, 40);u8g2.print("temperature");u8g2.setCursor(88, 40);u8g2.print(temperature);u8g2.print("℃");
  u8g2.setCursor(0, 58);u8g2.print("thermal status");u8g2.setCursor(88, 58);if (fanStatus == 1) {u8g2.print("开");} else {u8g2.print("关");}
  u8g2.sendBuffer();
}
void displayConfig6() {
  u8g2.setFont(u8g2_font_unifont_t_chinese2);
  u8g2.clearBuffer();
  u8g2.setCursor(8, 12);u8g2.print("==Network=="); 
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.setCursor(0, 26);u8g2.print("ssid:");u8g2.setCursor(42, 26);u8g2.print("MPPT");
  u8g2.setCursor(0, 42);u8g2.print("pass:");u8g2.setCursor(42, 42);u8g2.print("12345678");
  u8g2.setCursor(0, 58);u8g2.print(WiFi.localIP());
  u8g2.sendBuffer();
}
//一下为菜单7显示
void displayConfig7() {
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  //设置显示字体
  u8g2.clearBuffer(); 
  u8g2.setCursor(8, 16);u8g2.print("==system menu==");
  u8g2.setCursor(0, 40);u8g2.print(">confirm entry");
  u8g2.sendBuffer();
}
void factoryResetMessageLCD() {
  u8g2.clearBuffer();
  u8g2.setCursor(8, 16);u8g2.print("==factory reset==");
  u8g2.setCursor(48, 40);u8g2.print("DONE");
  u8g2.sendBuffer();
  delay(1000);
}
////////////////////////////////////////////  MAIN LCD MENU CODE /////////////////////////////////////////////
void LCD_Menu() {
  int
    menuPages = 6,
    subMenuPages = 13,
    longPressTime = 3000,
    longPressInterval = 500,
    shortPressInterval = 100;

  //SETTINGS MENU
  if (settingMode == 1) {
    chargingPause = 1;

    //BUTTON KEYPRESS
    if (setMenuPage == 0) {
      if (digitalRead(buttonRight) == 1) { subMenuPage++; }
      if (digitalRead(buttonLeft) == 1) { subMenuPage--; }
      if (digitalRead(buttonBack) == 1) {
        settingMode = 0;
        subMenuPage = 0;
      }                                                         //bool engage, main menu int page
      if (digitalRead(buttonSelect) == 1) { setMenuPage = 1; }  //enter sub menu settings - bool engage
      lcdBacklight_Wake();
      while (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1
          || digitalRead(buttonBack) == 1 || digitalRead(buttonSelect) == 1) {}
    }
    //SUB MENU PAGE CYCLING
    if (subMenuPage > subMenuPages) {
      subMenuPage = 0;
    } else if (subMenuPage < 0) {
      subMenuPage = subMenuPages;
    }
    //--------------------------- SETTINGS MENU PAGES: ---------------------------//
    ///// SETTINGS MENU ITEM: SUPPLY ALGORITHM SELECT /////
    if (subMenuPage == 0) {
      u8g2.clearBuffer();
      u8g2.setCursor(8, 16);
      u8g2.print("==charging mode==");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
      u8g2.setCursor(0, 40);
      u8g2.print("=");
      }
      if (MPPT_Mode == 1) {
        u8g2.print("MPPT同步整流");
      } else {
        u8g2.print("normal同步整流");
      }
      u8g2.sendBuffer();

      //SET MENU - BOOLTYPE
      if (setMenuPage == 0) {
        boolTemp = MPPT_Mode;
      } else {
        if (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {
          while (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {}
          if (MPPT_Mode == 1) {
            MPPT_Mode = 0;
          } else {
            MPPT_Mode = 1;
          }
        }
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          MPPT_Mode = boolTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
      }
    }

    ///// SETTINGS MENU ITEM: CHARER/PSU MODE /////
    else if (subMenuPage == 1) {
      u8g2.clearBuffer();
      u8g2.setCursor(8, 16);
      u8g2.print("==operating mode==");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      if (output_Mode == 1) {
        u8g2.print("charging mode");
      } else {
        u8g2.print("power mode");
      }
      u8g2.sendBuffer();
      //SET MENU - BOOLTYPE
      if (setMenuPage == 0) {
        boolTemp = output_Mode;
      } else {
        if (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {
          while (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {}
          if (output_Mode == 1) {
            output_Mode = 0;
          } else {
            output_Mode = 1;
          }
        }
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          output_Mode = boolTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
      }
    }

    ///// SETTINGS MENU ITEM: MAX BATTERY V /////
    else if (subMenuPage == 2) {
      u8g2.clearBuffer();
      u8g2.setCursor(4, 16);
      u8g2.print("=max battery voltage=");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      u8g2.setCursor(8, 40);
      u8g2.print(voltageBatteryMax, 2);
      u8g2.print("V");

      //SET MENU - FLOATTYPE
      if (setMenuPage == 0) {
        floatTemp = voltageBatteryMax;
      } else {
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          voltageBatteryMax = floatTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
        currentMenuSetMillis = millis();
        if (digitalRead(buttonRight) == 1) {  //Right button press (increments setting values)
          while (digitalRead(buttonRight) == 1) {
            if (millis() - currentMenuSetMillis > longPressTime) {                             //Long Press
              voltageBatteryMax += 1.00;                                                       //Increment by 1
              voltageBatteryMax = constrain(voltageBatteryMax, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMax, 2);
              delay(longPressInterval);                                                        //Display settings data
            } else {                                                                           //Short Press
              voltageBatteryMax += 0.01;                                                       /////                                                  //Increment by 0.01
              voltageBatteryMax = constrain(voltageBatteryMax, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMax, 2);
              delay(shortPressInterval);  //Display settings data
            }
            u8g2.print("V");  //Display unit
          }
        } else if (digitalRead(buttonLeft) == 1) {  //Left button press (decrements setting values)
          while (digitalRead(buttonLeft) == 1) {
            if (millis() - currentMenuSetMillis > longPressTime) {                             //Long Press
              voltageBatteryMax -= 1.00;                                                       //Increment by 1
              voltageBatteryMax = constrain(voltageBatteryMax, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMax, 2);
              delay(longPressInterval);                                                        //Display settings data
            } else {                                                                           //Short Press
              voltageBatteryMax -= 0.01;                                                       /////                                                 //Increment by 0.01
              voltageBatteryMax = constrain(voltageBatteryMax, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMax, 2);
              delay(shortPressInterval);  //Display settings data
            }
            u8g2.print("V");  //Display unit
          }
        }
      }
      u8g2.sendBuffer();
    }
    ///// SETTINGS MENU ITEM: MIN BATTERY V /////
    else if (subMenuPage == 3) {
      u8g2.clearBuffer();
      u8g2.setCursor(4, 16);
      u8g2.print("=min battery voltage=");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      u8g2.setCursor(8, 40);
      u8g2.print(voltageBatteryMin, 2);
      u8g2.print("V");
      u8g2.print("                ");

      //SET MENU - FLOATTYPE
      if (setMenuPage == 0) {
        floatTemp = voltageBatteryMin;
      } else {
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          voltageBatteryMin = floatTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
        currentMenuSetMillis = millis();
        if (digitalRead(buttonRight) == 1) {  //Right button press (increments setting values)
          while (digitalRead(buttonRight) == 1) {
            if (millis() - currentMenuSetMillis > longPressTime) {                             //Long Press
              voltageBatteryMin += 1.00;                                                       //Increment by 1
              voltageBatteryMin = constrain(voltageBatteryMin, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMin, 2);
              delay(longPressInterval);                                                        //Display settings data
            } else {                                                                           //Short Press
              voltageBatteryMin += 0.01;                                                       ////                                            //Increment by 0.01
              voltageBatteryMin = constrain(voltageBatteryMin, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMin, 2);
              delay(shortPressInterval);  //Display settings data
            }
            u8g2.print("V");  //Display unit
          }
        } else if (digitalRead(buttonLeft) == 1) {  //Left button press (decrements setting values)
          while (digitalRead(buttonLeft) == 1) {
            if (millis() - currentMenuSetMillis > longPressTime) {                             //Long Press
              voltageBatteryMin -= 1.00;                                                       //Increment by 1
              voltageBatteryMin = constrain(voltageBatteryMin, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMin, 2);
              delay(longPressInterval);                                                        //Display settings data
            } else {                                                                           //Short Press
              voltageBatteryMin -= 0.01;                                                       //////                                       //Increment by 0.01
              voltageBatteryMin = constrain(voltageBatteryMin, vOutSystemMin, vOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(voltageBatteryMin, 2);
              delay(shortPressInterval);  //Display settings data
            }
            u8g2.print("V");  //Display unit
          }
        }
      }
      u8g2.sendBuffer();
    }
    ///// SETTINGS MENU ITEM: CHARGING CURRENT /////
    else if (subMenuPage == 4) {
      u8g2.clearBuffer();
      u8g2.setCursor(4, 16);
      u8g2.print("=charge current control=");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      u8g2.setCursor(8, 40);
      u8g2.print(currentCharging, 2);
      u8g2.print("A");

      //SET MENU - FLOATTYPE
      if (setMenuPage == 0) {
        floatTemp = currentCharging;
      } else {
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          currentCharging = floatTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
        currentMenuSetMillis = millis();
        if (digitalRead(buttonRight) == 1) {  //Right button press (increments setting values)
          while (digitalRead(buttonRight) == 1) {
            if (millis() - currentMenuSetMillis > longPressTime) {               //Long Press
              currentCharging += 1.00;                                           //Increment by 1
              currentCharging = constrain(currentCharging, 0.0, cOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(currentCharging, 2);
              delay(longPressInterval);                                          //Display settings data
            } else {                                                             //Short Press
              currentCharging += 0.01;                                           //////                                             //Increment by 0.01
              currentCharging = constrain(currentCharging, 0.0, cOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(currentCharging, 2);
              delay(shortPressInterval);  //Display settings data
            }
            u8g2.print("A");  //Display unit
          }
        } else if (digitalRead(buttonLeft) == 1) {  //Left button press (decrements setting values)
          while (digitalRead(buttonLeft) == 1) {
            if (millis() - currentMenuSetMillis > longPressTime) {               //Long Press
              currentCharging -= 1.00;                                           //Increment by 1
              currentCharging = constrain(currentCharging, 0.0, cOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(currentCharging, 2);
              delay(longPressInterval);                                          //Display settings data
            } else {                                                             //Short Press
              currentCharging -= 0.01;                                           ////////                                          //Increment by 0.01
              currentCharging = constrain(currentCharging, 0.0, cOutSystemMax);  //Limit settings values to a range
              u8g2.setCursor(8, 40);
              u8g2.print(currentCharging, 2);
              delay(shortPressInterval);  //Display settings data
            }
            u8g2.print("A");  //Display unit
          }
        }
      }
      u8g2.sendBuffer();
    }
    ///// 负载mos控制 /////
    else if (subMenuPage == 5) {
      u8g2.clearBuffer();
      u8g2.setCursor(8, 16);
      u8g2.print("==output control==");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      if (enableMos == 1) {
        u8g2.print("OPEN");
      } else if (enableMos == 0) {
        u8g2.print("CLOSE");
      } else if (enableMos == 2) {
        u8g2.print("AUTO");
      }

      //SET MENU - BOOLTYPE
      if (setMenuPage == 0) {
        intTemp = enableMos;
      } else {
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          enableMos = intTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
        if (digitalRead(buttonRight) == 1) {
          while (digitalRead(buttonRight) == 1) {
            delay(200);
            enableMos++;
            enableMos = constrain(enableMos, 0, 2);
          }
        } else if (digitalRead(buttonLeft) == 1) {
          while (digitalRead(buttonLeft) == 1) {
            delay(200);
            enableMos--;
            enableMos = constrain(enableMos, 0, 2);
          }
        }
      }
      u8g2.sendBuffer();
    }

    ///// SETTINGS MENU ITEM: COOLING FAN /////
    else if (subMenuPage == 6) {
      u8g2.clearBuffer();
      u8g2.setCursor(8, 16);
      u8g2.print("==COOLING==");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      if (enableFan == 1) {
        u8g2.print("OPEN");
      } else {
        u8g2.print("CLOSE");
      }

      //SET MENU - BOOLTYPE
      if (setMenuPage == 0) {
        boolTemp = enableFan;
      } else {
        if (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {
          while (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {}
          if (enableFan == 1) {
            enableFan = 0;
          } else {
            enableFan = 1;
          }
        }
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          enableFan = boolTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
      }
      u8g2.sendBuffer();
    }

    ///// SETTINGS MENU ITEM: FAN TRIG TEMP /////
    else if (subMenuPage == 7) {
      u8g2.clearBuffer();
      u8g2.setCursor(4, 16);
      u8g2.print("=cooling start temp=");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      u8g2.setCursor(8, 40);
      u8g2.print(temperatureFan);
      u8g2.print("℃");

      //SET MENU - INTTYPE
      if (setMenuPage == 0) {
        intTemp = temperatureFan;
      } else {
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          temperatureFan = intTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
        if (digitalRead(buttonRight) == 1) {  //Right button press (increments setting values)
          while (digitalRead(buttonRight) == 1) {
            temperatureFan++;                                    //Increment by 1
            temperatureFan = constrain(temperatureFan, 0, 100);  //Limit settings values to a range
            u8g2.setCursor(8, 40);
            u8g2.print(temperatureFan);
            delay(shortPressInterval);  //Display settings data
            u8g2.print("℃");            //Display unit
          }
        } else if (digitalRead(buttonLeft) == 1) {  //Left button press (decrements setting values)
          while (digitalRead(buttonLeft) == 1) {
            temperatureFan--;                                    //Increment by 1
            temperatureFan = constrain(temperatureFan, 0, 100);  //Limit settings values to a range
            u8g2.setCursor(8, 40);
            u8g2.print(temperatureFan);
            delay(shortPressInterval);  //Display settings data
            u8g2.print("℃");            //Display unit
          }
        }
      }
      u8g2.sendBuffer();
    }
    ///// SETTINGS MENU ITEM: SHUTDOWN TEMP /////
    else if (subMenuPage == 8) {
      u8g2.clearBuffer();
      u8g2.setCursor(4, 16);
      u8g2.print("=cooling stop temp=");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      u8g2.setCursor(8, 40);
      u8g2.print(temperatureMax);
      u8g2.print("℃");

      //SET MENU - INTTYPE
      if (setMenuPage == 0) {
        intTemp = temperatureMax;
      } else {
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          temperatureMax = intTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
        if (digitalRead(buttonRight) == 1) {  //Right button press (increments setting values)
          while (digitalRead(buttonRight) == 1) {
            temperatureMax++;                                    //Increment by 1
            temperatureMax = constrain(temperatureMax, 0, 120);  //Limit settings values to a range
            u8g2.setCursor(8, 40);
            u8g2.print(temperatureMax);
            delay(shortPressInterval);  //Display settings data
            u8g2.print("℃");            //Display unit
          }
        } else if (digitalRead(buttonLeft) == 1) {  //Left button press (decrements setting values)
          while (digitalRead(buttonLeft) == 1) {
            temperatureMax--;                                    //Increment by 1
            temperatureMax = constrain(temperatureMax, 0, 120);  //Limit settings values to a range
            u8g2.setCursor(8, 40);
            u8g2.print(temperatureMax);
            delay(shortPressInterval);  //Display settings data
            u8g2.print("℃");            //Display unit
          }
        }
      }
      u8g2.sendBuffer();
    }
    ///// SETTINGS MENU ITEM: WIFI FEATURE /////
    else if (subMenuPage == 9) {
      u8g2.clearBuffer();
      u8g2.setCursor(8, 16);
      u8g2.print("==WIFI state==");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      if (enableWiFi == 1) {
        u8g2.print("open");
      } else {
        u8g2.print("close");
      }

      //SET MENU - BOOLTYPE
      if (setMenuPage == 0) {
        boolTemp = enableWiFi;
      } else {
        if (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {
          while (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {}
          if (enableWiFi == 1) {
            enableWiFi = 0;
          } else {
            enableWiFi = 1;
          }
        }
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          enableWiFi = boolTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
      }
      u8g2.sendBuffer();
    }

    ///// SETTINGS MENU ITEM: AUTOLOAD /////
    else if (subMenuPage == 10) {
      u8g2.clearBuffer();
      u8g2.setCursor(8, 16);
      u8g2.print("==operating mode==");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      if (flashMemLoad == 1) {
        u8g2.print("auto");
      } else {
        u8g2.print("manual");
      }

      //SET MENU - BOOLTYPE
      if (setMenuPage == 0) {
        boolTemp = flashMemLoad;
      } else {
        if (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {
          while (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {}
          if (flashMemLoad == 1) {
            flashMemLoad = 0;
          } else {
            flashMemLoad = 1;
          }
        }
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          flashMemLoad = boolTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveAutoloadSettings();
          setMenuPage = 0;
        }
      }
      u8g2.sendBuffer();
    }
    ///// SETTINGS MENU ITEM: BACKLIGHT SLEEP /////
    else if (subMenuPage == 11) {
      u8g2.clearBuffer();
      u8g2.setCursor(8, 16);
      u8g2.print("==backlight time==");
      if (setMenuPage == 1) {
        u8g2.setCursor(0, 40);
        u8g2.print(">");
      } else {
        u8g2.setCursor(0, 40);
        u8g2.print("=");
      }
      u8g2.setCursor(8, 40);
      if (backlightSleepMode == 1) {
        u8g2.print("10 SECONDS    ");
      } else if (backlightSleepMode == 2) {
        u8g2.print("5 MINUTES     ");
      } else if (backlightSleepMode == 3) {
        u8g2.print("1 HOUR        ");
      } else if (backlightSleepMode == 4) {
        u8g2.print("6 HOURS       ");
      } else if (backlightSleepMode == 5) {
        u8g2.print("12 HOURS      ");
      } else if (backlightSleepMode == 6) {
        u8g2.print("1 DAY         ");
      } else if (backlightSleepMode == 7) {
        u8g2.print("3 DAYS        ");
      } else if (backlightSleepMode == 8) {
        u8g2.print("1 WEEK        ");
      } else if (backlightSleepMode == 9) {
        u8g2.print("1 MONTH       ");
      } else {
        u8g2.print("NEVER         ");
      }

      //SET MENU - INTMODETYPE
      if (setMenuPage == 0) {
        intTemp = backlightSleepMode;
      } else {
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          backlightSleepMode = intTemp;
          setMenuPage = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          saveSettings();
          setMenuPage = 0;
        }
        if (digitalRead(buttonRight) == 1) {                         //Right button press (increments setting values)
          backlightSleepMode++;                                      //Increment by 1
          backlightSleepMode = constrain(backlightSleepMode, 0, 9);  //Limit settings values to a range
          u8g2.setCursor(8, 40);
          if (backlightSleepMode == 1) {
            u8g2.print("10 SECONDS    ");
          } else if (backlightSleepMode == 2) {
            u8g2.print("5 MINUTES     ");
          } else if (backlightSleepMode == 3) {
            u8g2.print("1 HOUR        ");
          } else if (backlightSleepMode == 4) {
            u8g2.print("6 HOURS       ");
          } else if (backlightSleepMode == 5) {
            u8g2.print("12 HOURS      ");
          } else if (backlightSleepMode == 6) {
            u8g2.print("1 DAY         ");
          } else if (backlightSleepMode == 7) {
            u8g2.print("3 DAYS        ");
          } else if (backlightSleepMode == 8) {
            u8g2.print("1 WEEK        ");
          } else if (backlightSleepMode == 9) {
            u8g2.print("1 MONTH       ");
          } else {
            u8g2.print("NEVER         ");
          }
          while (digitalRead(buttonRight) == 1) {}
        } else if (digitalRead(buttonLeft) == 1) {                   //Left button press (decrements setting values)
          backlightSleepMode--;                                      //Increment by 1
          backlightSleepMode = constrain(backlightSleepMode, 0, 9);  //Limit settings values to a range
          u8g2.setCursor(8, 40);
          if (backlightSleepMode == 1) {
            u8g2.print("10 SECONDS    ");
          } else if (backlightSleepMode == 2) {
            u8g2.print("5 MINUTES     ");
          } else if (backlightSleepMode == 3) {
            u8g2.print("1 HOUR        ");
          } else if (backlightSleepMode == 4) {
            u8g2.print("6 HOURS       ");
          } else if (backlightSleepMode == 5) {
            u8g2.print("12 HOURS      ");
          } else if (backlightSleepMode == 6) {
            u8g2.print("1 DAY         ");
          } else if (backlightSleepMode == 7) {
            u8g2.print("3 DAYS        ");
          } else if (backlightSleepMode == 8) {
            u8g2.print("1 WEEK        ");
          } else if (backlightSleepMode == 9) {
            u8g2.print("1 MONTH       ");
          } else {
            u8g2.print("NEVER         ");
          }
          while (digitalRead(buttonLeft) == 1) {}
        }
      }
      u8g2.sendBuffer();
    }
    ///// SETTINGS MENU ITEM: FACTORY RESET /////
    else if (subMenuPage == 12) {
      u8g2.clearBuffer();
      if (setMenuPage == 0) {
        u8g2.setCursor(4, 16);u8g2.print("==factory reset==");
        u8g2.setCursor(0, 40);u8g2.print(">press confirm");
      } else {
        if (confirmationMenu == 0) {
          u8g2.setCursor(28, 16);u8g2.print("are you sure?");
          u8g2.setCursor(0, 40);u8g2.print("  >NO      YES  ");
        } else {
          u8g2.setCursor(28, 16);u8g2.print("are you sure?");
          u8g2.setCursor(0, 40);u8g2.print("   NO     >YES  ");
        }
        if (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {
          while (digitalRead(buttonRight) == 1 || digitalRead(buttonLeft) == 1) {}
          if (confirmationMenu == 0) {
            confirmationMenu = 1;  //Cycle Yes NO
          } else {
            confirmationMenu = 0;
          }
        }
        if (digitalRead(buttonBack) == 1) {
          while (digitalRead(buttonBack) == 1) {}
          setMenuPage = 0;
          confirmationMenu = 0;
        }
        if (digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonSelect) == 1) {}
          if (confirmationMenu == 1) {
            factoryReset();
            factoryResetMessageLCD();
          }
          setMenuPage = 0;
          confirmationMenu = 0;
          subMenuPage = 0;
        }
      }
      u8g2.sendBuffer();
    }
    ///// SETTINGS MENU ITEM: FIRMWARE VERSION /////
    else if (subMenuPage == 13) {
      u8g2.clearBuffer();
      if (setMenuPage == 0) {
        u8g2.setCursor(8, 16);u8g2.print("==firmware version==");
        u8g2.setCursor(40, 48);u8g2.print("V1.20");
      } else {
        u8g2.setCursor(6, 16);u8g2.print("www.ykniot.cn");
        u8g2.setCursor(0, 40);u8g2.print("===========");
        if (digitalRead(buttonBack) == 1 || digitalRead(buttonSelect) == 1) {
          while (digitalRead(buttonBack) == 1 || digitalRead(buttonSelect) == 1) {}
          setMenuPage = 0;  //Cancel
        }
      }
      u8g2.sendBuffer();
    }
  }
  //MAIN MENU
  else if (settingMode == 0) {
    chargingPause = 0;

    //OLED BACKLIGHT SLEEP
    lcdBacklight();

    //BUTTON KEYPRESS
    if (digitalRead(buttonRight) == 1) {
      buttonRightCommand = 1;
      lcdBacklight_Wake();
    }
    if (digitalRead(buttonLeft) == 1) {
      buttonLeftCommand = 1;
      lcdBacklight_Wake();
    }
    if (digitalRead(buttonBack) == 1) {
      buttonBackCommand = 1;
      lcdBacklight_Wake();
    }
    if (digitalRead(buttonSelect) == 1) {
      buttonSelectCommand = 1;
      lcdBacklight_Wake();
    }

    currentLCDMillis = millis();
    if (currentLCDMillis - prevLCDMillis >= millisLCDInterval && enableLCD == 1) {  //Run routine every millisLCDInterval (ms)
      prevLCDMillis = currentLCDMillis;

      //MENU PAGE BUTTON ACTION
      if (buttonRightCommand == 1) {
        buttonRightCommand = 0;
        menuPage++;
        u8g2.clearBuffer();
      } else if (buttonLeftCommand == 1) {
        buttonLeftCommand = 0;
        menuPage--;
        u8g2.clearBuffer();
      } else if (buttonBackCommand == 1) {
        buttonBackCommand = 0;
        menuPage = 0;
        u8g2.clearBuffer();
      } else if (buttonSelectCommand == 1 && menuPage == 6) {
        buttonSelectCommand = 0;
        settingMode = 1;
        u8g2.clearBuffer();
      }
      if (menuPage > menuPages) {
        menuPage = 0;
      } else if (menuPage < 0) {
        menuPage = menuPages;
      }

      if (menuPage == 0) {
        displayConfig1();
      } else if (menuPage == 1) {
        displayConfig2();
      } else if (menuPage == 2) {
        displayConfig3();
      } else if (menuPage == 3) {
        displayConfig4();
      } else if (menuPage == 4) {
        displayConfig5();
      } else if (menuPage == 5) {
        displayConfig6();
      } else if (menuPage == 6) {
        displayConfig7();
      }
    }
  }
}
