void Out_MOSFET_Enable(){                                                                  //打开负载输出MOS
  enableMos = 1;
  enableMosen = 1;
  digitalWrite(Out_MOSFET,HIGH);
  saveSettings();//保持设置到EEPROM中
}
void Out_MOSFET_Disable(){                                                                 //关闭负载输出MOS
  enableMos = 0; 
  enableMosen = 0;  
  digitalWrite(Out_MOSFET,LOW);
  saveSettings();//保持设置到EEPROM中
}  
void Out_MOSFET_zt(){
  enableMos = 2;
  if(voltageOutput>=voltageBatteryMin){digitalWrite(Out_MOSFET,HIGH);enableMosen = 1;}
  else{digitalWrite(Out_MOSFET,LOW);enableMosen = 0;}  
  saveSettings();//保持设置到EEPROM中
}                                                                                        //输出MOS状态控制
void Out_Mosfet(){
  //if(enableMos==true){Out_MOSFET_zt();}
  if(enableMos == 1){Out_MOSFET_Enable();}
     else if (enableMos == 0){Out_MOSFET_Disable();}
     else if (enableMos == 2){Out_MOSFET_zt();}
}
  