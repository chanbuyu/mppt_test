void connect_wifi() {
  const byte DNS_PORT = 53;                          //DNS端口
  const String url = "espmppt.com";                  //域名
  const char* APSsid = "MPPT";                       //AP SSID
  const char* APPassword = "12345678";               //AP wifi密码
  IPAddress APIp(10, 10, 0, 1);                      //AP IP
  IPAddress APGateway(10, 10, 0, 1);                 //AP网关
  IPAddress APSubnetMask(255, 255, 255, 0);          //AP子网掩码
  WiFi.mode(WIFI_AP_STA);                            //打开AP和STA共存模式
  WiFi.softAPConfig(APIp, APGateway, APSubnetMask);  //设置AP的IP地址，网关和子网掩码
  WiFi.softAP(APSsid, APPassword);                   //设置AP模式的登陆名和密码
  dnsserver.start(DNS_PORT, url, APIp);              //设置DNS的端口、网址、和IP
  WiFi.begin();                                      //连接WIFI
  Serial.print("Connected");
  //循环，直到连接成功
  while (WiFi.status() != WL_CONNECTED) {
    if (WIFI_Status) {
      Serial.print(".");
      delay(500);
      count++;
      if (count >= 10) {
        WIFI_Status = false;
        Serial.println("WiFi连接失败，请用手机进行配网");
        http_en();
        break;
      }
    }
  }
  if (WIFI_Status != false) {
    Serial.println("连接成功");
    Serial.print("IP:");
    Serial.println(WiFi.localIP());
    WIFI = 1;
    http_en();
  }
}


void Wireless_Telemetry() {
}

void http_en() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });
  server.on("/mppt", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/plain", MPPT_Data().c_str());
  });
  server.on("/sw", HTTP_GET, Out_Mosfet_sw);
  server.begin();  // 初始化HTTP服务器
  Serial.println("启动http服务！");
}
