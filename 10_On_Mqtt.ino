
void mqtt_setup() {
  // you can skip this part if you're already maintaining the connection logic
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500); // waiting for the connection
  }
  Serial.println(WiFi.localIP());
  

  // set device's details (optional)
  device.setName(device_name);
  device.setSoftwareVersion("1.0.0");
  device.setManufacturer(manufacturer);

  // optional properties
  sensor_buckEnable.setCurrentState(buckEnable);
  sensor_buckEnable.setName("buckEnable");
  //sensor_buckEnable.setDeviceClass("door");
  sensor_buckEnable.setIcon("mdi:fire");

  // configure sensor (optional)
  analogSensor_powerInput.setIcon("mdi:home");
  analogSensor_powerInput.setName("powerInput");
  analogSensor_powerInput.setUnitOfMeasurement("W");

  // configure sensor (optional)
  analogSensor_powerOutput.setIcon("mdi:home");
  analogSensor_powerOutput.setName("powerOutput");
  analogSensor_powerOutput.setUnitOfMeasurement("W");

  // configure sensor (optional)
  analogSensor_PWM.setIcon("mdi:home");
  analogSensor_PWM.setName("PWM");
  //analogSensor_PWM.setUnitOfMeasurement("V");

  // configure sensor (optional)
  analogSensor_voltageInput.setIcon("mdi:home");
  analogSensor_voltageInput.setName("voltageInput");
  analogSensor_voltageInput.setUnitOfMeasurement("V");

  // configure sensor (optional)
  analogSensor_voltageOutput.setIcon("mdi:home");
  analogSensor_voltageOutput.setName("voltageOutput");
  analogSensor_voltageOutput.setUnitOfMeasurement("V");
  analogSensor_voltageOutput.setValue(0);

  // configure sensor (optional)
  analogSensor_currentInput.setIcon("mdi:home");
  analogSensor_currentInput.setName("currentInput");
  analogSensor_currentInput.setUnitOfMeasurement("A");
  analogSensor_currentInput.setValue(0);

  // configure sensor (optional)
  analogSensor_currentOutput.setIcon("mdi:home");
  analogSensor_currentOutput.setName("currentOutput");
  analogSensor_currentOutput.setUnitOfMeasurement("A");
  analogSensor_currentOutput.setValue(0);

  // MQTT broker connection (use your data here)
  mqtt.begin(mqtt_server, mqtt_user, mqtt_pass);
}

void mqtt_loop() {
    mqtt.loop();

    if ((millis() - lastUpdateAt) > 1000) { // 1000ms debounce time
        
      sensor_buckEnable.setState(buckEnable);
      analogSensor_powerInput.setValue(powerInput);
      analogSensor_powerOutput.setValue(powerOutput);
      analogSensor_PWM.setValue(PWM);
      analogSensor_voltageInput.setValue(voltageInput);
      analogSensor_voltageOutput.setValue(voltageOutput);
      analogSensor_currentInput.setValue(currentInput);
      analogSensor_currentOutput.setValue(currentOutput);
      lastUpdateAt = millis();

    }
    // your loop logic goes here
}