void setup() {
  Serial.begin(9600);   // USB serial to PC
  Serial1.begin(9600);  // GPS on Serial1 (D0 RX, D1 TX)
  Serial.println("Raw NMEA data from GPS:");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);  // just forward whatever GPS sends
  }
}
