void setup() {
  Serial.begin(9600);
  Serial.flush();

  delay(100);
  Serial.println("READY");
}


void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "START") {
      Serial.println("CALL OA1");
    }
  }
  
  delay(100);
}