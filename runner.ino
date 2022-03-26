void setup() {}
void loop() {

  int test = ultrasoundRead(J3);

  if (test < 120) {

    Serial.println(test);
  }

  delay(30);
}
