void setup() {
  // Velocidad del motor M1 en un 30%
  motorSpeed(M1, 30);
  // Velocidad del motor M2 en un 30%
  motorSpeed(M3, 30);
}
void loop() {
  goForward(M1, M3);

  delay(1000);

  motorsOff(M1, M3);

  delay(1000);
  
  goReverse(M1, M3);

  delay(1000);

  motorsOff(M1, M3);

  delay(1000);
}
