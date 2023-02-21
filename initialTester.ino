#include "Arduino.h"

void setup() {

  // setting motor speed percentages
  motorSpeed(M1, 25);
  motorSpeed(M2, 25);
}

void loop() {

  goForward(M1, M2);
  delay(1000);

  motorsOff(M1, M2);
  delay(2000);

  turnRight(M1, M2);
  delay(1000);

  motorsOff(M1, M2);
  delay(2000);
}
