
void setup() { 
  pinMode(LED_BUILTIN, OUTPUT); 
}

void loop() {
  delay(3000);
  blink();
  blink();
  blink();
}

void blink() {
  digitalWrite(LED_BUILTIN, HIGH); // turn led on
  delay(200);
  digitalWrite(LED_BUILTIN, LOW); // turn led off
  delay(200);
}