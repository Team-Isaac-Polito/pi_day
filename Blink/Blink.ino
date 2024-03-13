// https://docs.arduino.cc/built-in-examples/basics/Blink/

// try 1, 10, 60, 100
double f = 1; // Hz

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1/f * 1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1/f * 1000);
}
