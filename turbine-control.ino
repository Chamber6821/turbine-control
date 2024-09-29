
constexpr int LED_PIN = 13;

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

bool light = false;
void loop() {
  digitalWrite(LED_PIN, light ? HIGH : LOW);
  light = !light;
  delay(100);
}
