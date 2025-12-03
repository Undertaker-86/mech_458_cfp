// Simple ESP32 test: blink onboard LED and print to Serial

const int LED_PIN = 2;  // Onboard LED pin for DOIT ESP32 Devkit V1

void setup() {
  // Start serial
  Serial.begin(115200);
  delay(1000);  // Give serial time to open

  Serial.println();
  Serial.println("ESP32 test starting...");
  Serial.println("If you see this and the LED is blinking, your ESP32 is working!");

  // Set LED pin as output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Turn LED on
  digitalWrite(LED_PIN, HIGH);
  Serial.println("LED ON");
  delay(500);

  // Turn LED off
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED OFF");
  delay(500);
}
