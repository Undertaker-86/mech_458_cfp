/*
 * Simple Motor Tester for NMB DIA42B10
 * Controls: Serial Monitor
 */

const int PIN_PWM = 5;
const int PIN_BRAKE = 4;
const int PIN_DIR = 8;
const int PIN_ENC = 2; // Just to see if pulses are coming

volatile long encoder_count = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENC, INPUT_PULLUP);

  // Attach interrupt just to see if encoder works
  attachInterrupt(digitalPinToInterrupt(PIN_ENC), countPulse, RISING);

  // Initial State: STOPPED
  digitalWrite(PIN_BRAKE, LOW); // Brake Engaged (Stop)
  digitalWrite(PIN_DIR, LOW);   // CW
  analogWrite(PIN_PWM, 0);      // 0 Speed

  Serial.println("--- MOTOR TESTER ---");
  Serial.println("Select a command:");
  Serial.println(" '1' : Run Slow (PWM 50)");
  Serial.println(" '2' : Run Medium (PWM 100)");
  Serial.println(" '3' : Run Fast (PWM 200)");
  Serial.println(" 's' : STOP (Brake Low)");
  Serial.println(" 'r' : RELEASE Brake (Brake High) - Motor might free wheel");
  Serial.println(" 'd' : Toggle Direction");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Clear the rest of the buffer (newlines etc)
    while(Serial.available() > 0) Serial.read();

    if (cmd == '1') {
      Serial.println("Running SLOW...");
      digitalWrite(PIN_BRAKE, HIGH); // Release Brake
      analogWrite(PIN_PWM, 50);
    }
    else if (cmd == '2') {
      Serial.println("Running MEDIUM...");
      digitalWrite(PIN_BRAKE, HIGH); // Release Brake
      analogWrite(PIN_PWM, 100);
    }
    else if (cmd == '3') {
      Serial.println("Running FAST...");
      digitalWrite(PIN_BRAKE, HIGH); // Release Brake
      analogWrite(PIN_PWM, 200);
    }
    else if (cmd == 's') {
      Serial.println("STOPPING (Brake Engaged)...");
      analogWrite(PIN_PWM, 0);
      digitalWrite(PIN_BRAKE, LOW); // Engage Brake
    }
    else if (cmd == 'r') {
      Serial.println("Brake Released (Motor should spin freely by hand)...");
      analogWrite(PIN_PWM, 0);
      digitalWrite(PIN_BRAKE, HIGH); // Disengage Brake
    }
    else if (cmd == 'd') {
      // Toggle Direction
      int currentDir = digitalRead(PIN_DIR);
      digitalWrite(PIN_DIR, !currentDir);
      Serial.print("Direction changed to: ");
      Serial.println(!currentDir ? "LOW" : "HIGH");
    }
  }

  // Print Encoder count every 1 second to verify it is alive
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.print("Encoder Count: ");
    Serial.println(encoder_count);
    lastPrint = millis();
  }
}

void countPulse() {
  encoder_count++;
}