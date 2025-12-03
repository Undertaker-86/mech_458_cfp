#include <Arduino.h>

const float ENCODER_PPR = 6.0;  // change zis

// PINS
// Motor 1 
const int M1_PWM = 5;
const int M1_BRAKE = 4;
const int M1_DIR = 8;
const int M1_ENC_A = 2; // Int 0

// Motor 2 
const int M2_PWM = 6;
const int M2_BRAKE = 7;
const int M2_DIR = 9;
const int M2_ENC_A = 3; // Int 1

// Variables
volatile long m1_pos = 0;
volatile long m2_pos = 0;
long target_pos = 0;

// PID, please tune kp and ki
float kp = 2.0;  
float ki = 0.05; 
float integral_error = 0;

// State for zeroing
bool isSynced = false; // False = Manual Mode, True = Auto Move

void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(M1_PWM, OUTPUT); pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
  
  // Encoders
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

  // STOP Motors Initially, remember to change as needed
  digitalWrite(M1_BRAKE, LOW); // engage brake
  digitalWrite(M2_BRAKE, LOW);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);

  Serial.println("Startup Ok");
  Serial.println("Controls:");
  Serial.println("  'q' / 'a' : Jog Motor 1 (Up / Down)");
  Serial.println("  'w' / 's' : Jog Motor 2 (Up / Down)");
  Serial.println("  'z'       : SET zero");
  Serial.println("  [number]  : Enter a value to Move platform");
  // Note that this is encoder pulses, so it probably will move as follow
  // input / PPR * pitch
}

void loop() {
  // Read serial from keyboard
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Check for jog
    if (input.equals("q")) { manualJog(1, true); }
    else if (input.equals("a")) { manualJog(1, false); }
    else if (input.equals("w")) { manualJog(2, true); }
    else if (input.equals("s")) { manualJog(2, false); }
    
    // Check for Zero 
    else if (input.equalsIgnoreCase("z")) {
      noInterrupts();
      m1_pos = 0;
      m2_pos = 0;
      interrupts();
      target_pos = 0;
      isSynced = true; // Engage holding
      Serial.println("Zero Set! Position is now 0.");
    }
    
    // Check for input
    else {
      long new_target = input.toInt();
      if (new_target == 0 && input != "0") {
         // Invalid input check
      } else {
         target_pos = new_target;
         isSynced = true;
         Serial.print("Moving to: "); Serial.println(target_pos);
         // Release Brakes
         digitalWrite(M1_BRAKE, HIGH);
         digitalWrite(M2_BRAKE, HIGH);
      }
    }
  }

  // Synchronization loop 
  // THis only runs if we have set zero and entered a target
  if (isSynced) {
    
    // --- MASTER (M1) LOGIC ---
    long error_m1 = target_pos - m1_pos;
    int base_speed = 0;

    // Deadband (stop if close enough), change as needed. This is actually a good
    // metric to measure for our CFP so try to vary this (again this is in pulses)
    if (abs(error_m1) < 10) {
        base_speed = 0;
        integral_error = 0; // Reset integrator when stopped
    } else {
        // Direction, this needs to be change/checked
        if (error_m1 > 0) digitalWrite(M1_DIR, LOW); // UP
        else digitalWrite(M1_DIR, HIGH); // DOWN
        
        // Speed Profile
        if (abs(error_m1) < 200) base_speed = 80; // Slow landing, not sure if needed, set = base_speed if not
        else base_speed = 150; // Cruise speed, change as necessary
    }
    
    // SLAVE (M2) LOGIC
    // Calculate Sync Error 
    long sync_error = m1_pos - m2_pos;
    
    // PID Calculation
    integral_error += sync_error;
    // Anti-windup
    integral_error = constrain(integral_error, -500, 500);
    
    float adjustment = (sync_error * kp) + (integral_error * ki);
    
    // Calculate Final Speeds
    int pwm1 = base_speed;
    int pwm2 = base_speed + adjustment;
    
    // If Master stops, Slave must fight to stay at same position as Master
    if (base_speed == 0) {
       // If stopped, just use PID to hold position against M1
       pwm2 = adjustment; 
       // If pwm2 is negative, we need to reverse direction of M2...
       // But for simplicity in this specific setup:
       // If base is 0, we usually just Stop/Brake both.
       pwm1 = 0; 
       pwm2 = 0; 
       digitalWrite(M1_BRAKE, LOW); // Engage Brakes to hold
       digitalWrite(M2_BRAKE, LOW);
    } else {
       digitalWrite(M1_BRAKE, HIGH); // Release Brakes
       digitalWrite(M2_BRAKE, HIGH);
       
       // TO BE IMPLEMENTED
       // Handle Direction reversal for PID? 
       // (In this version: Assume we are moving generally in same direction)
       // If adjustment is huge negative, M2 might stall. 
       
       pwm2 = constrain(pwm2, 0, 255);
       analogWrite(M1_PWM, pwm1);
       analogWrite(M2_PWM, pwm2);
       
       // Ensure M2 Direction matches M1
       digitalWrite(M2_DIR, digitalRead(M1_DIR));
    }
  }
}

// MANUAL JOG FUNCTION 
void manualJog(int motor, bool up) {
  isSynced = false; // Disable auto-sync
  digitalWrite(M1_BRAKE, LOW);
  digitalWrite(M2_BRAKE, LOW);
  
  int pwm = 100; // Jog speed
  int duration = 200; // Move for 200ms then stop
  
  if (motor == 1) {
    digitalWrite(M1_BRAKE, HIGH);
    digitalWrite(M1_DIR, up ? LOW : HIGH);
    analogWrite(M1_PWM, pwm);
    delay(duration);
    analogWrite(M1_PWM, 0);
    digitalWrite(M1_BRAKE, LOW);
  } 
  else {
    digitalWrite(M2_BRAKE, HIGH);
    digitalWrite(M2_DIR, up ? LOW : HIGH);
    analogWrite(M2_PWM, pwm);
    delay(duration);
    analogWrite(M2_PWM, 0);
    digitalWrite(M2_BRAKE, LOW);
  }
  Serial.println("Jog done.");
}

// INTERRUPTS 
void countM1() {
  if (digitalRead(M1_DIR) == LOW) m1_pos++; 
  else m1_pos--; 
}

void countM2() {
  if (digitalRead(M2_DIR) == LOW) m2_pos++; 
  else m2_pos--; 
}