#define EEP_PIN  33
#define IN1_PIN  18
#define IN2_PIN  19
#define ENC_A    26
#define ENC_B    25

volatile long encoderCount = 0;
unsigned long lastPrintMs  = 0;
long lastCountSnap = 0;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B))
    encoderCount++;
  else
    encoderCount--;
}

void motorSetSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(IN1_PIN, speed);   // v3.x: pass pin directly, no channels
    ledcWrite(IN2_PIN, 0);
  } else if (speed < 0) {
    ledcWrite(IN1_PIN, 0);
    ledcWrite(IN2_PIN, -speed);
  } else {
    ledcWrite(IN1_PIN, 0);
    ledcWrite(IN2_PIN, 0);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Motor Test Bench ===");
  Serial.println("f=fwd  r=rev  s=stop  +=faster  -=slower  e=reset encoder");

  pinMode(EEP_PIN, OUTPUT);
  digitalWrite(EEP_PIN, HIGH);

  // v3.x API: ledcAttach(pin, freq, resolution)
  ledcAttach(IN1_PIN, 5000, 8);
  ledcAttach(IN2_PIN, 5000, 8);

  // Force both LOW to start
  ledcWrite(IN1_PIN, 0);
  ledcWrite(IN2_PIN, 0);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  // Quick sanity pulse on IN1 so you can verify with multimeter
  Serial.println("Pulsing IN1 briefly at startup to verify PWM output...");
  ledcWrite(IN1_PIN, 128);
  delay(2000);
  ledcWrite(IN1_PIN, 0);
  Serial.println("Pulse done. You should have seen ~1.6V on IN1 during those 2 seconds.");
}

void loop() {
  static int cmdSpeed = 180;

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'f': motorSetSpeed( cmdSpeed); Serial.printf("FWD speed=%d\n", cmdSpeed); break;
      case 'r': motorSetSpeed(-cmdSpeed); Serial.printf("REV speed=%d\n", cmdSpeed); break;
      case 's': motorSetSpeed(0);         Serial.println("STOP"); break;
      case '+': cmdSpeed = min(255, cmdSpeed + 15); Serial.printf("Speed=%d\n", cmdSpeed); break;
      case '-': cmdSpeed = max(0,   cmdSpeed - 15); Serial.printf("Speed=%d\n", cmdSpeed); break;
      case 'e': noInterrupts(); encoderCount = 0; interrupts(); Serial.println("Encoder reset"); break;
    }
  }

  if (millis() - lastPrintMs >= 500) {
    long cnt;
    noInterrupts(); cnt = encoderCount; interrupts();
    long delta = cnt - lastCountSnap;
    lastCountSnap = cnt;
    float rpm = ((float)delta / 22.0f) * 2.0f * 60.0f;
    Serial.printf("Enc: %ld  Delta: %ld  RPM: %.1f\n", cnt, delta, rpm);
    lastPrintMs = millis();
  }
}