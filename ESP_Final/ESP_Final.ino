#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <ESP32Servo.h>
#include <math.h>

// ==========================================
// Pin Definitions & Hardware Configuration
// ==========================================

// nRF24L01 Wireless E-stop
#define CE 25
#define IRQ 26
#define CSN 27
#define SCK 14
#define MOSI 12
#define MISO 13

// DRV8833 Motor Driver
#define IN1 21
#define IN2 19
#define IN3 18
#define IN4 5

// UART Configuration
// NOTE: If testing with USB plugged into your PC, use pins 16 (RX) and 17 (TX) for the Pi
// so they don't clash with the USB serial port (pins 3 & 1).
#define TX 1          // Change to 17 if using separate pins from USB
#define RX 3          // Change to 16 if using separate pins from USB
#define PI_BAUD 115200

#define LED_PIN 2     // Onboard status LED (GPIO 2 on most ESP32 DevKits)

// Encoder 1 (Front / Left)
#define A1 34
#define B1 32

// Encoder 2 (Rear / Right)
#define A2 35
#define B2 33

// Servos (Steering)
#define front 15 // Front servo GPIO (Note: change if GPIO 2 is used as LED_PIN)
#define rear 4  // Rear servo GPIO

// Servo neutral positions (physical angle when offset is 0.0 deg)
#define FRONT_SERVO_CENTER 45.0f  // 45 deg physical neutral
#define REAR_SERVO_CENTER  90.0f  // 90 deg physical neutral

// ==========================================
// Constants & Settings
// ==========================================
#define PWM_FREQ 5000
#define PWM_RES 8
#define MAX_SPEED 255
#define PI_TIMEOUT_MS 500
#define ENCODER_PPR 360.0f
#define ODOM_INTERVAL 500   // Odometry send interval in ms (e.g. 500 for testing, 29-50 for high-rate ROS)
#define x_rod 70

// Hardware Serial for Raspberry Pi
HardwareSerial PiSerial(2);

// ==========================================
// Odometry & Encoder Variables
// ==========================================
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
long previousCount1 = 0;
long previousCount2 = 0;
float rpm1 = 0.0f;
float rpm2 = 0.0f;
unsigned long lastOdomTime = 0;

// Encoder ISRs
void IRAM_ATTR encoderISR1() {
  if (digitalRead(A1) == digitalRead(B1)) {
    encoderCount1++;
  } else {
    encoderCount1--;
  }
}

void IRAM_ATTR encoderISR2() {
  if (digitalRead(A2) == digitalRead(B2)) {
    encoderCount2++;
  } else {
    encoderCount2--;
  }
}

// ==========================================
// nRF24L01 Radio Configuration
// ==========================================
RF24 radio(CE, CSN);
const byte address[6] = "00001";

const uint8_t CMD_ESTOP = 0x01;
const uint8_t CMD_RESTART = 0x02;

// ==========================================
// Actuators
// ==========================================
Servo frontServo;
Servo rearServo;

// Motor kinematics variables
float delta_r;
float delta_l;
float theta_s;

// Calculate delta R
float calc_delta_R() 
{
  float rad = theta_s * (M_PI / 180.0f);
  float term1 = 20.054f * sin(rad);
  float term2 = 20.054f * (1.0f - cos(rad));
  float sq_val = pow(39.0f, 2) - pow(term2, 2);
  float sqrt_term = (sq_val >= 0.0f) ? sqrt(sq_val) : 0.0f;
  float common_x = term1 + sqrt_term - 39.0f;

  float y1 = 1583.077f + 28.906f * common_x;
  float x1 = 176.216f - 0.030f * common_x;
  float angle1 = atan2(y1, x1);

  float num = 176.216f + 54.760f * common_x + 0.5f * pow(common_x, 2);
  float den = sqrt(pow(x1, 2) + pow(y1, 2));
  float ratio = constrain(num / den, -1.0f, 1.0f);
  float angle2 = acos(ratio);

  delta_r = (180.0f / M_PI) * (angle1 - angle2);
  return delta_r;
}

// Calculate delta L
float calc_delta_L() 
{
  float rad = theta_s * (M_PI / 180.0f);
  float term1 = 20.054f * sin(rad);
  float term2 = 20.054f * (1.0f - cos(rad));
  float sq_val = pow(39.0f, 2) - pow(term2, 2);
  float sqrt_term = (sq_val >= 0.0f) ? sqrt(sq_val) : 0.0f;
  float common_x = term1 + sqrt_term - 39.0f;

  float y1 = -1583.077f + 28.906f * common_x;
  float x1 = 176.216f + 0.030f * common_x;
  float angle1 = atan2(y1, x1);

  float num = 176.216f - 54.760f * common_x + 0.5f * pow(common_x, 2);
  float den = sqrt(pow(x1, 2) + pow(y1, 2));
  float ratio = constrain(num / den, -1.0f, 1.0f);
  float angle2 = acos(ratio);

  delta_l = (180.0f / M_PI) * (angle1 + angle2);
  return delta_l;
}

// Car command state (Angles are stored as offsets in degrees, where 0.0 = straight ahead)
struct car_command {
  float frontWheelSpeed;
  float rearWheelSpeed;
  float frontWheelAngle;
  float rearWheelAngle;
};

car_command currentCommand = {
  0.0f,
  0.0f,
  0.0f,   // 0.0 deg offset (straight ahead)
  0.0f    // 0.0 deg offset (straight ahead)
};

unsigned long lastPiCommandTime = 0;
bool eStopped = true;

// Forward declarations
void stopMotors();
void emergencyStop();
void restartSystem();
void setServoAngles(float frontAngleOffset, float rearAngleOffset);
void setMotorSpeed(float frontSpeed, float rearSpeed);

// ==========================================
// Motor Control
// ==========================================
void stopMotors() {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
  ledcWrite(IN3, 0);
  ledcWrite(IN4, 0);
}

void setMotorSpeed(float frontSpeed, float rearSpeed) {
  if (eStopped) {
    stopMotors();
    return;
  }

  int speed1 = constrain((int)frontSpeed, -MAX_SPEED, MAX_SPEED);
  int speed2 = constrain((int)rearSpeed, -MAX_SPEED, MAX_SPEED);

  // Motor 1 (Front)
  if (speed1 > 0) {
    ledcWrite(IN1, speed1);
    ledcWrite(IN2, 0);
  } else if (speed1 < 0) {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, -speed1);
  } else {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);
  }

  // Motor 2 (Rear)
  if (speed2 > 0) {
    ledcWrite(IN3, speed2);
    ledcWrite(IN4, 0);
  } else if (speed2 < 0) {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, -speed2);
  } else {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, 0);
  }
}

// ==========================================
// Servo Steering Control
// ==========================================
// Receives steering angle offsets in degrees relative to straight-ahead (0 deg)
void setServoAngles(float frontAngleOffset, float rearAngleOffset) {
  float frontPos = constrain(FRONT_SERVO_CENTER + frontAngleOffset, 0.0f, 180.0f);
  float rearPos  = constrain(REAR_SERVO_CENTER + rearAngleOffset, 0.0f, 180.0f);

  frontServo.write((int)frontPos);
  rearServo.write((int)rearPos);
}

// ==========================================
// Safety & State Management
// ==========================================
void emergencyStop() {
  eStopped = true;
  stopMotors();
  currentCommand.frontWheelSpeed = 0.0f;
  currentCommand.rearWheelSpeed = 0.0f;
  Serial.println("EMERGENCY STOP ACTIVATED");
  PiSerial.println("STATUS,ESTOP");
}

void restartSystem() {
  eStopped = false;
  currentCommand.frontWheelSpeed = 0.0f;
  currentCommand.rearWheelSpeed = 0.0f;
  lastPiCommandTime = millis();
  Serial.println("SYSTEM READY");
  PiSerial.println("STATUS,READY");
}

// ==========================================
// Raspberry Pi UART Communication
// ==========================================

// Parse numeric drive/steer command string from Pi
// Accepts:
// 1. "frontSpeed,rearSpeed,frontAngleOffset,rearAngleOffset" (or prefixed with "V,")
// 2. "speed,frontAngleOffset,rearAngleOffset" (or prefixed with "V,")
// Note: 0.0 deg is straight ahead
bool parsePiCommand(const String &line) {
  String cmd = line;
  if (cmd.startsWith("V,") || cmd.startsWith("v,")) {
    cmd = cmd.substring(2);
  }

  int commaIndices[4];
  int commaCount = 0;
  for (int i = 0; i < (int)cmd.length(); i++) {
    if (cmd.charAt(i) == ',') {
      if (commaCount < 4) {
        commaIndices[commaCount] = i;
      }
      commaCount++;
    }
  }

  if (commaCount == 3) {
    // Format: frontWheelSpeed,rearWheelSpeed,frontAngleOffset,rearAngleOffset
    float fSpeed = cmd.substring(0, commaIndices[0]).toFloat();
    float rSpeed = cmd.substring(commaIndices[0] + 1, commaIndices[1]).toFloat();
    float fAngle = cmd.substring(commaIndices[1] + 1, commaIndices[2]).toFloat();
    float rAngle = cmd.substring(commaIndices[2] + 1).toFloat();

    currentCommand.frontWheelSpeed = constrain(fSpeed, -MAX_SPEED, MAX_SPEED);
    currentCommand.rearWheelSpeed = constrain(rSpeed, -MAX_SPEED, MAX_SPEED);
    currentCommand.frontWheelAngle = constrain(fAngle, -90.0f, 90.0f);
    currentCommand.rearWheelAngle = constrain(rAngle, -90.0f, 90.0f);
    lastPiCommandTime = millis();
    return true;
  } else if (commaCount == 2) {
    // Format: speed,frontAngleOffset,rearAngleOffset
    float speed = cmd.substring(0, commaIndices[0]).toFloat();
    float fAngle = cmd.substring(commaIndices[0] + 1, commaIndices[1]).toFloat();
    float rAngle = cmd.substring(commaIndices[2] + 1).toFloat();

    float constrainedSpeed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    currentCommand.frontWheelSpeed = constrainedSpeed;
    currentCommand.rearWheelSpeed = constrainedSpeed;
    currentCommand.frontWheelAngle = constrain(fAngle, -90.0f, 90.0f);
    currentCommand.rearWheelAngle = constrain(rAngle, -90.0f, 90.0f);
    lastPiCommandTime = millis();
    return true;
  }

  return false;
}

// Check and verify incoming commands from Raspberry Pi
void checkIncomingCommands() {
  if (PiSerial.available()) {
    String incoming = PiSerial.readStringUntil('\n');
    incoming.trim();

    if (incoming.length() > 0) {
      // 1. Toggle LED for instant visual feedback on the board (if not sharing pin with servo)
#if defined(LED_PIN) && (LED_PIN >= 0)
      if (LED_PIN != front && LED_PIN != rear) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
#endif

      // 2. Print to PC Serial Monitor
      Serial.print("[FROM PI] >>> ");
      Serial.println(incoming);

      // 3. Send acknowledgment back to Pi
      PiSerial.printf("ACK: Received [%s]\n", incoming.c_str());

      // 4. Handle commands
      if (incoming.equalsIgnoreCase("ESTOP")) {
        emergencyStop();
      } else if (incoming.equalsIgnoreCase("RESTART")) {
        restartSystem();
      } else {
        if (parsePiCommand(incoming)) {
          Serial.printf("Parsed Pi Command: F_Spd=%.1f, R_Spd=%.1f, F_Ang_Offset=%.1f, R_Ang_Offset=%.1f\n",
                        currentCommand.frontWheelSpeed,
                        currentCommand.rearWheelSpeed,
                        currentCommand.frontWheelAngle,
                        currentCommand.rearWheelAngle);
        }
      }
    }
  }
}

// Send odometry packet over UART to Pi and echo to USB Serial
void sendOdometry() {
  unsigned long currentTime = millis();
  if (currentTime - lastOdomTime < ODOM_INTERVAL) {
    return;
  }
  unsigned long elapsedTime = currentTime - lastOdomTime;
  lastOdomTime = currentTime;

  // Read encoder values safely
  long count1;
  long count2;
  noInterrupts();
  count1 = encoderCount1;
  count2 = encoderCount2;
  interrupts();

  // Calculate RPM
  long delta1 = count1 - previousCount1;
  long delta2 = count2 - previousCount2;
  if (elapsedTime > 0) {
    rpm1 = (delta1 / ENCODER_PPR) * (60000.0f / elapsedTime);
    rpm2 = (delta2 / ENCODER_PPR) * (60000.0f / elapsedTime);
  }
  previousCount1 = count1;
  previousCount2 = count2;

  // Send packet over PiSerial pins (TX/RX)
  PiSerial.printf("ODOM,%ld,%ld,%.2f,%.2f,%.2f,%.2f\n",
                  count1, count2, rpm1, rpm2,
                  currentCommand.frontWheelAngle, currentCommand.rearWheelAngle);

  // Echo to USB Serial for debugging in Arduino Serial Monitor
  Serial.printf("ODOM,%ld,%ld,%.2f,%.2f,%.2f,%.2f\n",
                count1, count2, rpm1, rpm2,
                currentCommand.frontWheelAngle, currentCommand.rearWheelAngle);
}

// ==========================================
// Wireless nRF24 Radio Check
// ==========================================
void checkRadio() {
  if (radio.available()) {
    uint8_t command;
    radio.read(&command, sizeof(command));
    Serial.print("nRF command: ");
    Serial.println(command);
    if (command == CMD_ESTOP) {
      emergencyStop();
    } else if (command == CMD_RESTART) {
      restartSystem();
    }
  }
}

// ==========================================
// Arduino Setup
// ==========================================
void setup() {
  // Status LED
#if defined(LED_PIN) && (LED_PIN >= 0)
  if (LED_PIN != front && LED_PIN != rear) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
  }
#endif

  // Serial Ports
  Serial.begin(115200);
  PiSerial.begin(PI_BAUD, SERIAL_8N1, RX, TX);

  Serial.println("==================================");
  Serial.println("Achilles ESP32 Comms & Control Ready");
  Serial.println("==================================");

  // Motor PWM setup (ESP32 Arduino Core 3.x ledcAttach)
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcAttach(IN2, PWM_FREQ, PWM_RES);
  ledcAttach(IN3, PWM_FREQ, PWM_RES);
  ledcAttach(IN4, PWM_FREQ, PWM_RES);
  stopMotors();

  // Steering Servos
  frontServo.attach(front);
  rearServo.attach(rear);
  setServoAngles(currentCommand.frontWheelAngle, currentCommand.rearWheelAngle);

  // Encoders
  pinMode(A1, INPUT);
  pinMode(B1, INPUT);
  pinMode(A2, INPUT);
  pinMode(B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(A1), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A2), encoderISR2, CHANGE);

  // nRF24 Wireless Receiver
  SPI.begin(SCK, MISO, MOSI, CSN);
  if (!radio.begin()) {
    Serial.println("nRF24 not responding — check wiring!");
  } else {
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(76);
    radio.openReadingPipe(1, address);
    radio.startListening();
    Serial.println("nRF24 Receiver ready.");
  }

  // Initial safety state
  eStopped = true;
  stopMotors();
  lastPiCommandTime = millis();
  Serial.println("System initialized in ESTOP mode. Send RESTART to begin.");
}

// ==========================================
// Main Execution Loop
// ==========================================
void loop() {
  // 1. Wireless Emergency stop has highest priority
  checkRadio();

  // 2. Continuously listen for incoming commands from Pi
  checkIncomingCommands();

  // 3. Drive motors with safety timeout check
  if (!eStopped && (millis() - lastPiCommandTime > PI_TIMEOUT_MS)) {
    stopMotors();
  } else if (!eStopped) {
    setMotorSpeed(
      currentCommand.frontWheelSpeed,
      currentCommand.rearWheelSpeed
    );
  } else {
    stopMotors();
  }

  // 4. Steering (0.0 deg is straight ahead)
  setServoAngles(
    currentCommand.frontWheelAngle,
    currentCommand.rearWheelAngle
  );

  // 5. Send odometry packet (every ODOM_INTERVAL ms, non-blocking)
  sendOdometry();
}
