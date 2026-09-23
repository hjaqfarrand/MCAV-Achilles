#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <ESP32Servo.h>
#include <math.h>
#include <stdio.h>

// ========================================
// PIN DEFINITIONS
// ========================================

// nRF24L01
#define CE 25
#define IRQ 13
#define CSN 26
#define SCK 27
#define MOSI 14
#define MISO 12

// DRV8833
#define IN1 21
#define IN2 19
#define IN3 18
#define IN4 5

// Raspberry Pi UART
// Changed from GPIO 1/3 to avoid USB Serial conflicts
#define TX 1
#define RX 3

// Encoder 1 - Front
#define A1 34
#define B1 32

// Encoder 2 - Rear
#define A2 35
#define B2 33

// Servos
#define FRONT_SERVO_PIN 2
#define REAR_SERVO_PIN  4

#define FRONT_NEUTRAL 45
#define REAR_NEUTRAL  90

// ========================================
// CONSTANTS
// ========================================

#define PWM_FREQ 5000
#define PWM_RES 8
#define MAX_SPEED 255

#define PI_TIMEOUT_MS 500
#define ODOM_INTERVAL 29
#define PI_BAUD 115200

// Must be calibrated for your car
const float MAX_WHEEL_SPEED = 1.0f; // m/s

// Must match encoder rising edges per wheel revolution
const float ENCODER_PPR = 360.0f;

// ========================================
// OBJECTS
// ========================================

HardwareSerial PiSerial(2);

RF24 radio(CE, CSN);

Servo frontServo;
Servo backServo;

// ========================================
// NRF SETTINGS
// ========================================

const byte address[6] = "00001";

// Must match transmitter command values
const uint8_t CMD_ESTOP   = 0x01;
const uint8_t CMD_RESTART = 0x02;

// ========================================
// COMMAND STRUCTURE
// ========================================

struct car_command
{
    float frontWheelSpeed;
    float rearWheelSpeed;

    float frontWheelAngle;
    float rearWheelAngle;
};

car_command currentCommand = {
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

// ========================================
// GLOBAL VARIABLES
// ========================================

// Encoder counts
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

// Previous encoder counts
long previousCount1 = 0;
long previousCount2 = 0;

// RPM
float rpm1 = 0.0f;
float rpm2 = 0.0f;

// Timing
unsigned long lastOdomTime = 0;
unsigned long lastPiCommandTime = 0;

// Safety
bool eStopped = true;
bool piCommandValid = false;
bool radioReady = false;

// UART buffer
char piInputBuffer[100];
size_t piInputIndex = 0;
bool discardPiLine = false;

// ========================================
// FUNCTION DECLARATIONS
// ========================================

void stopMotors();
void emergencyStop();
void restartSystem();
void checkRadio();

void readRaspberryPi();
bool parsePiCommand(const char *packet);

void setMotorSpeed(float frontSpeed, float rearSpeed);
void setSingleMotor(int pin1, int pin2, float speed);

void sendOdometry();
void setServoAngles(float frontAngle, float rearAngle);

// ========================================
// ENCODER INTERRUPTS
// ========================================

void IRAM_ATTR readEncoder1()
{
    if (digitalRead(B1) == HIGH)
    {
        encoderCount1++;
    }
    else
    {
        encoderCount1--;
    }
}

void IRAM_ATTR readEncoder2()
{
    if (digitalRead(B2) == HIGH)
    {
        encoderCount2++;
    }
    else
    {
        encoderCount2--;
    }
}

// ========================================
// STOP MOTORS
// ========================================

void stopMotors()
{
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);

    ledcWrite(IN3, 0);
    ledcWrite(IN4, 0);
}

// ========================================
// SINGLE MOTOR CONTROL
// ========================================

void setSingleMotor(int pin1, int pin2, float speed)
{
    if (!isfinite(speed))
    {
        ledcWrite(pin1, 0);
        ledcWrite(pin2, 0);
        return;
    }

    // Convert m/s to PWM
    float magnitude = fabsf(speed);

    int pwm = (int)roundf(
        magnitude / MAX_WHEEL_SPEED * MAX_SPEED
    );

    pwm = constrain(pwm, 0, MAX_SPEED);

    // Forward
    if (speed > 0.0f)
    {
        ledcWrite(pin2, 0);
        ledcWrite(pin1, pwm);
    }

    // Reverse
    else if (speed < 0.0f)
    {
        ledcWrite(pin1, 0);
        ledcWrite(pin2, pwm);
    }

    // Stop
    else
    {
        ledcWrite(pin1, 0);
        ledcWrite(pin2, 0);
    }
}

// ========================================
// DUAL MOTOR CONTROL
// ========================================

void setMotorSpeed(float frontSpeed, float rearSpeed)
{
    // E-stop or invalid/stale Pi command
    if (eStopped ||
        !radioReady ||
        !piCommandValid ||
        millis() - lastPiCommandTime > PI_TIMEOUT_MS)
    {
        stopMotors();
        return;
    }

    // Reject invalid numbers
    if (!isfinite(frontSpeed) ||
        !isfinite(rearSpeed))
    {
        stopMotors();
        return;
    }

    // Front motor
    setSingleMotor(IN1, IN2, frontSpeed);

    // Rear motor
    setSingleMotor(IN3, IN4, rearSpeed);
}

// ========================================
// PARSE RASPBERRY PI COMMAND
// ========================================

// Expected packet:
// V,frontSpeed,rearSpeed,frontAngle,rearAngle
//
// Example:
// V,0.500,0.500,15.000,-10.000

bool parsePiCommand(const char *packet)
{
    car_command newCommand;

    int consumed = 0;

    int parsed = sscanf(
        packet,
        "V,%f,%f,%f,%f%n",
        &newCommand.frontWheelSpeed,
        &newCommand.rearWheelSpeed,
        &newCommand.frontWheelAngle,
        &newCommand.rearWheelAngle,
        &consumed
    );

    // Validate packet format
    if (parsed != 4 ||
        consumed == 0 ||
        packet[consumed] != '\0')
    {
        return false;
    }

    // Validate numeric values
    if (!isfinite(newCommand.frontWheelSpeed) ||
        !isfinite(newCommand.rearWheelSpeed) ||
        !isfinite(newCommand.frontWheelAngle) ||
        !isfinite(newCommand.rearWheelAngle))
    {
        return false;
    }

    // Reject speeds outside configured limits
    if (fabsf(newCommand.frontWheelSpeed) > MAX_WHEEL_SPEED ||
        fabsf(newCommand.rearWheelSpeed) > MAX_WHEEL_SPEED)
    {
        return false;
    }

    // Reject unreasonable wheel angle commands
    // Replace with validated mechanical limits.
    if (fabsf(newCommand.frontWheelAngle) > 30.0f ||
        fabsf(newCommand.rearWheelAngle) > 30.0f)
    {
        return false;
    }

    // Save command
    currentCommand = newCommand;

    lastPiCommandTime = millis();

    piCommandValid = true;

    return true;
}

// ========================================
// READ RASPBERRY PI
// ========================================

void readRaspberryPi()
{
    while (PiSerial.available())
    {
        char c = PiSerial.read();

        // End of packet
        if (c == '\n' || c == '\r')
        {
            if (discardPiLine)
            {
                discardPiLine = false;
                piInputIndex = 0;
                continue;
            }

            if (piInputIndex > 0)
            {
                piInputBuffer[piInputIndex] = '\0';

                if (parsePiCommand(piInputBuffer))
                {
                    Serial.print("Front speed: ");
                    Serial.println(currentCommand.frontWheelSpeed);

                    Serial.print("Rear speed: ");
                    Serial.println(currentCommand.rearWheelSpeed);

                    Serial.print("Front angle: ");
                    Serial.println(currentCommand.frontWheelAngle);

                    Serial.print("Rear angle: ");
                    Serial.println(currentCommand.rearWheelAngle);
                }
                else
                {
                    Serial.print("Invalid Pi command: ");
                    Serial.println(piInputBuffer);
                }

                piInputIndex = 0;
            }
        }

        // Read characters
        else if (!discardPiLine)
        {
            if (piInputIndex < sizeof(piInputBuffer) - 1)
            {
                piInputBuffer[piInputIndex++] = c;
            }
            else
            {
                // Discard entire oversized packet
                piInputIndex = 0;
                discardPiLine = true;

                Serial.println("Pi packet overflow");
            }
        }
    }
}

// ========================================
// SEND ODOMETRY
// ========================================

void sendOdometry()
{
    unsigned long currentTime = millis();

    if (currentTime - lastOdomTime < ODOM_INTERVAL)
    {
        return;
    }

    unsigned long elapsedTime =
        currentTime - lastOdomTime;

    lastOdomTime = currentTime;

    // Read encoder counts atomically
    long count1;
    long count2;

    noInterrupts();

    count1 = encoderCount1;
    count2 = encoderCount2;

    interrupts();

    // Encoder differences
    long delta1 = count1 - previousCount1;
    long delta2 = count2 - previousCount2;

    // Calculate RPM
    rpm1 = (
        (float)delta1 / ENCODER_PPR
    ) * (60000.0f / elapsedTime);

    rpm2 = (
        (float)delta2 / ENCODER_PPR
    ) * (60000.0f / elapsedTime);

    // Update previous counts
    previousCount1 = count1;
    previousCount2 = count2;

    // Send to Raspberry Pi
    PiSerial.printf(
        "ODOM,%ld,%ld,%.2f,%.2f,%.2f,%.2f\n",
        count1,
        count2,
        rpm1,
        rpm2,
        currentCommand.frontWheelAngle,
        currentCommand.rearWheelAngle
    );
}

// ========================================
// EMERGENCY STOP
// ========================================

void emergencyStop()
{
    eStopped = true;
    piCommandValid = false;

    stopMotors();

    currentCommand.frontWheelSpeed = 0;
    currentCommand.rearWheelSpeed = 0;

    Serial.println("EMERGENCY STOP");

    PiSerial.println("STATUS,ESTOP");
}

// ========================================
// RESTART SYSTEM
// ========================================

void restartSystem()
{
    // Do not restart if radio is unavailable
    if (!radioReady)
    {
        return;
    }

    // Clear previous motion commands
    currentCommand.frontWheelSpeed = 0;
    currentCommand.rearWheelSpeed = 0;

    piCommandValid = false;

    stopMotors();

    // Explicit restart command clears latch
    eStopped = false;

    Serial.println("SYSTEM READY");

    PiSerial.println("STATUS,READY");
}

// ========================================
// CHECK NRF24L01
// ========================================

void checkRadio()
{
    if (!radioReady)
    {
        emergencyStop();
        return;
    }

    // Drain all pending radio commands
    while (radio.available())
    {
        uint8_t command = 0;

        radio.read(&command, sizeof(command));

        Serial.print("nRF command: ");
        Serial.println(command);

        if (command == CMD_ESTOP)
        {
            emergencyStop();
        }

        else if (command == CMD_RESTART)
        {
            restartSystem();
        }
    }
}

// ========================================
// SERVO CONTROL
// ========================================

// Steering conversion is deliberately disabled
// until the geometry equations are validated.
//
// The Pi sends wheel angles, NOT servo angles.
//
// This function holds both servos at neutral.
// Do not replace it with direct angle writes.

void setServoAngles(float frontAngle, float rearAngle)
{
    (void)frontAngle;
    (void)rearAngle;

    frontServo.write(FRONT_NEUTRAL);
    backServo.write(REAR_NEUTRAL);
}

// ========================================
// SETUP
// ========================================

void setup()
{
    // ------------------------------------
    // 1. Serial Monitor
    // ------------------------------------

    Serial.begin(115200);

    Serial.println("ESP32 STARTING");

    // ------------------------------------
    // 2. Raspberry Pi UART
    // ------------------------------------

    PiSerial.begin(
        PI_BAUD,
        SERIAL_8N1,
        RX,
        TX
    );

    Serial.println("Raspberry Pi UART ready");

    // ------------------------------------
    // 3. DRV8833 Motor Driver
    // ------------------------------------

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Arduino-ESP32 core 3.x
    bool pwmReady = true;

    pwmReady &= ledcAttach(IN1, PWM_FREQ, PWM_RES);
    pwmReady &= ledcAttach(IN2, PWM_FREQ, PWM_RES);
    pwmReady &= ledcAttach(IN3, PWM_FREQ, PWM_RES);
    pwmReady &= ledcAttach(IN4, PWM_FREQ, PWM_RES);

    stopMotors();

    if (!pwmReady)
    {
        Serial.println("Motor PWM setup failed");

        while (true)
        {
            stopMotors();
            delay(100);
        }
    }

    Serial.println("Motor driver ready");

    // ------------------------------------
    // 4. Encoder Initialisation
    // ------------------------------------

    pinMode(A1, INPUT);
    pinMode(B1, INPUT);

    pinMode(A2, INPUT);
    pinMode(B2, INPUT);

    noInterrupts();

    encoderCount1 = 0;
    encoderCount2 = 0;

    interrupts();

    attachInterrupt(
        digitalPinToInterrupt(A1),
        readEncoder1,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(A2),
        readEncoder2,
        RISING
    );

    previousCount1 = 0;
    previousCount2 = 0;

    rpm1 = 0;
    rpm2 = 0;

    lastOdomTime = millis();

    Serial.println("Encoders ready");

    // ------------------------------------
    // 5. Servo Initialisation
    // ------------------------------------

    frontServo.setPeriodHertz(50);
    backServo.setPeriodHertz(50);

    frontServo.attach(FRONT_SERVO_PIN, 500, 2400);
    backServo.attach(REAR_SERVO_PIN, 500, 2400);

    frontServo.write(FRONT_NEUTRAL);
    backServo.write(REAR_NEUTRAL);

    Serial.println("Servos ready");

    // ------------------------------------
    // 6. nRF24L01 Initialisation
    // ------------------------------------

    SPI.begin(SCK, MISO, MOSI, CSN);

    radioReady = radio.begin();

    if (!radioReady)
    {
        Serial.println("ERROR: nRF24 not detected");

        eStopped = true;
        stopMotors();

        while (true)
        {
            stopMotors();
            delay(100);
        }
    }

    // Must match transmitter settings
    radio.setChannel(76);
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);

    radio.openReadingPipe(1, address);

    radio.startListening();

    Serial.println("nRF24 ready");

    // ------------------------------------
    // 7. Initial Safety State
    // ------------------------------------

    eStopped = true;
    piCommandValid = false;

    currentCommand.frontWheelSpeed = 0;
    currentCommand.rearWheelSpeed = 0;

    currentCommand.frontWheelAngle = 0;
    currentCommand.rearWheelAngle = 0;

    lastPiCommandTime = millis();

    stopMotors();

    Serial.println("======================");
    Serial.println("ESP32 READY");
    Serial.println("Waiting for restart");
    Serial.println("======================");
}

// ========================================
// MAIN LOOP
// ========================================

void loop()
{
    // 1. Check emergency-stop commands
    checkRadio();

    // 2. Receive Raspberry Pi commands
    readRaspberryPi();

    // 3. Check radio again before motor output
    checkRadio();

    // 4. Control both motors
    setMotorSpeed(
        currentCommand.frontWheelSpeed,
        currentCommand.rearWheelSpeed
    );

    // 5. Steering held at neutral until calibrated
    setServoAngles(
        currentCommand.frontWheelAngle,
        currentCommand.rearWheelAngle
    );

    // 6. Send encoder feedback
    sendOdometry();
}
