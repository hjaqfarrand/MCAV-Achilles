/*
  nRF24L01 Transmitter + Button — ESP32
  ─────────────────────────────────────
  nRF24  →  ESP32
  CE     →  GPIO 21
  CSN    →  GPIO 15
  SCK    →  GPIO 22
  MISO   →  GPIO 23
  MOSI   →  GPIO 18
  VCC    →  3.3V  (do NOT use 5V)
  GND    →  GND

  Button →  GPIO 4  → other leg to GND
  (uses internal pull-up, so no external resistor needed)
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(21, 15); // CE=26, CSN=25

const byte address[6]   = "00001";
const byte BTN_MSG      = 0xAB;
const int  BUTTON_PIN   = 4;

bool lastButtonState = HIGH;   // HIGH = not pressed (pull-up)
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  SPI.begin(22, 23, 18, 15); // SCK, MISO, MOSI, SS

  if (!radio.begin()) {
    Serial.println("nRF24 not responding — check wiring!");
    while (1) {}
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);          // must match receiver
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("Transmitter ready — press the button to send");
}

void loop() {
  bool reading = digitalRead(BUTTON_PIN);

  // Detect a press: pin goes from HIGH (released) to LOW (pressed)
  if (reading == LOW && lastButtonState == HIGH &&
      (millis() - lastDebounceTime) > debounceDelay) {

    lastDebounceTime = millis();

    byte msg = BTN_MSG;
    bool ok = radio.write(&msg, sizeof(msg));

    Serial.println(ok ? "Button pressed — Sent OK" : "Button pressed — Send FAILED");
  }

  lastButtonState = reading;
}