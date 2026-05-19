/*
  nRF24L01 Receiver — ESP32
  ─────────────────────────────────────
  nRF24  →  ESP32
  CE     →  GPIO 26
  CSN    →  GPIO 25
  SCK    →  GPIO 19
  MISO   →  GPIO 17
  MOSI   →  GPIO 16
  VCC    →  3.3V  (do NOT use 5V)
  GND    →  GND

  LED    →  GPIO 16 → 330Ω → GND  (moved off 23, CE is already there)
*/

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(26, 25); // CE=22, CSN=21

const byte address[6] = "00001";
const byte LED_PIN    = 23;
const byte EXPECTED   = 0xAB;

bool ledState = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  SPI.begin(19, 17, 16, 25);  // SCK, MISO, MOSI, SS

  if (!radio.begin()) {
    Serial.println("nRF24 not responding — check wiring!");
    while (1) {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
    }
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.openReadingPipe(1, address);
  radio.startListening();

  Serial.println("Receiver ready — listening...");
}

void loop() {
  if (radio.available()) {
    byte received = 0;
    radio.read(&received, sizeof(received));

    Serial.print("Received: 0x");
    Serial.println(received, HEX);

    if (received == EXPECTED) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      Serial.print("Valid signal! LED is now: ");
      Serial.println(ledState ? "ON" : "OFF");
    } else {
      Serial.println("Unexpected payload — LED unchanged");
    }
  }
}
