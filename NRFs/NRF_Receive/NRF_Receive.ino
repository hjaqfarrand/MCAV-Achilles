/*
  nRF24L01 Receiver — ESP32
  ─────────────────────────────────────
  nRF24  →  ESP32
  CE     →  GPIO 21
  CSN    →  GPIO 22
  SCK    →  GPIO 19
  MISO   →  GPIO 18
  MOSI   →  GPIO 23
  VCC    →  3.3V  (do NOT use 5V)
  GND    →  GND

  MPU6050 → ESP32
  SDA     → GPIO 21
  SCL     → GPIO 22
  VCC     → 3.3V
  GND     → GND

  LED     → GPIO 4 → 330Ω → GND
*/

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(21, 22); // CE=26, CSN=25

const byte address[6] = "00001";
const byte BTN_MSG    = 0xAB;   // value the transmitter sends on button press


void setup() {
  Serial.begin(115200);

  SPI.begin(19, 18, 23, 22); // SCK, MISO, MOSI, SS

  if (!radio.begin()) {
    Serial.println("nRF24 not responding — check wiring!");
    while (1) {}
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);          // must match transmitter
  radio.openReadingPipe(1, address);
  radio.startListening();

  Serial.println("Receiver ready — listening for button presses...");
}

void loop() {
  if (radio.available()) {
    byte received = 0;
    radio.read(&received, sizeof(received));

    Serial.print("Received: 0x");
    Serial.println(received, HEX);

    if (received == BTN_MSG) {
      Serial.println("Button press confirmed");
    } else {
      Serial.println("Unexpected payload — ignored");
    }
  }
}

