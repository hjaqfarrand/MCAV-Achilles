#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/*
  nRF24L01 Transmitter — ESP32
  ─────────────────────────────────────
  nRF24  →  ESP32
  CE     →  GPIO 26
  CSN    →  GPIO 25
  SCK    →  GPIO 19
  MISO   →  GPIO 17
  MOSI   →  GPIO 16
  VCC    →  3.3V  (do NOT use 5V)
  GND    →  GND
*/


// CE, CSN
RF24 radio(26, 25); // CE=22, CSN=21

const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);

  SPI.begin(19, 17, 16, 25);

  if (!radio.begin()) {
    Serial.println("nRF24 not responding — check wiring!");
    while (1) {}
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("Transmitter ready");
}

void loop() {
  byte msg = 0xAB;
  bool ok = radio.write(&msg, sizeof(msg));

  if (ok) {
    Serial.println("Sent OK — ACK received from receiver");
  } else {
    Serial.println("Send FAILED — no ACK");
  }

  delay(1000);
}
