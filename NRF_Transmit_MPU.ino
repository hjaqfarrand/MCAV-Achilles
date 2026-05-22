#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

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

  Wire.begin(21, 22);
  Wire.setClock(100000);

  Serial.println("MPU6050 detected");
}

void loop() {
  byte msg = 0xAB;
  bool ok = radio.write(&msg, sizeof(msg));
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);

  if (Wire.available() == 6) {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();

    Serial.print("AX: "); Serial.print(ax);
    Serial.print(" AY: "); Serial.print(ay);
    Serial.print(" AZ: "); Serial.println(az);
  }

  delay(500);

  if (ok) {
    Serial.println("Sent OK — ACK received from receiver");
  } 
  else {
    Serial.println("Send FAILED — no ACK");
  }

  delay(1000);
}
