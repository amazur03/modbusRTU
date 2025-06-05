#include <Arduino.h>
#include <AltSoftSerial.h>

#define RE_PIN 2
#define DE_PIN 3

AltSoftSerial rs485Serial; // RX=8, TX=9 na Arduino Uno

#define FRAME_MAX_SIZE 32
#define FRAME_TIMEOUT 50
#define DEVICE_ADDRESS 0x01

uint8_t buffer[FRAME_MAX_SIZE];
uint8_t idx = 0;
unsigned long lastRxTime = 0;

// Obliczanie CRC16 (Modbus RTU)
uint16_t modbus_crc(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void sendResponse() {
  uint8_t response[9] = {
    DEVICE_ADDRESS,
    0x03,
    0x04,
    0x00, 0x0A,
    0x00, 0x14,
    0x00, 0x00
  };

  uint16_t crc = modbus_crc(response, 7);
  response[7] = crc & 0xFF;
  response[8] = (crc >> 8) & 0xFF;

  digitalWrite(RE_PIN, HIGH);
  digitalWrite(DE_PIN, HIGH);

  rs485Serial.write(response, 9);

  delay(20);

  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);

  Serial.println("Odpowiedź wysłana");
}

void setup() {
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);

  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);

  rs485Serial.begin(9600);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("RS485 odbior - start");
}

void loop() {
  while (rs485Serial.available()) {
    if (idx < FRAME_MAX_SIZE) {
      buffer[idx++] = rs485Serial.read();
      lastRxTime = millis();
    } else {
      idx = 0;
    }
  }

  if (idx > 0 && (millis() - lastRxTime) > FRAME_TIMEOUT) {
    Serial.print("Odebrano ramkę (");
    Serial.print(idx);
    Serial.print(" bajtów): ");

    for (uint8_t i = 0; i < idx; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    if (idx >= 4) {
      uint16_t crc = modbus_crc(buffer, idx - 2);
      uint16_t recvCrc = ((uint16_t)buffer[idx - 2]) | ((uint16_t)buffer[idx - 1] << 8);

      if (crc == recvCrc) {
        if (buffer[0] == DEVICE_ADDRESS) {
          Serial.println("Poprawna ramka do mnie - wysyłam odpowiedź");
          sendResponse();
        } else {
          Serial.println("Poprawna ramka, ale nie do mnie (adres: 0x" + String(buffer[0], HEX) + ")");
        }
      } else {
        Serial.print("Błędne CRC! Obliczone: 0x");
        Serial.print(crc, HEX);
        Serial.print(", Odebrane: 0x");
        Serial.println(recvCrc, HEX);
      }
    } else {
      Serial.println("Za krótka ramka - ignoruję");
    }

    idx = 0;
  }
}
