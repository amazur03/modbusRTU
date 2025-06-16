#include <Arduino.h>

#define RE_DE_PIN 4
#define FRAME_TIMEOUT 100

HardwareSerial rs485Serial(2);

uint8_t response[64];
uint8_t idx = 0;
unsigned long lastRxTime = 0;

uint8_t currentAddress = 0x01;

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

void printFrame(const char* label, uint8_t* data, uint8_t len) {
  Serial.print(label);
  Serial.print(" (");
  Serial.print(len);
  Serial.println(" bajtów):");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print("0x");
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void sendRequest(uint8_t slaveAddr) {
  currentAddress = slaveAddr;

  uint8_t frame[8] = {
    currentAddress,
    0x03,        // Funkcja: Read Holding Registers
    0x00, 0x11,  // Start address: 0x0011
    0x00, 0x14,  // Ilość: 20 rejestrów
    0x00, 0x00   // CRC - placeholder
  };

  uint16_t crc = modbus_crc(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  digitalWrite(RE_DE_PIN, HIGH);
  delay(5);
  rs485Serial.write(frame, 8);
  rs485Serial.flush();
  delay(5);
  digitalWrite(RE_DE_PIN, LOW);

  Serial.print("\nWysłano zapytanie do slave'a o adresie 0x");
  Serial.print(currentAddress, HEX);
  Serial.println(":");
  printFrame("Zapytanie", frame, 8);

  idx = 0;
  lastRxTime = millis();
}

void setup() {
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);

  rs485Serial.begin(9600, SERIAL_8N1, 16, 17);  // GPIO 16 = RX, 17 = TX
  Serial.begin(115200);
  while (!Serial);

  Serial.println("RS485 master - start");
  Serial.println("Wpisz 1 lub 2 i naciśnij Enter, aby wysłać zapytanie.");
}

void loop() {
  // Czytanie z RS485
  while (rs485Serial.available()) {
    if (idx < sizeof(response)) {
      response[idx++] = rs485Serial.read();
      lastRxTime = millis();
    }
  }

  // Przetwarzanie odpowiedzi
  if (idx > 0 && (millis() - lastRxTime) > FRAME_TIMEOUT) {
    printFrame("Odpowiedź", response, idx);

    if (idx >= 5) {
      uint16_t crc = modbus_crc(response, idx - 2);
      uint16_t recvCrc = response[idx - 2] | (response[idx - 1] << 8);

      if (crc == recvCrc && response[0] == currentAddress && response[1] == 0x03) {
        Serial.println("✅ Poprawna odpowiedź:");
        uint8_t byteCount = response[2];
        for (uint8_t i = 0; i < byteCount; i += 2) {
          uint16_t value = (response[3 + i] << 8) | response[4 + i];
          Serial.print("  Rejestr ");
          Serial.print(i / 2);
          Serial.print(": ");
          Serial.println(value);
        }
      } else {
        Serial.println("❌ Błędne CRC lub niepoprawna odpowiedź.");
      }
    } else {
      Serial.println("⚠️ Zbyt krótka odpowiedź.");
    }

    idx = 0;
  }

  // Odczyt danych z klawiatury (monitora szeregowego)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "1" || input == "2") {
      uint8_t addr = input.toInt();
      sendRequest(addr);
    } else {
      Serial.println("❌ Wpisz tylko '1' lub '2' i naciśnij Enter.");
    }
  }
}
