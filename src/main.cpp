#include <Arduino.h>
#include <SoftwareSerial.h>

#define RE_PIN 2
#define DE_PIN 3

SoftwareSerial rs485Serial(10, 11); // RX, TX

#define FRAME_MAX_SIZE 32   // maksymalny rozmiar ramki
#define FRAME_TIMEOUT 50    // timeout w ms na koniec ramki

uint8_t buffer[FRAME_MAX_SIZE];
uint8_t idx = 0;
unsigned long lastRxTime = 0;

void setup() {
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);

  digitalWrite(RE_PIN, LOW); // tryb odbioru
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
      // Bufor pełny, możesz tu dodać obsługę błędu lub wyczyścić bufor
    }
  }

  if (idx > 0 && (millis() - lastRxTime) > FRAME_TIMEOUT) {
    Serial.print("Odebrano: ");
    for (uint8_t i = 0; i < idx; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    idx = 0; // resetujemy bufor na kolejną ramkę
  }
}
