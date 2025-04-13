#include <Arduino.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include "MSP.h"

// --- UART налаштування ---
#define RX_PIN 16
#define TX_PIN 17
#define SERIAL_BAUD 115200

// --- MSP команда ---
#define MSP_RC_CHANNEL 0x0200

// --- Тумблер налаштування ---
#define SWITCH_CHANNEL 5
#define LOW_THRESHOLD 1100
#define MID_THRESHOLD 1500
#define HIGH_THRESHOLD 1900
#define DEAD_BAND 150

// --- Лідар TF02-Pro ---
#define LIDAR_RX 2
#define LIDAR_TX 3
#define LIDAR_BAUDRATE 115200
#define OBSTACLE_DISTANCE 25.0  // Відстань до перешкоди в метрах

// --- Сервопривід ---
#define SERVO_PIN 9
#define DEFAULT_ANGLE 0     // Початковий кут сервоприводу
#define AVOID_ANGLE 90      // Кут для уникнення перешкоди

// --- Стани перемикача ---
enum SwitchMode {
  MODE_OFF,
  MODE_AUTO,
  MODE_MANUAL,
  MODE_UNKNOWN
};

// --- Глобальні змінні ---
SwitchMode currentMode = MODE_UNKNOWN;
SwitchMode lastMode = MODE_UNKNOWN;
MSP msp;
HardwareSerial SerialPort(2);
RCChannels rcChannels;

// --- Лідар ---
Servo steeringServo;
SoftwareSerial lidarSerial(LIDAR_RX, LIDAR_TX);
uint16_t distance = 0;
uint16_t strength = 0;
uint8_t checksum = 0;
bool frameStarted = false;
uint8_t frameCounter = 0;
uint8_t dataBuffer[9];

// ====================== ФУНКЦІЇ ======================

// --- Визначення режиму ---
SwitchMode getSwitchMode(uint16_t value) {
  if (value < (LOW_THRESHOLD + DEAD_BAND)) {
    return MODE_OFF;
  } else if (value > (HIGH_THRESHOLD - DEAD_BAND)) {
    return MODE_MANUAL;
  } else if (value > (MID_THRESHOLD - DEAD_BAND) && value < (MID_THRESHOLD + DEAD_BAND)) {
    return MODE_AUTO;
  }
  return MODE_UNKNOWN;
}

// --- Дії для кожного режиму ---
void actionForModeOff() {
  Serial.println("[ACTION] Mode: OFF");
  steeringServo.write(DEFAULT_ANGLE);  // Сервопривід у вихідне положення
}

void actionForModeAuto() {
  Serial.println("[ACTION] Mode: AUTO");
  // Автоматичне уникнення перешкод (реалізовано в loop())
}

void actionForModeManual() {
  Serial.println("[ACTION] Mode: MANUAL");
  // Керування вручну (серво не керується автоматично)
}

// --- Читання даних з лідара ---
bool readLidarData() {
  while (lidarSerial.available() >= 9) {
    uint8_t currentByte = lidarSerial.read();
    
    // Перевірка початку кадру (0x59, 0x59)
    if (!frameStarted) {
      if (currentByte == 0x59) {
        dataBuffer[0] = currentByte;
        frameCounter = 1;
        frameStarted = true;
      }
    } else if (frameCounter == 1) {
      if (currentByte == 0x59) {
        dataBuffer[1] = currentByte;
        frameCounter = 2;
      } else {
        frameStarted = false;
      }
    } else {
      // Зберігаємо байти даних
      dataBuffer[frameCounter] = currentByte;
      frameCounter++;
      
      // Якщо отримали всі 9 байтів
      if (frameCounter == 9) {
        frameStarted = false;
        
        // Обчислення контрольної суми
        uint8_t calculatedChecksum = 0;
        for (int i = 0; i < 8; i++) {
          calculatedChecksum += dataBuffer[i];
        }
        
        // Перевірка контрольної суми
        if (calculatedChecksum == dataBuffer[8]) {
          // Розбір даних лідара
          distance = dataBuffer[2] + (dataBuffer[3] << 8);  // Відстань в см
          strength = dataBuffer[4] + (dataBuffer[5] << 8);  // Сила сигналу
          return true;
        }
      }
    }
  }
  return false;
}

// --- Обробка MSP ---
void processMSP() {
  if (msp.request(MSP_RC_CHANNEL, &rcChannels)) {
    if (rcChannels.channelCount > SWITCH_CHANNEL) {
      uint16_t switchValue = rcChannels.channels[SWITCH_CHANNEL];
      SwitchMode newMode = getSwitchMode(switchValue);
      if (newMode != currentMode) {
        currentMode = newMode;
        Serial.print("Switch value: ");
        Serial.print(switchValue);
        Serial.print(" | New mode: ");
        switch (currentMode) {
          case MODE_OFF:
            Serial.println("OFF");
            actionForModeOff();
            break;
          case MODE_AUTO:
            Serial.println("AUTO");
            actionForModeAuto();
            break;
          case MODE_MANUAL:
            Serial.println("MANUAL");
            actionForModeManual();
            break;
          default:
            Serial.println("UNKNOWN");
            break;
        }
        lastMode = currentMode;
      } else if (currentMode == MODE_AUTO) {
        actionForModeAuto();
      }
    }
  } else {
    Serial.println("MSP RC_CHANNEL request failed.");
  }
}

// ====================== SETUP & LOOP ======================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32 switch controller: OFF / AUTO / MANUAL");
  SerialPort.begin(SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  msp.begin(SerialPort);

  // Ініціалізація сервоприводу
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(DEFAULT_ANGLE);

  // Ініціалізація лідара
  lidarSerial.begin(LIDAR_BAUDRATE);

  delay(1000);
  Serial.println("System ready.");
}

void loop() {
  processMSP();  // Обробка режимів (OFF/AUTO/MANUAL)

  // Якщо режим AUTO - обробка даних лідара
  if (currentMode == MODE_AUTO) {
    if (readLidarData()) {
      Serial.print("Distance: ");
      Serial.print(distance / 100.0);
      Serial.print(" m | Strength: ");
      Serial.println(strength);

      // Перевірка на перешкоду
      if ((distance / 100.0) <= OBSTACLE_DISTANCE) {
        Serial.println("Obstacle detected! Avoiding...");
        steeringServo.write(AVOID_ANGLE);
      } else {
        steeringServo.write(DEFAULT_ANGLE);
      }
    }
  }

  delay(50);
}