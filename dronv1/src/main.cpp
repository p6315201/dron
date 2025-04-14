#include <Arduino.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include "MSP.h"
#include "constants.h"

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

// --- Додаткові змінні для уникнення перешкод ---
unsigned long obstacleDetectedTime = 0;  // Час першого виявлення перешкоди
bool isObstacleConfirmed = false;        // Прапорець підтвердженої перешкоди
const unsigned long OBSTACLE_CONFIRM_DELAY = 50; // Затримка підтвердження в мс

// ====================== ФУНКЦІЇ ======================

// --- Визначення режиму ---
SwitchMode getSwitchMode(uint16_t value) {
  if (value < (LOW_THRESHOLD + DEAD_BAND)) {
    return MODE_OFF;
  } 
  if (value > (HIGH_THRESHOLD - DEAD_BAND)) {
    return MODE_MANUAL;
  } 
  if (value > (MID_THRESHOLD - DEAD_BAND) && value < (MID_THRESHOLD + DEAD_BAND)) {
    return MODE_AUTO;
  }
  return MODE_UNKNOWN;
}

// --- Дії для кожного режиму ---
void actionForModeOff() {
  Serial.println("[ACTION] Mode: OFF");
  steeringServo.write(DEFAULT_ANGLE);  // Сервопривід у вихідне положення
  
  // Скидаємо змінні відстеження перешкод при вимкненні режиму AUTO
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
}

void actionForModeAuto() {
  Serial.println("[ACTION] Mode: AUTO");
  // Автоматичне уникнення перешкод (реалізовано в loop())
}

void actionForModeManual() {
  Serial.println("[ACTION] Mode: MANUAL");
  // Керування вручну (серво не керується автоматично)
  
  // Скидаємо змінні відстеження перешкод при виході з режиму AUTO
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
}

// --- Читання даних з лідара ---
bool readLidarData() {
  if (lidarSerial.available() < 9) {
    return false;
  }

  while (lidarSerial.available() >= 9) {
    uint8_t currentByte = lidarSerial.read();
    
    // Випадок 1: Шукаємо початок кадру
    if (!frameStarted) {
      if (currentByte == 0x59) {
        dataBuffer[0] = currentByte;
        frameCounter = 1;
        frameStarted = true;
      }
      continue;
    }
    
    // Випадок 2: Перевіряємо другий байт заголовка
    if (frameCounter == 1) {
      if (currentByte != 0x59) {
        // Невірний другий байт - скидаємо кадр
        frameStarted = false;
        continue;
      }
      dataBuffer[1] = currentByte;
      frameCounter = 2;
      continue;
    }
    
    // Випадок 3: Збираємо дані кадру
    dataBuffer[frameCounter] = currentByte;
    frameCounter++;
    
    // Перевіряємо чи зібрали повний кадр
    if (frameCounter < 9) {
      continue;
    }
    
    // Повний кадр отримано - обробляємо
    frameStarted = false;
    
    // Обчислення контрольної суми
    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < 8; i++) {
      calculatedChecksum += dataBuffer[i];
    }
    
    // Неправильна контрольна сума
    if (calculatedChecksum != dataBuffer[8]) {
      continue;
    }
    
    // Розбір даних лідара
    distance = dataBuffer[2] + (dataBuffer[3] << 8);  // Відстань в см
    strength = dataBuffer[4] + (dataBuffer[5] << 8);  // Сила сигналу
    return true;
  }
  
  return false;
}

// --- Обробка MSP ---
void processMSP() {
  if (!msp.request(MSP_RC_CHANNEL, &rcChannels)) {
    Serial.println("MSP RC_CHANNEL request failed.");
    return;
  }
  
  if (rcChannels.channelCount <= SWITCH_CHANNEL) {
    return;
  }
  
  uint16_t switchValue = rcChannels.channels[SWITCH_CHANNEL];
  SwitchMode newMode = getSwitchMode(switchValue);
  
  if (newMode == currentMode) {
    if (currentMode == MODE_AUTO) {
      actionForModeAuto();
    }
    return;
  }
  
  // Обробка зміни режиму
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
  if (currentMode != MODE_AUTO) {
    delay(50);
    return;
  }
  
  if (readLidarData()) {
    Serial.print("Distance: ");
    Serial.print(distance / 100.0);
    Serial.print(" m | Strength: ");
    Serial.println(strength);

    // Перевірка на перешкоду з підтвердженням
    if ((distance / 100.0) <= OBSTACLE_DISTANCE) {
      // Якщо перешкода виявлена вперше
      if (obstacleDetectedTime == 0) {
        obstacleDetectedTime = millis();  // Зберегти час першого виявлення
      }
      // Перевірка чи пройшов необхідний час для підтвердження
      else if (!isObstacleConfirmed && (millis() - obstacleDetectedTime >= OBSTACLE_CONFIRM_DELAY)) {
        isObstacleConfirmed = true;
        Serial.println("Obstacle detected! Avoiding...");
        steeringServo.write(AVOID_ANGLE);
      }
    } else {
      // Перешкоди немає - скинути змінні відстеження
      obstacleDetectedTime = 0;
      if (isObstacleConfirmed) {
        isObstacleConfirmed = false;
        steeringServo.write(DEFAULT_ANGLE);
      }
    }
  }

  delay(50);
}