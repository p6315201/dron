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

// --- Стани додаткового перемикача ---
enum AuxSwitchState {
  AUX_OFF,
  AUX_ON
};

// --- Глобальні змінні ---
SwitchMode currentMode = MODE_UNKNOWN;
SwitchMode lastMode = MODE_UNKNOWN;
AuxSwitchState auxState = AUX_OFF;
AuxSwitchState lastAuxState = AUX_OFF;
MSP msp;
HardwareSerial SerialPort(2);
RCChannels rcChannels;

// --- Лідар ---
SoftwareSerial lidarSerial(LIDAR_RX, LIDAR_TX);
uint16_t distance = 0;
uint16_t strength = 0;
uint8_t checksum = 0;
bool frameStarted = false;
uint8_t frameCounter = 0;
uint8_t dataBuffer[9];

// --- Додаткові змінні для уникнення перешкод ---
unsigned long obstacleDetectedTime = 0;
bool isObstacleConfirmed = false;
const unsigned long OBSTACLE_CONFIRM_DELAY = 50;
bool obstacleHandled = false;  // Прапорець для відстеження обробленої перешкоди
unsigned long obstacleResetTime = 0;
const unsigned long OBSTACLE_RESET_DELAY = 2000;  // Затримка перед можливістю нового виявлення (2 секунди)

// --- Сервоприводи ---
const int NUM_SERVOS = 6;
Servo servos[NUM_SERVOS];
int servoAngleRest = 0;
int servoAngleActive = 90;
int currentServoIndex = 0;
bool servoActivated[NUM_SERVOS] = {false, false, false, false, false, false}; // Відслідковування активованих сервоприводів
bool triggerFlag = false;
unsigned long lastTriggerTime = 0;
const unsigned long TRIGGER_DEBOUNCE = 500;

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

// --- Визначення стану додаткового перемикача (ON/OFF) ---
AuxSwitchState getAuxSwitchState(uint16_t value) {
  if (value < AUX_SWITCH_THRESHOLD) {
    return AUX_OFF;
  } else {
    return AUX_ON;
  }
}

// --- Ініціалізація усіх сервоприводів ---
void initializeServos() {
  const int servoPins[NUM_SERVOS] = {9, 10, 11, 12, 13, 14};
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(servoAngleRest); // Початкове положення
    servoActivated[i] = false;
    Serial.print("Servo ");
    Serial.print(i+1);
    Serial.print(" initialized on pin ");
    Serial.println(servoPins[i]);
  }
}

// --- Активація наступного сервоприводу ---
void activateNextServo() {
  // Перевіряємо чи є ще неактивовані сервоприводи
  if (currentServoIndex < NUM_SERVOS && !servoActivated[currentServoIndex]) {
    // Активуємо поточний сервопривід
    servos[currentServoIndex].write(servoAngleActive);
    servoActivated[currentServoIndex] = true;
    
    Serial.print("Activated servo #");
    Serial.println(currentServoIndex + 1);
    
    // Переходимо до наступного сервоприводу
    currentServoIndex++;
  } else {
    Serial.println("All servos have been activated already.");
  }
}

// --- Скидання всіх сервоприводів для нового циклу ---
void resetAllServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(servoAngleRest);
    servoActivated[i] = false;
  }
  currentServoIndex = 0;
  Serial.println("All servos reset to initial position.");
}

// --- Обробка стану додаткового перемикача ---
void processAuxSwitch() {
  // В режимі MANUAL тільки коли додатковий перемикач в ON - активувати сервоприводи
  if (currentMode == MODE_MANUAL) {
    unsigned long currentTime = millis();
    
    // Якщо перемикач переходить з OFF в ON, активуємо наступний сервопривід
    if (auxState == AUX_ON && lastAuxState == AUX_OFF) {
      activateNextServo();
      lastTriggerTime = currentTime;
      triggerFlag = true;
    }
    
    // Оновлюємо стан попереднього значення для відстеження переходів
    lastAuxState = auxState;
  }
}

// --- Дії для кожного режиму основного перемикача ---
void actionForModeOff() {
  Serial.println("[ACTION] Mode: OFF");
  
  // Скидаємо всі сервоприводи (тільки при зміні режиму з MANUAL або AUTO)
  if (lastMode == MODE_MANUAL || lastMode == MODE_AUTO) {
    resetAllServos();
  }
  
  // Скидаємо змінні відстеження перешкод
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
  obstacleHandled = false;
  obstacleResetTime = 0;
  
  // Скидаємо стан додаткового перемикача
  auxState = AUX_OFF;
  lastAuxState = AUX_OFF;
  triggerFlag = false;
}

void actionForModeAuto() {
  Serial.println("[ACTION] Mode: AUTO");
  
  // Скидаємо стан додаткового перемикача
  auxState = AUX_OFF;
  lastAuxState = AUX_OFF;
  triggerFlag = false;
  
  // Скидаємо змінні відстеження перешкод
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
  obstacleHandled = false;
  obstacleResetTime = 0;
  
  // При переході з OFF в AUTO, скидаємо сервоприводи
  if (lastMode == MODE_OFF) {
    resetAllServos();
  }
}

void actionForModeManual() {
  Serial.println("[ACTION] Mode: MANUAL");
  
  // Скидаємо змінні відстеження перешкод
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
  obstacleHandled = false;
  obstacleResetTime = 0;
  
  // Скидаємо тригер для послідовної активації
  triggerFlag = false;
  lastAuxState = AUX_OFF;  // Додано для коректного відстеження переходу
  
  // При переході з OFF в MANUAL, скидаємо сервоприводи
  if (lastMode == MODE_OFF) {
    resetAllServos();
  }
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
    
    if (calculatedChecksum != dataBuffer[8]) {
      continue;
    }
    
    // Розбір даних лідара
    distance = dataBuffer[2] + (dataBuffer[3] << 8);
    strength = dataBuffer[4] + (dataBuffer[5] << 8);
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
  
  // Обробка основного перемикача
  if (rcChannels.channelCount > SWITCH_CHANNEL) {
    uint16_t switchValue = rcChannels.channels[SWITCH_CHANNEL];
    SwitchMode newMode = getSwitchMode(switchValue);
    
    if (newMode != currentMode) {
      lastMode = currentMode;
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
    }
  }
  
  // Обробка додаткового перемикача
  if (rcChannels.channelCount > AUX_SWITCH_CHANNEL) {
    uint16_t auxValue = rcChannels.channels[AUX_SWITCH_CHANNEL];
    AuxSwitchState newAuxState = getAuxSwitchState(auxValue);
    
    if (newAuxState != auxState) {
      auxState = newAuxState;
      Serial.print("Aux Switch value: ");
      Serial.print(auxValue);
      Serial.println(auxState == AUX_ON ? " | ON" : " | OFF");
    }
  }
}

// --- Обробка даних лідара в режимі AUTO ---
void processLidarDataInAutoMode() {
  // Перевіряємо чи пройшов час скидання після останньої обробленої перешкоди
  if (obstacleHandled) {
    if (obstacleResetTime == 0) {
      obstacleResetTime = millis();
    } else if (millis() - obstacleResetTime >= OBSTACLE_RESET_DELAY) {
      // Час скидання минув, скидаємо всі прапорці для відстеження нової перешкоди
      obstacleHandled = false;
      obstacleResetTime = 0;
      obstacleDetectedTime = 0;
      isObstacleConfirmed = false;
      Serial.println("Reset obstacle detection - ready for new obstacle");
    }
    return; // Вихід з функції, якщо перешкода вже оброблена і час скидання ще не минув
  }

  // Зчитуємо дані з лідара
  if (readLidarData()) {
    float distanceM = distance / 100.0;
    
    Serial.print("Distance: ");
    Serial.print(distanceM);
    Serial.print(" m | Strength: ");
    Serial.println(strength);

    // Перевірка на перешкоду
    if (distanceM <= OBSTACLE_DISTANCE) {
      if (obstacleDetectedTime == 0) {
        obstacleDetectedTime = millis();
      }
      else if (!isObstacleConfirmed && (millis() - obstacleDetectedTime >= OBSTACLE_CONFIRM_DELAY)) {
        isObstacleConfirmed = true;
        obstacleHandled = true;  // Позначаємо що перешкода оброблена
        Serial.println("Obstacle detected! Activating next servo...");
        activateNextServo();
      }
    } else {
      // Перешкода зникла до підтвердження - скидаємо таймер
      if (!isObstacleConfirmed) {
        obstacleDetectedTime = 0;
      }
    }
  }
}

// ====================== SETUP & LOOP ======================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32 Sequential One-Time Servo Control System");
  SerialPort.begin(SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  msp.begin(SerialPort);

  // Ініціалізація сервоприводів
  initializeServos();

  // Ініціалізація лідара
  lidarSerial.begin(LIDAR_BAUDRATE);

  delay(1000);
  Serial.println("System ready. Operating modes:");
  Serial.println("- OFF: All servos reset");
  Serial.println("- AUTO: Each obstacle activates one servo with reset delay");
  Serial.println("- MANUAL: Toggle AUX switch from OFF to ON to activate servos sequentially");
}

void loop() {
  processMSP();  // Обробка режимів та додаткового перемикача
  
  // Обробка додаткового перемикача для серво в режимі MANUAL
  if (currentMode == MODE_MANUAL) {
    processAuxSwitch();
  }

  // Якщо режим AUTO - обробка даних лідара
  if (currentMode == MODE_AUTO) {
    processLidarDataInAutoMode();
  }

  delay(50);
}