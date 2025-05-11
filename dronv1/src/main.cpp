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

// --- Стани для машини станів лідара ---
enum LidarState {
  WAITING_FOR_HEADER1,
  WAITING_FOR_HEADER2,
  READING_DATA,
  READING_CHECKSUM
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
LidarState lidarState = WAITING_FOR_HEADER1;
uint16_t distance = 0;
uint16_t strength = 0;
uint8_t lidarBuffer[9];
uint8_t lidarIndex = 0;

// --- Змінні для уникнення перешкод ---
unsigned long obstacleDetectedTime = 0;
bool isObstacleConfirmed = false;
const unsigned long OBSTACLE_CONFIRM_DELAY = 50;
bool obstacleHandled = false;
unsigned long obstacleResetTime = 0;
const unsigned long OBSTACLE_RESET_DELAY = 2000;

// --- Сервоприводи ---
const int NUM_SERVOS = 6;
Servo servos[NUM_SERVOS];
int servoAngleRest = 0;
int servoAngleActive = 90;
int currentServoIndex = 0;
bool servoActivated[NUM_SERVOS] = {false, false, false, false, false, false};
bool triggerFlag = false;
unsigned long lastTriggerTime = 0;
const unsigned long TRIGGER_DEBOUNCE = 500;

// ====================== ФУНКЦІЇ ======================

// --- Визначення режиму з гістерезисом ---
SwitchMode getSwitchMode(uint16_t value) {
  static SwitchMode lastMode = MODE_UNKNOWN;
  if (value < LOW_THRESHOLD + DEAD_BAND && lastMode != MODE_OFF) {
    lastMode = MODE_OFF;
  } else if (value > HIGH_THRESHOLD - DEAD_BAND && lastMode != MODE_MANUAL) {
    lastMode = MODE_MANUAL;
  } else if (value > MID_THRESHOLD - DEAD_BAND && value < MID_THRESHOLD + DEAD_BAND && lastMode != MODE_AUTO) {
    lastMode = MODE_AUTO;
  }
  return lastMode;
}

// --- Визначення стану додаткового перемикача ---
AuxSwitchState getAuxSwitchState(uint16_t value) {
  return (value < AUX_SWITCH_THRESHOLD) ? AUX_OFF : AUX_ON;
}

// --- Ініціалізація сервоприводів ---
void initializeServos() {
  const int servoPins[NUM_SERVOS] = {12, 13, 14, 15, 16, 17}; // Перевірені PWM піни для ESP32
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i], 500, 2500); // Налаштування PWM для сервоприводів
    servos[i].write(servoAngleRest);
    servoActivated[i] = false;
    Serial.print("Servo ");
    Serial.print(i + 1);
    Serial.print(" initialized on pin ");
    Serial.println(servoPins[i]);
  }
}

// --- Активація наступного сервоприводу ---
void activateNextServo() {
  if (currentServoIndex < NUM_SERVOS && !servoActivated[currentServoIndex]) {
    servos[currentServoIndex].write(servoAngleActive);
    servoActivated[currentServoIndex] = true;
    Serial.print("Activated servo #");
    Serial.println(currentServoIndex + 1);
    currentServoIndex++;
  } else {
    Serial.println("All servos have been activated already.");
  }
}

// --- Скидання всіх сервоприводів ---
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
  if (currentMode == MODE_MANUAL) {
    unsigned long currentTime = millis();
    if (auxState == AUX_ON && lastAuxState == AUX_OFF && (currentTime - lastTriggerTime >= TRIGGER_DEBOUNCE)) {
      activateNextServo();
      lastTriggerTime = currentTime;
      triggerFlag = true;
    }
    lastAuxState = auxState;
  }
}

// --- Дії для режимів ---
void actionForModeOff() {
  Serial.println("[ACTION] Mode: OFF");
  if (lastMode == MODE_MANUAL || lastMode == MODE_AUTO) {
    resetAllServos();
  }
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
  obstacleHandled = false;
  obstacleResetTime = 0;
  auxState = AUX_OFF;
  lastAuxState = AUX_OFF;
  triggerFlag = false;
}

void actionForModeAuto() {
  Serial.println("[ACTION] Mode: AUTO");
  auxState = AUX_OFF;
  lastAuxState = AUX_OFF;
  triggerFlag = false;
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
  obstacleHandled = false;
  obstacleResetTime = 0;
  if (lastMode == MODE_OFF) {
    resetAllServos();
  }
}

void actionForModeManual() {
  Serial.println("[ACTION] Mode: MANUAL");
  obstacleDetectedTime = 0;
  isObstacleConfirmed = false;
  obstacleHandled = false;
  obstacleResetTime = 0;
  triggerFlag = false;
  lastAuxState = AUX_OFF;
  if (lastMode == MODE_OFF) {
    resetAllServos();
  }
}

// --- Читання даних з лідара через машину станів ---
bool readLidarData() {
  while (lidarSerial.available()) {
    uint8_t byte = lidarSerial.read();
    switch (lidarState) {
      case WAITING_FOR_HEADER1:
        if (byte == 0x59) {
          lidarBuffer[0] = byte;
          lidarState = WAITING_FOR_HEADER2;
        }
        break;
      case WAITING_FOR_HEADER2:
        if (byte == 0x59) {
          lidarBuffer[1] = byte;
          lidarState = READING_DATA;
          lidarIndex = 2;
        } else {
          lidarState = WAITING_FOR_HEADER1;
        }
        break;
      case READING_DATA:
        lidarBuffer[lidarIndex++] = byte;
        if (lidarIndex == 8) {
          lidarState = READING_CHECKSUM;
        }
        break;
      case READING_CHECKSUM:
        lidarBuffer[8] = byte;
        uint8_t calcChecksum = 0;
        for (int i = 0; i < 8; i++) {
          calcChecksum += lidarBuffer[i];
        }
        if (calcChecksum == lidarBuffer[8]) {
          distance = lidarBuffer[2] + (lidarBuffer[3] << 8); // Відстань у см
          strength = lidarBuffer[4] + (lidarBuffer[5] << 8);
          lidarState = WAITING_FOR_HEADER1;
          return true;
        }
        lidarState = WAITING_FOR_HEADER1;
        break;
    }
  }
  return false;
}

// --- Обробка MSP ---
void processMSP() {
  if (!msp.request(MSP_RC_CHANNEL, &rcChannels)) {
    Serial.println("MSP RC_CHANNEL request failed.");
    return;
  }
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
        case MODE_OFF: Serial.println("OFF"); actionForModeOff(); break;
        case MODE_AUTO: Serial.println("AUTO"); actionForModeAuto(); break;
        case MODE_MANUAL: Serial.println("MANUAL"); actionForModeManual(); break;
        default: Serial.println("UNKNOWN"); break;
      }
    }
  }
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
  if (obstacleHandled) {
    if (millis() - obstacleResetTime >= OBSTACLE_RESET_DELAY) {
      obstacleHandled = false;
      obstacleResetTime = 0;
      obstacleDetectedTime = 0;
      isObstacleConfirmed = false;
      Serial.println("Reset obstacle detection - ready for new obstacle");
    }
    return;
  }
  if (readLidarData()) {
    float distanceM = distance / 100.0; // Переведення в метри
    Serial.print("Distance: ");
    Serial.print(distanceM);
    Serial.print(" m | Strength: ");
    Serial.println(strength);
    if (distanceM <= OBSTACLE_DISTANCE) {
      if (obstacleDetectedTime == 0) {
        obstacleDetectedTime = millis();
      } else if (!isObstacleConfirmed && (millis() - obstacleDetectedTime >= OBSTACLE_CONFIRM_DELAY)) {
        isObstacleConfirmed = true;
        obstacleHandled = true;
        obstacleResetTime = millis();
        Serial.println("Obstacle detected! Activating next servo...");
        activateNextServo();
      }
    } else if (!isObstacleConfirmed) {
      obstacleDetectedTime = 0;
    }
  }
}

// ====================== SETUP & LOOP ======================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32 Sequential One-Time Servo Control System");
  SerialPort.begin(SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  msp.begin(SerialPort);
  initializeServos();
  lidarSerial.begin(LIDAR_BAUDRATE);
  delay(1000);
  Serial.println("System ready. Operating modes:");
  Serial.println("- OFF: All servos reset");
  Serial.println("- AUTO: Each obstacle activates one servo with reset delay");
  Serial.println("- MANUAL: Toggle AUX switch from OFF to ON to activate servos sequentially");
}

void loop() {
  processMSP();
  if (currentMode == MODE_MANUAL) {
    processAuxSwitch();
  }
  if (currentMode == MODE_AUTO) {
    processLidarDataInAutoMode();
  }
  // Затримка прибрана для підвищення швидкодії
}