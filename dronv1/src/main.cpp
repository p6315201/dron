#include <Arduino.h>
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

// --- Функція визначення режиму ---
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
}

void actionForModeAuto() {
  Serial.println("[ACTION] Mode: AUTO");
}

void actionForModeManual() {
  Serial.println("[ACTION] Mode: MANUAL");
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32 switch controller: OFF / AUTO / MANUAL");
  SerialPort.begin(SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  msp.begin(SerialPort);
  delay(1000);
  Serial.println("Ready to receive switch modes.");
}

void loop() {
  processMSP();
  delay(50);
}