#ifndef CONSTANTS_H
#define CONSTANTS_H

// --- UART налаштування ---
#define RX_PIN 16
#define TX_PIN 17
#define SERIAL_BAUD 115200

// --- MSP команда ---
#define MSP_RC_CHANNEL 0x0200

// --- Тумблер налаштування --- (основний)
#define SWITCH_CHANNEL 5
#define LOW_THRESHOLD 1100
#define MID_THRESHOLD 1500
#define HIGH_THRESHOLD 1900
#define DEAD_BAND 150

// --- Додатковий тумблер ---
#define AUX_SWITCH_CHANNEL 6  // Канал для додаткового перемикача
#define AUX_SWITCH_THRESHOLD 1500  // Поріг для розділення ON/OFF

// --- Лідар TF02-Pro ---
#define LIDAR_RX 2
#define LIDAR_TX 3
#define LIDAR_BAUDRATE 115200
#define OBSTACLE_DISTANCE 25.0  // Відстань до перешкоди в метрах

// --- Сервоприводи ---
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SERVO4_PIN 12
#define SERVO5_PIN 13
#define SERVO6_PIN 14

#define SERVO_REST_ANGLE 0     // Кут спокою для сервоприводів
#define SERVO_ACTIVE_ANGLE 90  // Кут активації для сервоприводів

// --- Тригер --- 
#define TRIGGER_DEBOUNCE_TIME 500  // Захист від брязкоту в мс

#endif // CONSTANTS_H