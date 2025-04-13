#include <ESP32Servo.h>
#include <SoftwareSerial.h>

// Налаштування пінів
#define LIDAR_RX 2      // Пін RX для лідара
#define LIDAR_TX 3      // Пін TX для лідара
#define SERVO_PIN 9     // Пін для сервоприводу

// Налаштування лідара TF02-Pro
#define LIDAR_BAUDRATE 115200
#define OBSTACLE_DISTANCE 25.0  // Відстань до перешкоди в метрах

// Налаштування сервоприводу
#define DEFAULT_ANGLE 0     // Початковий кут сервоприводу
#define AVOID_ANGLE 90      // Кут для уникнення перешкоди

// Створення об'єктів
Servo steeringServo;
SoftwareSerial lidarSerial(LIDAR_RX, LIDAR_TX);

// Змінні для зберігання даних лідара
uint16_t distance = 0;
uint16_t strength = 0;
uint8_t checksum = 0;
bool frameStarted = false;
uint8_t frameCounter = 0;
uint8_t dataBuffer[9];

void setup() {
  // Ініціалізація серійного порту для налагодження
  Serial.begin(9600);
  Serial.println("Система автонаведення для дрону з лідаром TF02-Pro");
  
  // Ініціалізація сервоприводу
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(DEFAULT_ANGLE);
  
  // Ініціалізація з'єднання з лідаром
  lidarSerial.begin(LIDAR_BAUDRATE);
  
  delay(500);
}

void loop() {
  // Читання даних з лідара
  if (readLidarData()) {
    // Виведення даних для налагодження
    Serial.print("Відстань: ");
    Serial.print(distance / 100.0); // Перетворення см в метри
    Serial.print(" м, Сила сигналу: ");
    Serial.println(strength);
    
    // Перевірка наявності перешкоди
    if ((distance / 100.0) <= OBSTACLE_DISTANCE) {
      // Перешкода виявлена, повертаємо серво для уникнення
      Serial.println("Перешкода виявлена! Уникаємо...");
      steeringServo.write(AVOID_ANGLE);
    } else {
      // Перешкоди немає, повертаємося до звичайного положення
      steeringServo.write(DEFAULT_ANGLE);
    }
  }
  
  // Невелика затримка для стабільності
  delay(50);
}

// Функція для читання та обробки даних з лідара TF02-Pro
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