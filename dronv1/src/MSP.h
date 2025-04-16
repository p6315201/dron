#ifndef MSP_H
#define MSP_H

#include <Arduino.h>

// Структура для RC каналів
struct RCChannels {
  uint16_t channels[16];
  uint8_t channelCount;
};

class MSP {
public:
  MSP() {};
  
  void begin(Stream &stream) {
    _stream = &stream;
  }
  
  template<class T>
  bool request(uint16_t command, T *payload) {
    // Базова імплементація MSP протоколу
    if (command == 0x0200) { // MSP_RC_CHANNEL
      RCChannels* channels = (RCChannels*)payload;
      channels->channelCount = 16;
      
      // Тестові значення
      for (int i = 0; i < 16; i++) {
        if (i == 5) {
          // Симуляція основного перемикача (OFF/AUTO/MANUAL)
          unsigned long currentTime = millis() / 5000;
          int position = currentTime % 3;
          
          switch (position) {
            case 0: channels->channels[i] = 1000; break; // OFF
            case 1: channels->channels[i] = 1500; break; // AUTO
            case 2: channels->channels[i] = 2000; break; // MANUAL
          }
        } 
        else if (i == 6) {
          // Симуляція додаткового перемикача (OFF/ON)
          unsigned long currentTime = millis() / 7000; // Інший період для різної поведінки
          int position = currentTime % 2;
          
          switch (position) {
            case 0: channels->channels[i] = 1000; break; // OFF
            case 1: channels->channels[i] = 2000; break; // ON
          }
        }
        else {
          channels->channels[i] = 1500;
        }
      }
      return true;
    }
    return false;
  }

private:
  Stream *_stream;
};

#endif