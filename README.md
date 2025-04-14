ціна лідару зараза в україні 6000
https://mikronika.net/product/benewake-lidar-tf02-pro?srsltid=AfmBOopplgaCw6pXc5RinPFA9U05KqWNsWkvzrWxoPub1LURgUQXNRYN

## Основні компоненти системи

1. **ESP32 мікроконтролер**: 
   * Центральний елемент системи
   * Координує роботу всіх компонентів

2. **Лідар TF02-Pro**: 
   * Датчик відстані на основі лазерного променя
   * Підключений через програмний UART (SoftwareSerial)
   * Піни підключення: 2 (RX) і 3 (TX)

3. **Сервопривід**: 
   * Виконавчий механізм для зміни напрямку руху
   * Підключений до піну 9

4. **MSP інтерфейс**: 
   * Протокол зв'язку для імітації команд пульта керування
   * Використовує апаратний UART на пінах 16 (RX) і 17 (TX)

## Режими роботи

Система підтримує три режими роботи, що перемикаються через віртуальний тумблер в класі MSP:

| Режим | Опис |
|-------|------|
| **MODE_OFF** | Система вимкнена, сервопривід у вихідному положенні |
| **MODE_AUTO** | Автоматичне уникнення перешкод на основі даних лідара |
| **MODE_MANUAL** | Ручне керування без автоматичного уникнення перешкод |

## Особливості алгоритму уникнення перешкод

Алгоритм включає механізм підтвердження для запобігання хибних спрацьовувань:

1. При першому виявленні перешкоди (відстань < `OBSTACLE_DISTANCE` = 25 метрів), система запам'ятовує час виявлення
2. Якщо перешкода фіксується протягом визначеного часу (`OBSTACLE_CONFIRM_DELAY` = 50 мс), система:
   * Підтверджує наявність перешкоди
   * Активує сервопривід для уникнення (поворот на кут `AVOID_ANGLE` = 90 градусів)
3. При зникненні перешкоди система повертає сервопривід у початкове положення

## Обробка даних з лідара

Лідар TF02-Pro передає дані через UART у специфічному форматі кадрів:
- Початок кадру: два байти-маркери (0x59, 0x59)
- Далі йдуть дані про відстань, силу сигналу та інші параметри

### Функція `readLidarData()` реалізує:

```
1. Пошук початку кадру (два байти 0x59)
2. Збір даних кадру в буфер
3. Перевірку контрольної суми для підтвердження цілісності даних
4. Вилучення інформації про відстань та силу сигналу
```

## Деталі реалізації MSP

Клас MSP імітує протокол Multiwii Serial Protocol:
- Використовується в дронах та RC-моделях
- У поточній реалізації симулює перемикання між трьома режимами з періодом 5 секунд (демонстраційна функція)

## Налаштування апаратної частини

| Компонент | Параметри |
|-----------|-----------|
| Послідовний порт | 115200 бод |
| Лідар | RX=2, TX=3 |
| UART для MSP | RX=16, TX=17 |
| Сервопривід | Пін 9 |
