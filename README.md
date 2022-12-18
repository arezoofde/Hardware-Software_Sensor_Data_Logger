# Sensor Data Logger
Sensor Data Logger is a dashboard for your device sensors. It plots charts that show the values of selected sensors in real-time, even from connected Android Wear devices.

You can get the app from the [Play Store](https://play.google.com/store/apps/details?id=net.steppschuh.sensordatalogger) or install the latest [apk file](https://github.com/Steppschuh/Sensor-Data-Logger/tree/master/Releases).


![Screencast](https://raw.githubusercontent.com/Steppschuh/Sensor-Data-Logger/master/Media/Screencasts/sensor_data_bw_long_500.gif)



### Tested sensor types
- Humidity
- Light RGB
- Temperature

### Data Lagger 
| Project Name    | Project Details                                                               | 
|-----------------|-------------------------------------------------------------------------------|
| KEYPAD _RTC     | Password Login + Data and Time                                                |
| RGB_Buzzer      | Light Color Control + Alarm Sound                                             |
| DHT11           | Temperature and Humidity Show                                                 |
| LCD16x2         | Show Data & Time + Temp & Hum + RS232 Data + server connection + setPassword + LED + Push BTN |
| ESP8266         | Send Data to Server                                                           |
| RS232           | Send UART Data to ESP8266                                                     |
| SDCARD          | Save Packet in SDCARD                                                         |

### MCU PIN CONFIG
|  pinout | sensor | Description |
|---------|--------|-------------|
| PC0 | LED1 | GREEN LED |
| PC2 | LED2 | YELLOW LEd |
|PC3 | LED3 | RED LED |
|PA5 | PBTN1 | Push Button |
|PA3 | PBTN2 | Push Button |
|PA4 | PBTN3 | Push Button |
|PE9 | RGB | RED in RGB |
|PE11| RGB | GREEN in RGB |
|PE13 | RGB | BLUE in RGB |
|PE14 | Buzzer | Alarm |
|PD8 | LCD | RS pin |
|PD9 | LCD | EN pin |
|PD10 | LCD | D4 pin |
|PD11 | LCD | D5 pin |
|PD12 | LCD | D6 pin |
|PD13 | LCD | D7 pin |
|PD14 | DHT11 |  Show Temperatre & Humidity |
|PF2 | KEYPAD | R0 pin |
|PF3 | KEYPAD  | R1 pin |
|PF4 | KEYPAD  | R2 pin |
|PF5 | KEYPAD  | R3 pin |
|PF6 | KEYPAD  | C0 pin |
|PF7 | KEYPAD  | C1 pin |
|PF8 | KEYPAD  | C2 pin |
|PF9 | KEYPAD  | C3 pin |
|PA9 | ESP8266 | UART TX |
|PA10 | ESP8266 | UART RX |


