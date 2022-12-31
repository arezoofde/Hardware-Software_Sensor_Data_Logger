--> DHT11 Project <--

ّA brief explanation about the DHT11 sensor that we use in our project:

DHT11 Temperature & Humidity Sensor features a temperature & humidity sensor complex with a calibrated digital signal output. 
By using the exclusive digital-signal-acquisition technique and temperature & humidity sensing technology, it ensures high reliability and excellent long-term stability. 
This sensor includes a resistive-type humidity measurement component and an NTC temperature measurement component, and connects to a high performance microcontroller, offering excellent quality, fast response, anti-interference ability and cost-effectiveness.

In this project, we used "dht.h" and "lcd.h" libraries, which are respectively related to the necessary functions to work with the DHT sensor and the LCD device.

First of all, we have specified the necessary features to set up the sensor. These features include information about :
- the port and pin number to which the sensor is connected
- type of DHT sensor used in this project (which is DHT11)
- there is no need to pull data line to power
- some other features we prefer use as default

Then it's time to configure LCD device to be ready for presenting the results got from sensor.lcd_init() initializes the LCD controller and put it ready to work.lcd_clear() clears whole screen of LCD.

The important part is get data from DHT11 sensor.DHT11_getData() function (which is defined properly in DHT.c file) returns two values.these two values are humidity and temperature.
through lcd_puts() send humidity value and temperature value to LCD to represent in specified coordinates we define.
