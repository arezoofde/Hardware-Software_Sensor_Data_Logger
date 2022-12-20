#ifndef DHT_H_
#define DHT_H_

#include "main.h"

/* Settings */
#define DHT_TIMEOUT 				10000	//The number of iterations after which the function returns empty values
#define DHT_POLLING_CONTROL			1		//Enable sensor polling rate check
#define DHT_POLLING_INTERVAL_DHT11	2000	//DHT11 polling interval (0.5 Hz according to datasheet). You can put 1500, it will work
#define DHT_POLLING_INTERVAL_DHT22	1000	//DHT22 polling interval (1 Hz according to datasheet)
#define DHT_IRQ_CONTROL						//Disable interrupts while communicating with the sensor
/* The structure of the data returned by the sensor */
typedef struct {
	float hum;
	float temp;
} DHT_data;

/* Type of sensor used */
typedef enum {
	DHT11,
	DHT22
} DHT_type;

/* Sensor object structure */
typedef struct {
	GPIO_TypeDef *DHT_Port;	//Sensor port (GPIOA, GPIOB, etc)
	uint16_t DHT_Pin;		//Sensor pin number (GPIO_PIN_0, GPIO_PIN_1, etc)
	DHT_type type;			//Sensor type (DHT11 or DHT22)
	uint8_t pullUp;			//Do you need to pull the data line to power (GPIO_NOPULL - no, GPIO_PULLUP - yes)

	//Sensor polling frequency control. Do not fill in the values!
	#if DHT_POLLING_CONTROL == 1
	uint32_t lastPollingTime;//Time of the last sensor poll
	float lastTemp;			 //Last temperature value
	float lastHum;			 //Last humidity value
	#endif
} DHT_sensor;


/*Function Prototypes */
DHT_data DHT_getData(DHT_sensor *sensor); //Get data from the sensor

#endif
