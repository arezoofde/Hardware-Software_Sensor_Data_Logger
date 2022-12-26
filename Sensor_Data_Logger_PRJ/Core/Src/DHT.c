#include "DHT.h"

#define lineDown() 		HAL_GPIO_WritePin(sensor->DHT_Port, sensor->DHT_Pin, GPIO_PIN_RESET)
#define lineUp()		HAL_GPIO_WritePin(sensor->DHT_Port, sensor->DHT_Pin, GPIO_PIN_SET)
#define getLine()		(HAL_GPIO_ReadPin(sensor->DHT_Port, sensor->DHT_Pin) == GPIO_PIN_SET)
#define Delay(d)		HAL_Delay(d)

static void goToOutput(DHT_sensor *sensor) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  //By default the line is high
  lineUp();

  //Setting the port to exit
  GPIO_InitStruct.Pin = sensor->DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; 	//open drain
  GPIO_InitStruct.Pull = sensor->pullUp;		//Pull-up to nutrition

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //High port speed
  HAL_GPIO_Init(sensor->DHT_Port, &GPIO_InitStruct);
}

static void goToInput(DHT_sensor *sensor) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //Input port setting
  GPIO_InitStruct.Pin = sensor->DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = sensor->pullUp;		//Pull-up to nutrition
  HAL_GPIO_Init(sensor->DHT_Port, &GPIO_InitStruct);
}

DHT_data DHT_getData(DHT_sensor *sensor) {
	DHT_data data = {-128.0f, -128.0f};
	
	#if DHT_POLLING_CONTROL == 1
	/* Limitation on the frequency of polling the sensor */
	//Determining the polling interval depending on the sensor
	uint16_t pollingInterval;
	if (sensor->type == DHT11) {
		pollingInterval = DHT_POLLING_INTERVAL_DHT11;
	} else {
		pollingInterval = DHT_POLLING_INTERVAL_DHT22;
	}

	//If the interval is small, then return the last known good value
	if ((HAL_GetTick() - sensor->lastPollingTime < pollingInterval) && sensor->lastPollingTime != 0) {
		data.hum = sensor->lastHum;
		data.temp = sensor->lastTemp;
		return data;
	}
	sensor->lastPollingTime = HAL_GetTick()+1;
	#endif

	/* Requesting data from the sensor */
	//Translation of the pin "to the exit"
	goToOutput(sensor);
	//Data line down 18ms
	lineDown();
	Delay(18);
	//Lifting the line, transferring the port "to the input"
	lineUp();
	goToInput(sensor);


	#ifdef DHT_IRQ_CONTROL
	//Turn off interrupts so that nothing interferes with data processing
	__disable_irq();
	#endif
	/* Waiting for a response from the sensor */
	uint16_t timeout = 0;
	//Waiting for recession
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) {
			#ifdef DHT_IRQ_CONTROL
			__enable_irq();
			#endif
			//If the sensor did not respond, then it definitely does not exist.
			//Resetting the Last Known Good Value to
			//don't get phantom values
			sensor->lastHum = -128.0f;
			sensor->lastTemp = -128.0f;

			return data;
		}
	}
	timeout = 0;
	//Waiting for the rise
	while(!getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) {
			#ifdef DHT_IRQ_CONTROL
			__enable_irq();
			#endif
			//If the sensor did not respond, then it definitely does not exist.
			//Resetting the Last Known Good Value to
			//don't get phantom values
			sensor->lastHum = -128.0f;
			sensor->lastTemp = -128.0f;

			return data;
		}
	}
	timeout = 0;
	//Waiting for recession
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) {
			#ifdef DHT_IRQ_CONTROL
			__enable_irq();
			#endif
			return data;
		}
	}
	
	/* Reading the response from the sensor */
	uint8_t rawData[5] = {0,0,0,0,0};
	for(uint8_t a = 0; a < 5; a++) {
		for(uint8_t b = 7; b != 255; b--) {
			uint16_t hT = 0, lT = 0;
			//While the line is low, the lT variable increment
			while(!getLine() && lT != 65535) lT++;
			//While the line is high, hT variable increment
			timeout = 0;
			while(getLine()&& hT != 65535) hT++;
			//If hT is greater than lT, then one has arrived
			if(hT > lT) rawData[a] |= (1<<b);
		}
	}

    #ifdef DHT_IRQ_CONTROL
	//Enabling interrupts after receiving data
	__enable_irq();
    #endif

	/* Data integrity check */
	if((uint8_t)(rawData[0] + rawData[1] + rawData[2] + rawData[3]) == rawData[4]) {
		//If the checksum matches, then convert and return the resulting values
		if (sensor->type == DHT22) {
			data.hum = (float)(((uint16_t)rawData[0]<<8) | rawData[1])*0.1f;
			//Check for negative temperature
			if(!(rawData[2] & (1<<7))) {
				data.temp = (float)(((uint16_t)rawData[2]<<8) | rawData[3])*0.1f;
			}	else {
				rawData[2] &= ~(1<<7);
				data.temp = (float)(((uint16_t)rawData[2]<<8) | rawData[3])*-0.1f;
			}
		}
		if (sensor->type == DHT11) {
			data.hum = (float)rawData[0];
			data.temp = (float)rawData[2];
		}
	}
	
	#if DHT_POLLING_CONTROL == 1
	sensor->lastHum = data.hum;
	sensor->lastTemp = data.temp;
	#endif

	return data;	
}
