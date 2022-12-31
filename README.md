# RGB_BUZZER Project


#### The purpose of this project is to implement RGB and Buzzer, which we implemented using the code snippet below.




```

  
/* USER CODE BEGIN 0 */

void set_rgb(uint32_t red ,uint32_t green , uint32_t blue){
	 htim1.Instance->CCR1 = red;
	 htim1.Instance->CCR2 = green;
	 htim1.Instance->CCR3 = blue;
}

/* USER CODE END 0 */


int main(void)
{

  while (1)
  {
   
	//RGB
	set_rgb (240,50,240);

```

#### The code mentioned above is the code related to RGB and BUZZER project. As the code is clear, we first defined a function called  set_rgb  that specifies three colors, red, green, and blue. This function takes 3 inputs, and at the end, we call this function with three arbitrary values.
It is worth noting that we have 4  HAL_TIM_PWM_Start  channels which we define for 3 colors and a BUZZER.
The following code is related to the BUZZER part, in this part we first set the BUZZER pin and then define a variable and set its initial value to 0   And in the WHILE loop, we give the variable value to channel 4, which is related to BUZZER.

```
HAL_GPIO_WritePin(GPIOE, BUZZER_Pin,GPIO_PIN_SET);  
  uint8_t value = 0; 
while (value<255)
{
    htim1.Instance->CCR4 = value; 
    value += 20; 
    HAL_Delay (500); 
}
  value = 0; 
  
