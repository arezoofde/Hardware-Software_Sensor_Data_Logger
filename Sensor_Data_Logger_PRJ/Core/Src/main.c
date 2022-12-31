/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef _WIN32

#include "rs232.h"

#include <stdio.h>
#include <string.h>

#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include "DHT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef int        bool;
#define true    -1
#define false   0

typedef struct {
    int port;
    void * handle;
} COMDevice;

/*****************************************************************************/
#define COM_MAXDEVICES 64
static COMDevice comDevices[COM_MAXDEVICES];
static int noDevices = 0;

#define COM_MINDEVNAME 16384
const char * comPtn = "COM???";

/*****************************************************************************/
const char * findPattern(const char * string, const char * pattern, int * value);
const char * portInternalName(int index);

/*****************************************************************************/
typedef struct _COMMTIMEOUTS {
    uint32_t ReadIntervalTimeout;
    uint32_t ReadTotalTimeoutMultiplier;
    uint32_t ReadTotalTimeoutConstant;
    uint32_t WriteTotalTimeoutMultiplier;
    uint32_t WriteTotalTimeoutConstant;
} COMMTIMEOUTS;

typedef struct _DCB {
    uint32_t DCBlength;
    uint32_t BaudRate;
    uint32_t fBinary  :1;
    uint32_t fParity  :1;
    uint32_t fOutxCtsFlow  :1;
    uint32_t fOutxDsrFlow  :1;
    uint32_t fDtrControl  :2;
    uint32_t fDsrSensitivity  :1;
    uint32_t fTXContinueOnXoff  :1;
    uint32_t fOutX  :1;
    uint32_t fInX  :1;
    uint32_t fErrorChar  :1;
    uint32_t fNull  :1;
    uint32_t fRtsControl  :2;
    uint32_t fAbortOnError  :1;
    uint32_t fDummy2  :17;
    uint16_t wReserved;
    uint16_t XonLim;
    uint16_t XoffLim;
    uint8_t  ByteSize;
    uint8_t  Parity;
    uint8_t  StopBits;
    int8_t  XonChar;
    int8_t  XoffChar;
    int8_t  ErrorChar;
    int8_t  EofChar;
    int8_t  EvtChar;
    uint16_t wReserved1;
} DCB;

/*****************************************************************************/
/** Windows system constants */
#define ERROR_INSUFFICIENT_BUFFER   122
#define INVALID_HANDLE_VALUE        ((void *) -1)
#define GENERIC_READ                0x80000000
#define GENERIC_WRITE               0x40000000
#define OPEN_EXISTING               3
#define MAX_DWORD                   0xFFFFFFFF

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	void* __stdcall CreateFileA(const char *lpFileName,
			uint32_t dwDesiredAccess, uint32_t dwShareMode,
			void *lpSecurityAttributes, uint32_t dwCreationDisposition,
			uint32_t dwFlagsAndAttributes, void *hTemplateFile);
	bool __stdcall WriteFile(void *hFile, const void *lpBuffer,
			uint32_t nNumberOfBytesToWrite, uint32_t *lpNumberOfBytesWritten,
			void *lpOverlapped);
	bool __stdcall ReadFile(void *hFile, void *lpBuffer,
			uint32_t nNumberOfBytesToRead, uint32_t *lpNumberOfBytesRead,
			void *lpOverlapped);
	bool __stdcall CloseHandle(void *hFile);

	uint32_t __stdcall GetLastError(void);
	void __stdcall SetLastError(uint32_t dwErrCode);

	uint32_t __stdcall QueryDosDeviceA(const char *lpDeviceName,
			char *lpTargetPath, uint32_t ucchMax);

	bool __stdcall GetCommState(void *hFile, DCB *lpDCB);
	bool __stdcall GetCommTimeouts(void *hFile, COMMTIMEOUTS *lpCommTimeouts);
	bool __stdcall SetCommState(void *hFile, DCB *lpDCB);
	bool __stdcall SetCommTimeouts(void *hFile, COMMTIMEOUTS *lpCommTimeouts);
	bool __stdcall SetupComm(void *hFile, uint32_t dwInQueue,
			uint32_t dwOutQueue);

	/*****************************************************************************/
	int comEnumerate() {
		// Get devices information text
		size_t size = COM_MINDEVNAME;
		char *list = (char*) malloc(size);
		SetLastError(0);
		QueryDosDeviceA(NULL, list, size);
		while (GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
			size *= 2;
			char *nlist = realloc(list, size);
			if (!nlist) {
				free(list);
				return 0;
			}
			list = nlist;
			SetLastError(0);
			QueryDosDeviceA(NULL, list, size);
		}
		// Gather all COM ports
		int port;
		const char *nlist = findPattern(list, comPtn, &port);
		noDevices = 0;
		while (port > 0 && noDevices < COM_MAXDEVICES) {
			COMDevice *com = &comDevices[noDevices++];
			com->port = port;
			com->handle = 0;
			nlist = findPattern(nlist, comPtn, &port);
		}
		free(list);
		return noDevices;
	}

	void comTerminate() {
		comCloseAll();
	}

	int comGetNoPorts() {
		return noDevices;
	}

	/*****************************************************************************/
	const char* comGetPortName(int index) {
#define COM_MAXNAME    32
		static char name[COM_MAXNAME];
		if (index < 0 || index >= noDevices)
			return 0;
		sprintf(name, "COM%i", comDevices[index].port);
		return name;
	}

	int comFindPort(const char *name) {
		for (int i = 0; i < noDevices; i++)
			if (strcmp(name, comGetPortName(i)) == 0)
				return i;
		return -1;
	}

	const char* comGetInternalName(int index) {
#define COM_MAXNAME    32
		static char name[COM_MAXNAME];
		if (index < 0 || index >= noDevices)
			return 0;
		sprintf(name, "//./COM%i", comDevices[index].port);
		return name;
	}

	/*****************************************************************************/
	int comOpen(int index, int baudrate) {
		DCB config;
		COMMTIMEOUTS timeouts;
		if (index < 0 || index >= noDevices)
			return 0;
		// Close if already open
		COMDevice *com = &comDevices[index];
		if (com->handle)
			comClose(index);
		// Open COM port
		void *handle = CreateFileA(comGetInternalName(index),
				GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
		if (handle == INVALID_HANDLE_VALUE)
			return 0;
		com->handle = handle;
		// Prepare read / write timeouts
		SetupComm(handle, 64, 64);
		timeouts.ReadIntervalTimeout = MAX_DWORD;
		timeouts.ReadTotalTimeoutConstant = 0;
		timeouts.WriteTotalTimeoutConstant = 0;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutMultiplier = 0;
		SetCommTimeouts(handle, &timeouts);
		// Prepare serial communication format
		GetCommState(handle, &config);
		config.BaudRate = baudrate;
		config.fBinary = true;
		config.fParity = 0;
		config.fErrorChar = 0;
		config.fNull = 0;
		config.fAbortOnError = 0;
		config.ByteSize = 8;
		config.Parity = 0;
		config.StopBits = 0;
		config.EvtChar = '\n';
		// Set the port state
		if (SetCommState(handle, &config) == 0) {
			CloseHandle(handle);
			return 0;
		}
		return 1;
	}

	void comClose(int index) {
		if (index < 0 || index >= noDevices)
			return;
		COMDevice *com = &comDevices[index];
		if (!com->handle)
			return;
		CloseHandle(com->handle);
		com->handle = 0;
	}

	void comCloseAll() {
		for (int i = 0; i < noDevices; i++)
			comClose(i);
	}

	/*****************************************************************************/
	int comWrite(int index, const char *buffer, size_t len) {
		if (index < 0 || index >= noDevices)
			return 0;
		COMDevice *com = &comDevices[index];
		uint32_t bytes = 0;
		WriteFile(com->handle, buffer, len, &bytes, NULL);
		return bytes;
	}

	int comRead(int index, char *buffer, size_t len) {
		if (index < 0 || index >= noDevices)
			return 0;
		COMDevice *com = &comDevices[index];
		uint32_t bytes = 0;
		ReadFile(com->handle, buffer, len, &bytes, NULL);
		return bytes;
	}

	/*****************************************************************************/
	const char* findPattern(const char *string, const char *pattern, int *value) {
		char c, n = 0;
		const char *sp = string;
		const char *pp = pattern;
		// Check for the string pattern
		while (1) {
			c = *sp++;
			if (c == '\0') {
				if (*pp == '?')
					break;
				if (*sp == '\0')
					break;
				n = 0;
				pp = pattern;
			} else {
				if (*pp == '?') {
					// Expect a digit
					if (c >= '0' && c <= '9') {
						n = n * 10 + (c - '0');
						if (*pp++ == '\0')
							break;
					} else {
						n = 0;
						pp = comPtn;
					}
				} else {
					// Expect a character
					if (c == *pp) {
						if (*pp++ == '\0')
							break;
					} else {
						n = 0;
						pp = comPtn;
					}
				}
			}
		}
		// Return the value
		*value = n;
		return sp;
	}

#endif // _WIN32

	/* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_clear();
  lcd_puts(0U, 0U, "LCD Project Test");
  HAL_Delay (1000);
  lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x14;
  sTime.Seconds = 0x17;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1107-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF2 PF3 PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF6 PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PBTN2_Pin PBTN3_Pin PBTN1_Pin */
  GPIO_InitStruct.Pin = PBTN2_Pin|PBTN3_Pin|PBTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin E_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin PD14 */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
