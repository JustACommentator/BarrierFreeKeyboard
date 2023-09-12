/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include <stdint.h>
#include <stdbool.h>
#include "math.h"
//#include <stdlib.h>
#include "timers.h"
#include "usbd_custom_hid_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define keyA 0x04
#define keyB 0x05
#define keyC 0x06
#define keyD 0x07
#define keyE 0x08
#define keyF 0x09
#define keyG 0x0A
#define keyH 0x0B
#define keyI 0x0C
#define keyJ 0x0D
#define keyK 0x0E
#define keyL 0x0F
#define keyM 0x10
#define keyN 0x11
#define keyO 0x12
#define keyP 0x13
#define keyQ 0x14
#define keyR 0x15
#define keyS 0x16
#define keyT 0x17
#define keyU 0x18
#define keyV 0x19
#define keyW 0x1A
#define keyX 0x1B
#define keyY 0x1C
#define keyZ 0x1D
#define key1 0x1E
#define key2 0x1F
#define key3 0x20
#define key4 0x21
#define key5 0x22
#define key6 0x23
#define key7 0x24
#define key8 0x25
#define key9 0x26
#define key0 0x27
#define keySpace 0x2C
#define keyDash 0x2D
#define keyPeriod 0x37
#define keyComma 0x36
#define keySlash 0x38
#define keyShift 0x02
#define keyBackSpace 0x2A
#define keyEnter 0x28
#define keyCTRL 0x01

typedef struct {
	uint8_t reportId;                                 // Report ID = 0x01 (1)
													  // Collection: CA:Keyboard
	uint8_t KB_KeyboardKeyboardLeftControl :1; // Usage 0x000700E0: Keyboard Left Control, Value = 0 to 1
	uint8_t KB_KeyboardKeyboardLeftShift :1; // Usage 0x000700E1: Keyboard Left Shift, Value = 0 to 1
	uint8_t KB_KeyboardKeyboardLeftAlt :1; // Usage 0x000700E2: Keyboard Left Alt, Value = 0 to 1
	uint8_t KB_KeyboardKeyboardLeftGui :1; // Usage 0x000700E3: Keyboard Left GUI, Value = 0 to 1
	uint8_t KB_KeyboardKeyboardRightControl :1; // Usage 0x000700E4: Keyboard Right Control, Value = 0 to 1
	uint8_t KB_KeyboardKeyboardRightShift :1; // Usage 0x000700E5: Keyboard Right Shift, Value = 0 to 1
	uint8_t KB_KeyboardKeyboardRightAlt :1; // Usage 0x000700E6: Keyboard Right Alt, Value = 0 to 1
	uint8_t KB_KeyboardKeyboardRightGui :1; // Usage 0x000700E7: Keyboard Right GUI, Value = 0 to 1
	uint8_t pad_2;                                    // Pad
	uint8_t Keyboard[2];                              // Value = 0 to 101
} KeyboardReport_t;

typedef struct {
	uint8_t reportId;                                 // Report ID = 0x02 (2)
													  // Collection: CA:Mouse CP:Pointer
	uint8_t BTN_MousePointerButton1 :1; // Usage 0x00090001: Button 1 Primary/trigger, Value = 0 to 1
	uint8_t BTN_MousePointerButton2 :1; // Usage 0x00090002: Button 2 Secondary, Value = 0 to 1
	uint8_t BTN_MousePointerButton3 :1; // Usage 0x00090003: Button 3 Tertiary, Value = 0 to 1
	uint8_t :5;                                      // Pad
	int8_t GD_MousePointerX;         // Usage 0x00010030: X, Value = -127 to 127
	int8_t GD_MousePointerY;         // Usage 0x00010031: Y, Value = -127 to 127
	int8_t GD_MousePointerWheel; // Usage 0x00010038: Wheel, Value = -127 to 127
} MouseReport_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern USBD_HandleTypeDef hUsbDeviceFS;

KeyboardReport_t keyboard_report;
MouseReport_t mouse_report;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* Definitions for StartPolling */
osThreadId_t StartPollingHandle;
const osThreadAttr_t StartPolling_attributes = {
  .name = "StartPolling",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InputProcessing */
osThreadId_t InputProcessingHandle;
const osThreadAttr_t InputProcessing_attributes = {
  .name = "InputProcessing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for capitalTimer */
osTimerId_t capitalTimerHandle;
const osTimerAttr_t capitalTimer_attributes = {
  .name = "capitalTimer"
};
/* Definitions for modeTimer */
osTimerId_t modeTimerHandle;
const osTimerAttr_t modeTimer_attributes = {
  .name = "modeTimer"
};
/* USER CODE BEGIN PV */

const float CDegConv = 57.2957795;

bool zoneReset = true;

bool modeReset = true;

bool validInputAction = false;

bool mouseMode = true;

bool buttonReset = true;

int mouse_speed = 25; //higher number, lower speed
int angleMargin = 20;
int turnMargin = 10;
int mouseThreshhold = 20;
int angleDist1[2] = { 0 };
int angleDist2[2] = { 0 };
int zone1 = 0;
int zone2 = 0;
int currentKeycode = 0;

char currentChar = '0';

int timeTilCapital = 500;

int timeTilModeSwitch = 500;

int angleDist[2];

int numberOfZones = 8; //To customize layout via Desktop program in future versions
float zoneSize = 0;
int distThresh = 64; // between 0 and 127 (ignoring imprecision. Actual range may be smaller)

int keycodeDict[8][8] = { { keyA, keyB, keyC, keyD, keyE, keyF, keyG, keyH },
		{ keyI, keyJ, keyK, keyL, keyM, keyN, keyO, keyP },
		{ keyQ, keyR, keyS, keyT, keyU, keyV, keyW, keyX },
		{ keyY, keyZ, keySpace, keySpace, keySpace, keySpace, keySpace, keySpace },
		{ key0, key1, key2, key3, key4, key5, key6, key7 },
		{ key8, key9, keySpace, keySpace, keySpace, keySpace, keySpace, keySpace },
		{ keyDash, keyPeriod, keyComma, keySlash, keyBackSpace, keyEnter, keySpace, keySpace },
		{ keySpace, keySpace, keySpace, keySpace, keySpace, keySpace, keySpace, keySpace } };

volatile uint16_t adcResultsDMA[4];
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);
volatile int adcConversionComplete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
void StarPollingTask(void *argument);
void StartInputProcessing(void *argument);
void capitalTimerCallback(void *argument);
void modeTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

void sendMouseButtons(int);
void sendMouse();
void checkModeSwitch(int, int);
void getAngleDist(int, int, int*);
int getZone(int*);
void getChar(int, int);
void initializeDictionary();
void capitalTimerCallback();
void sendCharacter(int, int);

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

	keyboard_report.reportId = 0x01;
	mouse_report.reportId = 0x02;

	mouse_report.BTN_MousePointerButton1 = 0;
	mouse_report.BTN_MousePointerButton2 = 0;
	mouse_report.BTN_MousePointerButton3 = 0;
	mouse_report.GD_MousePointerWheel = 0;
	mouse_report.GD_MousePointerX = 0;
	mouse_report.GD_MousePointerY = 0;

	zoneSize = 360 / numberOfZones;
	initializeDictionary();

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
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of capitalTimer */
  capitalTimerHandle = osTimerNew(capitalTimerCallback, osTimerOnce, NULL, &capitalTimer_attributes);

  /* creation of modeTimer */
  modeTimerHandle = osTimerNew(modeTimerCallback, osTimerOnce, NULL, &modeTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StartPolling */
  StartPollingHandle = osThreadNew(StarPollingTask, NULL, &StartPolling_attributes);

  /* creation of InputProcessing */
  InputProcessingHandle = osThreadNew(StartInputProcessing, NULL, &InputProcessing_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void initializeDictionary() {
	// for config file reading
}

void sendCharacter(int keycode, int modifier) { //only supports shift as modifier for now
//	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report, sizeof(MouseReport_t));
//	osDelay(500);

	keyboard_report.KB_KeyboardKeyboardLeftShift = modifier;
	keyboard_report.Keyboard[0] = keycode;
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &keyboard_report,
			sizeof(KeyboardReport_t));
	osDelay(25);

	keyboard_report.KB_KeyboardKeyboardLeftShift = 0;
	keyboard_report.Keyboard[0] = 0x00;
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &keyboard_report,
			sizeof(KeyboardReport_t));
	osDelay(50);
}

void sendMouse() {

	int X = adcResultsDMA[2] - 127;
	int Y = -(adcResultsDMA[3] - 127);

	mouse_report.GD_MousePointerX = 0;
	mouse_report.GD_MousePointerY = 0;

	if (abs(X) > mouseThreshhold)
		mouse_report.GD_MousePointerX = (X / mouse_speed);
	if (abs(Y) > mouseThreshhold)
		mouse_report.GD_MousePointerY = (Y / mouse_speed);
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
			sizeof(MouseReport_t));

}

void sendMouseButtons(int left) {

	if (left == -1)
		buttonReset = true;

	if (buttonReset && left != -1) {
		buttonReset = false;

		switch (left) {
		case 1:
			mouse_report.BTN_MousePointerButton3 = 1;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			osDelay(25);
			mouse_report.BTN_MousePointerButton3 = 0;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			break;
		case 2:
			break;
		case 3:
			mouse_report.BTN_MousePointerButton2 = 1;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			osDelay(25);
			mouse_report.BTN_MousePointerButton2 = 0;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			break;
		case 4:
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			mouse_report.BTN_MousePointerButton1 = 1;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			osDelay(25);
			mouse_report.BTN_MousePointerButton1 = 0;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			break;
		case 5:
			mouse_report.BTN_MousePointerButton1 = 1;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			osDelay(25);
			mouse_report.BTN_MousePointerButton1 = 0;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			osDelay(25);
			mouse_report.BTN_MousePointerButton1 = 1;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			osDelay(25);
			mouse_report.BTN_MousePointerButton1 = 0;
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &mouse_report,
					sizeof(MouseReport_t));
			break;
		case 6:
			break;
		case 7:
			break;
		}
	}
}

void checkModeSwitch(int right, int left) {

	if (!(left == 0 && right == -1))
		osTimerStop(modeTimerHandle);
	if (right == -1 && left == -1) {
		modeReset = true;
		osTimerStop(modeTimerHandle);
	}

	if ((left == 0 && right == -1) && modeReset) {
		modeReset = false;
		osTimerStart(modeTimerHandle, timeTilModeSwitch);
	}

}

void getChar(int right, int left) {

	if (left == -1 || right == -1) {
		zoneReset = true;
		osTimerStop(capitalTimerHandle);
		return;
	}
	if ((left != -1 && right != -1) && zoneReset) {
		currentKeycode = keycodeDict[zone1][zone2];
		sendCharacter(keycodeDict[zone1][zone2], 0);
		zoneReset = false;
		osTimerStart(capitalTimerHandle, timeTilCapital);
	}
//	else if ((left == -1 && right == 4) && zoneReset) { //add Timer for this, else it'll get triggered by accident
//		sendCharacter(keyBackSpace, 0);
//		zoneReset = false;
//	}
}

void getAngleDist(int x, int y, int *resArr) {

	if (y == 0) {
		if (x >= 0)
			resArr[0] = 90;
		else
			resArr[0] = 270;

		resArr[1] = abs(x);

		return;
	}

	if (y >= 0) {
		if (x >= 0) {
			resArr[0] = (int) (atan(x / (float) y) * CDegConv);
		} else {
			resArr[0] = (int) (atan(x / (float) y) * CDegConv + 360);
		}
	} else {
		resArr[0] = (int) (atan(x / (float) y) * CDegConv + 180);
	}

	resArr[1] = (int) sqrt(x * x + y * y);

	return;
}

int getZone(int *angleDist) {
	if (*(angleDist + 1) >= distThresh) {
		return (int) ((*angleDist + zoneSize / 2) / zoneSize) % numberOfZones;
	} else {
		return -1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcConversionComplete = 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StarPollingTask */
/**
 * @brief  Function implementing the StartPolling thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StarPollingTask */
void StarPollingTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, adcChannelCount);
		adcConversionComplete = 0;

		getAngleDist(adcResultsDMA[2] - 127, adcResultsDMA[3] - 127,
				angleDist1);
		getAngleDist(adcResultsDMA[0] - 127, adcResultsDMA[1] - 127,
				angleDist2);

		zone1 = getZone(angleDist1);
		zone2 = getZone(angleDist2);

		osDelay(1);
	}
	osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartInputProcessing */
/**
 * @brief Function implementing the InputProcessing thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInputProcessing */
void StartInputProcessing(void *argument)
{
  /* USER CODE BEGIN StartInputProcessing */
	/* Infinite loop */
	for (;;) {
		checkModeSwitch(zone1, zone2);
		if (!mouseMode)
			getChar(zone1, zone2);
		else {
			sendMouse();
			osDelay(25);
			sendMouseButtons(zone2);
		}
		osDelay(1);
	}
	osThreadTerminate(NULL);
  /* USER CODE END StartInputProcessing */
}

/* capitalTimerCallback function */
void capitalTimerCallback(void *argument)
{
  /* USER CODE BEGIN capitalTimerCallback */
	if (currentChar >= 'a' && currentChar <= 'z')
		currentChar = currentChar - 32;
	sendCharacter(keyBackSpace, 0);
	sendCharacter(currentKeycode, 1);
  /* USER CODE END capitalTimerCallback */
}

/* modeTimerCallback function */
void modeTimerCallback(void *argument)
{
  /* USER CODE BEGIN modeTimerCallback */
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	mouseMode = !mouseMode;
  /* USER CODE END modeTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
	while (1) {
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
