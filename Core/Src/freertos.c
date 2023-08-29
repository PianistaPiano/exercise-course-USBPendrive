/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
#include "bmp280.h"
#include "i2c.h"
#include "usart.h"
#include "SSD1306_OLED.h"
#include "GFX_BW.h"
#include "fonts/fonts.h"
#include "arm_math.h"
#include "tim.h"
#include "adc.h"
#include "usb_host.h"
#include "fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SAMPLES 1024

#define f_unmount(path) f_mount(0, path, 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct
{
	float Pressure;
	float Temperature;
}BmpData_t;

typedef struct
{
	uint8_t OutFreqArray[10];
} FFTData_t;


// variables for USB
volatile uint8_t UnmountUSB = 0;

/* USER CODE END Variables */
/* Definitions for HeartbeatTask */
osThreadId_t HeartbeatTaskHandle;
const osThreadAttr_t HeartbeatTask_attributes = {
  .name = "HeartbeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Bmp280Task */
osThreadId_t Bmp280TaskHandle;
const osThreadAttr_t Bmp280Task_attributes = {
  .name = "Bmp280Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for FFTTask */
osThreadId_t FFTTaskHandle;
const osThreadAttr_t FFTTask_attributes = {
  .name = "FFTTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for PendriveTask */
osThreadId_t PendriveTaskHandle;
const osThreadAttr_t PendriveTask_attributes = {
  .name = "PendriveTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for QueueBmpData */
osMessageQueueId_t QueueBmpDataHandle;
const osMessageQueueAttr_t QueueBmpData_attributes = {
  .name = "QueueBmpData"
};
/* Definitions for QueueFFTData */
osMessageQueueId_t QueueFFTDataHandle;
const osMessageQueueAttr_t QueueFFTData_attributes = {
  .name = "QueueFFTData"
};
/* Definitions for QueueBmpPenData */
osMessageQueueId_t QueueBmpPenDataHandle;
const osMessageQueueAttr_t QueueBmpPenData_attributes = {
  .name = "QueueBmpPenData"
};
/* Definitions for TimerBmpData */
osTimerId_t TimerBmpDataHandle;
const osTimerAttr_t TimerBmpData_attributes = {
  .name = "TimerBmpData"
};
/* Definitions for TimerBmpPenData */
osTimerId_t TimerBmpPenDataHandle;
const osTimerAttr_t TimerBmpPenData_attributes = {
  .name = "TimerBmpPenData"
};
/* Definitions for MutexPritnf */
osMutexId_t MutexPritnfHandle;
const osMutexAttr_t MutexPritnf_attributes = {
  .name = "MutexPritnf"
};
/* Definitions for MutexI2C1 */
osMutexId_t MutexI2C1Handle;
const osMutexAttr_t MutexI2C1_attributes = {
  .name = "MutexI2C1"
};
/* Definitions for MutexBmpData */
osMutexId_t MutexBmpDataHandle;
const osMutexAttr_t MutexBmpData_attributes = {
  .name = "MutexBmpData"
};
/* Definitions for SemaphoreBmpQueue */
osSemaphoreId_t SemaphoreBmpQueueHandle;
const osSemaphoreAttr_t SemaphoreBmpQueue_attributes = {
  .name = "SemaphoreBmpQueue"
};
/* Definitions for SemaphoreBmpPenQueue */
osSemaphoreId_t SemaphoreBmpPenQueueHandle;
const osSemaphoreAttr_t SemaphoreBmpPenQueue_attributes = {
  .name = "SemaphoreBmpPenQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
float complexABS(float real, float compl);
/* USER CODE END FunctionPrototypes */

void StartHeartbeatTask(void *argument);
void StartBmp280Task(void *argument);
void StartOledTask(void *argument);
void StartFFTTask(void *argument);
void StartPendriveTask(void *argument);
void TimerBmpDataCallback(void *argument);
void TimerBmpPenDataCallback(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	printf(pcTaskName);
	while(1){}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	while(1){}
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexPritnf */
  MutexPritnfHandle = osMutexNew(&MutexPritnf_attributes);

  /* creation of MutexI2C1 */
  MutexI2C1Handle = osMutexNew(&MutexI2C1_attributes);

  /* creation of MutexBmpData */
  MutexBmpDataHandle = osMutexNew(&MutexBmpData_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreBmpQueue */
  SemaphoreBmpQueueHandle = osSemaphoreNew(1, 1, &SemaphoreBmpQueue_attributes);

  /* creation of SemaphoreBmpPenQueue */
  SemaphoreBmpPenQueueHandle = osSemaphoreNew(1, 1, &SemaphoreBmpPenQueue_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerBmpData */
  TimerBmpDataHandle = osTimerNew(TimerBmpDataCallback, osTimerPeriodic, NULL, &TimerBmpData_attributes);

  /* creation of TimerBmpPenData */
  TimerBmpPenDataHandle = osTimerNew(TimerBmpPenDataCallback, osTimerPeriodic, NULL, &TimerBmpPenData_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueBmpData */
  QueueBmpDataHandle = osMessageQueueNew (8, sizeof(BmpData_t), &QueueBmpData_attributes);

  /* creation of QueueFFTData */
  QueueFFTDataHandle = osMessageQueueNew (16, sizeof(FFTData_t), &QueueFFTData_attributes);

  /* creation of QueueBmpPenData */
  QueueBmpPenDataHandle = osMessageQueueNew (8, sizeof(BmpData_t), &QueueBmpPenData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HeartbeatTask */
  HeartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &HeartbeatTask_attributes);

  /* creation of Bmp280Task */
  Bmp280TaskHandle = osThreadNew(StartBmp280Task, NULL, &Bmp280Task_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);

  /* creation of FFTTask */
  FFTTaskHandle = osThreadNew(StartFFTTask, NULL, &FFTTask_attributes);

  /* creation of PendriveTask */
  PendriveTaskHandle = osThreadNew(StartPendriveTask, NULL, &PendriveTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
  * @brief  Function implementing the HeartbeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartHeartbeatTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
  /* USER CODE END StartHeartbeatTask */
}

/* USER CODE BEGIN Header_StartBmp280Task */
/**
* @brief Function implementing the Bmp280Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmp280Task */
void StartBmp280Task(void *argument)
{
  /* USER CODE BEGIN StartBmp280Task */
	BMP280_t Bmp280;
	BmpData_t _BmpData;
	uint32_t DelayTick = osKernelGetTickCount();

	osMutexAcquire(MutexI2C1Handle, osWaitForever);
	BMP280_Init(&Bmp280, &hi2c1, 0x76);
	osMutexRelease(MutexI2C1Handle);

	// Start Timer to get data form bmp to display
	osTimerStart(TimerBmpDataHandle, 100);

  /* Infinite loop */
  for(;;)
  {
	//printf("BMP\n\r");
	osMutexAcquire(MutexI2C1Handle, osWaitForever);
	BMP280_ReadTemperatureAndPressure(&Bmp280, &_BmpData.Pressure, &_BmpData.Temperature);
	osMutexRelease(MutexI2C1Handle);

	if(osOK == osSemaphoreAcquire(SemaphoreBmpQueueHandle, 0))
	{
		osMessageQueuePut(QueueBmpDataHandle, &_BmpData, 0, osWaitForever);
	}
	if(osOK == osSemaphoreAcquire(SemaphoreBmpPenQueueHandle, 0))
	{
		osMessageQueuePut(QueueBmpPenDataHandle, &_BmpData, 0, osWaitForever);
	}
//	printf("Temperature: %.2f, Pressure: %.2f\n\r", _BmpData.Temperature, _BmpData.Pressure);
	DelayTick += 10;
    osDelayUntil(DelayTick);
  }
  /* USER CODE END StartBmp280Task */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
	char Message[32];
	BmpData_t _BmpData;
	FFTData_t _FFTData;
	//uint8_t i = 0;

	osMutexAcquire(MutexI2C1Handle, osWaitForever);
	SSD1306_Init(&hi2c1);
	osMutexRelease(MutexI2C1Handle);

	GFX_SetFont(font_8x5);
	SSD1306_Clear(BLACK);

	SSD1306_Display();


  /* Infinite loop */
  for(;;)
  {
	SSD1306_Clear(BLACK);
	//printf("Oled\n\r");
	osMessageQueueGet(QueueBmpDataHandle, &_BmpData, 0, 0);

	osMessageQueueGet(QueueFFTDataHandle, &_FFTData, 0, 0);


	sprintf(Message, "Temp: %.2f C",  _BmpData.Temperature);
	GFX_DrawString(0,0, Message, WHITE, BLACK);
	sprintf(Message, "Press: %.2f hPa",_BmpData.Pressure);
	GFX_DrawString(0,15, Message, WHITE, BLACK);

	// FFT
	for(uint8_t i = 0; i < 10; i++) // Each frequency
	{
	  GFX_DrawFillRectangle(10+(i*11), 64-_FFTData.OutFreqArray[i], 10, _FFTData.OutFreqArray[i], WHITE);
	}


	SSD1306_Display();

  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartFFTTask */
/**
* @brief Function implementing the FFTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFFTTask */
void StartFFTTask(void *argument)
{
  /* USER CODE BEGIN StartFFTTask */
	arm_rfft_fast_instance_f32 FFTHandler;
	FFTData_t FFTData;
	int FreqPoint = 0;
	int Offset = 55; // variable noise floor offset

	uint16_t *AdcMicrophone;
	float *FFTInBuffer;
	float *FFTOutBuffer;
	int *Freqs;

	// zaalakujmy je na stercie od FRTOS'a
	AdcMicrophone = pvPortMalloc(FFT_SAMPLES * sizeof(uint16_t));
	FFTInBuffer = pvPortMalloc(FFT_SAMPLES * sizeof(float));
	FFTOutBuffer = pvPortMalloc(FFT_SAMPLES * sizeof(float));
	Freqs = pvPortMalloc(FFT_SAMPLES * sizeof(int));

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcMicrophone, FFT_SAMPLES);
	arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES);

  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
	  for(uint32_t i = 0; i < FFT_SAMPLES; i++)
	  {
		  FFTInBuffer[i] = (float)AdcMicrophone[i];
		  //printf("%d\n\r", AdcMicrophone[i]);
	  }
	  arm_rfft_fast_f32(&FFTHandler, FFTInBuffer, FFTOutBuffer, 0); // zwraca liczby zespolone

	  FreqPoint = 0;
	  // calculate abs values and linear-to-dB
	  for (int i = 0; i < FFT_SAMPLES; i = i+2)
	  {
		  Freqs[FreqPoint] = (int)(20*log10f(complexABS(FFTOutBuffer[i], FFTOutBuffer[i+1]))) - Offset;

		  if(Freqs[FreqPoint] < 0)
		  {
			  Freqs[FreqPoint] = 0;
		  }
		  FreqPoint++;

	  }

//	  FFTData.OutFreqArray[0] = (uint8_t)Freqs[1]; // 22 Hz
//	  FFTData.OutFreqArray[1] = (uint8_t)Freqs[2]; // 63 Hz
//	  FFTData.OutFreqArray[2] = (uint8_t)Freqs[3]; // 125 Hz
//	  FFTData.OutFreqArray[3] = (uint8_t)Freqs[6]; // 250 Hz
//	  FFTData.OutFreqArray[4] = (uint8_t)Freqs[12]; // 500 Hz
//	  FFTData.OutFreqArray[5] = (uint8_t)Freqs[23]; // 1000 Hz
//	  FFTData.OutFreqArray[6] = (uint8_t)Freqs[51]; // 2200 Hz
//	  FFTData.OutFreqArray[7] = (uint8_t)Freqs[104]; // 4500 Hz
//	  FFTData.OutFreqArray[8] = (uint8_t)Freqs[207]; // 9000 Hz
//	  FFTData.OutFreqArray[9] = (uint8_t)Freqs[344]; // 15000 Hz

	  FFTData.OutFreqArray[0] = (uint8_t)Freqs[1]; // 50 Hz
	  FFTData.OutFreqArray[1] = (uint8_t)Freqs[2]; // 100 Hz
	  FFTData.OutFreqArray[2] = (uint8_t)Freqs[3]; // 150 Hz
	  FFTData.OutFreqArray[3] = (uint8_t)Freqs[6]; // 300 Hz
	  FFTData.OutFreqArray[4] = (uint8_t)Freqs[12]; // 600 Hz
	  FFTData.OutFreqArray[5] = (uint8_t)Freqs[20]; // 1000 Hz
	  FFTData.OutFreqArray[6] = (uint8_t)Freqs[22]; // 1100 Hz
	  FFTData.OutFreqArray[7] = (uint8_t)Freqs[40]; // 2000 Hz
	  FFTData.OutFreqArray[8] = (uint8_t)Freqs[207]; // 10350 Hz
	  FFTData.OutFreqArray[9] = (uint8_t)Freqs[344]; // 17200 Hz

	  osMessageQueuePut(QueueFFTDataHandle, &FFTData, 0, osWaitForever);

	  //CalculateFFT();
  }
  /* USER CODE END StartFFTTask */
}

/* USER CODE BEGIN Header_StartPendriveTask */
/**
* @brief Function implementing the PendriveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPendriveTask */
void StartPendriveTask(void *argument)
{
  /* USER CODE BEGIN StartPendriveTask */
	extern ApplicationTypeDef Appli_state;
	extern uint8_t retUSBH; /* Return value for USBH */
	extern char USBHPath[4]; /* USBH logical drive path */
	extern FATFS USBHFatFS; /* File system object for USBH logical drive */
	extern FIL USBHFile; /* File object for USBH */
	FILINFO fno;
	DIR Directory;
	uint16_t nfile = 0;
	char FileName[64];
	BmpData_t _BmpData;

	typedef enum
	{
		DEVICE_MOUNT = 0,
		DIR_AND_FILE_OPEN,
		SAVE_DATA_TO_FILE,
		TRY_UNMOUNT_DEVICE,
		UNMOUNT_DEVICE,
		ERROR_DEVICE,
		MAX_NUMBER_OF_FILES
	} USBDeviceStatus_t;
	USBDeviceStatus_t USBDeviceStatus = DEVICE_MOUNT;

	HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
	  switch (Appli_state)
	  {
		case APPLICATION_IDLE:
			//printf("IDLE\n\r");
			HAL_GPIO_TogglePin(LEDBlue_GPIO_Port, LEDBlue_Pin);
			HAL_GPIO_TogglePin(LEDYellow_GPIO_Port, LEDYellow_Pin);
			osDelay(1000);
			break;
		case APPLICATION_START:
			break;
		case APPLICATION_READY:
			//printf("READY\n\r");

			switch (USBDeviceStatus)
			{
				case DEVICE_MOUNT:
					retUSBH = f_mount(&USBHFatFS, (const TCHAR*)USBHPath, 1);
					if(retUSBH == FR_OK)
					{
						HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(LEDYellow_GPIO_Port, LEDYellow_Pin, GPIO_PIN_RESET);
						printf("FatFS mounted\n\r");
						USBDeviceStatus = DIR_AND_FILE_OPEN;
					}
					else
					{
						printf("FatFS mount error\n\r");
						HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_RESET);
						USBDeviceStatus = ERROR_DEVICE;
					}

					break;
				case DIR_AND_FILE_OPEN:
					// make sub dir BMPData, if exist nothing happened
					retUSBH = f_mkdir("BMPData");
					if(retUSBH == FR_OK)
						printf("Created SubDir\n\r");
					else if(retUSBH != FR_OK && retUSBH != FR_EXIST )
					{
						printf("SubDir Error\n\r");
						HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_RESET);
						USBDeviceStatus = ERROR_DEVICE;
					}

					// Open Directory
					retUSBH = f_opendir(&Directory, "BMPData");
					if(retUSBH == FR_OK)
					{
						printf("Opened Sub Dir\n\r");
						//Find Last file...
						while(1)
						{
							retUSBH = f_readdir(&Directory, &fno);
							if(retUSBH != FR_OK || fno.fname[0] == 0)
								break;
							if(fno.fname[0] != 0)
								nfile++;
						}

					}
					else
					{
						printf("OpenDir Error\n\r");
						HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_RESET);
						USBDeviceStatus = ERROR_DEVICE;
					}
					//... and create new with higher number
					if((nfile + 1) > 255)
					{
						printf("Number of files: 255. MAX! Clear the device\n\r");
						HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(LEDYellow_GPIO_Port, LEDYellow_Pin, GPIO_PIN_RESET);
						USBDeviceStatus = MAX_NUMBER_OF_FILES;
					}
					else
					{
						sprintf(FileName, "BMPData\\BmpData%d.txt", nfile + 1);
						printf("File Name: %s\n\r", FileName);
						retUSBH = f_open(&USBHFile, (const TCHAR*)FileName, FA_CREATE_NEW | FA_WRITE);
						if(retUSBH != FR_OK)
						{
							printf("Cant open a file: %d\n\r", retUSBH);
							HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_RESET);
							USBDeviceStatus = ERROR_DEVICE;
						}
						else
						{
							printf("File opened\n\r");
							osTimerStart(TimerBmpPenDataHandle, 5000);
							USBDeviceStatus = SAVE_DATA_TO_FILE;
						}
					}

					break;
				case SAVE_DATA_TO_FILE:
					if(osOK == osMessageQueueGet(QueueBmpPenDataHandle, &_BmpData, 0, 0))
					{
						printf("T=%.2f, P=%.2f\n\r", _BmpData.Temperature, _BmpData.Pressure); // @suppress("Float formatting support")
						retUSBH = f_printf(&USBHFile, "T=%5d, P=%6d\n", (uint16_t)(_BmpData.Temperature *100), (uint32_t)(_BmpData.Pressure*100));
						if(retUSBH > 0)
							printf("Zapisalem do pliku\n\r");
						else
							printf("Nie zapisalem do pliku: %d\n\r", retUSBH);
					}
					if(UnmountUSB == 1)
					{
						USBDeviceStatus = TRY_UNMOUNT_DEVICE;
					}
					osDelay(1000);
					break;
				case TRY_UNMOUNT_DEVICE:
					// stop timer
					osTimerStop(TimerBmpPenDataHandle);
					// close file
					retUSBH = f_close(&USBHFile);
					if(retUSBH != FR_OK)
					{
						printf("Cant close a file: %d\n\r", retUSBH);
						USBDeviceStatus = ERROR_DEVICE;
					}
					// close directory
					retUSBH = f_closedir(&Directory);
					if(retUSBH != FR_OK)
					{
						printf("Can't close directory: %d\n\r", retUSBH);
						USBDeviceStatus = ERROR_DEVICE;
					}
					retUSBH = f_unmount((const TCHAR*)USBHPath);
					if(retUSBH != FR_OK)
					{
						printf("Device can't be unmounted\n\r");
						USBDeviceStatus = ERROR_DEVICE;
					}
					else
					{
						printf("You can unmount your device\n\r");
						UnmountUSB = 0;
						nfile = 0;
						HAL_GPIO_WritePin(LEDYellow_GPIO_Port, LEDYellow_Pin, GPIO_PIN_RESET);
						USBDeviceStatus = UNMOUNT_DEVICE;
					}

					break;
				case UNMOUNT_DEVICE:
					HAL_GPIO_TogglePin(LEDBlue_GPIO_Port, LEDBlue_Pin);
					osDelay(100);
					break;
				case ERROR_DEVICE:
					HAL_GPIO_TogglePin(LEDYellow_GPIO_Port, LEDYellow_Pin);
					if(UnmountUSB == 1)
					{
						USBDeviceStatus = TRY_UNMOUNT_DEVICE;
					}
					osDelay(100);
					break;
				case MAX_NUMBER_OF_FILES:
					HAL_GPIO_TogglePin(LEDBlue_GPIO_Port, LEDBlue_Pin);
					HAL_GPIO_TogglePin(LEDYellow_GPIO_Port, LEDYellow_Pin);
					if(UnmountUSB == 1)
					{
						USBDeviceStatus = TRY_UNMOUNT_DEVICE;
						HAL_GPIO_WritePin(LEDYellow_GPIO_Port, LEDYellow_Pin, GPIO_PIN_RESET);
					}
					osDelay(100);
					break;
				default:
					break;
			}
			break;
		case APPLICATION_DISCONNECT:
			printf("DISCON\n\r");
			USBDeviceStatus = DEVICE_MOUNT;
			HAL_GPIO_WritePin(LEDBlue_GPIO_Port, LEDBlue_Pin, GPIO_PIN_SET);
			Appli_state = APPLICATION_IDLE;
			osDelay(1000);
			break;
		default:
			printf("DEFAULT\n\r");
			osDelay(1000);
			break;
	  }

  }
  /* USER CODE END StartPendriveTask */
}

/* TimerBmpDataCallback function */
void TimerBmpDataCallback(void *argument)
{
  /* USER CODE BEGIN TimerBmpDataCallback */
	osSemaphoreRelease(SemaphoreBmpQueueHandle);
  /* USER CODE END TimerBmpDataCallback */
}

/* TimerBmpPenDataCallback function */
void TimerBmpPenDataCallback(void *argument)
{
  /* USER CODE BEGIN TimerBmpPenDataCallback */
	osSemaphoreRelease(SemaphoreBmpPenQueueHandle);
  /* USER CODE END TimerBmpPenDataCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
  // send char to console etc.
	osMutexAcquire(MutexPritnfHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 1000);
	osMutexRelease(MutexPritnfHandle);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
//		SamplesReady = 1;
		osThreadFlagsSet(FFTTaskHandle, 0x01);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if(GPIO_Pin == B1_Pin)
  {
	  UnmountUSB = 1;
  }

}
float complexABS(float real, float compl)
{
	return sqrtf(real*real+compl*compl);
}
/* USER CODE END Application */

