/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "I2Cdev.h"
#include "MPU6050.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE BEGIN Variables */

osThreadId_t i2cHandle;
const osThreadAttr_t i2cHandler_attributes = {
	 .name = "i2cHandler",
	 .priority = (osPriority_t) osPriorityNormal,
	 .stack_size = 128 * 4
};
/*
static ros::NodeHandle nh;
 USER CODE END Variables
 Definitions for getInfoTask
	osThreadId_t getInfoTaskHandle;
	const osThreadAttr_t getInfoTask_attributes = {
	  .name = "getInfoTask",
	  .priority = (osPriority_t) osPriorityNormal,
	  .stack_size = 128 * 4
	};
	 Definitions for errorHandler
	osThreadId_t errorHandlerHandle;
	const osThreadAttr_t errorHandler_attributes = {
	  .name = "errorHandler",
	  .priority = (osPriority_t) osPriorityNormal,
	  .stack_size = 128 * 4
	};
	 Definitions for relayManager
	osThreadId_t relayManagerHandle;
	const osThreadAttr_t relayManager_attributes = {
	  .name = "relayManager",
	  .priority = (osPriority_t) osPriorityHigh,
	  .stack_size = 128 * 4
	};
	 Definitions for ledManager
	osThreadId_t ledManagerHandle;
	const osThreadAttr_t ledManager_attributes = {
	  .name = "ledManager",
	  .priority = (osPriority_t) osPriorityNormal,
	  .stack_size = 128 * 4
	};
	 Definitions for terminalManager
	osThreadId_t terminalManagerHandle;
	const osThreadAttr_t terminalManager_attributes = {
	  .name = "terminalManager",
	  .priority = (osPriority_t) osPriorityLow,
	  .stack_size = 128 * 4
	};
	 Definitions for imuManager
	osThreadId_t imuManagerHandle;
	const osThreadAttr_t imuManager_attributes = {
	  .name = "imuManager",
	  .priority = (osPriority_t) osPriorityNormal,
	  .stack_size = 128 * 4
	};
	 Definitions for usonicManager
	osThreadId_t usonicManagerHandle;
	const osThreadAttr_t usonicManager_attributes = {
	  .name = "usonicManager",
	  .priority = (osPriority_t) osPriorityNormal,
	  .stack_size = 128 * 4
	};
	 Definitions for rosSpin
	osThreadId_t rosSpinHandle;
	const osThreadAttr_t rosSpin_attributes = {
	  .name = "rosSpin",
	  .priority = (osPriority_t) osPriorityNormal,
	  .stack_size = 128 * 4
	};
*/

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */
void StartI2CTask(void *argument);
/*void StartROS(void *argument);
void StartInfoTask(void *argument);
extern void StartErrorHandler(void *argument);
extern void StartRelayManager(void *argument);
extern void StartLedManager(void *argument);
extern void StartTerminal(void *argument);
extern void StartImuManager(void *argument);
extern void StartUsonicManager(void *argument);*/

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
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
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* ROS node handle initialization and topic registration */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of rosSpin */
	i2cHandle = osThreadNew(StartI2CTask, NULL, &i2cHandler_attributes);
 /* rosSpinHandle = osThreadNew(StartROS, &nh, &rosSpin_attributes);

   creation of getInfoTask
  getInfoTaskHandle = osThreadNew(StartInfoTask, &nh, &getInfoTask_attributes);

   creation of errorHandler
  errorHandlerHandle = osThreadNew(StartErrorHandler, &nh, &errorHandler_attributes);

   creation of relayManager
  relayManagerHandle = osThreadNew(StartRelayManager, &nh, &relayManager_attributes);

   creation of ledManager
  ledManagerHandle = osThreadNew(StartLedManager, &nh, &ledManager_attributes);

   creation of terminalManager
  terminalManagerHandle = osThreadNew(StartTerminal, &nh, &terminalManager_attributes);

   creation of imuManager
  imuManagerHandle = osThreadNew(StartImuManager, &nh, &imuManager_attributes);

   creation of usonicManager
  usonicManagerHandle = osThreadNew(StartUsonicManager, &nh, &usonicManager_attributes);
*/
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartI2CTask */
/**
  * @brief  Function implementing the I2C Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartI2CTask */

void StartI2CTask(void *argument)
{

  for(;;)
  {
    osDelay(1);
  }
}

/*USER CODE BEGIN Header_StartROS
*
* @brief Function implementing the rosSpin thread.
* @param argument: Not used
* @retval None

 USER CODE END Header_StartROS
void StartROS(void *argument)
{
   USER CODE BEGIN StartROS
  ros::NodeHandle* _nh = (ros::NodeHandle*)argument;
   Infinite loop
  for(;;)
  {
	  _nh->spinOnce();
  }
   USER CODE END StartROS
}
*/

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
