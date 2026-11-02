/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
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

/* USER CODE END Variables */
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for ActuatorTask */
osThreadId_t ActuatorTaskHandle;
const osThreadAttr_t ActuatorTask_attributes = {
  .name = "ActuatorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for SensorImuTask */
osThreadId_t SensorImuTaskHandle;
const osThreadAttr_t SensorImuTask_attributes = {
  .name = "SensorImuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for OLEDPowerTask */
osThreadId_t OLEDPowerTaskHandle;
const osThreadAttr_t OLEDPowerTask_attributes = {
  .name = "OLEDPowerTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for GrabTask */
osThreadId_t GrabTaskHandle;
const osThreadAttr_t GrabTask_attributes = {
  .name = "GrabTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for RemoteTask */
osThreadId_t RemoteTaskHandle;
const osThreadAttr_t RemoteTask_attributes = {
  .name = "RemoteTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for remote_chassis */
osMessageQueueId_t remote_chassisHandle;
uint8_t remote_chassisBuffer[ 6 * sizeof( uint16_t ) ];
osStaticMessageQDef_t remote_chassisControlBlock;
const osMessageQueueAttr_t remote_chassis_attributes = {
  .name = "remote_chassis",
  .cb_mem = &remote_chassisControlBlock,
  .cb_size = sizeof(remote_chassisControlBlock),
  .mq_mem = &remote_chassisBuffer,
  .mq_size = sizeof(remote_chassisBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartChassisTask(void *argument);
void StartActuatorTask(void *argument);
void StartSafetyTask(void *argument);
void StartSensorImuTask(void *argument);
void StartOLEDPower(void *argument);
void StartGrabTask(void *argument);
void StartRemoteTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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

  /* Create the queue(s) */
  /* creation of remote_chassis */
  remote_chassisHandle = osMessageQueueNew (6, sizeof(uint16_t), &remote_chassis_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of ActuatorTask */
  ActuatorTaskHandle = osThreadNew(StartActuatorTask, NULL, &ActuatorTask_attributes);

  /* creation of SafetyTask */
  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);

  /* creation of SensorImuTask */
  SensorImuTaskHandle = osThreadNew(StartSensorImuTask, NULL, &SensorImuTask_attributes);

  /* creation of OLEDPowerTask */
  OLEDPowerTaskHandle = osThreadNew(StartOLEDPower, NULL, &OLEDPowerTask_attributes);

  /* creation of GrabTask */
  GrabTaskHandle = osThreadNew(StartGrabTask, NULL, &GrabTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(StartRemoteTask, NULL, &RemoteTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartChassisTask */
/**
  * @brief  Function implementing the ChassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassisTask */
__weak void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartActuatorTask */
/**
* @brief Function implementing the ActuatorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartActuatorTask */
__weak void StartActuatorTask(void *argument)
{
  /* USER CODE BEGIN StartActuatorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartActuatorTask */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the SafetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
__weak void StartSafetyTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSafetyTask */
}

/* USER CODE BEGIN Header_StartSensorImuTask */
/**
* @brief Function implementing the SensorImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorImuTask */
__weak void StartSensorImuTask(void *argument)
{
  /* USER CODE BEGIN StartSensorImuTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensorImuTask */
}

/* USER CODE BEGIN Header_StartOLEDPower */
/**
* @brief Function implementing the OLEDPowerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLEDPower */
__weak void StartOLEDPower(void *argument)
{
  /* USER CODE BEGIN StartOLEDPower */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartOLEDPower */
}

/* USER CODE BEGIN Header_StartGrabTask */
/**
* @brief Function implementing the GrabTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGrabTask */
__weak void StartGrabTask(void *argument)
{
  /* USER CODE BEGIN StartGrabTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGrabTask */
}

/* USER CODE BEGIN Header_StartRemoteTask */
/**
* @brief Function implementing the RemoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteTask */
__weak void StartRemoteTask(void *argument)
{
  /* USER CODE BEGIN StartRemoteTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRemoteTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

