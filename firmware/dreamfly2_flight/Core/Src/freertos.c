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
#include "led.h"
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

/* USER CODE END Variables */
/* Definitions for flight_systems */
osThreadId_t flight_systemsHandle;
const osThreadAttr_t flight_systems_attributes = {
  .name = "flight_systems",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for flight_control */
osThreadId_t flight_controlHandle;
const osThreadAttr_t flight_control_attributes = {
  .name = "flight_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for flight_sensor */
osThreadId_t flight_sensorHandle;
const osThreadAttr_t flight_sensor_attributes = {
  .name = "flight_sensor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for flight_debug */
osThreadId_t flight_debugHandle;
const osThreadAttr_t flight_debug_attributes = {
  .name = "flight_debug",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for flight_link */
osThreadId_t flight_linkHandle;
const osThreadAttr_t flight_link_attributes = {
  .name = "flight_link",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void flight_system_task_entry(void *argument);
void flight_control_task_entry(void *argument);
void flight_sensor_task_entry(void *argument);
void flight_debug_task_entry(void *argument);
void flight_link_task_entry(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of flight_systems */
  flight_systemsHandle = osThreadNew(flight_system_task_entry, NULL, &flight_systems_attributes);

  /* creation of flight_control */
  flight_controlHandle = osThreadNew(flight_control_task_entry, NULL, &flight_control_attributes);

  /* creation of flight_sensor */
  flight_sensorHandle = osThreadNew(flight_sensor_task_entry, NULL, &flight_sensor_attributes);

  /* creation of flight_debug */
  flight_debugHandle = osThreadNew(flight_debug_task_entry, NULL, &flight_debug_attributes);

  /* creation of flight_link */
  flight_linkHandle = osThreadNew(flight_link_task_entry, NULL, &flight_link_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_flight_system_task_entry */
/**
  * @brief  Function implementing the flight_systems thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_flight_system_task_entry */
__weak void flight_system_task_entry(void *argument)
{
  /* USER CODE BEGIN flight_system_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END flight_system_task_entry */
}

/* USER CODE BEGIN Header_flight_control_task_entry */
/**
* @brief Function implementing the flight_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flight_control_task_entry */
__weak void flight_control_task_entry(void *argument)
{
  /* USER CODE BEGIN flight_control_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END flight_control_task_entry */
}

/* USER CODE BEGIN Header_flight_sensor_task_entry */
/**
* @brief Function implementing the flight_sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flight_sensor_task_entry */
__weak void flight_sensor_task_entry(void *argument)
{
  /* USER CODE BEGIN flight_sensor_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END flight_sensor_task_entry */
}

/* USER CODE BEGIN Header_flight_debug_task_entry */
/**
* @brief Function implementing the flight_debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flight_debug_task_entry */
__weak void flight_debug_task_entry(void *argument)
{
  /* USER CODE BEGIN flight_debug_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END flight_debug_task_entry */
}

/* USER CODE BEGIN Header_flight_link_task_entry */
/**
* @brief Function implementing the flight_link thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flight_link_task_entry */
__weak void flight_link_task_entry(void *argument)
{
  /* USER CODE BEGIN flight_link_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END flight_link_task_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

