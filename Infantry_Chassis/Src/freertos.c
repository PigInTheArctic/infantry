/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "infantry.h"
#include "ins.h"
#include "detect.h"
#include "bsp_pwm.h"
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
osThreadId calibrate_tast_handle;
osThreadId chassisTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId referee_usart_task_handle;
osThreadId UI_task_handle;
osThreadId detect_task_handle;
/* USER CODE END Variables */
osThreadId DefaultTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId INS_TaskHandle;
osThreadId DetectTaskHandle;
osThreadId UI_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartChassisTask(void const * argument);
void StartINSTask(void const * argument);
void StartDeteactTask(void const * argument);
void StartUITask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize) {
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityAboveNormal, 0, 512);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of INS_Task */
  osThreadDef(INS_Task, StartINSTask, osPriorityHigh, 0, 128);
  INS_TaskHandle = osThreadCreate(osThread(INS_Task), NULL);

  /* definition and creation of DetectTask */
  osThreadDef(DetectTask, StartDeteactTask, osPriorityLow, 0, 128);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* definition and creation of UI_Task */
  osThreadDef(UI_Task, StartUITask, osPriorityAboveNormal, 0, 256);
  UI_TaskHandle = osThreadCreate(osThread(UI_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  uint8_t red = 0, green = 0, blue = 0;
  uint8_t brightness = 0, color = 0;
  int8_t direction = 1;
  /* Infinite loop */
  for (;;) {
    red = 0;      green = 0;      blue = 0;
    switch (color) {
    case 0:        red = brightness;
      break;
    case 1:        green = brightness;
      break;
    default:        blue = brightness;
      break;
    }
    TIM_Set_PWM(&htim5, TIM_CHANNEL_1, red);
    TIM_Set_PWM(&htim5, TIM_CHANNEL_2, green);
    TIM_Set_PWM(&htim5, TIM_CHANNEL_3, blue);

    if (direction > 0) {
      if (brightness < 255)           brightness++;
      else           direction = -1;
    }
    else
      if (brightness > 0)           brightness--;
      else {
        direction = 1;
        color = (color + 1) % 3;
      }
    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  Chassis_Init();
  /* Infinite loop */
  for (;;) {
    Chassis_Task();
    osDelay(3);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
* @brief Function implementing the INS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const * argument)
{
  /* USER CODE BEGIN StartINSTask */
  INS_Init();
  /* Infinite loop */
  for (;;) {
    INS_Task();
    osDelay(2);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartDeteactTask */
/**
* @brief Function implementing the DetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDeteactTask */
void StartDeteactTask(void const * argument)
{
  /* USER CODE BEGIN StartDeteactTask */
  Detect_Init();
  /* Infinite loop */
  for (;;) {
    Detect_Task();
    osDelay(100);
  }
  /* USER CODE END StartDeteactTask */
}

/* USER CODE BEGIN Header_StartUITask */
/**
* @brief Function implementing the UI_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUITask */
void StartUITask(void const * argument)
{
  /* USER CODE BEGIN StartUITask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartUITask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
