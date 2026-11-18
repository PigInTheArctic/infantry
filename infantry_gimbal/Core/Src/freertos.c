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
#include "infantry.h"
#include "ins.h"
#include "remote_keyboard.h"
#include "vision.h"
#include "offline_detect.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId insTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId modeTaskHandle;
osThreadId chassisTaskHandle;
osThreadId canTaskHandle;
osThreadId detectTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartInsTask(void const * argument);
void StartGimbalTask(void const * argument);
void StartModeTask(void const * argument);
void StartChassisTask(void const * argument);
void StartCanTask(void const * argument);
void StartDetectTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of insTask */
  osThreadDef(insTask, StartInsTask, osPriorityRealtime, 0, 512);
  insTaskHandle = osThreadCreate(osThread(insTask), NULL);

  /* definition and creation of gimbalTask */
  osThreadDef(gimbalTask, StartGimbalTask, osPriorityHigh, 0, 512);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  /* definition and creation of modeTask */
  osThreadDef(modeTask, StartModeTask, osPriorityAboveNormal, 0, 256);
  modeTaskHandle = osThreadCreate(osThread(modeTask), NULL);

  /* definition and creation of chassisTask */
  osThreadDef(chassisTask, StartChassisTask, osPriorityNormal, 0, 256);
  chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, StartCanTask, osPriorityHigh, 0, 256);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* definition and creation of detectTask */
  osThreadDef(detectTask, StartDetectTask, osPriorityBelowNormal, 0, 128);
  detectTaskHandle = osThreadCreate(osThread(detectTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  UNUSED(argument);

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
    osDelay(4);
  }//上电后正常运行的情况下颜色红绿蓝交替亮暗闪烁
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartInsTask */
/**
 * @brief Function implementing the insTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInsTask */
void StartInsTask(void const * argument)
{
  /* USER CODE BEGIN StartInsTask */
  INS_Init();
  /* Infinite loop */
  for (;;) {
    INS_Task();
    osDelay(2);
  }//
  /* USER CODE END StartInsTask */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
 * @brief Function implementing the gimbalTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void const * argument)
{
  /* USER CODE BEGIN StartGimbalTask */
  GimbalInit();
  /* Infinite loop */
  for (;;) {
    GimbalTask();
    osDelay(1);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartModeTask */
/**
 * @brief Function implementing the modeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartModeTask */
void StartModeTask(void const * argument)
{
  /* USER CODE BEGIN StartModeTask */
  /* Infinite loop */
  for (;;) {
    ModeTask();
    osDelay(1);
  }
  /* USER CODE END StartModeTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
 * @brief Function implementing the chassisTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for (;;) {
    ChassisTask();
    osDelay(20);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
 * @brief Function implementing the canTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCanTask */
void StartCanTask(void const * argument)
{
  /* USER CODE BEGIN StartCanTask */
  /* Infinite loop */
  for (;;) {
    CanTask();
    osDelay(1);
  }
  /* USER CODE END StartCanTask */
}

/* USER CODE BEGIN Header_StartDetectTask */
/**
* @brief Function implementing the detectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDetectTask */
void StartDetectTask(void const * argument)
{
  /* USER CODE BEGIN StartDetectTask */
  osDelay(20000);
  Detect_Init();
  /* Infinite loop */
  for (;;) {
    Detect_task();
    osDelay(20);
  }
  /* USER CODE END StartDetectTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
