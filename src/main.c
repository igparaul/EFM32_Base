/***************************************************************************//**
 * @file
 * @brief FreeRTOS Blink Demo for Energy Micro EFM32GG_STK3700 Starter Kit
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"

#include "em_chip.h"
#include "bsp.h"
#include "bsp_trace.h"

#include "sleep.h"

#include "../inc/i2c.h"

#define STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

/* Structure with parameters for LedBlink */
typedef struct {
  /* Delay between blink of led */
  portTickType delay;
  /* Number of led */
  int          ledNo;
  uint8_t address;
} TaskParams_t;

typedef struct {
  uint8_t address;
  uint8_t address_X_1;
  uint8_t address_X_2;
  uint8_t address_Y_1;
  uint8_t address_Y_2;
  uint8_t address_Z_1;
  uint8_t address_Z_2;
} Axis;

/***************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 ******************************************************************************/
static void LedBlink(void *pParameters)
{
  TaskParams_t     * pData = (TaskParams_t*) pParameters;
  const portTickType delay = pData->delay;

  for (;; ) {
    BSP_LedToggle(pData->ledNo);
    vTaskDelay(delay);
  }
}

static void testWhoAmI(void *pParameters)
{
  TaskParams_t     * pData = (TaskParams_t*) pParameters;
  BSP_I2C_Init(pData->address);

  bool res = I2C_Test();
  printf("Valor correcte (0/1): %d\n",(int)res);

}

static void calculateValues(void *pParameters)
{
  Axis     * pData = (Axis*) pParameters;
  BSP_I2C_Init(pData->address);
  uint16_t data[6];
  uint16_t dataConvert[3];

  for(;;)
  {
	  I2C_ReadRegister(pData->address_X_1, &data[0]);
	  I2C_ReadRegister(pData->address_X_2, &data[1]);
      I2C_ReadRegister(pData->address_Y_1, &data[2]);
	  I2C_ReadRegister(pData->address_Y_2, &data[3]);
      I2C_ReadRegister(pData->address_Z_1, &data[4]);
	  I2C_ReadRegister(pData->address_Z_2, &data[5]);

      for (int i = 0; i < 3; i++)
      {
            dataConvert[i] = (data[i*2] << 16) + data[i*2+1];
      }
  }

  printf("Valor X1: %d\n",(int)res);

}

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  /* If first word of user data page is non-zero, enable Energy Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize LED driver */
  BSP_LedsInit();
  /* Setting state of leds*/
  BSP_LedSet(0);
  BSP_LedSet(1);

  /* Initialize SLEEP driver, no calbacks are used */
  SLEEP_Init(NULL, NULL);
#if (configSLEEP_MODE < 3)
  /* do not let to sleep deeper than define */
  SLEEP_SleepBlockBegin((SLEEP_EnergyMode_t)(configSLEEP_MODE + 1));
#endif

  /* Parameters value for taks*/
  static TaskParams_t parametersToTask1 = { pdMS_TO_TICKS(1000), 0, NULL };
  static TaskParams_t parametersToTask2 = { pdMS_TO_TICKS(500), 1, NULL };
  static TaskParams_t parametersToTask3 = { NULL, NULL, 0xD6 };
  static Axis parametersToValues = {0xD6, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};

  /*Create two task for blinking leds*/
  xTaskCreate(LedBlink, (const char *) "LedBlink1", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
  xTaskCreate(LedBlink, (const char *) "LedBlink2", STACK_SIZE_FOR_TASK, &parametersToTask2, TASK_PRIORITY, NULL);

  // Test WhoAmI
  xTaskCreate(testWhoAmI, (const char *) "testWhoAmI", STACK_SIZE_FOR_TASK, &parametersToTask3, TASK_PRIORITY, NULL);

  // Calcular valors X Y Z
  xTaskCreate(calculateValues, (const char *) "calculateValues", STACK_SIZE_FOR_TASK, &parametersToValues, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();



  return 0;
}