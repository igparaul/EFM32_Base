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

QueueHandle_t CuaLectura;
QueueHandle_t CuaPrint;

/* Structure with parameters for LedBlink */
typedef struct {
  /* Delay between blink of led */
  portTickType delay;
  /* Number of led */
  int          ledNo;
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

typedef struct{
	int16_t X;
	int16_t Y;
	int16_t Z;
} dataConverted;

uint16_t C2 (uint16_t data)
{
	if((data & 0x8000) == 0x8000)
	{
		data =~ data;
		data++;
	}
	return data;
}

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
  Axis     * pData = (Axis*) pParameters;
  BSP_I2C_Init(pData->address);

  bool res = I2C_Test();
  printf("Valor correcte (0/1): %d\n",(int)res);

}

static void calculateMovement()
{

}

static void displayMov(void *pParameters)
{
	Axis     * pData = (Axis*) pParameters;
	dataConverted data;
	for (;;)
	{
		xQueueReceive(CuaLectura, &data, 0);
		printf("Valor X: %d\n", data.X);
	    printf("Valor Y: %d\n", data.Y);
	    printf("Valor Z: %d\n\n", data.Z);
	}
}

static void ReadValues(void *pParameters)
{
  Axis     * pData = (Axis*) pParameters;
  BSP_I2C_Init(pData->address);
  uint8_t data[6];
  dataConverted dataConv;
  int16_t dataConvert[3];

  //I2C_WriteRegister(0x1E, 0x38);
  //I2C_WriteRegister(0x1F, 0x38);
  //I2C_WriteRegister(0x24, 0x01);

  I2C_WriteRegister(0x20, 0x20);

  for(;;)
  {
	  I2C_ReadRegister(pData->address_X_2, &data[0]);
	  I2C_ReadRegister(pData->address_X_1, &data[1]);
      I2C_ReadRegister(pData->address_Y_2, &data[2]);
	  I2C_ReadRegister(pData->address_Y_1, &data[3]);
      I2C_ReadRegister(pData->address_Z_2, &data[4]);
	  I2C_ReadRegister(pData->address_Z_1, &data[5]);

	  dataConv.X = (data[0] << 8) + data[1];
	  //dataConv.X = C2(dataConv.X);
	  dataConv.Y = (data[2] << 8) + data[3];
	  //dataConv.Y = C2(dataConv.Y);
	  dataConv.Z = (data[4] << 8) + data[5];
	  //dataConv.Z = C2(dataConv.Z);

      xQueueSend(CuaLectura, &dataConv, 0);
  }
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

  CuaLectura =  xQueueCreate( 10, sizeof(dataConverted));

  /* Parameters value for taks*/
  static TaskParams_t parametersToTask1 = { pdMS_TO_TICKS(1000), 0 };
  static TaskParams_t parametersToTask2 = { pdMS_TO_TICKS(500), 1 };
  static Axis parametersToTask3 = {0xD6, NULL, NULL, NULL, NULL, NULL, NULL };
  static Axis parametersToValues = {0xD6, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
  static Axis parametersToDisp = {NULL, NULL, NULL, NULL, NULL, NULL, NULL};

  /*Create two task for blinking leds*/
  xTaskCreate(LedBlink, (const char *) "LedBlink1", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
  xTaskCreate(LedBlink, (const char *) "LedBlink2", STACK_SIZE_FOR_TASK, &parametersToTask2, TASK_PRIORITY, NULL);

  // Test WhoAmI
  xTaskCreate(testWhoAmI, (const char *) "testWhoAmI", STACK_SIZE_FOR_TASK, &parametersToTask3, TASK_PRIORITY, NULL);

  // Llegir valors X Y Z
  xTaskCreate(ReadValues, (const char *) "calculateValues", STACK_SIZE_FOR_TASK, &parametersToValues, TASK_PRIORITY, NULL);

  //
  xTaskCreate(displayMov, (const char *) "displayMov", STACK_SIZE_FOR_TASK, &parametersToDisp, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();



  return 0;
}
