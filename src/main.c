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
#include <math.h>

#include "../inc/i2c.h"

#define STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

QueueHandle_t CuaLecturaGraus;
QueueHandle_t CuaLecturaGs;
QueueHandle_t CuaPrintGraus;
QueueHandle_t CuaPrintGs;

SemaphoreHandle_t SemaphoreISR;

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

typedef struct{
	char X_sign;
	int16_t X_ent;
	int16_t X_dec;
	char Y_sign;
	int16_t Y_ent;
	int16_t Y_dec;
	char Z_sign;
	int16_t Z_ent;
	int16_t Z_dec;
} dataInG;

typedef struct{
	float degX;
	float degY;
} dataGraus;

void GPIO_ODD_IRQHandler(void) {
 uint32_t aux;

 aux = GPIO_IntGet();

 /* clear flags */
 GPIO_IntClear(aux);

 xSemaphoreGiveFromISR(SemaphoreISR, NULL);
}

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

static void testWhoAmI(void *pParameters)
{
  Axis     * pData = (Axis*) pParameters;
  BSP_I2C_Init(pData->address);

  bool res = I2C_Test();
  printf("Valor correcte ( No(0)/ Si(1)): %d\n",(int)res);

}

static void calculateG()
{
	dataConverted data;
	dataInG dataG;
	for (;;)
	{
		xQueueReceive(CuaLecturaGs, &data, 0);
		dataG.X_ent = data.X * 2 / 32767;
		dataG.X_dec = (data.X * 2000 / 32767) - (dataG.X_ent * 1000);
		dataG.Y_ent = data.Y * 2 / 32767;
		dataG.Y_dec = (data.Y * 2000 / 32767) - (dataG.Y_ent * 1000);
		dataG.Z_ent = data.Z * 2 / 32767;
		dataG.Z_dec = (data.Z * 2000 / 32767) - (dataG.Z_ent * 1000);

		if (dataG.X_dec < 0)
		{
			dataG.X_dec = -dataG.X_dec;
			dataG.X_ent = -dataG.X_ent;
			dataG.X_sign = '-';
		}
		else
			dataG.X_sign = '+';

		if (dataG.Y_dec < 0)
		{
			dataG.Y_dec = -dataG.Y_dec;
			dataG.Y_ent = -dataG.Y_ent;
			dataG.Y_sign = '-';
		}
		else
			dataG.Y_sign = '+';

		if (dataG.Z_dec < 0)
		{
			dataG.Z_dec = -dataG.Z_dec;
			dataG.Z_ent = -dataG.Z_ent;
			dataG.Z_sign = '-';
		}
		else
			dataG.Z_sign = '+';

		xQueueSend(CuaPrintGs, &dataG, 0);
	}
}

static void calculateIncl()
{
	dataConverted data;
	dataGraus dataGraus;
	float xf, yf, zf;
	for (;;)
	{
		xQueueReceive(CuaLecturaGraus, &data, 0);
		xf = (float)data.X * 2 / 32767;
		yf = (float)data.Y * 2 / 32767;
		zf = (float)data.Z * 2 / 32767;

		dataGraus.degX = atanf(xf/zf)*180/3.14159265359;
		dataGraus.degY = atanf(yf/zf)*180/3.14159265359;
		xQueueSend(CuaPrintGraus, &dataGraus, 0);
	}
}

static void displayMov()
{
	dataGraus data;
	dataInG dataG;

	for (;;)
	{
		xSemaphoreTake(SemaphoreISR, portMAX_DELAY);
		xQueueReceive(CuaPrintGraus, &data, 0);
		xQueueReceive(CuaPrintGs, &dataG, 0);
		printf("Inclinacio X: %f graus\n", data.degX);
		printf("Inclinacio Y: %f graus\n\n", data.degY);
		printf("Valor X: %c%d.%d G\n", dataG.X_sign, dataG.X_ent, dataG.X_dec);
		printf("Valor Y: %c%d.%d G\n", dataG.Y_sign, dataG.Y_ent, dataG.Y_dec);
		printf("Valor Z: %c%d.%d G\n\n", dataG.Z_sign, dataG.Z_ent, dataG.Z_dec);
	}
}

static void readValues(void *pParameters)
{
  Axis     * pData = (Axis*) pParameters;
  BSP_I2C_Init(pData->address);
  uint8_t data[6];
  dataConverted dataConv;

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
	  dataConv.Y = (data[2] << 8) + data[3];
	  dataConv.Z = (data[4] << 8) + data[5];

      xQueueSend(CuaLecturaGs, &dataConv, 0);
      xQueueSend(CuaLecturaGraus, &dataConv, 0);
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

  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0); /* Boto 0 */

  SemaphoreISR = xSemaphoreCreateBinary();

  /* Set Interrupt configuration for button 0 */
  GPIO_IntConfig(gpioPortB, 9, false, true, true);

  /* Enable interrupts */
  NVIC_SetPriority(GPIO_ODD_IRQn, 255);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* Initialize SLEEP driver, no calbacks are used */
  SLEEP_Init(NULL, NULL);
#if (configSLEEP_MODE < 3)
  /* do not let to sleep deeper than define */
  SLEEP_SleepBlockBegin((SLEEP_EnergyMode_t)(configSLEEP_MODE + 1));
#endif

  CuaLecturaGs =  xQueueCreate( 10, sizeof(dataConverted));
  CuaLecturaGraus =  xQueueCreate( 10, sizeof(dataConverted));
  CuaPrintGs = xQueueCreate( 10, sizeof(dataInG));
  CuaPrintGraus = xQueueCreate( 10, sizeof(dataGraus));

  /* Parameters value for tasks*/
  //static Axis parametersWhoAmI = {0xD6, NULL, NULL, NULL, NULL, NULL, NULL };
  static Axis parametersToValues = {0xD6, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};

  // Test WhoAmI
  //xTaskCreate(testWhoAmI, (const char *) "testWhoAmI", STACK_SIZE_FOR_TASK, &parametersWhoAmI, TASK_PRIORITY, NULL);

  // Llegir valors X Y Z
  xTaskCreate(readValues, (const char *) "readValues", STACK_SIZE_FOR_TASK, &parametersToValues, TASK_PRIORITY, NULL);

  // Calcular valors en G
  xTaskCreate(calculateG, (const char *) "calculateG", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  // Calcular inclinació eix X i Y
  xTaskCreate(calculateIncl, (const char *) "calculateIncl", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  // Imprimir valors per pantalla
  xTaskCreate(displayMov, (const char *) "displayMov", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();



  return 0;
}
