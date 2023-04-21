/*
 * i2c.h
 *
 *  Created on: 21 de abr. de 2023
 *      Author: practiques
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdio.h>
#include <stdbool.h>
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "FreeRTOS.h"
#include "semphr.h"

void BSP_I2C_Init(uint8_t addr);
bool I2C_WriteRegister(uint8_t reg, uint8_t data);
bool I2C_ReadRegister(uint8_t reg, uint8_t *val);
bool I2C_Test();

#endif /* INC_I2C_H_ */
