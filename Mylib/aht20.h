#ifndef __AHT20_H__
#define __AHT20_H__
#define AHT_ADDR	0x38<<1
#include "stm32f1xx_hal.h"
void AHT20_Init(I2C_HandleTypeDef *hi2c);
void AHT20_Read(I2C_HandleTypeDef *hi2c, float *temp, float *humidity );
#endif