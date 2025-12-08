#include "aht20.h"
void AHT20_Init(I2C_HandleTypeDef *hi2c){
	HAL_Delay(40);
	uint8_t status;
	HAL_I2C_Mem_Read(hi2c,AHT_ADDR,0x71,1,&status,1,1000);
	if ((status>>3 & 0x01) == 0){
		uint8_t init_commands[3] = {0xBE, 0x08, 0x00};
		HAL_I2C_Master_Transmit(hi2c,AHT_ADDR,init_commands,3,1000);
		HAL_Delay(10);
	}
}
void AHT20_Read(I2C_HandleTypeDef *hi2c, float *temp, float *humidity ){
	uint8_t measure_command[3] = {0xAC, 0x33, 0x00};
	HAL_I2C_Master_Transmit(hi2c,AHT_ADDR,measure_command,3,1000);
	HAL_Delay(80);
	uint8_t status;
	do {
		HAL_I2C_Mem_Read(hi2c, AHT_ADDR, 0x71, 1, &status, 1, 1000);
		HAL_Delay(100);
	}
	while ((status>>7 & 0x01) == 1);
	uint8_t RxData[7];
	HAL_I2C_Master_Receive(hi2c, AHT_ADDR, RxData, 7, 1000);
	uint32_t h20=(RxData[1]<<12 | RxData[2] << 4 | RxData[3] >> 4);
	uint32_t t20=( (RxData[3]&0x0F)<<16 | RxData[4] << 8 | RxData[5] );
	*temp=(t20 / 1048576.0)*200.0 - 50.0;
	*humidity= h20 / 10485.76;
}