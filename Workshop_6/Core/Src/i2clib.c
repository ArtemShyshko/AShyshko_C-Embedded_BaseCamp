#include "i2clib.h"
#include "main.h"

I2C_HandleTypeDef hi2c1;

void turnOff(void){
	uint8_t devID = 0x80;
	uint8_t TxBuffer[2];

	TxBuffer[0] = 0x00;
	TxBuffer[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);

	TxBuffer[0] = 0xFA;
	TxBuffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);

	TxBuffer[0] = 0xFB;
	TxBuffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);

	TxBuffer[0] = 0xFC;
	TxBuffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);

	TxBuffer[0] = 0xFD;
	TxBuffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);
}

void setFreq(uint16_t PWMFreq){
	uint8_t devID = 0x80;
	uint8_t TxBuffer[2];

	uint16_t Prescale = (25000000/(4096*PWMFreq))-1;

	TxBuffer[0] = 0x00;
	TxBuffer[1] = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);

	TxBuffer[0] = 0xFE;
	TxBuffer[1] = Prescale;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);

	TxBuffer[0] = 0x00;
	TxBuffer[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);
}

void setDuty(uint16_t DutyPercent){
	uint8_t devID = 0x80;
	uint8_t TxBuffer[2];
	DutyPercent = ((0x1000*DutyPercent)/0x64)+0x199;

	TxBuffer[0] = 0x00;
	TxBuffer[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);
	
	TxBuffer[0] = 0xFA;
	TxBuffer[1] = 0x99;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);
	
	TxBuffer[0] = 0xFB;
	TxBuffer[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);
	
	TxBuffer[0] = 0xFC;
	TxBuffer[1] = DutyPercent%0x100;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);
	
	TxBuffer[0] = 0xFD;
	TxBuffer[1] = DutyPercent/0x100;
	HAL_I2C_Master_Transmit(&hi2c1, devID, (uint8_t*) &TxBuffer, 2, 1000);
}


