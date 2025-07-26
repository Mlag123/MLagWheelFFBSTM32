/*
 * as5600.c
 *
 *  Created on: Jul 27, 2025
 *      Author: MLag
 */

#include "as5600.h"


//test func
uint16_t AS5600ReadAngle(void){
	uint8_t reg_h = AS5600_REG_ANGLE_H;
	uint8_t reg_l = AS5600_REG_ANGLE_L;
	uint8_t buf[2];

	HAL_I2C_Master_Transmit(&hi2c1,AS5600_ADDR,&reg_h,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1,AS5600_ADDR,&buf[0],1,HAL_MAX_DELAY);

	HAL_I2C_Master_Transmit(&hi2c1,AS5600_ADDR,&reg_l,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1,AS5600_ADDR,&buf[1],1,HAL_MAX_DELAY);

	return ((buf[0]<<8)|buf[1]&0x0FFF); //test code;

}

uint16_t AS5600_Scale(void){
	uint16_t raw = AS5600ReadAngle();
	return (uint32_t)raw* 65535/4095;
}

