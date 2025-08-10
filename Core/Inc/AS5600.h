/*
 * as5600.h
 *
 *  Created on: Aug 10, 2025
 *      Author: MLag
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_



#endif /* INC_AS5600_H_ */


#include "stm32f1xx_it.h"
//#include "stm32f1xx_hal.h"
#include "main.h"
#define AS5600_ADDR        (0x36 << 1)
#define AS5600_ANGLE_REG   0x0E
uint16_t getAngle(void);


#define CONFIG_REG_HIGH     0x07
#define CONFIG_REG_LOW      0x08

void AS5600_SetSlowFilter(void);
