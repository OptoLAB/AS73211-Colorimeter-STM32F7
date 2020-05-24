/*
 * AS73211.h
 *
 *  Created on: Feb 27, 2020
 *      Author: Jovan
 */

#ifndef HARDWARE_INC_AS73211_H_
#define HARDWARE_INC_AS73211_H_


#include "stm32f7xx_hal.h"


#define AS73211_OSR_REG		0x00
#define AS73211_AGEN_REG	0x02
#define AS73211_CFG1_REG	0x06
#define AS73211_CFG2_REG	0x07
#define AS73211_CFG3_REG	0x08
#define AS73211_BREAK_REG	0x09
#define AS73211_EDGES_REG	0x0A
#define AS73211_OPT_REG		0x0B

#define AS73211_X_CHANNEL   0x02
#define AS73211_Y_CHANNEL   0x03
#define AS73211_Z_CHANNEL   0x04

#define AS73211_ADDR (0x74<<1)


uint8_t AS73211_init(I2C_HandleTypeDef i2c_handle);
void AS73211_writeByte(uint8_t addr, uint8_t byte_2_write);
void AD73211_startConfiguration(void);
void AD73211_startMeasurement(void);
uint16_t AD73211_readXYZChannel(uint8_t XYZ_channel);
float color6_convertingToEe(uint8_t channel, uint16_t MRES_data);
float color6_getTemperature();

#endif /* HARDWARE_INC_AS73211_H_ */
