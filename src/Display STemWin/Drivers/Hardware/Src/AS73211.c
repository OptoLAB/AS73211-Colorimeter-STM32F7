/*
 * AS73211.c
 *
 *  Created on: Feb 27, 2020
 *      Author: Jovan
 */

#include "AS73211.h"

I2C_HandleTypeDef hi2c2;

uint8_t _setGain;
uint8_t _setTime;
float   _FSR;
uint32_t _numOfClk;


uint8_t _channel_gain[12] ={0x00,0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0,0xB0};
uint8_t _channel_time[15] ={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E};


float _X_channel_FSR[12] ={0.866,1.732,3.463,6.927,13.854,27.707,55.414,110.828,221.657,443.314,886.628,1773.255};

float _Y_channel_FSR[12] ={0.932,1.865,3.730,7.460,14.919,29.838,59.677,119.354,238.707,477.415,954.830,1909.659};

float _Z_channel_FSR[12] ={0.501,1.002,2.003,4.006,8.012,16.024,32.048,64.097,128.194,256.387,512.774,1025.548};

int32_t _number_of_clock[15] ={1024,2048,4096,8192,16384,32768,65536,131072,262144,524288,1048576,2097152,4194304,8388608,16777216};



uint8_t AS73211_init(I2C_HandleTypeDef i2c_handle)
{
	hi2c2=i2c_handle;

	if(HAL_I2C_IsDeviceReady(&hi2c2,AS73211_ADDR, 5, 100) == HAL_OK )
		return 1;
	else
		return 0;
}

void AS73211_writeByte(uint8_t reg_addr, uint8_t byte_2_write)
{
	uint8_t data[2];
    data[0] = reg_addr;
    data[1] = byte_2_write;
	HAL_I2C_Master_Transmit(&hi2c2, AS73211_ADDR, (uint8_t *)data, 2, 1000);

	if (reg_addr ==  AS73211_CFG1_REG)
    {
        _setGain = (byte_2_write & 0xF0);
        _setTime = (byte_2_write & 0x0F);
    }
}

void AD73211_startConfiguration(void)
{
	AS73211_writeByte(AS73211_OSR_REG, 0x02);
}

void AD73211_startMeasurement(void)
{
	AS73211_writeByte(AS73211_OSR_REG, 0x83);

}

uint16_t AD73211_readXYZChannel(uint8_t XYZ_channel)
{
	uint8_t data[2];
	uint16_t value;

	HAL_I2C_Mem_Read(&hi2c2,AS73211_ADDR, XYZ_channel,1,data,2,1000);
	value=(data[1]<<8)|data[0];

	return value;

}


float color6_convertingToEe(uint8_t channel, uint16_t MRES_data)
{
    float dataValue;
    uint8_t cnt;

    for (cnt = 0; cnt < 12; cnt++)
    {
        if (_channel_gain[ cnt ] == _setGain)
        {
             if (channel == AS73211_X_CHANNEL )
             {
                 _FSR = _X_channel_FSR[ cnt ];
             }
             else if (channel == AS73211_Y_CHANNEL)
             {
                 _FSR = _Y_channel_FSR[ cnt ];
             }
             else if (channel == AS73211_Z_CHANNEL)
             {
                 _FSR = _Z_channel_FSR[ cnt ];
             }
        }
        if (_channel_time[ cnt ] == _setTime)
        {
             _numOfClk = _number_of_clock[ cnt ];
        }
    }

    dataValue = (float)(_FSR / _numOfClk) * (float)MRES_data;

    return dataValue;
}

float color6_getTemperature()
{
    uint16_t channelData;
    uint8_t readReg[ 2 ] = {0,0};
    uint16_t readData = 0;
    float floatData;

    HAL_I2C_Mem_Read(&hi2c2,AS73211_ADDR,0x01,1,readReg,2,1000);

    readData = readReg[ 1 ];
    readData = readData << 8;
    readData = readData | readReg[ 0 ];

    channelData = readData & 0x0FFF;
    floatData = (channelData * 0.05) - 66.9;
    return floatData;
}

