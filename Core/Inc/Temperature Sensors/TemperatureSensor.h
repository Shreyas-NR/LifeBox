/*
 * TemperatureSensor.h
 *
 *  Created on: 07-Aug-2020
 *      Author: shreyasnr
 */

#ifndef INC_TEMPERATURE_SENSORS_TEMPERATURESENSOR_H_
#define INC_TEMPERATURE_SENSORS_TEMPERATURESENSOR_H_

#include "main.h"
#include "I2C/i2c.h"

/**
 * MCP9808 CONNECTIONS ON SINGLE BUS
 *  A2 A1 A0 address
 *  0  0  0   0x18  this is the default address
 *  0  0  1   0x19
 *  0  1  0   0x1A
 *  0  1  1   0x1B
 *  1  0  0   0x1C
 *  1  0  1   0x1D
 *  1  1  0   0x1E
 *  1  1  1   0x1F
 *
 * Mode Resolution SampleTime
    0    0.5°C       30 ms
    1    0.25°C      65 ms
    2    0.125°C     130 ms
    3    0.0625°C    250 ms
 */

void SetTempSensorResolution(uint8_t SensorNum, uint8_t ResolutionBits);
void ReadTemperatureSensorOne();
void ReadTemperatureSensorTwo();
void ReadHeartTemperatureSensor();
void ReadHumidtySensor();
void CalculateTempAverage();
/**
 * MCP9808 REGISTER ADDRESS
 */
#define AMBIENT_TEMP_REG_ADDR	0X05
#define RESOLUTION_REG_ADDR		0X08

#define THIRTY_THREE_SAMPLES_PER_SEC	0x00
#define FIFTEEN_SAMPLES_PER_SEC			0x01
#define SEVEN_SAMPLES_PER_SEC			0x02
#define FOUR_SAMPLES_PER_SEC			0x03

#define COOLING_TEMPERATURE_SENSOR_ONE	0X01
#define COOLING_TEMPERATURE_SENSOR_TWO	0X02
/**
 * MLX90614 REGISTER ADDRESS
 */
// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x2E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F

//uint16_t AmbTempOneRegValue[2];
//uint16_t TempOneRegValue;
//float TemperatureOne;
#endif /* INC_TEMPERATURE_SENSORS_TEMPERATURESENSOR_H_ */
