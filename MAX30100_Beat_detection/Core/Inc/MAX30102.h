/*
 * MAX30102.h
 *
 *  Created on: Jun 14, 2022
 *      Author: Janek
 */


#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_
#include "main.h"

#define MAX30100_ADDR 0xAE
#define MAX30100_READ_ADDR 0xAF

#define ITR_STATUS_REG      0x0
#define ITR_EN_REG  	    0x1
#define FIFO_WR_PTR_REG     0x2
#define OVR_COUNTER_REG  	0x3
#define FIFO_READ_PTR_REG   0x4
#define FIFO_DATA_REG   	0x5

#define MODE_CFG_REG   			0x6
#define SPO2_CFG_REG   			0x7
#define LED_CFG_REG 		  	0x9
#define TEMP_INEGER			   	0x16
#define TEMP_FRACTION			0x17

#define REVISION_ID				0xFE
#define PART_ID					0xFF

#define FILTER_SIZE				20
#define PULSE_MAX_THRESHOLD		3000
#define PULSE_MIN_THRESHOLD		35
#define PULSE_FALL_THRESHOLD	30
#define BPM_SAMPLE_SIZE			10

enum States
{
	IDLE_State,
	PEAK_FOUND,
	PULSE_TRACE_DOWN,
};


typedef struct LEDBuf
{
	uint16_t IrVal;
	uint16_t RVal;
}LEDBuf;

typedef struct LEDValue
{
	uint16_t CurrentVal;
	uint16_t PrevVal;
}LEDValue;

typedef struct dcFilter
{
	float w;
	float prev_w;
	float result;
}dcFilter;

typedef struct mean_diff_filter_t
{
	float values[FILTER_SIZE];
	uint8_t index;
	float sum;
	uint8_t count;
}mean_diff_filter_t;

typedef struct butterworth_filter_t
{
	 float v[2];
	 float result;
}butterworth_filter_t;


extern I2C_HandleTypeDef hi2c1;
extern LEDValue LedIr, LedR;

void MAX30100_Init(void);
LEDBuf ReadDataFromFIFO(uint8_t Buf[4]);
void ReadOperation();
void StoreIrData(LEDBuf Led);
void StoreRedData(LEDBuf Led);
float DCRemoval(LEDValue* RawLed, dcFilter* Filter, float alpha);
float meanDiff(float LEDval, mean_diff_filter_t* Filter);
float ButterworthFilter(float LEDval, butterworth_filter_t* Filter);
uint32_t timestamp();
uint16_t BeatDetection(float curSensorValue);

#endif /* INC_MAX30102_H_ */
