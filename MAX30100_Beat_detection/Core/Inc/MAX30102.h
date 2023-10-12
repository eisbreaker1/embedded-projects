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
#define MIN_PULSE_DURATION		200

#define MODE_UNUSED 			0
#define MODE_HR_ONLY 			(1<<1)
#define MODE_SPO2_EN			(1<<1)|(1<<0)

#define SAMPLE_RATE_50			0
#define SAMPLE_RATE_100			(1<<2)
#define SAMPLE_RATE_167			(1<<3)
#define SAMPLE_RATE_200			(1<<3)|(1<<2)
#define SAMPLE_RATE_400			(1<<4)
#define SAMPLE_RATE_600			(1<<4)|(1<<2)
#define SAMPLE_RATE_800			(1<<4)|(1<<3)
#define SAMPLE_RATE_1000		(1<<4)|(1<<3)|(1<<2)

#define PULSE_WIDTH_200			0
#define PULSE_WIDTH_400			(1<<0)
#define PULSE_WIDTH_800			(1<<1)
#define PULSE_WIDTH_1600		(1<<1)|(1<<0)

#define	IR_PA_0				0
#define	IR_PA_4_4				(1<<0)
#define	IR_PA_7_6				(1<<1)
#define	IR_PA_11				(1<<1)|(1<<0)
#define	IR_PA_14_2				(1<<2)
#define	IR_PA_17_4				(1<<2)|(1<<0)
#define	IR_PA_20_8				(1<<2)|(1<<1)
#define	IR_PA_24				(1<<2)|(1<<1)|(1<<0)
#define	IR_PA_27_1				(1<<3)
#define	IR_PA_30_6				(1<<3)|(1<<0)
#define	IR_PA_33_8				(1<<3)|(1<<1)
#define	IR_PA_37				(1<<3)|(1<<1)|(1<<0)
#define	IR_PA_40_2				(1<<3)|(1<<2)
#define	IR_PA_43_6				(1<<3)|(1<<2)|(1<<0)
#define	IR_PA_46_8				(1<<3)|(1<<2)|(1<<1)
#define	IR_PA_50				(1<<3)|(1<<2)|(1<<1)|(1<<0)

#define	RED_PA_0				0
#define	RED_PA_4_4				(1<<4)
#define	RED_PA_7_6				(1<<5)
#define	RED_PA_11				(1<<5)|(1<<4)
#define	RED_PA_14_2				(1<<6)
#define	RED_PA_17_4				(1<<6)|(1<<4)
#define	RED_PA_20_8				(1<<6)|(1<<5)
#define	RED_PA_24				(1<<6)|(1<<5)|(1<<4)
#define	RED_PA_27_1				(1<<7)
#define	RED_PA_30_6				(1<<7)|(1<<4)
#define	RED_PA_33_8				(1<<7)|(1<<5)
#define	RED_PA_37				(1<<7)|(1<<5)|(1<<4)
#define	RED_PA_40_2				(1<<7)|(1<<6)
#define	RED_PA_43_6				(1<<7)|(1<<6)|(1<<4)
#define	RED_PA_46_8				(1<<7)|(1<<6)|(1<<5)
#define	RED_PA_50				(1<<7)|(1<<6)|(1<<5)|(1<<4)



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
