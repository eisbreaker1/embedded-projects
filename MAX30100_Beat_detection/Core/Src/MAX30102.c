/*
 * MAX30102.c
 *
 *  Created on: 29 kwi 2023
 *      Author: Janek
 */
#include "MAX30102.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern LEDValue IrLED;
extern LEDValue RedLED;
enum States Beat_State = IDLE_State;
extern float bpmTable[BPM_SAMPLE_SIZE];
extern uint16_t bpmIndex;
extern UART_HandleTypeDef huart2;

/*
 * @fn- 				ReadDataFromFIFO
 * @brief 				Reads the data from FIFO and stores it in a LEDBuf type variable
 * @param[in] 			uint8_t* Buf
 * @return				LEDBuf Data
 * @Note				None
 */
LEDBuf ReadDataFromFIFO(uint8_t* Buf)
{
	LEDBuf Led;
	HAL_I2C_Mem_Read(&hi2c1, MAX30100_READ_ADDR, FIFO_DATA_REG , 1, Buf, 4, 50);
	Led.IrVal = (Buf[0]<<8)|(Buf[1]<<0);
	Led.RVal = (Buf[2]<<8)|(Buf[3]<<0);
	return Led;
}

void StoreIrData(LEDBuf Led)
{
	IrLED.PrevVal = IrLED.CurrentVal;
	IrLED.CurrentVal = Led.IrVal;
}

void StoreRedData(LEDBuf Led)
{
	RedLED.PrevVal = RedLED.CurrentVal;
	RedLED.CurrentVal = Led.RVal;
}

/*
 * @fn- 				DCRemoval
 * @brief 				Applies a DC value removing filter to the unfiltered data
 * @param[in] 			LEDValue* RawLed
 *						dcFilter* Filter
 *						float alpha
 * @return				float FilteredValue
 * @Note				None
 */
float DCRemoval(LEDValue* RawLed, dcFilter* Filter, float alpha)
{
	Filter->w = RawLed->CurrentVal + alpha * Filter->prev_w;
	Filter->result = Filter->w - Filter->prev_w;
	Filter->prev_w = Filter->w;
	return Filter->result;
}

/*
 * @fn- 				meanDiff
 * @brief 				Applies a mean diff filter to the data
 * @param[in] 			float LEDval Unfiltered LED data
 *						mean_diff_filter_t* Filter - a pointer to a filter data structure
 * @return				float FilteredValue
 * @Note				None
 */
float meanDiff(float LEDval, mean_diff_filter_t* Filter)
{
	float avg = 0;

	Filter->sum -= Filter->values[Filter->index];
	Filter->values[Filter->index] = LEDval;
	Filter->sum += Filter->values[Filter->index];

	Filter->index += 1;
	Filter->index = Filter->index % FILTER_SIZE;

	if(Filter->count < FILTER_SIZE)
		Filter->count += 1;

	avg = Filter->sum / Filter->count;
	return avg;
}


/*
 * @fn- 				ButterworthFilter
 * @brief 				Applies a Butterworth filter to the data
 * @param[in] 			float LEDval Unfiltered LED data
 *						butterworth_filter_t* Filter
 * @return				float FilteredValue
 * @Note				None
 */
float ButterworthFilter(float LEDval, butterworth_filter_t* Filter)
{
	Filter->v[0] = Filter->v[1];

	//Fs = 100Hz and Fc = 10Hz
	Filter->v[1] = (2.452372752527856026e-1 * LEDval) + (0.50952544949442879485 * Filter->v[0]);

	Filter->result = Filter->v[0] + Filter->v[1];
	return Filter->result;
}


/*
 * @fn- 				ReadOperation
 * @brief 				Stores data from the buffer and stores it in global variable
 * @param[in] 			void
 *
 * @return				void
 *
 * @Note				None
 */
void ReadOperation(void)
{
	uint8_t Buf[2];
	LEDBuf Led;
	Led = ReadDataFromFIFO(Buf);
	StoreIrData(Led);
	//StoreRedData(Led);
}

/*
 * @fn- 				ReadOperation
 * @brief 				Function detects and calculates the pulse.
 * @param[in] 			float curSensorValue - IR LED data read from the sensor
 *
 * @return				uint16_t Beat - BPM value if peak is found
 * 						0 if peak is not found yet
 *
 * @Note				None
 */
uint16_t BeatDetection(float curSensorValue)
{
	static uint32_t currentBeatValue = 0;
	static uint32_t lastBeat = 0;
	static int valuesBPMCount = 0;
	static int valuesBPMSum = 0;

	if(curSensorValue > PULSE_MAX_THRESHOLD)
	{
		Beat_State = IDLE_State;
		lastBeat = 0;
		currentBeatValue = 0;
		return 0;
	}

	switch (Beat_State)
	{
	case IDLE_State:
		if(curSensorValue > PULSE_MIN_THRESHOLD)
		{
			Beat_State = PEAK_FOUND;
		}
		break;
	case PEAK_FOUND:
	{
			currentBeatValue = timestamp();
			uint32_t Duration = currentBeatValue - lastBeat;
			if (Duration < MIN_PULSE_DURATION)
			{
				Beat_State = IDLE_State;
				break;
			}
			lastBeat = currentBeatValue;

			if(valuesBPMCount < BPM_SAMPLE_SIZE)
			{
				valuesBPMCount++;
			}

			float RawBPM = 0;
			if(Duration>0)
			{
				RawBPM = 60000.0 / (float)Duration;
			}
			bpmTable[bpmIndex] = RawBPM;
			bpmIndex++;
			bpmIndex %=valuesBPMCount;
			valuesBPMSum = 0;
			for(int i=0; i<valuesBPMCount; i++)
			{
				valuesBPMSum += bpmTable[i];
			}
			lastBeat = currentBeatValue;

			Beat_State = PULSE_TRACE_DOWN;

			//return valuesBPMSum/valuesBPMCount;
			return valuesBPMSum/valuesBPMCount;

		break;
	}
	case PULSE_TRACE_DOWN:
	{
		if(curSensorValue<=PULSE_FALL_THRESHOLD)
		{
			Beat_State = IDLE_State;
		}
		break;
	}
	}

	return 0;
}

/*
 * @fn- 				MAX30100_Init
 * @brief 				Returns the time in ms
 * @param[in] 			uint32 time
 *
 * @return				void
 *
 * @Note				None
 */
uint32_t timestamp()
{
	return (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
}



/*
 * @fn- 				MAX30100_Init
 * @brief 				Initializes the MAX30100_Init sensor
 * @param[in] 			void
 *
 * @return				void
 *
 * @Note				None
 */
void MAX30100_Init(void)
{
	uint8_t Settings = MODE_HR_ONLY;
	HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR, MODE_CFG_REG, 1, &Settings, 1, 50);

	Settings = (1<<2)|(3<<0);
	HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR, SPO2_CFG_REG, 1, &Settings, 1, 50);

	Settings =(6<<0);
	HAL_I2C_Mem_Write(&hi2c1, MAX30100_ADDR, LED_CFG_REG, 1, &Settings, 1, 50);


}
