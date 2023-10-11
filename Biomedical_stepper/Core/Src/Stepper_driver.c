/*
 * Stepper_drivers.c
 *
 *  Created on: Oct 5, 2022
 *      Author: Janek
 */
#include "main.h"
#include "Stepper_driver.h"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

uint32_t Timer;
uint32_t Counter;
uint16_t StepsLeft;

//Basic stepper motor functions

/*
 * ****************
 * @fn- 				Stepper_Dir
 * @brief 				Set up direction of the stepper motor
 * @param[in] 			Base adress of GPIO peripheral
 * @param[in] 			uint8_t Dir : 0 - clockwise, 1 - counter-clockwise
 *
 * @return				void
 *
 * @Note				None
 */
void Stepper_Dir(uint8_t Dir)
{
	if ((Dir==1) || (!Dir))

	{
		HAL_GPIO_WritePin(Stepper_DIR_GPIO_Port, Stepper_DIR_GPIO_Pin, Dir);
	}
}





/*
 * ****************
 * @fn- 				Microstep_Set
 * @brief 				Initialize the motor in the micro-stepping mode
 * @param[in] 			uint8_t Step : 0 - Full-step, 1 - Half-step,
 * 						2 - 1/4-step, 3 - 1/8-step, 4 - 1/16-step
 *
 * @return				void
 *
 * @Note				None
 */
void Microstep_Set(uint8_t Step)
{

	switch(Step){
	case 0:
		HAL_GPIO_WritePin(Stepper_MS1_GPIO_Port, Stepper_MS1_GPIO_Pin, 0);
		HAL_GPIO_WritePin(Stepper_MS2_GPIO_Port, Stepper_MS2_GPIO_Pin, 0);
		HAL_GPIO_WritePin(Stepper_MS3_GPIO_Port, Stepper_MS3_GPIO_Pin, 0);
		break;
	case 1:
		HAL_GPIO_WritePin(Stepper_MS1_GPIO_Port, Stepper_MS1_GPIO_Pin, 1);
		HAL_GPIO_WritePin(Stepper_MS2_GPIO_Port, Stepper_MS2_GPIO_Pin, 0);
		HAL_GPIO_WritePin(Stepper_MS3_GPIO_Port, Stepper_MS3_GPIO_Pin, 0);
		break;
	case 2:
		HAL_GPIO_WritePin(Stepper_MS1_GPIO_Port, Stepper_MS1_GPIO_Pin, 0);
		HAL_GPIO_WritePin(Stepper_MS2_GPIO_Port, Stepper_MS2_GPIO_Pin, 1);
		HAL_GPIO_WritePin(Stepper_MS3_GPIO_Port, Stepper_MS3_GPIO_Pin, 0);
		break;
	case 3:
		HAL_GPIO_WritePin(Stepper_MS1_GPIO_Port, Stepper_MS1_GPIO_Pin, 1);
		HAL_GPIO_WritePin(Stepper_MS2_GPIO_Port, Stepper_MS2_GPIO_Pin, 1);
		HAL_GPIO_WritePin(Stepper_MS3_GPIO_Port, Stepper_MS3_GPIO_Pin, 0);
		break;
	case 4:
		HAL_GPIO_WritePin(Stepper_MS1_GPIO_Port, Stepper_MS1_GPIO_Pin, 1);
		HAL_GPIO_WritePin(Stepper_MS2_GPIO_Port, Stepper_MS2_GPIO_Pin, 1);
		HAL_GPIO_WritePin(Stepper_MS3_GPIO_Port, Stepper_MS3_GPIO_Pin, 1);
		break;
	}
}

/*
 * ****************
 * @fn- 				delay_10us
 * @brief 				Basic function - function that delays time of x*10us.
 * @param[in] 			uint32_t us - number of 10s of microseconds to be delayed.
 * For high speed operation it takes shorter periods of time to be delayed.
 *
 * @return				void
 *
 * @Note				None
 */
void delay_1us (uint32_t us)
{
	uint32_t wait = us;
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < wait);  // wait for the counter to reach the us input in the parameter
}

/*
 * ****************
 * @fn- 				One_step
 * @brief 				Basic function - one step of the stepper motor
 * @param[in] 			uint8_t Time - Used in the Rotate_Once function to make
 *
 * @return				void
 *
 * @Note				None
 */
void One_step(uint32_t Time)
{
	HAL_GPIO_WritePin(Stepper_STEP_GPIO_Port, Stepper_STEP_GPIO_Pin, 1);
	delay_1us(Time);
	HAL_GPIO_WritePin(Stepper_STEP_GPIO_Port, Stepper_STEP_GPIO_Pin, 0);
	delay_1us(Time);
}

/*
 * ****************
 * @fn- 				Microstep_speed
 * @brief 				Basic function - Function returns number of 10s of microseconds to be divided by the RPM value,
 * 						based on the microstepping option. If the motor is using 4x microstepping option, it has to step
 * 						4 times as fast to maintain the same RPM value.
 * @param[in] 			uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				uint16_t Time - time to be divided by the RPM value in other stepper functions
 *
 * @Note				None
 */
uint32_t Microstep_speed(uint8_t Microstep)
{
	switch(Microstep){
	case 0:
		return 300000;
	case 1:
		return 150000;
	case 2:
		return 75000;
	case 3:
		return 37500;
	case 4:
		return 18750;
	default:
		return 0;
	}
}


/*
 * ****************
 * @fn- 				Microstep_speed
 * @brief 				Basic function - Function returns number of steps per revolution, based on the microstepping option
 * @param[in] 			uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				uint16_t Steps - Number of steps that it takes for the motor to rotate once
 *
 * @Note				None
 */
uint16_t Microstep_steps(uint8_t Microstep)
{
	switch(Microstep){
	case 0:
		return 200;
	case 1:
		return 400;
	case 2:
		return 800;
	case 3:
		return 1600;
	case 4:
		return 3200;
	default:
			return 0;
	}
}


/*
 * ****************
 * @fn- 				Set_Speed
 * @brief 				Basic function - Move with a set speed in RPM for specific time
 * @param[in] 			uint8_t Speed - sets up speed of the rotations in RPM
 *						uint8_t Time - Time in seconds
 *
 * @return				void
 *
 * @Note				None
 */
void Set_Speed(double Speed, uint32_t Time, uint8_t Microstep)
{
	Timer = 0;
	uint32_t Delay = Microstep_speed(Microstep)/Speed;
	HAL_TIM_Base_Start_IT(&htim4);
	while (Timer <= Time)
	{
		One_step(Delay);
	}
	Timer = 0;
}

/*
 * ****************
 * @fn- 				Multiple_steps
 * @brief 				Basic function - multiple steps of the stepper motor
 * @param[in] 			uint16_t Steps - Number of steps to be made by the stepper
 *						uint16_t Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 * @return				void
 *
 * @Note				None
 */
void Multiple_steps(uint16_t Steps, double Speed, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	uint32_t Time = Microstep_speed(Microstep)/Speed;
	for(uint16_t k = 0; k<Steps; k++)
	{
		One_step(Time);
	}
}

/*
 * ****************
 * @fn- 				Rotate_Once
 * @brief 				Basic function - one rotation of the stepper motor
 * @param[in] 			uint8_t Speed - sets up speed of the rotation in RPM
 *
 * @return				void
 *
 * @Note				None
 */
void Rotate_Once(double Speed, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	uint32_t Time = Microstep_speed(Microstep)/Speed;
	uint16_t Limit = Microstep_steps(Microstep);
	for(uint16_t i = 0; i<Limit; i++)
	{
		One_step(Time);
	}
}

/*
 * ****************
 * @fn- 				Rotate_Multiple
 * @brief 				Basic function - multiple rotates of the stepper motor
 * 						with set up speed
 * @param[in] 			uint8_t Speed - sets up speed of the rotations in RPM
 *						uint8_t RotateNumber - sets up number of rotations to perform
 *
 * @return				void
 *
 * @Note				None
 */
void Rotate_Multiple(double Speed, double RotateNumber, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	uint32_t Steps = RotateNumber * Microstep_steps(Microstep);
	uint32_t Time = Microstep_speed(Microstep)/Speed;

	for(uint8_t i = 0; i<Steps; i++)
	{
		One_step(Time);
	}
}

/*
 * ****************
 * @fn- 				Angle
 * @brief 				Basic function - rotate stepper by the certain angle
 * @param[in] 			uint16_t Angle - Angle to be rotated by the stepper
 *						uint16_t Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Angle(uint16_t Angle, double Speed, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	double x = (double)Microstep_steps(Microstep)/(double)360.0;
	double Limit = Angle * x;
	uint16_t Time = Microstep_speed(Microstep)/Speed;
	for(uint16_t i = 0; i<Limit; i++)
	{
		One_step(Time);
	}
}

/*
 * ****************
 * @fn- 				Fraction
 * @brief 				Basic function - rotate stepper by the certain angle
 * @param[in] 			uint16_t Angle - Angle to be rotated by the stepper
 *						uint16_t Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Fraction(double Fraction, double Speed, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	float x = (float)Microstep_steps(Microstep);
	float Limit = Fraction * x;
	uint16_t Time = Microstep_speed(Microstep)/Speed;
	for(uint16_t i = 1; i<Limit; i++)
	{
		One_step(Time);
	}
}





/*
 * ****************
 * @fn- 				Multiple_steps_IT
 * @brief 				Basic function - perform multiple steps in the interrupt mode
 * @param[in] 			uint32_t Steps - Number of steps to perform
 *						uint16_t Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Multiple_steps_IT(uint32_t Steps, double Speed, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	Counter = Microstep_speed(Microstep)/Speed;
	StepsLeft = Steps;
}


/*
 * ****************
 * @fn- 				Rotate_multiple_IT
 * @brief 				Basic function - perform multiple steps in the interrupt mode
 * @param[in] 			uint32_t Steps - Number of steps to perform
 *						uint16_t Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Rotate_multiple_IT(double Speed, uint16_t RotateNumber, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	Counter = Microstep_speed(Microstep)/Speed;
	StepsLeft = RotateNumber*Microstep_steps(Microstep);
}

/*
 * ****************
 * @fn- 				Fraction_IT
 * @brief 				Basic function - rotate stepper by the certain angle
 * @param[in] 			uint16_t Angle - Angle to be rotated by the stepper
 *						uint16_t Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Fraction_IT(double Fraction, double Speed, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	Counter = Microstep_speed(Microstep)/Speed;
	StepsLeft = Fraction * (float)Microstep_steps(Microstep);
}


/*
 * ****************
 * @fn- 				Fraction_PWM
 * @brief 				Basic function - rotate stepper by the certain angle using PWM mode
 * @param[in] 			uint16_t Angle - Angle to be rotated by the stepper
 *						uint16_t Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Fraction_PWM(double Fraction, double Speed, uint8_t Microstep)
{

	Microstep_Set(Microstep);
	float x = (float)Microstep_steps(Microstep);
	float Limit = Fraction * x;
	uint32_t Time = Microstep_speed(Microstep)/Speed;

	__HAL_TIM_SET_AUTORELOAD(&htim5, Limit);
	__HAL_TIM_SET_AUTORELOAD(&htim2, Time);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Time/2);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}


/*
 * ****************
 * @fn- 				Fraction_PWM
 * @brief 				Basic function - Multiple steps of the motor using PWM mode
 * @param[in] 			uint32_t Steps - Number of steps for the motor
 *						double Speed - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Multiple_steps_PWM(uint32_t Steps, double Speed, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	uint32_t Time = Microstep_speed(Microstep)/Speed;
	__HAL_TIM_SET_AUTORELOAD(&htim5, Steps);
	__HAL_TIM_SET_AUTORELOAD(&htim2, Time);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Time/2);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}


/*
 * ****************
 * @fn- 				Fraction_PWM
 * @brief 				Basic function - Multiple steps of the motor using PWM mode
 * @param[in] 			double Speed - Speed of the stepper motor
 *						uint16_t RotateNumber - Speed of the stepper motor
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Rotate_multiple_PWM(double Speed, double RotateNumber, uint8_t Microstep)
{
	Microstep_Set(Microstep);
	uint32_t Time = Microstep_speed(Microstep)/Speed;
	uint32_t Limit = RotateNumber * Microstep_steps(Microstep);

	__HAL_TIM_SET_AUTORELOAD(&htim5, Limit);
	__HAL_TIM_SET_AUTORELOAD(&htim2, Time);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Time/2);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}




//Stepper motor functions for medical applications

//Pompa perystaltyczna

/*
 * ****************
 * @fn- 				Peristaltic_pump
 * @brief 				Function to operate peristaltic pump. Calculates the motion of the stepper motor to provide
 * 						specific volume and flow rate of a liquid. Peristaltic pump should be calibrated and then it
 * 						can operate stepper by a formula of FlowRate = FlowRate = RPM * Coefficient
 * @param[in] 			uint16_t FlowRate
 * 						uint16_t Volume
 * 						float Coefficient
 *
 *
 * @return				void
 *
 * @Note				None
 */
void Peristaltic_infusion(double FlowRate, double Volume)
{
	Stepper_Dir(Clockwise);
	double Speed = FlowRate / PeristalticCoefficient;
	double Time = Volume/FlowRate;
	double RotateNumber = Speed*Time;
	Rotate_multiple_PWM(Speed, RotateNumber, PeristalticMicrostep);
}

//Pompa strzykawkowa

/*
 * ****************
 * @fn- 				Syringe_infusion
 * @brief				Function to start infusion of the syringe pump. The operator specifies the resolution of the linear
 * 						stepper motor, volume of the liquid, flow rate and diameter of the syringe. The RPM value and the
 * 						time of the operation are calculated.
 * @param[in]			uint16_t FlowRate   (ml/min)
 *						uint16_t Volume	    (ml)
 *						double Radius	    (mm)
 *						uint16_t Resolution (mm/revolution)
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Syringe_infusion(double FlowRate, double Volume)
{
	Stepper_Dir(Clockwise);
	double Surface = SyringeRadius*SyringeRadius * 3.14159;

	double Speed = (FlowRate*1000)/(Surface*Resolution);

	double Height = (Volume*1000)/(Surface);
	double RotateNumber = Height/Resolution;
	Rotate_multiple_PWM(Speed, RotateNumber, SyringeMicrostep);
}






/*
 * ****************
 * @fn- 				Syringe_remove
 * @brief				Function that retracts the piston from the syringe. Used to remove the piston from the syringe and
 * 						remove it from the pump.
 * @param[in]			uint16_t Height - height of the syringe (mm)
 *						double Resolution - resolution of the linear stepper motor (mm/rotation)
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 *
 * @return				void
 *
 * @Note				None
 */
void Syringe_remove()
{
	Stepper_Dir(CtrClockwise);
	Microstep_Set(SyringeMicrostep);
	double RotateNumber = SyringeHeight/Resolution;
	double Speed = RemoveSpeed/Resolution;

	Rotate_multiple_PWM(Speed,RotateNumber, SyringeMicrostep);
}






/*
 * ****************
 * @fn- 				Syringe_fill
 * @brief				Funtion used to fill up the syringe with liquid.
 * @param[in]			uint16_t Volume - Volume of the liquid to be filled into the syringe
 *						double Radius	- Radius of the syringe
 *						double Resolution - resolution of the linear stepper motor (mm/rotation)
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Syringe_fill(double Volume)
{
	Stepper_Dir(CtrClockwise);

	double Surface = SyringeRadius*SyringeRadius*3.14159;
	double Height = (Volume*1000)/Surface;
	double RotateNumber = Height/Resolution;

	double Speed = 1000*FillingSpeed/(Resolution*Surface);
	Rotate_multiple_PWM(Speed, RotateNumber, SyringeMicrostep);
}




//Pompa insulinowa

/*
 * ****************
 * @fn- 				Insulin_infusion
 * @brief				Funtion used to perform single infusion of insulin pump, for example after reading the data from the insulin sensor
 * @param[in]			uint16_t Units (1 unit = 0,01ml) - Number of ulits of insulin to be infused to the patient
 *						double Radius	- Radius of the syringe
 *						double Resolution - resolution of the linear stepper motor (mm/rotation)
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Insulin_infusion(double Units)
{
	Stepper_Dir(Clockwise);
	double Surface = InsulinRadius*InsulinRadius * 3.14159;
	double Height = (Units*10)/Surface;
	double RotateNumber = Height/Resolution;
	double Speed = (10*InsulinSpeed)/(Surface*Resolution);
	Rotate_multiple_PWM(Speed, RotateNumber, InsulinMicrostep);
}

//Robot chirugiczny i ramię typu C


/*
 * ****************
 * @fn- 				Arm_Rotation
 * @brief				Funtion used to perform single infusion of insulin pump, for example after reading the data from the insulin sensor
 * @param[in]			uint16_t Units (1 unit = 0,01ml) - Number of ulits of insulin to be infused to the patient
 *						double Radius	- Radius of the syringe
 *						double Resolution - resolution of the linear stepper motor (mm/rotation)
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */


void Stepper_Rotation(double Angle, double Speed, uint8_t Dir)
{
	Stepper_Dir(Dir);
	double StepsPerDg = Microstep_steps(RotationMicrostep)/360.0;
	double Steps = (Angle*StepsPerDg)/RotationResolution;
	double RotationSpeed = Speed/RotationResolution;
	Multiple_steps_PWM(Steps, RotationSpeed, RotationMicrostep);
}


/*
 * ****************
 * @fn- 				Arm_Progression
 * @brief				Funtion used to perform single infusion of insulin pump, for example after reading the data from the insulin sensor
 * @param[in]			uint16_t Units (1 unit = 0,01ml) - Number of ulits of insulin to be infused to the patient
 *						double Radius	- Radius of the syringe
 *						double Resolution - resolution of the linear stepper motor (mm/rotation)
 *						uint8_t Microstep - Microstepping option (0,1,2,3,4)
 *
 * @return				void
 *
 * @Note				None
 */
void Stepper_Progression(double Distance, double Speed, uint8_t Dir)
{
	Stepper_Dir(Dir);
	Microstep_Set(ProgressionMicrostep);
	double RotateNumber = Distance / ProgressionResolution;
	double RotorSteps = Speed/ProgressionResolution;
	Rotate_multiple_PWM(RotorSteps, RotateNumber, ProgressionMicrostep);
}

/* ****************
* @fn- 					Arm_Progression
* @brief				Funtion used to perform single infusion of insulin pump, for example after reading the data from the insulin sensor
* @param[in]			uint16_t Units (1 unit = 0,01ml) - Number of ulits of insulin to be infused to the patient
*						double Radius	- Radius of the syringe
*						double Resolution - resolution of the linear stepper motor (mm/rotation)
*						uint8_t Microstep - Microstepping option (0,1,2,3,4)
*
* @return				void
*
* @Note				None
*/
void Chemical_analyzer(Analyzer_Probes Probe)
{
	Microstep_Set(AnalyzerMicrostep);
	static int Position = 0;
	double StepsPerDg = Microstep_steps(AnalyzerMicrostep)/360.0;
	double Angle;

	if((Probe > Position)&&((Probe - Position)<=180))
	{
		Stepper_Dir(0);
		Angle = Probe - Position;
	}
	else if((Probe > Position)&&((Probe - Position)>180))
	{
		Stepper_Dir(1);
		Angle = 360 - (Probe - Position);
	}
	else if((Probe < Position)&&((Position - Probe)<=180))
	{
		Stepper_Dir(1);
		Angle = abs(Probe - Position);
	}
	else
	{
		Stepper_Dir(0);
		Angle = 360 - abs(Probe - Position);
	}
	double Steps = ( Angle * StepsPerDg ) / AnalyzerResolution;
	double RotationSpeed = AnalyzerSpeed/AnalyzerResolution;
	Multiple_steps_PWM(Steps, RotationSpeed, AnalyzerMicrostep);
	Position = Probe;
}


