/*
 * Stepper_drivers.h
 *
 *  Created on: Oct 5, 2022
 *      Author: Janek
 */
//Basic funtion prototypes


typedef enum
{
	PROBE_1 = 0,
	PROBE_2 = 36,
	PROBE_3 = 72,
	PROBE_4 = 108,
	PROBE_5 = 144,
	PROBE_6 = 180,
	PROBE_7 = 216,
	PROBE_8 = 252,
	PROBE_9 = 288,
	PROBE_10 = 324,

} Analyzer_Probes;

void Stepper_Dir(uint8_t Dir);
void Microstep_Set(uint8_t Step);
void One_step( uint32_t Time);
void Set_Speed(double Speed, uint32_t Time, uint8_t Microstep);
void Rotate_Once(double Speed, uint8_t Microstep);
void Rotate_Multiple(double Speed, double RotateNumber, uint8_t Microstep);
void delay_1us (uint32_t us);
void Angle(uint16_t Angle, double Speed, uint8_t Microstep);
void Multiple_steps(uint16_t Steps, double Speed, uint8_t Microstep);
uint32_t Microstep_speed(uint8_t Microstep);
uint16_t Microstep_steps(uint8_t Microstep);
void Fraction(double Fraction, double Speed, uint8_t Microstep);
void Fraction_IT(double Fraction, double Speed, uint8_t Microstep);
void Multiple_steps_IT(uint32_t Steps, double Speed, uint8_t Microstep);
void Rotate_multiple_IT(double Speed, uint16_t RotateNumber, uint8_t Microstep);

void Fraction_PWM(double Fraction, double Speed, uint8_t Microstep);
void Multiple_steps_PWM(uint32_t Steps, double Speed, uint8_t Microstep);
void Rotate_multiple_PWM(double Speed, double RotateNumber, uint8_t Microstep);

//Medical function prototypes
void Peristaltic_infusion(double FlowRate, double Volume);
void Syringe_infusion(double FlowRate, double Volume);
void Syringe_remove();
void Syringe_fill(double Volume);
void Insulin_infusion(double Units);
void Stepper_Rotation(double Angle, double Speed, uint8_t Dir);
void Stepper_Progression(double Distance, double Speed, uint8_t Dir);
void Chemical_analyzer(Analyzer_Probes Probe);

#ifndef STEPPER_DRIVER_H_
#define STEPPER_DRIVER_H_

#define Clockwise 1
#define CtrClockwise 0
#define SyringeRadius 30.0
#define Resolution 1.0
#define SyringeMicrostep 4
#define SyringeHeight 150
#define RemoveSpeed 200
#define FillingSpeed 20

#define PeristalticMicrostep 4
#define PeristalticCoefficient 3

#define InsulinMicrostep 1
#define InsulinSpeed 20.0
#define InsulinRadius 10.0

#define RotationMicrostep 4
#define RotationResolution 0.1

#define ProgressionMicrostep 4
#define ProgressionResolution 0.5
#define Forwards 1
#define Backwards 0


#define AnalyzerMicrostep 4
#define AnalyzerSpeed 10
#define AnalyzerResolution 1


#endif /* STEPPER_DRIVER_H_ */
