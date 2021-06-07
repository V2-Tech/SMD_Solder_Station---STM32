/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */
#include "main.h"
#include "stm32f1xx_hal.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32PID_H
#define __STM32PID_H

// ------------------------- Defines -------------------------
#define FILTER_LENGTH 4

#define TEMPERATURE_SAMPLE_TIME 0.25
#define MAXTEMPERATURE 240
#define DERIVATIVE_TIME_CONSTANT 0.02f

// ------------------------- Variable ------------------------
typedef struct {
	float buffer[FILTER_LENGTH];
	uint8_t bufferIndex;
	float FilteredValue;
} LPFilter;

typedef struct {
	//Public
	float Setpoint;
	float OutputVal;

	/* Actual Controller contribution values */
	float PID_P;
	float PID_I;
	float PID_D;

	//Private:
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float PID_D_Tau;

	/* Output limits */
	float MaxOutputVal;
	float MinOutputVal;

	/* Wind-Up limits */
	float PID_I_Max;
	float PID_I_Min;

	/* Sample time (in seconds) */
	float SampleTime;

	/* Controller "memory" variables */
	float lastError;
	float lastInValue;
	float PID_Error;
} PID;

// ------------------------- Functions  ----------------------
void LPFilterInit(LPFilter *filter);
float LPFilterUpdate(LPFilter *filter, float InValue);

void PIDInit(PID *PID, float SampleTime, float MaxOuputValue, float MinOutputVal, float Kp, float Ki, float Kd, float PID_I_Max, float PID_I_Min, float PID_D_Tau);
void PIDNewSetpoint(PID *PID, float Setpoint);
void PIDNewCoeff(PID *PID, float Kp, float Ki, float Kd);
uint32_t PIDUpdate(PID *PID, float ActValue);

#endif
