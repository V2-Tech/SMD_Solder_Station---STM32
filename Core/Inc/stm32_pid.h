/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "stdbool.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32PID_H
#define __STM32PID_H

// ------------------------- Defines -------------------------
#define FILTER_LENGTH 4
#define PID_LOOP_FREQUENCY_HZ 2

#define MAXTEMPERATURE 240
#define STARTTEMPERATURE 180

#define TEMPERATURE_SAMPLE_TIME 0.25
#define DERIVATIVE_TIME_CONSTANT 0.1f

// ------------------------- Variable ------------------------
typedef struct {
	//Public
	float buffer[FILTER_LENGTH];
	float FilteredValue;
	_Bool FilterOK;

	//Private:
	uint8_t bufferIndex;
} LPFilter;

typedef struct {
	/* Public */
	float Setpoint;
	float OutputVal;

	/* Private: */
	// Controller gains
	float Kp;
	float Ki;
	float Kd;

	// Derivative low-pass filter time constant
	float PID_D_Tau;

	// Actual Controller contribution values
	float PID_P;
	float PID_I;
	float PID_D;

	// Output limits
	float MaxOutputVal;
	float MinOutputVal;

	// Wind-Up limits
	float PID_I_Max;
	float PID_I_Min;

	// Sample time (in seconds)
	float SampleTime;

	// Controller "memory" variables
	float lastError;
	float lastInValue;
	float PID_Error;
} PID;

typedef struct {
	/* Public */
	uint32_t Kp_Tuned;
	uint32_t Ki_Tuned;
	uint32_t Kd_Tuned;

	/* Private */
	double *input, *output;
	_Bool isMax, isMin;
	double setpoint;
	double noiseBand;
	int16_t controlType;
	_Bool running;
	uint32_t peak1, peak2, lastTime;
	int16_t sampleTime;
	int16_t nLookBack;
	int16_t peakType;
	double lastInputs[101];
	double peaks[10];
	int16_t peakCount;
	_Bool justchanged;
	_Bool justevaled;
	double absMax, absMin;
	double oStep;
	double outputStart;
	double Ku, Pu;
} PID_AutoTune;

// ------------------------- Functions  ----------------------
void LPFilterInit(LPFilter *filter);
float LPFilterUpdate(LPFilter *filter, float InValue);

void PIDInit(PID *PID, float SampleTime, float MaxOuputValue, float MinOutputVal, float Kp, float Ki, float Kd, float PID_I_Max, float PID_I_Min, float PID_D_Tau);
void PIDNewSetpoint(PID *PID, float Setpoint);
void PIDNewCoeff(PID *PID, float Kp, float Ki, float Kd);
uint32_t PIDUpdate(PID *PID, float ActValue);

void PID_AutoTuneInit(PID_AutoTune *PID_AutoTuneStruct, double Setpoint, double NyquistSensorResolution, float SampleTime, int16_t TunningDeep, double OutputStartVal, double OutputStepVal);
int8_t PID_AutoTuneUpdate(PID_AutoTune *PID_AutoTuneStruct, float *Input, uint32_t *Output);
void PID_AutoTuneCompute(PID_AutoTune *PID_AutoTuneStruct);
void PID_AutoTuneGetCoeff(PID_AutoTune *PID_AutoTuneStruct, PID *PID);

#endif
