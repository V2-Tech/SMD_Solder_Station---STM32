/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */
#include "stm32_pid.h"

// ------------------------- Variables -------------------------
static float LPFILTER_COEFFICIENTS[FILTER_LENGTH] = {0.25f, 0.25f, 0.25f, 0.25f};

// ------------------------- Functions  ----------------------
void LPFilterInit(LPFilter *filter)
{
	/* Clear buffer */
	for (int8_t n = 0; n<FILTER_LENGTH; n++)
	{
		filter->buffer[n] = 0;
	}

	/* Reset index */
	filter->bufferIndex = 0;

	/* Reset Output value */
	filter->FilteredValue = 0;
}

float LPFilterUpdate(LPFilter *filter, float InValue)
{
	/* Store last input value into the buffer */
	filter->buffer[filter->bufferIndex] = InValue;

	/* Update buffer index */
	filter->bufferIndex++;

	if (filter->bufferIndex>=FILTER_LENGTH)
	{
		filter->bufferIndex = 0;
	}

	/* Compute output */
	filter->FilteredValue = 0;

	uint8_t ConvolvIndex = filter->bufferIndex;

	for (int8_t n = 0; n<FILTER_LENGTH; n++)
	{
		/* Shift the index (formula require [n-k] value ) */
		if (ConvolvIndex>0)
		{
			ConvolvIndex--;
		}
		else
		{
			ConvolvIndex = FILTER_LENGTH - 1;
		}
		/* Do the Convolution */
		filter->FilteredValue += LPFILTER_COEFFICIENTS[n] * filter->buffer[ConvolvIndex];
	}

	/* Return the actual filtered value output */

	return filter->FilteredValue;
}

void PIDInit(PID *PID, float SampleTime, float MaxOuputValue, float MinOutputVal, float Kp, float Ki, float Kd, float PID_I_Max, float PID_I_Min, float PID_D_Tau)
{
	/* Assign the value to the struct variable */
	PID->SampleTime = SampleTime;

	PID->MaxOutputVal = MaxOuputValue;
	PID->MinOutputVal = MinOutputVal;

	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;

	PID->PID_I_Max = PID_I_Max;
	PID->PID_I_Min = PID_I_Min;

	PID->PID_D_Tau = PID_D_Tau;

	/* Reset the memory */
	PID->Setpoint = 0;
	PID->OutputVal = 0;
	PID->lastError = 0;
	PID->lastInValue = 0;
	PID->PID_Error = 0;
}

void PIDNewSetpoint(PID *PID, float Setpoint)
{
	PID->Setpoint = Setpoint;
}

void PIDNewCoeff(PID *PID, float Kp, float Ki, float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;

	/* Reset the memory */
	PID->OutputVal = 0;
	PID->lastError = 0;
	PID->lastInValue = 0;
}

uint32_t PIDUpdate(PID *PID, float ActValue)
{
	/* Calculation of the actual error */
	PID->PID_Error = PID->Setpoint - ActValue;

	/* Calculate the PROPORTIONAL part */
	PID->PID_P = PID->Kp * PID->PID_Error;

	/* Calculate the INTEGRATIVE part */
	PID->PID_I = PID->PID_I + (0.5f * PID->Ki * PID->SampleTime * (PID->PID_Error - PID->lastError));

	//Anti-wind-up via integrator clamping
	if (PID->PID_I>PID->PID_I_Max)
	{
		PID->PID_I = PID->PID_I_Max;
	}
	else if (PID->PID_I<PID->PID_I_Min)
	{
		PID->PID_I = PID->PID_I_Min;
	}

	/* Calculate the DERIVATIVE part (band limited and measurement dependent) */
	PID->PID_D = -(2.0f * PID->Kd * (ActValue - PID->lastInValue)
				+ (2.0f * PID->PID_D_Tau - PID->SampleTime) * PID->PID_D)
				/ (2.0f * PID->PID_D_Tau + PID->SampleTime);

	/* Calculate the total PID value and check the upper/lower output limits*/
	PID->OutputVal = PID->PID_P + PID->PID_I + PID->PID_D;

	if (PID->OutputVal>PID->MaxOutputVal)
	{
		PID->OutputVal = PID->MaxOutputVal;
	}
	else if (PID->OutputVal<PID->MinOutputVal)
	{
		PID->OutputVal = PID->MinOutputVal;
	}

	/* Update the last values*/
	PID->lastError = PID->PID_Error;
	PID->lastInValue = ActValue;

	/* Return the actual PID output value */
	return (uint32_t)PID->OutputVal;
}
