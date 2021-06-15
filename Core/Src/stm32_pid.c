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

	/* Reset Output values */
	filter->FilteredValue = 0;
	filter->FilterOK = false;
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
		filter->FilterOK = true;
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

void PID_AutoTuneInit(PID_AutoTune *PID_AutoTuneStruct, double Setpoint, double NyquistSensorResolution,
						float SampleTime/*Second*/, int16_t TunningDeep, double OutputStartVal, double OutputStepVal)
{
	PID_AutoTuneStruct->setpoint = Setpoint;
	PID_AutoTuneStruct->noiseBand = NyquistSensorResolution;
	PID_AutoTuneStruct->sampleTime = SampleTime * 1000;
	PID_AutoTuneStruct->nLookBack = TunningDeep * 4;
	PID_AutoTuneStruct->outputStart = OutputStartVal;
	PID_AutoTuneStruct->oStep = OutputStepVal;
}

int8_t PID_AutoTuneUpdate(PID_AutoTune *PID_AutoTuneStruct, float *Input, uint32_t *Output)
{
	PID_AutoTuneStruct->justevaled = false;
	if(PID_AutoTuneStruct->peakCount>9 && PID_AutoTuneStruct->running)
	{
		PID_AutoTuneStruct->running = false;
		*Output = PID_AutoTuneStruct->outputStart;
		PID_AutoTuneCompute(PID_AutoTuneStruct);
		return 1;
	}

	uint32_t now = xTaskGetTickCount();
	if((now - PID_AutoTuneStruct->lastTime) < PID_AutoTuneStruct->sampleTime)
	{
		return false;
	}
	PID_AutoTuneStruct->lastTime = now;

	double refVal = *Input;
	PID_AutoTuneStruct->justevaled = true;

	if(!PID_AutoTuneStruct->running)
	{ //initialize working variables the first time around
		PID_AutoTuneStruct->peakType = 0;
		PID_AutoTuneStruct->peakCount = 0;
		PID_AutoTuneStruct->justchanged = false;
		PID_AutoTuneStruct->absMax = refVal;
		PID_AutoTuneStruct->absMin = refVal;
		PID_AutoTuneStruct->setpoint = refVal;
		PID_AutoTuneStruct->running = true;
		PID_AutoTuneStruct->outputStart = *Output;
		*Output = PID_AutoTuneStruct->outputStart + PID_AutoTuneStruct->oStep;
	}
	else
	{
		if(refVal>PID_AutoTuneStruct->absMax)
		{
			PID_AutoTuneStruct->absMax = refVal;
		}
		if(refVal<PID_AutoTuneStruct->absMin)
		{
			PID_AutoTuneStruct->absMin = refVal;
		}
	}

	//oscillate the output base on the input's relation to the setpoint

	if(refVal>PID_AutoTuneStruct->setpoint + PID_AutoTuneStruct->noiseBand)
	{
		*Output = PID_AutoTuneStruct->outputStart - PID_AutoTuneStruct->oStep;
	}
	else if (refVal<PID_AutoTuneStruct->setpoint - PID_AutoTuneStruct->noiseBand)
	{
		*Output = PID_AutoTuneStruct->outputStart + PID_AutoTuneStruct->oStep;
	}

	PID_AutoTuneStruct->isMax = true;
	PID_AutoTuneStruct->isMin = true;

	for(int i=PID_AutoTuneStruct->nLookBack-1; i>=0 ;i--)
	{
		double val = PID_AutoTuneStruct->lastInputs[i];
		if(PID_AutoTuneStruct->isMax)
		{
			PID_AutoTuneStruct->isMax = refVal>val;
		}
		if(PID_AutoTuneStruct->isMin)
		{
			PID_AutoTuneStruct->isMin = refVal<val;
		}
		PID_AutoTuneStruct->lastInputs[i+1] = PID_AutoTuneStruct->lastInputs[i];
	}
	PID_AutoTuneStruct->lastInputs[0] = refVal;
	if (PID_AutoTuneStruct->nLookBack<9)
	{
		return 0;
	}

	if(PID_AutoTuneStruct->isMax)
	{
		if(PID_AutoTuneStruct->peakType==0)
		{
			PID_AutoTuneStruct->peakType = 1;
		}
	    if(PID_AutoTuneStruct->peakType==-1)
	    {
	    	PID_AutoTuneStruct->peakType = 1;
	    	PID_AutoTuneStruct->justchanged = true;
	    	PID_AutoTuneStruct->peak2 = PID_AutoTuneStruct->peak1;
	    }
	    PID_AutoTuneStruct->peak1 = now;
	    PID_AutoTuneStruct->peaks[PID_AutoTuneStruct->peakCount] = refVal;

	  }
	  else if(PID_AutoTuneStruct->isMin)
	  {
	    if(PID_AutoTuneStruct->peakType==0)
	    {
	    	PID_AutoTuneStruct->peakType = -1;
	    }
	    if(PID_AutoTuneStruct->peakType==1)
	    {
	    	PID_AutoTuneStruct->peakType = -1;
	    	PID_AutoTuneStruct->peakCount++;
	    	PID_AutoTuneStruct->justchanged = true;
	    }

	    if(PID_AutoTuneStruct->peakCount<10)
	    {
	    	PID_AutoTuneStruct->peaks[PID_AutoTuneStruct->peakCount] = refVal;
	    }
	  }

	  if(PID_AutoTuneStruct->justchanged && PID_AutoTuneStruct->peakCount>2)
	  { //we've transitioned.  check if we can autotune based on the last peaks
	    double avgSeparation = (abs(PID_AutoTuneStruct->peaks[PID_AutoTuneStruct->peakCount-1] -
	    		PID_AutoTuneStruct->peaks[PID_AutoTuneStruct->peakCount-2]) +
	    		abs(PID_AutoTuneStruct->peaks[PID_AutoTuneStruct->peakCount-2] -
	    				PID_AutoTuneStruct->peaks[PID_AutoTuneStruct->peakCount-3]))/2;
	    if (avgSeparation < 0.05*(PID_AutoTuneStruct->absMax - PID_AutoTuneStruct->absMin))
	    {
	    	*Output = PID_AutoTuneStruct->outputStart;
	    	PID_AutoTuneCompute(PID_AutoTuneStruct);
			PID_AutoTuneStruct->running = false;
			return 1;
	    }
	  }
	  PID_AutoTuneStruct->justchanged = false;
	  return 0;
}

void PID_AutoTuneCompute(PID_AutoTune *PID_AutoTuneStruct)
{
	PID_AutoTuneStruct->Ku = 4 * (2 * PID_AutoTuneStruct->oStep)/((PID_AutoTuneStruct->absMax - PID_AutoTuneStruct->absMin)*3.14159);
	PID_AutoTuneStruct->Pu = (double)(PID_AutoTuneStruct->peak1 - PID_AutoTuneStruct->peak2) / 1000;
}

void PID_AutoTuneGetCoeff(PID_AutoTune *PID_AutoTuneStruct, PID *PID)
{
	if (PID->Kd != 0)
	{
		PID_AutoTuneStruct->Kp_Tuned = 0.6 * PID_AutoTuneStruct->Ku;
		PID_AutoTuneStruct->Ki_Tuned = 1.2 * PID_AutoTuneStruct->Ku / PID_AutoTuneStruct->Pu;
		PID_AutoTuneStruct->Kd_Tuned = 0.075 * PID_AutoTuneStruct->Ku * PID_AutoTuneStruct->Pu;
	}
	else
	{
		PID_AutoTuneStruct->Kp_Tuned = 0.4 * PID_AutoTuneStruct->Ku;
		PID_AutoTuneStruct->Ki_Tuned = 0.48 * PID_AutoTuneStruct->Ku / PID_AutoTuneStruct->Pu;
		PID_AutoTuneStruct->Kd_Tuned = 0;
	}
}

void PID_AutoTuneStop(PID_AutoTune *PID_AutoTuneStruct)
{
	PID_AutoTuneStruct->running = false;
}
