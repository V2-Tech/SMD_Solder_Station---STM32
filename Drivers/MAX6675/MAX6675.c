/*************************************************************************************
 Title	 :  MAXIM Integrated MAX6675 Library for STM32 Using HAL Libraries
 Author  :  Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Software:  STM32CubeIDE
 Hardware:  Any STM32 device
*************************************************************************************/
#include "MAX6675.h"

// ------------------- Variables ----------------
_Bool TCF=0;                                          // Thermocouple Connection acknowledge Flag
uint8_t DATARX[2];                                    // Raw Data from MAX6675

float ActTemperature;

// ------------------- Functions ----------------
void Max6675_Read_Temp(void)
{                                       // Temperature Variable
	HAL_GPIO_WritePin(MAX6675_NSS_GPIO_Port,MAX6675_NSS_Pin,GPIO_PIN_RESET);       // Low State for SPI Communication
	HAL_SPI_Receive_DMA(&hspi1, DATARX, 1);  // DATA Transfer
}

void Max6675_Get_TempValue(float* tempartureVariable)
{
	float Temp=0;
	TCF=(((DATARX[0]|(DATARX[1]<<8))>>2)& 0x0001);        // State of Connecting
	Temp=((((DATARX[0]|DATARX[1]<<8)))>>3);               // Temperature Data Extraction
	Temp*=0.25;                                           // Data to Centigrade Conversation
	*tempartureVariable = Temp;
}

void Max6675_Read_TempValue(float* tempartureVariable)
{
	float Temp=0;                                  // Temperature Variable
	HAL_GPIO_WritePin(MAX6675_NSS_GPIO_Port,MAX6675_NSS_Pin,GPIO_PIN_RESET);       // Low State for SPI Communication
	HAL_SPI_Receive(&hspi1, DATARX, 1, 50);  // DATA Transfer
	HAL_GPIO_WritePin(MAX6675_NSS_GPIO_Port,MAX6675_NSS_Pin,GPIO_PIN_SET);         // High State for SPI Communication
	TCF=(((DATARX[0]|(DATARX[1]<<8))>>2)& 0x0001);        // State of Connecting
	Temp=((((DATARX[0]|DATARX[1]<<8)))>>3);               // Temperature Data Extraction
	Temp*=0.25;                                           // Data to Centigrade Conversation
	*tempartureVariable = Temp;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi->Instance == SPI1)
	{
		HAL_GPIO_WritePin(MAX6675_NSS_GPIO_Port,MAX6675_NSS_Pin,GPIO_PIN_SET);         // High State for SPI Communication
		Max6675_Get_TempValue(&ActTemperature);
	}
}
