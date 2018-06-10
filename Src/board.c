#include <stdarg.h>

#include "board.h"
#include "main.h"
#include "usbd_audio_if.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_dac.h"

#define TDP2016_ADDR			0xB1
#define TDP2016_FTNCTR			0x01
#define TDP2016_AGCATK			0x02
#define TDP2016_AGCREL			0x03
#define TDP2016_AGCHLD			0x04
#define TDP2016_AGCFIX			0x05
#define TDP2016_AGCCT1			0x06
#define TDP2016_AGCCT2			0x07
#define TDP2016_I2CTO			100

extern DAC_HandleTypeDef hdac1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart4;

static volatile uint32_t half_size=0;
static volatile uint32_t full_size=0;
static volatile uint32_t pointer=0;
static volatile uint8_t *audio_buff=0;


BSP_Status BSP_AudioOut_Init(uint32_t freq, uint32_t vol)
{
	// TODO: Initialize and setup external audio amplifier with the volume

	// start DMA update timer
	//HAL_TIM_Base_Start(&htim7);
	//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

	// start Timer with interrupt
	htim7.Instance->ARR = 1000;
	HAL_TIM_Base_Start_IT(&htim7);

	UART_Printf("AudioOut_Init\n\r");

	return BSP_OK;
}


BSP_Status BSP_AudioOut_DeInit(void)
{
	// TODO: Deinitialize external audio amplifier

	// stop DMA update timer
	//HAL_TIM_Base_Stop(&htim7);

	// stop Timer and its interrupt
	HAL_TIM_Base_Stop_IT(&htim7);

	UART_Printf("AudioOut_DeInit\n\r");

	return BSP_OK;
}


BSP_Status BSP_AudioOut_Start(uint8_t *buff, uint32_t size)
{
	// TODO: start audio stream

	// using DMA
	//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)buff, (size>>2), DAC_ALIGN_12B_R);
	// without using DMA
	__HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
	pointer = 0;
	audio_buff = buff;
	full_size = size;
	half_size = size>>1;
	//UART_Printf("Audio_Start:%d,%d,%d\n\r",pointer,full_size,half_size);
	__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);

	return BSP_OK;
}

BSP_Status BSP_AudioOut_Play(uint8_t *buff, uint32_t size)
{
	//HAL_GPIO_TogglePin(USR_LED1_GPIO_Port, USR_LED1_Pin);
	// TODO: keep running using
	//HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)buff, (size>>2), DAC_ALIGN_12B_R);
	// without using DMA

	__HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
	audio_buff = buff;
	pointer = 0;
	full_size = size;
	half_size = size>>1;
	//UART_Printf("Audio_Play:%d,%d,%d\n\r",pointer,full_size,half_size);
	__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);

	return BSP_OK;
}

BSP_Status BSP_AudioOut_Volume(uint8_t vol)
{
	return BSP_OK;
}

BSP_Status BSP_AudioOut_Mute(uint8_t vol)
{
	return BSP_OK;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint16_t value_x;
	int32_t value_y;
	static uint8_t state = 0;


	if((htim == &htim7) && (full_size > 0))
	{
		//HAL_GPIO_TogglePin(USR_LED1_GPIO_Port, USR_LED1_Pin);

		if(pointer < full_size)
		{
			HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

			if(pointer < 3)
			{
				//pointer = 0;
				state = 0;
			}

			value_x = (audio_buff[pointer]<<8) + audio_buff[pointer+1];

			if(value_x <= 0x7fff)
			{
				value_y = value_x;
			}
			else
			{
				value_y = value_x - 0x10000;
			}
			// bias at the center
			value_x = 0x7fff + value_y;

			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_L, value_x);
			//HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

			pointer = pointer + 4;

			if((state == 0) && (pointer >= half_size))
			{
				HAL_GPIO_WritePin(USR_LED1_GPIO_Port, USR_LED1_Pin, GPIO_PIN_SET);
				HalfTransfer_CallBack_FS();
				state = 1;
			}
			else if((state == 1) && (pointer >= full_size))
			{
				HAL_GPIO_WritePin(USR_LED1_GPIO_Port, USR_LED1_Pin, GPIO_PIN_RESET);
				TransferComplete_CallBack_FS();
			}
		}
	}
}

/**
 * DMA half transfer call back
 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	HalfTransfer_CallBack_FS();
}

/**
 * DMA full transfer call back
 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	TransferComplete_CallBack_FS();
}

BSP_Status BSP_TDP2016_ReadReg(uint8_t addr, uint8_t *data, int len)
{
	HAL_I2C_Master_Transmit(&hi2c1, TDP2016_ADDR, &addr, 1, TDP2016_I2CTO);
	HAL_I2C_Master_Receive(&hi2c1, TDP2016_ADDR, data, len, TDP2016_I2CTO);

	return BSP_OK;
}


BSP_Status BSP_TDP2016_WriteReg(uint8_t addr, uint8_t *data, int len)
{
	HAL_I2C_Master_Transmit(&hi2c1, TDP2016_ADDR, &addr, 1, TDP2016_I2CTO);
	HAL_I2C_Master_Transmit(&hi2c1, TDP2016_ADDR, data, len, TDP2016_I2CTO);

	return BSP_OK;
}

void UART_Printf(const char* format,...)
{
	char buffer[256];
	int length;
	va_list args;
	va_start(args, format);

	length = vsprintf(buffer, format, args);
	if(length)
		HAL_UART_Transmit(&huart4, (uint8_t*)buffer, length, 1000);

	va_end(args);
}
