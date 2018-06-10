#ifndef __BOARD_H
#define __BOARD_H

#include <stdbool.h>
#include <stdint.h>

// note that BSP_Status is compatible with HAL_StatusTypeDef
typedef enum
{
	BSP_OK		= 0x00,
	BSP_ERROR	= 0x01,
	BSP_BUSY	= 0x02,
	BSP_TIMEOUT	= 0x03
} BSP_Status;

BSP_Status BSP_AudioOut_Init(uint32_t freq, uint32_t vol);
BSP_Status BSP_AudioOut_DeInit(void);
BSP_Status BSP_AudioOut_Start(uint8_t *buff, uint32_t size);
BSP_Status BSP_AudioOut_Play(uint8_t *buff, uint32_t size);
BSP_Status BSP_AudioOut_Volume(uint8_t vol);
BSP_Status BSP_AudioOut_Mute(uint8_t vol);

BSP_Status BSP_TDP2016_ReadReg(uint8_t addr, uint8_t *data, int len);
BSP_Status BSP_TDP2016_WriteReg(uint8_t addr, uint8_t *data, int len);

void UART_Printf(const char* format,...);

#endif	// __BOARD_H
