//
//  libSTM_led.c
//  MatrixPilot
//
//  Created by Robert Dickenson on 17/6/2014.
//  Copyright (c) 2014 MatrixPilot. All rights reserved.
//

#include "../libUDB/libUDB.h"

//#if (BOARD_TYPE == PX4_BOARD)

#include "stm32f4xx_hal.h"
#include "gpio.h"

int16_t FindFirstBitFromLeft(int16_t val)
{
	int16_t i = 0;

	if (val != 0)
	{
		for (i = 1; i <= 16; i++)
		{
			if (val & 0x8000) break;
			val <<= 1;
		}
	}
	return i;
}

//ToTo: Use parameter x and remove fixed GPIO_PIN_5
void udb_led_toggle(uint8_t x)
{
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	switch (x)
	{
	case LED_RED:
		HAL_GPIO_TogglePin(LED1_Port, LED1);
		break;
	case LED_GREEN:
		HAL_GPIO_TogglePin(LED2_Port, LED2);
		break;
	case LED_ORANGE:
		HAL_GPIO_TogglePin(LED3_Port, LED3);
		break;
	case LED_BLUE:
		HAL_GPIO_TogglePin(LED4_Port, LED4);
		break;
	}
}

void led_on(uint8_t x)
{
//void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	switch (x)
	{
	case LED_RED:
		HAL_GPIO_WritePin(LED1_Port, LED1, RESET);
		break;
	case LED_GREEN:
		HAL_GPIO_WritePin(LED2_Port, LED2, RESET);
		break;
	case LED_ORANGE:
		HAL_GPIO_WritePin(LED3_Port, LED3, RESET);
		break;
	case LED_BLUE:
		HAL_GPIO_WritePin(LED4_Port, LED4, RESET);
		break;
	}
}
/*
LED2 GREEN
LED4 BLUE
LED1 RED
LED3 ORANGE
 */
void led_off(uint8_t x)
{
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	switch (x)
	{
	case LED_RED:
		HAL_GPIO_WritePin(LED1_Port, LED1, SET);
		break;
	case LED_GREEN:
		HAL_GPIO_WritePin(LED2_Port, LED2, SET);
		break;
	case LED_ORANGE:
		HAL_GPIO_WritePin(LED3_Port, LED3, SET);
		break;
	case LED_BLUE:
		HAL_GPIO_WritePin(LED4_Port, LED4, SET);
		break;
	}
}


//#endif // (BOARD_TYPE == PX4_BOARD)
