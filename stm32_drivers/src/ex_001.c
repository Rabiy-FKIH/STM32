/*
 * ex_001.c
 *
 *  Created on: Nov 5, 2020
 *      Author: Rabiy BEN FKIH ALI
 */
#include "stm32f446xx.h"

void Delay(void)
{
	for (uint32_t i = 0 ;i < 500000 ; i++);
}

int main(void)
{
	GPIO_Handle_t led;
	led.pGPIOx = GPIOA;
	led.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_5;
	led.GPIO_Pin_Config.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_Pin_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	led.GPIO_Pin_Config.GPIO_PinOtype = GPIO_OP_TYPE_PP;
	led.GPIO_Pin_Config.GPIO_PinPuPdCOntrol = GPIO_NO_PUPD;

	GPIO_PeriClockcontrol(led.pGPIOx , DISABLE);
	GPIO_Init(&led);
	//loop
	while(1)
	{
		GPIO_TogglePin(led.pGPIOx , led.GPIO_Pin_Config.GPIO_PinNumber);
		Delay();
	}
	return 0;
}
