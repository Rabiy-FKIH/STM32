/*
 * ex_002.c
 *
 *  Created on: Nov 6, 2020
 *      Author: Rabiy BEN FKI ALI
 */
#include "stm32f446xx.h"

void Delay(void)
{
	for (uint32_t i = 0 ;i < 500000 ; i++);
}

#define HIGH				1
#define BUTTON_PRESSED		HIGH

int main(void)
{
	GPIO_Handle_t led , Button;
	led.pGPIOx = GPIOA;
	led.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_5;
	led.GPIO_Pin_Config.GPIO_PinMode = GPIO_MODE_OUT;
	led.GPIO_Pin_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	led.GPIO_Pin_Config.GPIO_PinOtype = GPIO_OP_TYPE_PP;
	led.GPIO_Pin_Config.GPIO_PinPuPdCOntrol = GPIO_NO_PUPD;

	Button.pGPIOx = GPIOC;
	Button.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_13;
	Button.GPIO_Pin_Config.GPIO_PinMode = GPIO_MODE_IN;
	Button.GPIO_Pin_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_Pin_Config.GPIO_PinPuPdCOntrol = GPIO_NO_PUPD;

	GPIO_PeriClockcontrol(led.pGPIOx , ENABLE);
	GPIO_Init(&led);

	GPIO_PeriClockcontrol(Button.pGPIOx , ENABLE);
	GPIO_Init(&Button);
	//loop
	while(1)
	{
		if (GPIO_ReadFromPin(Button.pGPIOx , 13) == BUTTON_PRESSED)
		{
			GPIO_TogglePin(led.pGPIOx , led.GPIO_Pin_Config.GPIO_PinNumber);
			Delay();
		}

	}
	return 0;
}

