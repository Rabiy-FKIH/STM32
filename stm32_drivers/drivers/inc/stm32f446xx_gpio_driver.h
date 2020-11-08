/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Nov 2, 2020
 *      Author: Rabiy BEN FKIH ALI
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;			/*possible values from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;			/*possible values from @GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;			/*possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdCOntrol;	/*possible values from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOtype;			/*possible values from @GPIO_PIN_TYPE*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_Pin_Config_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx ;					//This hold the base address of the GPIO port to which the pin belong
	GPIO_Pin_Config_t GPIO_Pin_Config;		//	this hold GPIO pin configuration setting
}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODE
 * GPIO Pin possible MODE
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AlTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_TYPE
 * GPIO Pin possible OUTPUT types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin possible OUTPUT Speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * GPIO Pull-up pull-down configuration macro
 */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2


/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin number macro
 */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15

/*
 *
 */
#define GPIOA_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 0); RCC->AHB1RSTR &=~(1<< 0); }while(0);
#define GPIOB_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 1); RCC->AHB1RSTR &=~(1<< 1); }while(0);
#define GPIOC_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 2); RCC->AHB1RSTR &=~(1<< 2); }while(0);
#define GPIOD_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 3); RCC->AHB1RSTR &=~(1<< 3); }while(0);
#define GPIOE_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 4); RCC->AHB1RSTR &=~(1<< 4); }while(0);
#define GPIOF_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 5); RCC->AHB1RSTR &=~(1<< 5); }while(0);
#define GPIOG_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 6); RCC->AHB1RSTR &=~(1<< 6); }while(0);
#define GPIOH_REG_RESET()		do{ RCC->AHB1RSTR |= (1<< 7); RCC->AHB1RSTR &=~(1<< 7); }while(0);

/********************************************************************************************
 * 							APIs supported by this driver
 * 							for more information about the APIs check the function definition
 ********************************************************************************************/
/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * Peripheral clock device
 */
void GPIO_PeriClockcontrol(GPIO_RegDef_t *GPIOx, uint8_t State);
/*
 * Data read and write
 */
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t State);
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ configuration and IRQ Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t State);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
