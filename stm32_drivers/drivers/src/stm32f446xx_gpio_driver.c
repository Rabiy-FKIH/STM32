/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Nov 2, 2020
 *      Author: Rabiy BEN FKIH ALI
 */

#include "stm32f446xx_gpio_driver.h"

/*****************************************************************************************
 * @fn                            - GPIO_PeriClockcontrol
 * @brief                         - this function enables or disables peripherals clock for the given GPIO port
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     - ENABLE or DISABLE macros
 * @param[in]                     -
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_PeriClockcontrol(GPIO_RegDef_t *pGPIOx, uint8_t State)
{
	if (State == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN;
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN;
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN;
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN;
		}
		if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN;
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN;
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN;
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN;
		}
	}
	else if (State == DISABLE)
	{
			if (pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI;
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI;
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN;
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI;
			}
			if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI;
			}
			else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI;
			}
			else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI;
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI;
			}
		}
}

/*****************************************************************************************
 * @fn                            - GPIO_Init
 * @brief                         - this function initialize the GPIO
 * @param[in]                     - base address of the GPIO peripheral

 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//1. Configure the mode of the GPIO pin
	uint8_t temp=0;
	if (pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode <=GPIO_MODE_ANALOG)
	{
		temp=(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode << (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if (pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.Configure the FTSR
			EXTI->FTSR |=(1 << pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
			//Clear the RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.Configure the RTSR
			EXTI->RTSR |=(1 << pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
			//Clear the RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1.Configure the FTSR and RTSR
			EXTI->FTSR |=(1 << pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
			EXTI->RTSR |=(1 << pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);

		}
		//Configure the GPIO port selection in SYSCFSNFIG_EXTI
		uint8_t temp1, temp2;
		temp1=(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber / 4);
		temp2=(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber % 4);
		uint8_t portcode = (GPIO_PORT_TO_CODE(pGPIOHandle->pGPIOx));
		SYSCFG_PCLK_EN;
		SYSCFG->EXTICR[temp1] |= (portcode << 4 * temp2);


	}
	temp=0;

	//2. Configure the speed of the GPIO pin
	temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<< (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp=0;

	//3. Configure the type of the GPIO pin
	temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinOtype << pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

	//4. Configure the pull-up pull-down setting of the GPIO pin

	temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinPuPdCOntrol << (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<(2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	//5. Configure the alternate functionality of the GPIO pin
	if (pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode == GPIO_MODE_AlTFN	)
	{
		uint8_t temp1, temp2;
		temp1=pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber / 8;
		temp2=pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 4 * temp2);	//clear
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_Pin_Config.GPIO_PinAltFunMode <<(4 * temp2);
	}
	else
	{
		//do later
	}
	temp=0;
}

/*****************************************************************************************
 * @fn                            - GPIO_DeInit
 * @brief                         - this function return the peripheral to it's reset state
 * @param[in]                     - base address of the GPIO peripheral
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}


/*****************************************************************************************
 * @fn                            - GPIO_ReadFromPin
 * @brief                         - this function read data  for the given GPIO pin number
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     - Pin Number
 * @param[in]                     -
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return	value;
}

/*****************************************************************************************
 * @fn                            - GPIO_ReadFromPort
 * @brief                         - this function read data  for the given GPIO port
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     -
 *
 * @return                        - 0 or 1
 *
 * @Note                          - none
 ******************************************************************************************/
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value= (uint16_t)pGPIOx->IDR;
	return	value;
}

/*****************************************************************************************
 * @fn                            - GPIO_WriteToPin
 * @brief                         - this function write data  on the given GPIO pin number
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     - Pin Number
 * @param[in]                     -	State written it take as value "GPIO_PIN_SET" or "GPIO_PIN_RESET"
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t State)
{
	if (State == GPIO_PIN_SET)
		pGPIOx->ODR |= (0x1 <<PinNumber);
	else if (State == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(0x1 <<PinNumber);

}

/*****************************************************************************************
 * @fn                            - GPIO_WriteToPort
 * @brief                         - this function enables or disables peripherals clock for the given GPIO port
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     - Value input from the user
 * @param[in]                     -
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = (Value);
}

/*****************************************************************************************
 * @fn                            - GPIO_TogglePin
 * @brief                         - this function inverse the state of the given GPIO pin number
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     - Pin number
 * @param[in]                     -
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*****************************************************************************************
 * @fn                            - GPIO_IRQConfig
 * @brief                         - this function enables or disables peripherals clock for the given GPIO port
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     - ENABLE or DISABLE macros it
 * @param[in]                     -
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t State)
{

}

/*****************************************************************************************
 * @fn                            - GPIO_IRQHandling
 * @brief                         - this function enables or disables peripherals clock for the given GPIO port
 * @param[in]                     - base address of the GPIO peripheral
 * @param[in]                     - ENABLE or DISABLE macros Value written it take as value "GPIO_PIN_SET" or "GPIO_PIN_RESET"
 * @param[in]                     -
 *
 * @return                        - none
 *
 * @Note                          - none
 ******************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
