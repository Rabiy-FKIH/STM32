/*
 * stm32f446xx.h
 *
 *  Created on: Oct 31, 2020
 *      Author: The Think Tank
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM 						SRAM1BASE_ADDR

#define PERIPH_BASE					0x40000000U
#define APB1_BASEADDR				PERIPH_BASE
#define APB2_BASEADDR				0x40010000U
#define AHB1_BASEADDR				0x40020000U
#define AHB2_BASEADDR				0x50000000U
#define AHB3_BASEADDR				0xA0001000U

/* Base addresses of peripherals which are hanging on AHB1 bus */
#define GPIOA_BASEADDR				(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1_BASEADDR + 0x1C00)
#define RCC_BASEADDR				(AHB1_BASEADDR + 0x3800)

/* Base addresses of peripherals which are hanging on APB1 bus */
#define SPI2_BASEADDR				(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR				(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1_BASEADDR + 0x5000)
#define I2C1_BASEADDR				(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1_BASEADDR + 0x5C00)


/* Base addresses of peripherals which are hanging on APB2 bus */
#define USART1_BASEADDR				(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2_BASEADDR + 0x1400)
#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR				(APB2_BASEADDR + 0x3C00)


/******************  Peripheral register definition structure ************************/

typedef struct {
	__vo uint32_t MODER;		//GPIO port mode register (GPIOx_MODER) (x = A..H)					Address offset: 0x00	|	Reset values:{ 0xA800 0000 for port A ; 0x0000 0280 for port B ;  0x0000 0000 for other ports }
	__vo uint32_t OTYPER;		//GPIO port output type register (GPIOx_OTYPER) (x = A..H)			Address offset: 0x04	|	Reset value: 0x0000 0000
	__vo uint32_t OSPEEDR;		//GPIO port output speed register (GPIOx_OSPEEDR) (x = A..H)		Address offset: 0x08	|	Reset values: { 0x0000 00C0 for port B ; 0x0000 0000 for other ports }
	__vo uint32_t PUPDR;		//GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..H)		Address offset: 0x0C	|	Reset values: { 0x6400 0000 for port A ; 0x0000 0100 for port B ; 0x0000 0000 for other ports }
	__vo uint32_t IDR;			//GPIO port input data register (GPIOx_IDR) (x = A..H)				Address offset: 0x10	|	Reset value: 0x0000 XXXX (where X means undefined) }
	__vo uint32_t ODR;			//GPIO port output data register (GPIOx_ODR) (x = A..H)				Address offset: 0x14	|	Reset value: 0x0000 0000
	__vo uint32_t BSRR;			//GPIO port output data register (GPIOx_ODR) (x = A..H)				Address offset: 0x14	|	Reset value: 0x0000 0000
	__vo uint32_t LCKR;			//GPIO port configuration lock register (GPIOx_LCKR) (x = A..H)		Address offset: 0x1C	|	Reset value: 0x0000 0000
	__vo uint32_t AFR[2];		//GPIO alternate function register AFR[0]= AFLR ; AFR[1]=AFHR		Address offset: 0x20	|	Reset value: 0x0000 0000

}GPIO_RegDef_t;


typedef struct {
	__vo uint32_t CR;            	/*RCC clock control register                                       Address offset: 0x00*/
	__vo uint32_t PLLCFG;			/*RCC PLL configuration register			                       Address offset: 0x04*/
	__vo uint32_t CFG;              /*RCC clock configuration register                                 Address offset: 0x08*/
	__vo uint32_t CIR;              /*RCC clock interrupt register                                     Address offset: 0x0C*/
	__vo uint32_t AHB1RSTR;         /*RCC AHB1 peripheral reset register                               Address offset: 0x10*/
	__vo uint32_t AHB2RSTR;         /*RCC AHB2 peripheral reset register                               Address offset: 0x14*/
	__vo uint32_t AHB3RSTR;         /*RCC AHB3 peripheral reset register                               Address offset: 0x18*/
	__vo uint32_t RESERVED0;		/*0x1C                                                                                 */
	__vo uint32_t APB1RSTR;         /*RCC APB1 peripheral reset register                               Address offset: 0x20*/
	__vo uint32_t APB2RSTR;         /*RCC APB2 peripheral reset register                               Address offset: 0x24*/
	__vo uint32_t RESERVED1[2];		/*0x28-> 0x2C*/
	__vo uint32_t AHB1ENR;          /*RCC AHB1 peripheral clock register                               Address offset: 0x30*/
	__vo uint32_t AHB2ENR;          /*RCC AHB2 peripheral clock enable register                        Address offset: 0x34*/
	__vo uint32_t AHB3ENR;			/*RCC AHB3 peripheral clock enable register                        Address offset: 0x38*/
	__vo uint32_t RESERVED2;		/*0x3C*/
	__vo uint32_t APB1ENR;          /*RCC APB1 peripheral clock enable register                        Address offset: 0x40*/
	__vo uint32_t APB2ENR;          /*RCC APB2 peripheral clock enable register                        Address offset: 0x44*/
	__vo uint32_t RESERVED3[2];    /*0x48  0x4C*/
	__vo uint32_t AHB1LPENR;        /*RCC AHB1 peripheral clock enable in low power mode register      Address offset: 0x50*/
	__vo uint32_t AHB2LPENR;        /*RCC AHB2 peripheral clock enable in low power mode register      Address offset: 0x54*/
	__vo uint32_t AHB3LPENR;        /*RCC AHB3 peripheral clock enable in low power mode register      Address offset: 0x58*/
	__vo uint32_t RESERVED4;         /*0x5C*/
	__vo uint32_t APB1LPENR;        /*RCC APB1 peripheral clock enable in low power mode register      Address offset: 0x60*/
	__vo uint32_t APB2LPENR;        /*RCC APB2 peripheral clock enabled in low power mode register     Address offset: 0x64*/
	__vo uint32_t RESERVED5[2];      /*0x68  0x6C*/
	__vo uint32_t BDCR;              /*RCC Backup domain control register                               Address offset: 0x70*/
	__vo uint32_t CSR;              /*RCC clock control & status register                              Address offset: 0x74*/
	__vo uint32_t RESERVED6[2];       /*0x78  0x7C*/
	__vo uint32_t SSCGR;            /*RCC spread spectrum clock generation register                    Address offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;       /*RCC PLLI2S configuration register	                               Address offset: 0x84*/
	__vo uint32_t PLLSAICFGR;       /*RCC PLL configuration register                                   Address offset: 0x88*/
	__vo uint32_t DCKCFGR;			/*RCC Dedicated Clock Configuration Register (RCC_DCKCFGR)		   Address offset: 0x8C	*/
	__vo uint32_t CKGATENR;			/*RCC clocks gated enable register (CKGATENR)					   Address offset: 0x90*/
	__vo uint32_t DCKCFGR2;         /*RCC dedicated clocks configuration register 2 (DCKCFGR2)		   Address offset: 0x94*/

}RCC_RegDef_t;

/*
 * Peripheral register structure for EXTI
 */
typedef struct {
	__vo uint32_t IMR;			//Interrupt mask register (EXTI_IMR)				Address offset: 0x00
	__vo uint32_t EMR;       	//Event mask register (EXTI_EMR)					Address offset: 0x04
	__vo uint32_t RTSR;       	//Rising trigger selection register (EXTI_RTSR)		Address offset: 0x08
	__vo uint32_t FTSR;			//Falling trigger selection register (EXTI_FTSR)	Address offset: 0x0C
	__vo uint32_t SWIER;		//Software interrupt event register (EXTI_SWIER)	Address offset: 0x10
	__vo uint32_t PR; 			//Pending register (EXTI_PR)						Address offset: 0x14

}EXTI_RegDef_t;

/*
 * Peripheral register structure for SYSCFG
 */
typedef struct {
	__vo uint32_t MEMRMP;		//SYSCFG memory remap register (SYSCFG_MEMRMP)				Address offset: 0x00
	__vo uint32_t PMC;       	//SYSCFG peripheral mode configuration register				Address offset: 0x04
	__vo uint32_t EXTICR[4];    //SYSCFG external interrupt configuration register :
								//EXTICR[x]=EXTICRx ;  x=[1..4]								Address offset: 0x08-0x14
	__vo uint32_t reserved1[2]; //reserved
	__vo uint32_t CMPCR;     	//Compensation cell control register						Address offset: 0x20
	__vo uint32_t reserved2[2]; //reserved
	__vo uint32_t CFGR;			//SYSCFG configuration register 							Address offset: 0x2C


}SYSCFG_RegDef_t;
/*******************************************************/

#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 					((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG 					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)




/*
 * Clock Enable Macro for GPIOx peripherals
 */

#define GPIOA_PCLK_EN			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN			(RCC->AHB1ENR |= (1<<7))


#define GPIO_PORT_TO_CODE(x)	((x == GPIOA)? 0 : \
								 (x == GPIOB)? 1 : \
								 (x == GPIOC)? 2 : \
								 (x == GPIOD)? 3 : \
								 (x == GPIOE)? 4 : \
								 (x == GPIOF)? 5 : \
								 (x == GPIOG)? 6 : 7)

/*
 * Clock Enable Macro for I2Cx peripherals
 */

#define I2C1_PCLK_EN			(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN			(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN			(RCC->APB1ENR |= (1<<23))

/*
 * Clock Enable Macro for SPIx peripherals
 */

#define SPI1_PCLK_EN			(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN			(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN			(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN			(RCC->APB2ENR |= (1<<13))

/*
 * Clock Enable Macro for UARTx and USARTx peripherals
 */

#define USART1_PCLK_EN			(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN			(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN			(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN			(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN			(RCC->APB2ENR |= (1<<5))

/*
 * Clock Enable Macro for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN			(RCC->APB2ENR |= (1<<14))


/*
 * Clock Disable Macro for GPIOx peripherals
 */

#define GPIOA_PCLK_DI			(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI			(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI			(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI			(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI			(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI			(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI			(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI			(RCC->AHB1ENR &= ~(1<<7))

/*
 * Clock Disable Macro for I2Cx peripherals
 */

#define I2C1_PCLK_DI			(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI			(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI			(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Disable Macro for SPIx peripherals
 */

#define SPI1_PCLK_DI			(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI			(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI			(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI			(RCC->APB2ENR &= ~(1<<13))

/*
 * Clock Disable Macro for UARTx and USARTx peripherals
 */

#define USART1_PCLK_DI			(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI			(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI			(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI			(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI			(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI			(RCC->APB2ENR &= ~(1<<5))

/*
 * Clock Enable Macro for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI			(RCC->APB2ENR &= ~(1<<14))

/*
 *
 */
#define ENABLE		1
#define DISABLE		0
#define SET		ENABLE
#define RESET	DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET



#include "stm32f446xx_gpio_driver.h"

#endif /* INC_STM32F446XX_H_ */
