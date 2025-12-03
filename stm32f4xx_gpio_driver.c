/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Oct 2, 2025
 *      Author: HARRY
 */


#include "stm32f4xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/***************************************************************************************************
 * @fn						- GPIO_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- none
 *
 * @Note					- none
 */
void GPIO_PeriClockControl(GPIO_Typedef *GPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(GPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(GPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(GPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(GPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(GPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(GPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(GPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(GPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(GPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(GPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(GPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(GPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(GPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(GPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(GPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
		else if(GPIOx == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}
		else if(GPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
		else if(GPIOx == GPIOI)
		{
			GPIOI_PCLK_DIS();
		}
	}
}
/*
 * GPIO Init and DeInit
 */

/***************************************************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function sets up parameters for GPIO pin/port that we want to use
 *
 * @param[in]				- Pointer to the GPIO that we want to set up
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_Init(GPIO_Handle_t *GPIOHandle)
{
	uint32_t temp = 0; //temp register

	// 1. configure the mode of gpio pin

	if(GPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		GPIOHandle->GPIOx->MODER &= ~( 0x03 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		GPIOHandle->GPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		if(GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FE)
		{
			// 1. configure the FTSR
			EXTI->FTSR |= ( 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RE)
		{
			// 1. configure the RTSR
			EXTI->RTSR |= ( 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RFT)
		{
			// 1. configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ( 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port  selection in SYSCFG_EXTICR
		uint8_t temp1 = GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(GPIOHandle->GPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);




		// 3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;

	// 2. configure the speed

	temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	GPIOHandle->GPIOx->OSPEEDR &= ~( 0x03 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	GPIOHandle->GPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. configure the PUPD settings
	temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinPUPDControl << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	GPIOHandle->GPIOx->PUPDR &= ~( 0x03 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	GPIOHandle->GPIOx->PUPDR |= temp;
	temp = 0;

	// 4. configure the Output type
	temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	GPIOHandle->GPIOx->OTYPER &= ~( 0x01 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	GPIOHandle->GPIOx->OTYPER |= temp;
	temp = 0;

	// 5. configure the alternative functionality
	if(GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		GPIOHandle->GPIOx->AFR[temp1] &= ~( 0xF << ( 4 * temp2 ));
		GPIOHandle->GPIOx->AFR[temp1] |= (GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ));
	}
}

/***************************************************************************************************
 * @fn						- GPIO_Deinit
 *
 * @brief					- This Disable GPIO that we don't need to use
 *
 * @param[in]				- base address of the gpio peripheral
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_DeInit(GPIO_Typedef *GPIOx)
{
	if(GPIOx == GPIOA)
	{
		GPIOA_REG_RST();
	}
	else if(GPIOx == GPIOB)
	{
		GPIOB_REG_RST();
	}
	else if(GPIOx == GPIOC)
	{
		GPIOC_REG_RST();
	}
	else if(GPIOx == GPIOD)
	{
		GPIOD_REG_RST();
	}
	else if(GPIOx == GPIOE)
	{
		GPIOE_REG_RST();
	}
	else if(GPIOx == GPIOF)
	{
		GPIOF_REG_RST();
	}
	else if(GPIOx == GPIOG)
	{
		GPIOG_REG_RST();
	}
	else if(GPIOx == GPIOH)
	{
		GPIOH_REG_RST();
	}
	else if(GPIOx == GPIOI)
	{
		GPIOI_REG_RST();
	}
}
/*
 * GPIO Data read and write
 */

/***************************************************************************************************
 * @fn						- GPIO_ReadFromInputPin
 *
 * @brief					- This function read data from a GPIO pin
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- The Pin number
 *
 * @return					- data in a unsigned 8-bit integer
 *
 * @Note					- none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_Typedef *GPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)(GPIOx->IDR >> PinNumber) & 0x01U;

	return value;
}

/***************************************************************************************************
 * @fn						- GPIO_ReadFromInputPort
 *
 * @brief					- This function read data from a GPIO port
 *
 * @param[in]				- base address of the gpio peripheral
 *
 * @return					- data in a unsigned 16-bit integer
 *
 * @Note					- none
*/

uint16_t GPIO_ReadFromInputPort(GPIO_Typedef *GPIOx)
{
	uint16_t value;

	value = (uint16_t)(GPIOx->IDR);

	return value;
}

/***************************************************************************************************
 * @fn						- GPIO_WriteToOutputPin
 *
 * @brief					- This function write data to the GPIO pin
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- The pin number that we want to use
 * @param[in]				- 8-bits value that we want to write
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_WriteToOutputPin(GPIO_Typedef *GPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		//write 1 to the ouput data register at the bit field corresponding to the pin
		GPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		GPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***************************************************************************************************
 * @fn						- GPIO_WriteToOutputPort
 *
 * @brief					- This function write data to the GPIO port
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- 16-bits value that we want to write
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_WriteToOutputPort(GPIO_Typedef *GPIOx, uint16_t Value)
{
	GPIOx->ODR = Value;
}

/***************************************************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- This function toggles the Output pin
 *
 * @param[in]				- base address of the gpio peripheral
 * @param[in]				- The pin number that we want to use
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_ToggleOutputPin(GPIO_Typedef *GPIOx, uint8_t PinNumber)
{
	GPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */

/***************************************************************************************************
 * @fn						- GPIO_IRQConfig
 *
 * @brief					- This function configures for GPIO interrupts
 *
 * @param[in]				- Interrupt vector number
 * @param[in]				- Interrupt vector order
 * @param[in]				- Enable or Disable macros
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) //32 to 64
		{
			//Program ISER1 register
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//Program ISER2 register
			*NVIC_ISER3 |= ( 1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64) //32 to 64
		{
			//Program ISER1 register
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//Program ISER2 register
			*NVIC_ISER3 |= ( 1 << IRQNumber % 64);
		}
	}
}

/***************************************************************************************************
 * @fn						- GPIO_IRQPriority_config
 *
 * @brief					- This function handle the priority of interrupt
 *
 * @param[in]				- Pin number
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_IRQPriority_config(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + (iprx * 4)) |= ( IRQPriority << shift_amount );
}

/***************************************************************************************************
 * @fn						- GPIO_IRQHandling
 *
 * @brief					- This function handle the interrupt
 *
 * @param[in]				- Pin number
 *
 * @return					- none
 *
 * @Note					- none
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}
