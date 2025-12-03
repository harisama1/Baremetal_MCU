/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Oct 2, 2025
 *      Author: HARRY LE
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include "stm32f4xx.h"

/*
 * This is handle structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;				//possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;				//possible values from @GPIO_PIN_MODE
	uint8_t GPIO_PinPUPDControl;		//possible values from @GPIO_PUPD
	uint8_t GPIO_PinOPType;				//possible values from @GPIO_OUTPUT_TYPE
	uint8_t GPIO_PinSpeed;				//possible values from @GPIO_OUTPUT_SPEED
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_Typedef;


typedef struct
{
	GPIO_Typedef *GPIOx;						// This hold the base address of the GPIO port to which the pin belong
	GPIO_PinConfig_Typedef GPIO_PinConfig;		// This hold GPIO pin configuration setting
}GPIO_Handle_t;



/*
 * GPIO pin possible modes
 * @GPIO_PIN_MODE
 * */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IN_FE		4
#define GPIO_MODE_IN_RE		5
#define GPIO_MODE_IN_RFT	6



/*
 * GPIO pin possible output types
 * @GPIO_OUTPUT_TYPE
 * */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1



/*
 * GPIO pin possible output speeds
 * @GPIO_OUTPUT_SPEED
 * */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3



/*
 * GPIO pin pull up and pull down configuration macros
 * @GPIO_PUPD
 * */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2

/*
 * GPIO Pin Number
 * @GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_Typedef *GPIOx, uint8_t EnorDi);

/*
 * GPIO Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *GPIOHandle);
void GPIO_DeInit(GPIO_Typedef *GPIOx);

/*
 * GPIO Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Typedef *GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Typedef *GPIOx);
void GPIO_WriteToOutputPin(GPIO_Typedef *GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Typedef *GPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Typedef *GPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriority_config(uint8_t IRQNumber, uint8_t IRQPriority);

#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
