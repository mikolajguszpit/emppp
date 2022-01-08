/*
 * bosz.h
 *
 *  Created on: Mar 24, 2020
 *      Author: NAM
 */

#ifndef INC_BOSZ_H_
#define INC_BOSZ_H_

#define Pin_Test GPIO_PIN_7
#define Pin2_Test GPIO_PIN_2

#define Blue_LED GPIO_PIN_15
#define Red_LED GPIO_PIN_14
#define Orange_LED GPIO_PIN_13
#define Green_LED GPIO_PIN_12

#define LED1_ON()	(GPIOD->BSRR = (Blue_LED))
#define LED1_OFF()	(GPIOD->BSRR = (Blue_LED)<<16)
#define LED1_TOGGLE()	(GPIOD->ODR ^= (Blue_LED))
#define LED1_FLASH()	do{LED1_TOGGLE(); LED1_TOGGLE();} while(0)

#define LED2_ON()	(GPIOD->BSRR = (Red_LED))
#define LED2_OFF()	(GPIOD->BSRR = (Red_LED)<<16)
#define LED2_TOGGLE()	(GPIOD->ODR ^= (Red_LED))
#define LED2_FLASH()	do{LED2_TOGGLE(); LED2_TOGGLE();} while(0)

#define LED3_ON()	(GPIOD->BSRR = (Orange_LED))
#define LED3_OFF()	(GPIOD->BSRR = (Orange_LED)<<16)
#define LED3_TOGGLE()	(GPIOD->ODR ^= (Orange_LED))
#define LED3_FLASH()	do{LED3_TOGGLE(); LED3_TOGGLE();} while(0)
#define LED3_FLASH2()	do{LED3_TOGGLE();for(uint32_t nsec=(SysTick->VAL);(SysTick->VAL)<=(nsec+10);); LED3_TOGGLE();} while(0)

#define LED4_ON()	(GPIOD->BSRR = (Green_LED))
#define LED4_OFF()	(GPIOD->BSRR = (Green_LED)<<16)
#define LED4_TOGGLE()	(GPIOD->ODR ^= (Green_LED))
#define LED4_FLASH()	do{LED4_TOGGLE(); LED4_TOGGLE();} while(0)

#define Pin_ON()	(GPIOD->BSRR = (Pin_Test))
#define Pin_OFF()	(GPIOD->BSRR = (Pin_Test)<<16)
#define Pin_TOGGLE()	(GPIOD->ODR ^= (Pin_Test))
#define Pin_FLASH()	do{Pin_TOGGLE(); Pin_TOGGLE();} while(0)

#define Pin2_ON()	(GPIOB->BSRR = (Pin2_Test))
#define Pin2_OFF()	(GPIOB->BSRR = (Pin2_Test)<<16)
#define Pin2_TOGGLE()	(GPIOB->ODR ^= (Pin2_Test))
#define Pin2_FLASH()	do{Pin2_TOGGLE(); Pin2_TOGGLE();} while(0)

#endif /* INC_BOSZ_H_ */
