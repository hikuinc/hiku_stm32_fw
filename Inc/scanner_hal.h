/**
  ******************************************************************************
  * @file    Inc/scanner_hal.h 
  * @author  Nils Gura
  * @version V1.0
  * @date    26-Apr-2015
  * @brief   Header for scanner hardware abstraction
  ******************************************************************************
*/ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCANNER_HAL_H
#define __SCANNER_HAL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//
// Connections to Marson MT710T scanner
//

#define SENSOR_PD_PORT GPIOA
#define SENSOR_PD_PIN GPIO_PIN_0
#define SENSOR_PD_MODE GPIO_MODE_INPUT
#define SENSOR_PD_PULL GPIO_PULLDOWN
#define SENSOR_PD_SPEED GPIO_SPEED_LOW
#define SENSOR_PD_CLK_ENABLE() __GPIOA_CLK_ENABLE()

#define SCAN_SYNC_PORT GPIOA
#define SCAN_SYNC_PIN GPIO_PIN_12
#define SCAN_SYNC_MODE GPIO_MODE_IT_FALLING
#define SCAN_SYNC_PULL GPIO_PULLUP
#define SCAN_SYNC_SPEED GPIO_SPEED_HIGH
#define SCAN_SYNC_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define SCAN_SYNC_EXTI_IRQn EXTI4_15_IRQn
#define __SCAN_SYNC_EXTI_IRQ_HANDLER() EXTI4_15_IRQHandler()

#define SCAN_ON_LED_PORT GPIOA
#define SCAN_ON_LED_PIN GPIO_PIN_6
#define SCAN_ON_LED_MODE GPIO_MODE_OUTPUT_PP
#define SCAN_ON_LED_PULL GPIO_NOPULL
#define SCAN_ON_LED_SPEED GPIO_SPEED_LOW
#define SCAN_ON_LED_CLK_ENABLE() __GPIOA_CLK_ENABLE()

#define SCAN_GS_PORT GPIOA
#define SCAN_GS_PIN GPIO_PIN_15
#define SCAN_GS_MODE GPIO_MODE_INPUT
#define SCAN_GS_PULL GPIO_PULLDOWN
#define SCAN_GS_SPEED GPIO_SPEED_LOW
#define SCAN_GS_CLK_ENABLE() __GPIOA_CLK_ENABLE()

#define SCAN_SP_PORT GPIOA
#define SCAN_SP_PIN GPIO_PIN_4
#define SCAN_SP_MODE GPIO_AF4_TIM14
#define SCAN_SP_PULL GPIO_NOPULL
#define SCAN_SP_SPEED GPIO_SPEED_HIGH
#define SCAN_SP_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define SCAN_SP_TIM_CLK_EN() __TIM14_CLK_ENABLE();
#define SCAN_SP_TIM TIM14
  /* 48MHz/((9+1)*(10684+1)) = 449.23Hz */
#define SCAN_SP_PRESCALER 9
#define SCAN_SP_PERIOD 10684
#define SCAN_SP_PERIOD_SCREEN 32053
#define SCAN_SP_PULSE_MT710T 50
#define SCAN_SP_PULSE_MT710TH 2000
#define SCAN_SP_TIM_CHANNEL TIM_CHANNEL_1
#define SCAN_SP_IRQn TIM14_IRQn
#define __SCAN_SP_IRQ_HANDLER() TIM14_IRQHandler()

#define SCAN_CP_PORT GPIOA
#define SCAN_CP_PIN GPIO_PIN_8
#define SCAN_CP_MODE GPIO_AF2_TIM1
#define SCAN_CP_PULL GPIO_NOPULL
#define SCAN_CP_SPEED GPIO_SPEED_HIGH
#define SCAN_CP_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define SCAN_CP_TIM_CLK_EN() __TIM1_CLK_ENABLE();
#define SCAN_CP_TIM TIM1
 /* 48MHz/(95+1) = 500kHz */
#define SCAN_CP_PRESCALER TIM_ICPSC_DIV1
#define SCAN_CP_PERIOD 95
#define SCAN_CP_PERIOD_SCREEN 287
#define SCAN_CP_PULSE (SCAN_CP_PERIOD/2)
#define SCAN_CP_TIM_CHANNEL TIM_CHANNEL_1
#define SCAN_CP_TIM_TRGO TIM_TRGO_OC1REF
#define SCAN_CP_TIM_ADC_CHANNEL TIM_CHANNEL_4
#define SCAN_CP_IRQn TIM1_IRQn

#define SCAN_VOUT_PORT GPIOA
#define SCAN_VOUT_PIN GPIO_PIN_3
#define SCAN_VOUT_MODE GPIO_MODE_ANALOG
#define SCAN_VOUT_PULL GPIO_NOPULL
#define SCAN_VOUT_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define SCAN_VOUT_ADC_CLK_ENABLE() __ADC1_CLK_ENABLE()
#define SCAN_VOUT_DMA_CLK_ENABLE() __DMA1_CLK_ENABLE()
#define SCAN_VOUT_ADC ADC1
#define SCAN_VOUT_ADC_CHANNEL ADC_CHANNEL_3
#define SCAN_VOUT_ADC_TRIG ADC_EXTERNALTRIGCONV_T1_TRGO
#define SCAN_VOUT_DMA DMA1_Channel1
#define SCAN_VOUT_IRQn DMA1_Channel1_IRQn
#define __SCAN_VOUT_IRQ_HANDLER() DMA1_Channel1_IRQHandler()

#define DEBUG_OUT_PORT GPIOB
#define DEBUG_OUT_PIN GPIO_PIN_7
#define DEBUG_OUT_MODE GPIO_MODE_OUTPUT_PP
#define DEBUG_OUT_PULL GPIO_NOPULL
#define DEBUG_OUT_SPEED GPIO_SPEED_LOW
#define DEBUG_OUT_CLK_ENABLE() __GPIOA_CLK_ENABLE()

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 

