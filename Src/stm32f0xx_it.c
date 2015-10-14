/**
  ******************************************************************************
  * @file    Src/stm32f0xx_it.c 
  * @author  Nils Gura
  * @version V1.1
  * @date    26-Apr-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
#include "scanner_hal.h"
#include "barcode.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
/** @addtogroup SPI_FullDuplex_ComDMA
  * @{
  */

volatile uint32_t cmos_sensor_state = CMOS_SENSOR_STOP;
uint8_t is_mt710th;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declared in "main.c" file */
extern UART_HandleTypeDef UartHandle;
/* SPI handler declared in "main.c" file */
extern SPI_HandleTypeDef SpiHandle;

extern TIM_HandleTypeDef TimHandleSp;
extern ADC_HandleTypeDef AdcHandle;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles DMA interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA 
  *         used for USART data transmission     
  */
void USARTx_DMA_RX_IRQHandler(void)
{
	//HAL_GPIO_TogglePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN);
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

void SPIx_DMA_RX_IRQHandler(void)
{
	//HAL_GPIO_TogglePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN);
  HAL_DMA_IRQHandler(SpiHandle.hdmarx);
  HAL_DMA_IRQHandler(SpiHandle.hdmatx);
}

/**
  * @brief  This function handles External EXTI_Line0 interrupt request.
  * @param  None
  * @retval None
  */

// interrupt routines needed for linear image sensor

void __SCAN_SYNC_EXTI_IRQ_HANDLER() 
{
  HAL_GPIO_EXTI_IRQHandler(SCAN_SYNC_PIN);
}

void __SCAN_SP_IRQ_HANDLER() 
{
	HAL_TIM_IRQHandler(&TimHandleSp);
}

void __SCAN_VOUT_IRQ_HANDLER()
{
  HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
		case SCAN_SYNC_PIN:
			if (cmos_sensor_state == CMOS_SENSOR_ARM)	{
				  cmos_sensor_state = CMOS_SENSOR_CAPTURE;
	        if (!is_mt710th && (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *) img_buf[img_buf_wr_ptr], IMAGE_COLUMNS) != HAL_OK)) Error_Handler();
			}
		}
}

// start capturing data with the ADC when the linear image sensor
// signals availability of the first data sample;
// interrupt handler driven by an external input pin

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == SCAN_SP_TIM) {
	  if (cmos_sensor_state == CMOS_SENSOR_READY) {
	    cmos_sensor_state = CMOS_SENSOR_ARM;
			if (is_mt710th && (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *) img_buf[img_buf_wr_ptr], IMAGE_COLUMNS) != HAL_OK)) Error_Handler();
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	cmos_sensor_state = CMOS_SENSOR_STOP;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_GPIO_TogglePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN);
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
