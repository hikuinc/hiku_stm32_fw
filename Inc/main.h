/**
  ******************************************************************************
  * @file    Inc/main.h 
  * @author  Nils Gura
  * @version V1.0
  * @date    21-Sep-2015
  * @brief   Main header file for scanner/microphone software
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
	SCAN_CMD_MIC,
	SCAN_CMD_MIC_SCAN,
	SCAN_CMD_SCAN,
	SCAN_CMD_LIGHT_ONLY,
	SCAN_CMD_SCAN_DEBUG,
  SCAN_CMD_NONE} scan_enum_t;

typedef struct scan_command {
	char cmdChar;
	scan_enum_t scanCmd;
} scan_cmd_t;
	
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor TIMx instance used and associated
   resources */
/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __TIM3_CLK_ENABLE()

/* Definition for TIMx Channel Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __GPIOB_CLK_ENABLE()
#define TIMx_GPIO_PORT_CHANNEL3        GPIOB
#define TIMx_GPIO_PIN_CHANNEL3         GPIO_PIN_1
#define TIMx_GPIO_AF_CHANNEL3          GPIO_AF1_TIM3

/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF1_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF1_USART1

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM              DMA1_Channel4
#define USARTx_RX_DMA_STREAM              DMA1_Channel5

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel4_5_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Channel4_5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel4_5_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Channel4_5_IRQHandler

/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __SPI1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF0_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF0_SPI1

/* Definition for SPIx's DMA */
#define SPIx_TX_DMA_STREAM               DMA1_Channel3
#define SPIx_RX_DMA_STREAM               DMA1_Channel2

/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA1_Channel2_3_IRQn
#define SPIx_DMA_RX_IRQn                 DMA1_Channel2_3_IRQn

#define SPIx_DMA_TX_IRQHandler           DMA1_Channel2_3_IRQHandler
#define SPIx_DMA_RX_IRQHandler           DMA1_Channel2_3_IRQHandler

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define DEFAULT_AUDIO_IN_FREQ                 ((uint32_t)16000)
#define DEFAULT_AUDIO_IN_CHANNEL_NBR          1 /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_IN_VOLUME               1

// Timer values to output a 1MHz clock signal for the microphone
#define  PERIOD_VALUE       (uint32_t) 15 /* Period Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/2)+1 /* Capture Compare 3 Value  */

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE                    64*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR
/* PCM buffer output size */
#define PCM_OUT_SIZE                          DEFAULT_AUDIO_IN_FREQ/1000
#define AUDIO_SAMP_PER_PACKET                 125
#define PACKET_HDR                            0xDEADBEEF
#define PACKET_TYPE_FIELD                     4
#define PACKET_LEN_FIELD                      5
#define PACKET_HDR_LEN                        7
#define AUDIO_PAYLOAD_LEN                     ((AUDIO_SAMP_PER_PACKET * PCM_OUT_SIZE * DEFAULT_AUDIO_IN_CHANNEL_NBR)/2)
#define SCAN_PAYLOAD_LEN                      31

#define AUDIO_DISCARD_SAMP                    24

#define SCAN_REPEAT_RESULT                    64

#define PKT_TYPE_SW_VERSION                   0x01
#define PKT_TYPE_AUDIO                        0x10
#define PKT_TYPE_SCAN                         0x20
#define PKT_TYPE_SCAN_DEBUG                   0xDB

#define UART_BAUD_RATE                        921600
#define UART_RECEIVE_TIMEOUT                  500

// software version to send to the Imp 
#define SOFTWARE_VERSION                      (uint8_t) 1
#define SOFTWARE_REVISION                     (uint8_t) 24

// scanner debug settings
#define DEBUG_SCAN_LINES                      128
#define UART_BAUD_RATE_DEBUG                  1843200

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 

