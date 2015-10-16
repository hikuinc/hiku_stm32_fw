/**
  ******************************************************************************
  * @file    Src/main.c 
  * @author  Nils Gura
  * @version V1.0
  * @date    21-Sep-2015
  * @brief   Scanner/Microphone software
  */ 

/* Includes ------------------------------------------------------------------*/
#include "zbar.h"
#include "decoder.h"
#include "stm32f0xx_hal.h"
#include "barcode.h"
#include "pdm_filter.h"
#include "scanner_hal.h"
#include "scanner.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef    TimHandleSp;
	
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
PDMFilter_InitStruct Filter[DEFAULT_AUDIO_IN_CHANNEL_NBR];
static int16_t RecBuf[PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR];
static uint8_t AudioPacketBuf[PACKET_HDR_LEN + AUDIO_PAYLOAD_LEN];
static uint8_t ScanPacketBuf[PACKET_HDR_LEN + SCAN_PAYLOAD_LEN];

/* ADC handle declaration */
ADC_HandleTypeDef             AdcHandle;

/* Buffer used for reception */
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
/* Buffer used for processing */
static uint16_t InternalBufferCopy[INTERNAL_BUFF_SIZE];

/* Scanner buffers */
// use double-buffering (ping-pong buffering) to have one buffer to process
// for bar codes while the other is being filled from the AD converter
uint8_t  img_buf[2][IMAGE_COLUMNS];
uint32_t img_buf_wr_ptr = 0;

static uint8_t scan_decoded;
static uint8_t stop_audio;

const scan_cmd_t scan_commands[] = {{.cmdChar='M', .scanCmd=SCAN_CMD_MIC},
                                    {.cmdChar='N', .scanCmd=SCAN_CMD_MIC_SCAN},
																		{.cmdChar='S', .scanCmd=SCAN_CMD_SCAN},
																		{.cmdChar='P', .scanCmd=SCAN_CMD_SCAN_SCREEN},																		
                                    {.cmdChar='L', .scanCmd=SCAN_CMD_LIGHT_ONLY},
																		{.cmdChar='D', .scanCmd=SCAN_CMD_SCAN_DEBUG},
																		{.cmdChar='E', .scanCmd=SCAN_CMD_FLASH_ERASE},
																		{.cmdChar='B', .scanCmd=SCAN_CMD_BUTTON_RELEASE}
																	 };

static scan_enum_t scanCmd = SCAN_CMD_NONE;
																	 
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/
	
void UART_Config(scan_enum_t scanCmd) {
		/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = UART_BAUD_RATE/UART_BAUD_RATE_DEBUG baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

	if (scanCmd == SCAN_CMD_SCAN_DEBUG)
		UartHandle.Init.BaudRate   = UART_BAUD_RATE_DEBUG;
	else
		UartHandle.Init.BaudRate   = UART_BAUD_RATE;

  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK) Error_Handler();
  if(HAL_UART_Init(&UartHandle) != HAL_OK) Error_Handler();
}

/**
  * @brief  Initialize the PDM library.
  * @param  AudioFreq: Audio sampling frequency
  * @param  ChnlNbr: Number of audio channels (1: mono; 2: stereo)
  * @retval None
  */
static void PDMDecoder_Init(uint32_t AudioFreq, uint32_t ChnlNbr)
{ 
  uint32_t i = 0;
  
  /* Enable CRC peripheral to unlock the PDM library */
  __CRC_CLK_ENABLE();
  
  for(i = 0; i < ChnlNbr; i++)
  {
    /* Filter LP and HP Init */
    Filter[i].LP_HZ = AudioFreq / 2;
    Filter[i].HP_HZ = 10;
    Filter[i].Fs = AudioFreq;
    Filter[i].Out_MicChannels = 1;
    Filter[i].In_MicChannels = ChnlNbr; 
    PDM_Filter_Init((PDMFilter_InitStruct *)&Filter[i]);
  }  
}

int8_t ALaw_Encode(int16_t number)
{
   const uint16_t ALAW_MAX = 0xFFF;
   uint16_t mask = 0x800;
   uint8_t sign = 0;
   uint8_t position = 11;
   uint8_t lsb = 0;
   if (number < 0)
   {
      number = -number;
      sign = 0x80;
   }
   if (number > ALAW_MAX)
   {
      number = ALAW_MAX;
   }
   for (; ((number & mask) != mask && position >= 5); mask >>= 1, position--);
   lsb = (number >> ((position == 4) ? (1) : (position - 4))) & 0x0f;
   return (sign | ((position - 4) << 4) | lsb) ^ 0x55;
}

static void symbol_handler (zbar_decoder_t *dcode)
{
	  static uint8_t scan_wr_ptr = 0;
	  static uint8_t decode_count = 0;
	  static char decode_buffer[DECODE_BUFFER_SIZE][DECODE_BUFFERS];

    uint32_t i, j;
	  uint32_t decodes_equal;
    zbar_symbol_type_t type = zbar_decoder_get_type(dcode);

	  if(type <= ZBAR_PARTIAL)
      return;
		
    const char *data = zbar_decoder_get_data(dcode);
    unsigned datalen = zbar_decoder_get_data_length(dcode);
		
		if (datalen > DECODE_BUFFER_SIZE)
			return;

		if (decode_count >= DECODE_BUFFERS/2) {
			decodes_equal = 0;
			for (i=0; i<decode_count; i++) {
				for (j=0; j<datalen; j++) 
			     if (data[j] != decode_buffer[j][i])
							break;
				if (j == datalen)
					decodes_equal++;
				if (decodes_equal == DECODE_BUFFERS/2) {
		      ScanPacketBuf[PACKET_LEN_FIELD] = 0;
		      ScanPacketBuf[PACKET_LEN_FIELD+1] = datalen+3;
					for (j=0; j<datalen; j++)
					  ScanPacketBuf[PACKET_HDR_LEN + j] = data[j];
		      ScanPacketBuf[PACKET_HDR_LEN + j] = '\r';
		      ScanPacketBuf[PACKET_HDR_LEN + j+1] = '\n';
		      ScanPacketBuf[PACKET_HDR_LEN + j+2] = 0;
					scan_decoded = 1;
					//scan result is sent in the main loop
					return;
				}
		  }
		}

    for (i=0; i<datalen; i++) 
      decode_buffer[i][scan_wr_ptr] = data[i];
		scan_wr_ptr = (scan_wr_ptr+1) % DECODE_BUFFERS;
		decode_count = (decode_count >= DECODE_BUFFERS) ? DECODE_BUFFERS : (decode_count + 1);
}

/* turn on scanner LED on/off */
void setScannerLED (uint8_t value) {
	  HAL_GPIO_WritePin(SCAN_ON_LED_PORT, SCAN_ON_LED_PIN, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* set scanner image sensor gain high/low */
void setScannerGAIN (uint8_t value) {
	// use pull-up/pull-down to set gain to avoid shorting the TRIG output
	// on MT710TH
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = SCAN_GS_PIN;
  GPIO_InitStruct.Mode = SCAN_GS_MODE;
  GPIO_InitStruct.Pull = value ? GPIO_PULLUP : GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = SCAN_GS_SPEED;
  HAL_GPIO_Init(SCAN_GS_PORT, &GPIO_InitStruct);  
}

void toggleDebug() {
	HAL_GPIO_TogglePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN);
}

// send the software version to the Imp via UART
void sendSWVersion(){
	  uint8_t pkt_buf[9];
	
		pkt_buf[0] = (PACKET_HDR >> 24) & 0xFF;
		pkt_buf[1] = (PACKET_HDR >> 16) & 0xFF;
		pkt_buf[2] = (PACKET_HDR >> 8) & 0xFF;
		pkt_buf[3] = PACKET_HDR & 0xFF;
		pkt_buf[4] = PKT_TYPE_SW_VERSION;
		pkt_buf[5] = 0x00;
		pkt_buf[6] = 0x02;
		pkt_buf[7] = SOFTWARE_VERSION;
		pkt_buf[8] = SOFTWARE_REVISION;
	
	  // use a blocking transmit with a 20ms timeout to transmit the software version
	  if(HAL_UART_Transmit(&UartHandle, pkt_buf, PACKET_HDR_LEN + pkt_buf[PACKET_LEN_FIELD+1], 20)!= HAL_OK) Error_Handler();
}

void Img_Scanner_Configuration(scan_enum_t scanCmd)
{ 												
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_HandleTypeDef    TimHandle;
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfig;
  ADC_ChannelConfTypeDef        adc_sConfig;
		
  /*
  *  Image sensor chip enable (active low), MT710T SENSOR_PD/pin 8
  */
  /* enable image sensor */
	SENSOR_PD_CLK_ENABLE();
  GPIO_InitStruct.Pin = SENSOR_PD_PIN;
  GPIO_InitStruct.Mode = SENSOR_PD_MODE;
  GPIO_InitStruct.Pull = SENSOR_PD_PULL;
  GPIO_InitStruct.Speed = SENSOR_PD_SPEED;
  HAL_GPIO_Init(SENSOR_PD_PORT, &GPIO_InitStruct);  

  /*
  *  Scanner LED enable PA6, MT700 pin 3
  */
	SCAN_ON_LED_CLK_ENABLE();
  GPIO_InitStruct.Pin = SCAN_ON_LED_PIN;
  GPIO_InitStruct.Mode = SCAN_ON_LED_MODE;
  GPIO_InitStruct.Pull = SCAN_ON_LED_PULL;
  GPIO_InitStruct.Speed = SCAN_ON_LED_SPEED;
  HAL_GPIO_Init(SCAN_ON_LED_PORT, &GPIO_InitStruct);  
  /* turn on scanner LED off */
  HAL_GPIO_WritePin(SCAN_ON_LED_PORT, SCAN_ON_LED_PIN, GPIO_PIN_RESET); 
	
  /*
  *  Image gain, MT710T GS/pin 2
  */
	SCAN_GS_CLK_ENABLE();
  GPIO_InitStruct.Pin = SCAN_GS_PIN;
  GPIO_InitStruct.Mode = SCAN_GS_MODE;
  GPIO_InitStruct.Pull = SCAN_GS_PULL;
  GPIO_InitStruct.Speed = SCAN_GS_SPEED;
  HAL_GPIO_Init(SCAN_GS_PORT, &GPIO_InitStruct);  
  /* set high gain */
	setScannerGAIN(0);

  /*
  *  Scan line start/end, MT710T SYNC/pin 1
  */
	SCAN_SYNC_CLK_ENABLE();
  GPIO_InitStruct.Pin = SCAN_SYNC_PIN;
  GPIO_InitStruct.Mode = SCAN_SYNC_MODE;
  GPIO_InitStruct.Pull = SCAN_SYNC_PULL;
  GPIO_InitStruct.Speed = SCAN_SYNC_SPEED;
  HAL_GPIO_Init(SCAN_SYNC_PORT, &GPIO_InitStruct);  

  /* Enable and set EOS EXTI Interrupt to the second highest priority */
  HAL_NVIC_SetPriority(SCAN_SYNC_EXTI_IRQn, 0x01, 0x00);
  HAL_NVIC_EnableIRQ(SCAN_SYNC_EXTI_IRQn);
  	
  /*
  *  Scanner clock pulse signal, MT710T CP/pin 7
  */
  TimHandle.Instance = SCAN_CP_TIM;
  TimHandle.Init.Prescaler         = SCAN_CP_PRESCALER;
	TimHandle.Init.Period            = (scanCmd == SCAN_CMD_SCAN_SCREEN) ? SCAN_CP_PERIOD_SCREEN : SCAN_CP_PERIOD;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) Error_Handler();

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.Pulse = SCAN_CP_PULSE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, SCAN_CP_TIM_CHANNEL) != HAL_OK) Error_Handler();
	
	sMasterConfig.MasterOutputTrigger = SCAN_CP_TIM_TRGO;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig)!= HAL_OK) Error_Handler();
	
	if (HAL_TIM_PWM_Start(&TimHandle, SCAN_CP_TIM_CHANNEL)!= HAL_OK) Error_Handler();

	// On the MT710TH the TRIG output is connected to either 
	// GS (DVT version) or SENSOR_PD (EVT version).
	// TRIG is an inverted copy of the clock signal CP and will
	// set GS or SENSOR_PD high on some samples.
	// On the MT710T, both GS and SENSOR_PD should be pulled
	// low at sampling time.
	is_mt710th = 0;
	uint32_t i;
	for (i=0; i<2*SCAN_CP_PERIOD; i++)
		is_mt710th |= HAL_GPIO_ReadPin(SCAN_GS_PORT, SCAN_GS_PIN) | HAL_GPIO_ReadPin(SENSOR_PD_PORT, SENSOR_PD_PIN);

  /*
  *  Scanner start pulse signal, MT710T SP/pin 6
  */			 

  TimHandleSp.Instance = SCAN_SP_TIM;
  TimHandleSp.Init.Prescaler         = SCAN_SP_PRESCALER;
	TimHandleSp.Init.Period            = (scanCmd == SCAN_CMD_SCAN_SCREEN) ? SCAN_SP_PERIOD_SCREEN : SCAN_SP_PERIOD;
  TimHandleSp.Init.ClockDivision     = 0;
  TimHandleSp.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandleSp.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandleSp) != HAL_OK) Error_Handler();

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.Pulse = is_mt710th ? SCAN_SP_PULSE_MT710TH : SCAN_SP_PULSE_MT710T;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandleSp, &sConfig, SCAN_SP_TIM_CHANNEL) != HAL_OK) Error_Handler();
	 
  HAL_NVIC_SetPriority(SCAN_SP_IRQn, 0x01, 0x00);
  HAL_NVIC_EnableIRQ(SCAN_SP_IRQn);

 /*
  *  Debug output pin
  */
	DEBUG_OUT_CLK_ENABLE();
  GPIO_InitStruct.Pin = DEBUG_OUT_PIN;
  GPIO_InitStruct.Mode = DEBUG_OUT_MODE;
  GPIO_InitStruct.Pull = DEBUG_OUT_PULL;
  GPIO_InitStruct.Speed = DEBUG_OUT_SPEED;
  HAL_GPIO_Init(DEBUG_OUT_PORT, &GPIO_InitStruct);  
  HAL_GPIO_WritePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN, GPIO_PIN_RESET); 


  /*
  *  Analog image sensor signal, MT710T CP/pin 7
  */
  AdcHandle.Instance = SCAN_VOUT_ADC;	
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION8b; //ADC_RESOLUTION12b;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.ExternalTrigConv      = SCAN_VOUT_ADC_TRIG;
	AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_FALLING; 
  AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV; // convert once per trigger
  AdcHandle.Init.DMAContinuousRequests = DISABLE; 
  AdcHandle.Init.Overrun               = OVR_DATA_OVERWRITTEN;
 
  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) Error_Handler();
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK) Error_Handler();
  
  /* ### - 3 - Channel configuration ######################################## */
  adc_sConfig.Channel      = SCAN_VOUT_ADC_CHANNEL;
  adc_sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  adc_sConfig.SamplingTime = (scanCmd == SCAN_CMD_SCAN_SCREEN) ? ADC_SAMPLETIME_41CYCLES_5 : ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &adc_sConfig) != HAL_OK) Error_Handler();

	if (HAL_TIM_PWM_Start_IT(&TimHandleSp, SCAN_SP_TIM_CHANNEL)!= HAL_OK) Error_Handler();	
}

void Mic_Configuration() {
  TIM_HandleTypeDef    TimHandle;
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;
	
	/*## Configure the TIM peripheral to output 1MHz microphone clock #######################################*/
  TimHandle.Instance = TIMx;
	/* Compute the prescaler value to have TIM3 counter clock equal to 16000000 Hz */
  TimHandle.Init.Prescaler         = (uint32_t)(SystemCoreClock / 16000000) - 1;
  TimHandle.Init.Period            = PERIOD_VALUE;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) Error_Handler();

  /*## Configure PWM channel 4 on timer 3 to output on PB1 #########################################*/
  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.Pulse = PULSE3_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
  /* Output microphone clock signal */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

	/* Configure the PDM library */
  PDMDecoder_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_CHANNEL_NBR);

  /*## Configure the SPI as a slave interface #######################################*/
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES_RXONLY;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
  SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLED;
  SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle.Init.Mode              = SPI_MODE_SLAVE;
  if (HAL_SPI_Init(&SpiHandle) != HAL_OK) Error_Handler();
	
	HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 0x02, 0x00);
	
if(HAL_SPI_Receive_DMA(&SpiHandle, (uint8_t *)InternalBuffer, 2*INTERNAL_BUFF_SIZE) != HAL_OK) Error_Handler();

}

GPIO_PinState is_command_mode() {
	GPIO_InitTypeDef GPIO_InitStruct;

	USARTx_RX_GPIO_CLK_ENABLE();

  GPIO_InitStruct.Pin       = USARTx_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

	return HAL_GPIO_ReadPin(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{	
  zbar_decoder_t *decoder;
  zbar_scanner_t *scanner;
	uint8_t cmdBuffer[1];
  TIM_HandleTypeDef    TimHandle;
  uint32_t scan_lines = 0;
  uint32_t scans = 0;
	uint32_t i;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;

		
  /* STM32F0xx HAL library initialization */
  HAL_Init();
	
  /* Configure the system clock to have a system clock = 48 Mhz */
  SystemClock_Config();

	if (is_command_mode() == GPIO_PIN_RESET) {
		HAL_FLASH_Unlock();
	  scanCmd = SCAN_CMD_MIC_SCAN_INSTANT;
	}
	
	Img_Scanner_Configuration(SCAN_CMD_NONE);

	UART_Config(SCAN_CMD_NONE);

	if (scanCmd == SCAN_CMD_MIC_SCAN_INSTANT){
		
			GPIO_InitTypeDef  GPIO_InitStruct;
			/*##-2- Configure peripheral GPIO ##########################################*/  
			/* UART TX GPIO pin configuration  */
			GPIO_InitStruct.Pin       = USARTx_RX_PIN;
			GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
			GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

			HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
	
	}
	
	
	AudioPacketBuf[0] = ScanPacketBuf[0] = (PACKET_HDR >> 24) & 0xFF;
	AudioPacketBuf[1] = ScanPacketBuf[1] = (PACKET_HDR >> 16) & 0xFF;
	AudioPacketBuf[2] = ScanPacketBuf[2] = (PACKET_HDR >> 8) & 0xFF;
	AudioPacketBuf[3] = ScanPacketBuf[3] = PACKET_HDR & 0xFF;
	AudioPacketBuf[PACKET_TYPE_FIELD] = PKT_TYPE_AUDIO;
	ScanPacketBuf[PACKET_TYPE_FIELD] = PKT_TYPE_SCAN;
	AudioPacketBuf[PACKET_LEN_FIELD] = (AUDIO_PAYLOAD_LEN >> 8) & 0xFF;
	AudioPacketBuf[PACKET_LEN_FIELD+1] = AUDIO_PAYLOAD_LEN & 0xFF;

	scan_decoded = 0;
	stop_audio = 0;

	sendSWVersion();
		
	cmdBuffer[0] = 0;
	while (scanCmd == SCAN_CMD_NONE) {
				uint32_t uart_state;
				HAL_UART_Receive_DMA(&UartHandle, cmdBuffer, 1);
				do {
					uart_state = HAL_UART_GetState(&UartHandle);
				} while ((uart_state == HAL_UART_STATE_BUSY) || (uart_state == HAL_UART_STATE_BUSY_RX) || (uart_state == HAL_UART_STATE_BUSY_TX_RX));
				for (i=0; i<SCAN_CMD_NONE; i++)
					if (scan_commands[i].cmdChar == cmdBuffer[0]) {
						scanCmd = scan_commands[i].scanCmd;
						break;
					}
	}

	switch (scanCmd) {
			case SCAN_CMD_MIC: 
				Mic_Configuration(); 
				while(1);
			case SCAN_CMD_MIC_SCAN: 
			case SCAN_CMD_MIC_SCAN_INSTANT:
				Mic_Configuration();
			case SCAN_CMD_SCAN_SCREEN: 
		  case SCAN_CMD_SCAN:
				if (scanCmd == SCAN_CMD_SCAN_SCREEN)
		      Img_Scanner_Configuration(SCAN_CMD_SCAN_SCREEN);
				cmos_sensor_state = CMOS_SENSOR_READY;
				setScannerLED(1);
				break;
			case SCAN_CMD_SCAN_DEBUG:
				// set higher baud rate for transferring debug data
				UART_Config(SCAN_CMD_SCAN_DEBUG);
				cmos_sensor_state = CMOS_SENSOR_READY;
			  AudioPacketBuf[PACKET_TYPE_FIELD] = PKT_TYPE_SCAN_DEBUG;
				AudioPacketBuf[PACKET_LEN_FIELD] = (IMAGE_COLUMNS >> 8) & 0xFF;
		    AudioPacketBuf[PACKET_LEN_FIELD+1] = IMAGE_COLUMNS & 0xFF;
				setScannerLED(1);
				break;
			case SCAN_CMD_LIGHT_ONLY:
				setScannerLED(1);
				while(1);
			case SCAN_CMD_FLASH_ERASE:
				HAL_FLASH_Unlock();
				EraseInitStruct.TypeErase = TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
        EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
        HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
			  HAL_FLASH_Lock();
				while(1);
			default:
				while(1);
		}
					
	decoder = zbar_decoder_create();
	scanner = zbar_scanner_create(decoder);
	zbar_decoder_set_handler(decoder, symbol_handler);

	zbar_scanner_new_scan(scanner);
	
	cmdBuffer[0] = 0;
	HAL_UART_Receive_DMA(&UartHandle, cmdBuffer, 1);
		
	while(1) {
		if (cmdBuffer[0] == 'B') {
			setScannerLED(0);
			stop_audio = 1;
			while(1);
		}
		if (scan_decoded && (scanCmd != SCAN_CMD_SCAN_DEBUG)) {
			uint32_t uart_state;
			uint32_t scan_repeat;
			setScannerLED(0);
			
			GPIO_InitTypeDef  GPIO_InitStruct;
			/*##-2- Configure peripheral GPIO ##########################################*/  
			/* UART TX GPIO pin configuration  */
			GPIO_InitStruct.Pin       = USARTx_RX_PIN;
			GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull      = GPIO_PULLUP;
			GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

			HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
			
			
			
			if (scanCmd == SCAN_CMD_MIC_SCAN_INSTANT)
				while (HAL_GetTick() < SCAN_PACKET_DELAY);
			for (scan_repeat=0;scan_repeat<SCAN_REPEAT_RESULT; scan_repeat++) {
				do {
					uart_state = HAL_UART_GetState(&UartHandle);
				} while ((uart_state == HAL_UART_STATE_BUSY) || (uart_state == HAL_UART_STATE_BUSY_TX) || (uart_state == HAL_UART_STATE_BUSY_TX_RX));
				if(HAL_UART_Transmit_DMA(&UartHandle, ScanPacketBuf, PACKET_HDR_LEN + ScanPacketBuf[PACKET_LEN_FIELD+1])!= HAL_OK) Error_Handler();
			}
			while(1);
		}
			
		// wait for DMA transfer to finish
		while (cmos_sensor_state != CMOS_SENSOR_STOP); 
		// switch between ping-pong buffers
		img_buf_wr_ptr ^= 1;				
		// high gain for 3 scans, low gain for 1 scan
		setScannerGAIN(scans%4 == 0);
		if (scanCmd == SCAN_CMD_SCAN_SCREEN)
		  setScannerLED(scans%3 == 0);
		cmos_sensor_state = CMOS_SENSOR_READY;
		
		uint8_t min_val, max_val, val, scale;
		uint32_t j;
		min_val = 255;
		max_val=0;
		scale = 1;
				for (j=SCAN_CONTRAST_BORDERS; j<IMAGE_COLUMNS-SCAN_CONTRAST_BORDERS; j++) {
					val = img_buf[img_buf_wr_ptr^1][j];
					if (val > max_val)
						max_val = val;
					if (val < min_val)
						min_val = val;
				}
				if (max_val-min_val <= SCAN_CONTRAST_SCALE) {
				  scale = SCAN_CONTRAST_SCALE/(max_val-min_val);
				  if (scale>4)
						scale = 4;
					// perform contrast scaling outside of scan_y routine in debug mode
					// to make it visible in debug image
					if (scanCmd == SCAN_CMD_SCAN_DEBUG) {
						for (j=0; j<IMAGE_COLUMNS; j++)
							img_buf[img_buf_wr_ptr^1][j] = (img_buf[img_buf_wr_ptr^1][j]-min_val)*scale;
						scale = 1;
					}
				}
				 
		if (scanCmd == SCAN_CMD_SCAN_DEBUG) {
			if (scans % DEBUG_CAPTURE_NTH == 0)
				if(HAL_UART_Transmit_DMA(&UartHandle, AudioPacketBuf, PACKET_HDR_LEN)!= HAL_OK) Error_Handler();
			if ((scan_lines < DEBUG_SCAN_LINES) && ((scan_lines < DEBUG_SCAN_LINES/2) ? (scans % DEBUG_CAPTURE_NTH == 6) : (scans % DEBUG_CAPTURE_NTH == 5)) ) {
					if(HAL_UART_Transmit_DMA(&UartHandle, img_buf[img_buf_wr_ptr^1], IMAGE_COLUMNS) != HAL_OK) Error_Handler();
					scan_lines++;
				}
		}
		scans++;
						
		// Process the full image array, calling symbol_handler when a barcode is detected
		zbar_scan_y_new(scanner, img_buf[img_buf_wr_ptr^1], IMAGE_COLUMNS, min_val, scale);
		// Process unfinished edges and start a new scan
		zbar_scanner_new_scan(scanner);			
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) // HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static uint8_t audio_samp_count = 0;
	static uint8_t discard_samp = 0;
	static uint32_t flashAddress = FLASH_USER_START_ADDR;
	static uint32_t flashReadAddress = FLASH_USER_START_ADDR;
	static uint8_t audio_stopped = 0;

	uint32_t i;
	
	//HAL_GPIO_WritePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN, GPIO_PIN_SET);
	
	  if (!scan_decoded) {
			
			// Copy data from DMA buffer and start recording next sample
			// (could be done better with DMA ping-pong buffering using SPI interrupts)
			for (i=0; i<INTERNAL_BUFF_SIZE; i++)
				InternalBufferCopy[i]=InternalBuffer[i];

		  if(HAL_SPI_Receive_DMA(&SpiHandle, (uint8_t *)InternalBuffer, 2*INTERNAL_BUFF_SIZE) != HAL_OK) Error_Handler();
			

				// Convert 1ms of PDM data to 16-bit PCM data
				// For 16kHz/16-bit PCM with a decimation factor of 64, this corresponds to
				// 16*64bits = 128 bytes read from the SPI interface (at 1MHz SCK)
				// (1MHz SCK -> 15.625kHz actual recording frequency; 1.024MHz -> 16kHz)			
				PDM_Filter_64_LSB((uint8_t*)&InternalBufferCopy[0], (uint16_t*)&(RecBuf[0]), DEFAULT_AUDIO_IN_VOLUME , (PDMFilter_InitStruct *)&Filter[0]);

				// Discard the first samples to allow the microphone's power supply to stabilize
				// and the PDM filter to stabilize
				if (discard_samp < AUDIO_DISCARD_SAMP) {
					discard_samp++;
					return;
				}

				// Down-sample from 16kHz to 8KHz, convert 16-bit PCM to 8-bit a-law
				if ((scanCmd == SCAN_CMD_MIC_SCAN_INSTANT) && (flashAddress < FLASH_USER_END_ADDR)) {
						  for (i=0; i<(PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR)/2; i+=4) {
							  uint32_t samples = ALaw_Encode(RecBuf[2*(i+3)])&0xFF;
							  samples = (samples << 8) | (ALaw_Encode(RecBuf[2*(i+2)])&0xFF);
							  samples = (samples << 8) | (ALaw_Encode(RecBuf[2*(i+1)])&0xFF);
							  samples = (samples << 8) | (ALaw_Encode(RecBuf[2*i])&0xFF);
							  HAL_FLASH_Program(TYPEPROGRAM_WORD, 
																  flashAddress+audio_samp_count*(PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR)/2+i,
																  samples);
						}
				} else
				   for (i=0; i<(PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR)/2; i++)
					   AudioPacketBuf[PACKET_HDR_LEN + audio_samp_count*(PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR)/2 + i] = ALaw_Encode(RecBuf[2*i]);
				
				if ((flashAddress == FLASH_USER_END_ADDR) && (flashReadAddress < FLASH_USER_END_ADDR)) {
					if (audio_samp_count % AUDIO_QUEUED_PACKET_SPACING == 0)
						if(HAL_UART_Transmit_DMA(&UartHandle, AudioPacketBuf, PACKET_HDR_LEN)!= HAL_OK) Error_Handler();
					if (audio_samp_count % AUDIO_QUEUED_PACKET_SPACING == 2) {
						if(HAL_UART_Transmit_DMA(&UartHandle,(uint8_t *) flashReadAddress, AUDIO_PAYLOAD_LEN)!= HAL_OK) Error_Handler();
						flashReadAddress += FLASH_PAGE_SIZE;
					}
				}
					
				audio_samp_count++;
			  // send the packet header ahead of time to avoid overwriting of data in AudioPacketBuf
				// if the interrupt routine is called again prior to the data having been transmitted
			  if ((audio_samp_count == AUDIO_SAMP_PER_PACKET-2) &&
					 ((scanCmd != SCAN_CMD_MIC_SCAN_INSTANT) || (flashAddress >= FLASH_USER_END_ADDR)) && !audio_stopped)
				    if(HAL_UART_Transmit_DMA(&UartHandle, AudioPacketBuf, PACKET_HDR_LEN)!= HAL_OK) Error_Handler();
				if (audio_samp_count >= AUDIO_SAMP_PER_PACKET) {
					audio_samp_count = 0;
					if ((scanCmd == SCAN_CMD_MIC_SCAN_INSTANT) && (flashAddress <= FLASH_USER_END_ADDR))
							flashAddress += FLASH_PAGE_SIZE;
					if (((scanCmd != SCAN_CMD_MIC_SCAN_INSTANT) || (flashAddress > FLASH_USER_END_ADDR)) && !audio_stopped) {
					  if(HAL_UART_Transmit_DMA(&UartHandle, &AudioPacketBuf[PACKET_HDR_LEN], AUDIO_PAYLOAD_LEN)!= HAL_OK) Error_Handler();
						if (stop_audio)
							audio_stopped = 1;
					}
		    }
			}
	//HAL_GPIO_WritePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	// flash scanner LEDs to signal an error
  while (1) {
		uint32_t j;
		for (j=0; j<500000; j++);
		setScannerLED(0);
		for (j=0; j<500000; j++);
		setScannerLED(1);				
	}	
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
