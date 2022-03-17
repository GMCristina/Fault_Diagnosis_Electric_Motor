/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADE9000_API.h"
#include <stdio.h>
#include <ADE9000CalibrationInputs.h>
#include <ADE9000_Calibration.h>
#include <Fault_Diagnosis.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x300400c0
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x300400c0))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  ADE9000_Power();

//  test_read_write_reg();
  setvbuf( stdin, NULL, _IONBF, 0 );

  ADE9000_Setup();

  printf("%d,%d,%d\r\n",N_BUFFER, N_SAMPLE,BURST_READ_N);

  printf("fdti: %f\t fdtv: %f \r\n",CURRENT_TRANSFER_FUNCTION,VOLTAGE_TRANSFER_FUNCTION);
  //ADE9000_Calibration();

  uint16_t index = 0;
  uint32_t start;
  uint32_t value_reg_32 = 0x00020000;
  uint16_t value_reg_16;

  Start_Waveform_Buffer();

  while(index < N_SAMPLE){
 		  while(flag_read == 0){}
 		 //uint32_t tickstart = HAL_GetTick();

		  //printf("nint: %d\t", n_int);
 		  flag_read = 0;
 		  ADE9000_SPI_Write_32(ADDR_STATUS0,value_reg_32);

		  value_reg_16 = ADE9000_SPI_Read_16(ADDR_WFB_TRG_STAT);
		  value_reg_16 = (value_reg_16>>12)&0x0F;
		  printf("pg: %i\r\n",value_reg_16);
 		  start = WAVEFORM_BUFFER_START_ADDR;



 		 ADE9000_SPI_Burst_Read_one_ch(start,BURST_READ_N,&ia[index].data_int);
 		 //ADE9000_SPI_Burst_Read_two_ch(start, BURST_READ_N,ia + index,va +index);
 		 //ADE9000_SPI_Burst_Read_two_ch(start, BURST_READ_N,&ia[index].data_int,&va[index].data_int);

 		  //printf("1 index %d\r\n",index);
 		  index += BURST_READ_N;
 		/*
 		 uint32_t tickend = HAL_GetTick();
 		 uint32_t ntick = tickend-tickstart;
 		 printf("TIME: %d MS\r\n",ntick);
 		*/
 		  while(flag_read == 0){}

  		 //tickstart = HAL_GetTick();
 		  //printf("nint: %d\t", n_int);
 		  flag_read = 0;
 		  ADE9000_SPI_Write_32(ADDR_STATUS0,value_reg_32);
 		  value_reg_16 = ADE9000_SPI_Read_16(ADDR_WFB_TRG_STAT);
 		  value_reg_16 = (value_reg_16>>12)&0x0F;
 		  printf("pg: %i\r\n",value_reg_16);
 		  start = WAVEFORM_BUFFER_START_ADDR + BURST_READ_N*8;


 		 ADE9000_SPI_Burst_Read_one_ch(start,BURST_READ_N,&ia[index].data_int);
 		// ADE9000_SPI_Burst_Read_two_ch(start, BURST_READ_N,ia + index,va +index);
 		//ADE9000_SPI_Burst_Read_two_ch(start, BURST_READ_N,&ia[index].data_int,&va[index].data_int);

 		 //printf("2 index %d\r\n",index);
 		 index += BURST_READ_N;
/*
   		tickend = HAL_GetTick();
   		 ntick = tickend-tickstart;
   		 printf("TIME: %d MS\r\n",ntick);
*/

 }
  Stop_Waveform_Buffer();

  /*
  printf("VA,IA\r\n");
  for(uint32_t i = 0; i<N_SAMPLE; i++){
	  printf("%d,%d\r\n",va[i].data_int,ia[i].data_int);
  }
  */

  //ADE9000_Conv_ADC_V(va,N_SAMPLE);
  ADE9000_Conv_ADC_I(ia,N_SAMPLE);
/*
  printf("VA,IA\r\n");
  for(uint32_t i = 0; i<N_SAMPLE; i++){
	  printf("%f,%f\r\n",va[i].data_float,ia[i].data_float);
  }

*/
  printf("IA\r\n");
  for(uint32_t i = 0; i<N_SAMPLE; i++){
	  printf("%f\r\n",ia[i].data_float);
  }

  FD_Wavedec(Wavelet_dec,Wavelet_dec_dim,&ia[0].data_float);

  printf("Dec\r\n");
  for(uint32_t i = 0; i<N_DEC_WAVELET; i++){
	  printf("%f\r\n",Wavelet_dec[i]);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ADE9000_Reset_Pin|ADE9000_PM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADE9000_CS_GPIO_Port, ADE9000_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ADE9000_Reset_Pin ADE9000_PM1_Pin */
  GPIO_InitStruct.Pin = ADE9000_Reset_Pin|ADE9000_PM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ADE9000_IRQ1_Pin */
  GPIO_InitStruct.Pin = ADE9000_IRQ1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADE9000_IRQ1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADE9000_CS_Pin */
  GPIO_InitStruct.Pin = ADE9000_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADE9000_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADE9000_IRQ0_Pin */
  GPIO_InitStruct.Pin = ADE9000_IRQ0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADE9000_IRQ0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

int __io_getchar(void)
{

	uint8_t ch;
	// Clear the Overrun flag just before receiving the first character
	__HAL_UART_CLEAR_OREFLAG(&huart3);

	HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// If the interrupt source is pin IRQ0
	if (GPIO_Pin == ADE9000_IRQ0_Pin)
	{
		//printf("IRQ0s\r\n");
		flag_read = 1;
		n_int ++;
		//uint32_t value_reg_32 = 0x00020000;
		//ADE9000_SPI_Write_32(ADDR_STATUS0,value_reg_32);
		/*
		  uint16_t value_reg_16 = ADE9000_SPI_Read_16(ADDR_WFB_TRG_STAT);
		  value_reg_16 = (value_reg_16>>12)&0x0F;
		  printf("pgi: %i\r\n",value_reg_16);
		  uint32_t value_reg_32 = 0x00020000;
		  ADE9000_SPI_Write_32(ADDR_STATUS0,value_reg_32);
		*/
		//printf("IRQ0e\r\n");


/*
		uint32_t value_reg_32 = ADE9000_SPI_Read_32(ADDR_STATUS0);
		  if((value_reg_32 & 0x00020000)!=0){
			  //last page full
			  uint16_t value_reg_16 = ADE9000_SPI_Read_16(ADDR_WFB_TRG_STAT);
			  flag_read = (value_reg_16>>12)&0x0F;
			  printf("Int: %i\r\n",flag_read);
			  //clear interrupt
			 value_reg_32 = value_reg_32 & 0x00020000;
			 ADE9000_SPI_Write_32(ADDR_STATUS0,value_reg_32);
		  }
*/

	}
	if (GPIO_Pin == ADE9000_IRQ1_Pin)
		{
			//printf("IRQ1\r\n");
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

