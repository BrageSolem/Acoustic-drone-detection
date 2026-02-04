/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "pdm2pcm_glo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PDM_BUF_LEN 64 // allocates a buffer in RAM of 256 elemetns of type uint16_t (16-bit per element)
#define OUTPUT_SAMPLES 16
#define PDM_DMA_BYTES 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/*
uint8_t pdm_buf[PDM_BUF_LEN];
int16_t pcm_buf_half[OUTPUT_SAMPLES];
int16_t pcm_buf_full[OUTPUT_SAMPLES];
volatile uint8_t pdm_half_ready = 0; // has to use volatilebecause the variable can change at any time outside of the codes control
volatile uint8_t pdm_full_ready = 0;
uint16_t sync = 0xA55A; // marks start of a sequence 5A A5, little endinan, least sig first, Ax16^3 + 5x16^2 + 5x16^1 + Ax16^0 so 5A A5, for msb A5 5A
*/

#define PDM_RX_WORDS   128          // must match your pdmRxBuf[128]
//volatile uint8_t sai_rxstate = 0;   // 1 = half, 2 = full
//volatile uint8_t sai_pdm12_rxstate = 0;
//volatile uint8_t sai_pdm34_rxstate = 0;
volatile uint8_t sai_pdm12_half_ready = 0;
volatile uint8_t sai_pdm12_full_ready = 0;
volatile uint8_t sai_pdm34_half_ready = 0;
volatile uint8_t sai_pdm34_full_ready = 0;


uint16_t pdm12_buf[PDM_DMA_BYTES * 2];   // mics 1+2
uint16_t pdm34_buf[PDM_DMA_BYTES * 2];   // mics 3+4
int16_t pcm1[OUTPUT_SAMPLES];
int16_t pcm2[OUTPUT_SAMPLES];
int16_t pcm3[OUTPUT_SAMPLES];
int16_t pcm4[OUTPUT_SAMPLES];

//uint16_t txBuf[128];
//uint8_t pdmRxBuf[PDM_DMA_BYTES];
int16_t MidBuffer[16];
//uint8_t txstate = 0;
//uint8_t rxstate = 0;

uint16_t fifobuf[256];
uint8_t fifo_w_ptr = 0;
uint8_t fifo_r_ptr = 0;
uint8_t fifo_read_enabled = 0;

void FifoWrite(uint16_t data){
	fifobuf[fifo_w_ptr] = data;
	fifo_w_ptr++;
}

uint16_t FifoRead(){
	uint16_t val = fifobuf[fifo_r_ptr];
	fifo_r_ptr++;
	return val;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Send_PCM_2CH(int16_t *ch1, int16_t *ch2, uint16_t n)
{
    uint16_t sync = 0xA55A;

    HAL_UART_Transmit(&huart2, (uint8_t *)&sync, 2, HAL_MAX_DELAY);

    for (uint16_t i = 0; i < n; i++){
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch1[i], 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch2[i], 2, HAL_MAX_DELAY);
    }
}

void UART_Send_PCM_4CH(int16_t *ch1, int16_t *ch2, int16_t *ch3, int16_t *ch4, uint16_t n){
	uint16_t sync = 0xA55A;

	HAL_UART_Transmit(&huart2, (uint8_t *)&sync, 2, HAL_MAX_DELAY);
	for (uint16_t i = 0; i < n; i++){
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch1[i], 2, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch2[i], 2, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch3[i], 2, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch4[i], 2, HAL_MAX_DELAY);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_SAI1_Init();
  /* USER CODE BEGIN 2 */

  const char test_msg[] = "UART_OK\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)test_msg, sizeof(test_msg)-1, HAL_MAX_DELAY);

  //HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)pdmRxBuf, PDM_DMA_BYTES); // PDM_RX_WORDS*2
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)pdm12_buf, PDM_DMA_BYTES * 2);
  HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)pdm34_buf, PDM_DMA_BYTES * 2);

  //HAL_I2S_Transmit_DMA(&hi2s3, &txBuf[0],64);
  //HAL_I2S_Receive_DMA(&hi2s2, &pdmRxBuf[0],64);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (sai_pdm12_half_ready && sai_pdm34_half_ready){
		  sai_pdm12_half_ready = 0;
		  sai_pdm34_half_ready= 0;

		  PDM_Filter((uint8_t*)&pdm12_buf[0], pcm1, &PDM1_filter_handler);
		  PDM_Filter((uint8_t*)&pdm12_buf[0], pcm2, &PDM2_filter_handler);
		  PDM_Filter((uint8_t*)&pdm34_buf[0], pcm3, &PDM3_filter_handler);
		  PDM_Filter((uint8_t*)&pdm34_buf[0], pcm4, &PDM4_filter_handler);
		  //UART_Send_PCM_4CH(pcm1,pcm2,pcm3, pcm4, OUTPUT_SAMPLES);
		  UART_Send_PCM_2CH(pcm1, pcm4, OUTPUT_SAMPLES);
	  }

	  if (sai_pdm12_full_ready && sai_pdm34_full_ready){
		  sai_pdm12_full_ready = 0;
		  sai_pdm34_full_ready = 0;

	      PDM_Filter((uint8_t*)&pdm12_buf[PDM_DMA_BYTES], pcm1, &PDM1_filter_handler);
	      PDM_Filter((uint8_t*)&pdm12_buf[PDM_DMA_BYTES], pcm2, &PDM2_filter_handler);
	      PDM_Filter((uint8_t*)&pdm34_buf[PDM_DMA_BYTES], pcm3, &PDM3_filter_handler);
	      PDM_Filter((uint8_t*)&pdm34_buf[PDM_DMA_BYTES], pcm4, &PDM4_filter_handler);
	      //UART_Send_PCM_4CH(pcm1,pcm2,pcm3, pcm4, OUTPUT_SAMPLES);
	      UART_Send_PCM_2CH(pcm1, pcm4, OUTPUT_SAMPLES);
	      }
/*
	  if (sai_pdm34_rxstate == 1){
	  		sai_pdm34_rxstate = 0;

	  		PDM_Filter((uint8_t*)&pdm34_buf[0], pcm3, &PDM3_filter_handler);
	  		PDM_Filter((uint8_t*)&pdm34_buf[0], pcm4, &PDM4_filter_handler);
	  		UART_Send_PCM_2CH(pcm3, pcm4, 16);


	  	  }
	  if (sai_pdm34_rxstate == 2){
		  sai_pdm34_rxstate = 0;

		  PDM_Filter((uint8_t*)&pdm34_buf[PDM_DMA_BYTES], pcm3, &PDM3_filter_handler);
	      PDM_Filter((uint8_t*)&pdm34_buf[PDM_DMA_BYTES], pcm4, &PDM4_filter_handler);
	      UART_Send_PCM_2CH(pcm3, pcm4, 16);
	      }
*/


	  // Synthetic data
	 /* static uint32_t t = 0;
	  if (HAL_GetTick() - t > 1000) {
	      t = HAL_GetTick();
	      int16_t test[4] = {1000, -1000, 500, -500};
	      UART_Send_PCM_Frame(test, 4);
	  }
*/
/*// One mic
	  if (sai_rxstate ==1){
		  sai_rxstate = 0;
		  PDM_Filter(&pdmRxBuf[0], (uint16_t*)MidBuffer, &PDM1_filter_handler);
		  UART_Send_PCM_Frame(MidBuffer, 16);
		  //for (int i=0; i<16; i++){FifoWrite(MidBuffer[i]);}
		  //if (fifo_w_ptr-fifo_r_ptr > 128) fifo_read_enabled = 1;


	  }

	  if (sai_rxstate ==2){
		  sai_rxstate = 0;
		  PDM_Filter(&pdmRxBuf[PDM_DMA_BYTES/2], (uint16_t*)MidBuffer, &PDM1_filter_handler);
	  		  //PDM_Filter(&pdmRxBuf[64], &MidBuffer[0], &PDM1_filter_handler);
		  UART_Send_PCM_Frame(MidBuffer, 16);
	  		  //for (int i=0; i<16; i++){FifoWrite(MidBuffer[i]);}


	  }
	  */

	  /*
	  if (txstate ==1){
	  		  if (fifo_read_enabled == 1){
	  			  for (int i=0; i<64; i=i+4){
	  				  uint16_t data = FifoRead();
	  				  txBuf[i] = data;
	  				  txBuf[i+2] = data;

	  		  }

	  	  }
	  		  txstate = 0;
	  }
	  if (txstate ==2){
	  	  		  if (fifo_read_enabled == 1){
	  	  			  for (int i=64; i<128; i=i+4){
	  	  				  uint16_t data = FifoRead();
	  	  				  txBuf[i] = data;
	  	  				  txBuf[i+2] = data;

	  	  		  }

	  	  	  }
	  	  		  txstate = 0;
	  	  }
	  	  */


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 16;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 16;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 2;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.ClockSource = SAI_CLKSOURCE_NA;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.FrameInit.FrameLength = 16;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 2;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Stops the other processes, so do not process and transmit the data here, do it in main instead, only stick to the flags here !
/*
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) // triggers when DMA has filled the first half of the circular buffer
{
	if (hsai == &hsai_BlockB1){ // will be needed when more SAI instances will be used
		pdm_half_ready = 1;
	}
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	if (hsai == &hsai_BlockB1){
		pdm_full_ready = 1;
	}
}
*/
/*
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
        sai_rxstate = 1;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
        sai_rxstate = 2;
}
*/

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
	if (hsai == &hsai_BlockB1){
		sai_pdm12_half_ready = 1;
	}
	if (hsai == &hsai_BlockA1){
		sai_pdm34_half_ready = 1;
	}
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	if (hsai == &hsai_BlockB1){
		sai_pdm12_full_ready = 1;
	}
	if (hsai == &hsai_BlockA1){
		sai_pdm34_full_ready = 1;
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
#ifdef USE_FULL_ASSERT
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
