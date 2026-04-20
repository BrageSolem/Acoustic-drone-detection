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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include <string.h>
	#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define OUTPUT_SAMPLES 16
	#define PDM_DMA_BYTES 512
	//#define USB_CHANNELS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

CRC_HandleTypeDef hcrc;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

/* USER CODE BEGIN PV */

	volatile uint8_t sai_pdm_half_ready = 0;
	volatile uint8_t sai_pdm_full_ready = 0;
	extern uint8_t CDC_Ready;
	volatile uint32_t usb_ok = 0, usb_busy = 0, usb_fail = 0;

	//uint8_t pdm_buffer[256 * 4];   // mics 1,2,3,4 // 1024 bytes
	uint8_t pdm_buffer[256];
	int16_t pcm_buffer[32];
	// one micuint8_t usb_tx_buff[2 + OUTPUT_SAMPLES * 2];
	uint8_t usb_tx_buff[2 + OUTPUT_SAMPLES * 2 * 2]; // give an explenation


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
	static void PeriphCommonClock_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	static void PeriphCommonClock_Config(void)
	{
		RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
		PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;

		PeriphClkInitStruct.PLL2.PLL2M = 8;
		PeriphClkInitStruct.PLL2.PLL2N = 192;
		PeriphClkInitStruct.PLL2.PLL2P = 2;
		PeriphClkInitStruct.PLL2.PLL2Q = 2;
		PeriphClkInitStruct.PLL2.PLL2R = 2;
		PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
		PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
		PeriphClkInitStruct.PLL2.PLL2FRACN = 0;

		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		{
			Error_Handler();
		}
	}


	void USB_Send_PCM_4CH(int16_t *ch1, int16_t *ch2, int16_t *ch3, int16_t *ch4, uint16_t n) //, int16_t *ch3, int16_t *ch4
	{
		uint16_t sync = 0xA55A;
		uint16_t idx = 0;

		//HAL_UART_Transmit(&huart2, (uint8_t *)&sync, 2, HAL_MAX_DELAY);
		memcpy(&usb_tx_buff[idx], &sync, 2);
		idx += 2;

		for (uint16_t i = 0; i < n; i++){
		//HAL_UART_Transmit(&huart2, (uint8_t *)&ch1[i], 2, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart2, (uint8_t *)&ch2[i], 2, HAL_MAX_DELAY);
			memcpy(&usb_tx_buff[idx], &ch1[i], 2); // number of bytes = 2, due to the output sample being 16 bit = 2 bytes
			idx += 2;
			memcpy(&usb_tx_buff[idx], &ch2[i], 2);
			idx += 2;
			memcpy(&usb_tx_buff[idx], &ch3[i], 2);
			idx += 2;
			memcpy(&usb_tx_buff[idx], &ch4[i], 2);
			idx += 2;

		}
		uint8_t r = CDC_Transmit_HS(usb_tx_buff, idx);
		if (r == USBD_OK) {
			usb_ok++;
		} else if (r == USBD_BUSY) {
			usb_busy++;
		} else {
			usb_fail++;
		}
	}

	void USB_Send_PCM_2CH(int16_t *ch1, int16_t *ch2, uint16_t n) //, int16_t *ch3, int16_t *ch4
		{
			uint16_t sync = 0xA55A;
			uint16_t idx = 0;

			//HAL_UART_Transmit(&huart2, (uint8_t *)&sync, 2, HAL_MAX_DELAY);
			memcpy(&usb_tx_buff[idx], &sync, 2);
			idx += 2;

			for (uint16_t i = 0; i < n; i++){
			//HAL_UART_Transmit(&huart2, (uint8_t *)&ch1[i], 2, HAL_MAX_DELAY);
			//HAL_UART_Transmit(&huart2, (uint8_t *)&ch2[i], 2, HAL_MAX_DELAY);
				memcpy(&usb_tx_buff[idx], &ch1[i], 2); // number of bytes = 2, due to the output sample being 16 bit = 2 bytes
				idx += 2;
				memcpy(&usb_tx_buff[idx], &ch2[i], 2);
				idx += 2;


			}
			uint8_t r = CDC_Transmit_HS(usb_tx_buff, idx);
			if (r == USBD_OK) {
				usb_ok++;
			} else if (r == USBD_BUSY) {
				usb_busy++;
			} else {
				usb_fail++;
			}
		}

	void USB_Send_PCM_1CH(int16_t *ch1, uint16_t n) //, int16_t *ch3, int16_t *ch4
		{
			uint16_t sync = 0xA55A;
			uint16_t idx = 0;

			//HAL_UART_Transmit(&huart2, (uint8_t *)&sync, 2, HAL_MAX_DELAY);
			memcpy(&usb_tx_buff[idx], &sync, 2);
			idx += 2;

			for (uint16_t i = 0; i < n; i++){
			//HAL_UART_Transmit(&huart2, (uint8_t *)&ch1[i], 2, HAL_MAX_DELAY);
			//HAL_UART_Transmit(&huart2, (uint8_t *)&ch2[i], 2, HAL_MAX_DELAY);
				memcpy(&usb_tx_buff[idx], &ch1[i], 2); // number of bytes = 2, due to the output sample being 16 bit = 2 bytes
				idx += 2;

			}
			uint8_t r = CDC_Transmit_HS(usb_tx_buff, idx);
			if (r == USBD_OK) {
				usb_ok++;
			} else if (r == USBD_BUSY) {
				usb_busy++;
			} else {
				usb_fail++;
			}
		}

	void PDM_Deinterleave_4_mics(const uint8_t *src, uint8_t *mic1, uint8_t *mic2,uint8_t *mic3 ,uint8_t *mic4, uint16_t len)
	{
		for (uint16_t i = 0; i < len; i += 4)
		{
			mic1[i/4] = ((uint8_t*)src)[i];
			mic2[i/4] = ((uint8_t*)src)[i + 1];
			mic3[i/4] = ((uint8_t*)src)[i + 2];
			mic4[i/4] = ((uint8_t*)src)[i + 3];
		}
	}


	void PCM_Deinterleave_2_mics(const int16_t *src, int16_t *mic1, int16_t *mic2)
	{
	    for (uint16_t i = 0; i < 16; i++)
	    {
	        mic1[i] = src[2 * i];
	        mic2[i] = src[2 * i + 1];
	    }
	}
	void USB_Send_Raw_PDM(const uint8_t *buf, uint16_t n)
	{
	    uint16_t sync = 0xA55A;
	    uint16_t idx = 0;
	    uint8_t tx[2 + 64];

	    memcpy(&tx[idx], &sync, 2);
	    idx += 2;

	    memcpy(&tx[idx], buf, n);
	    idx += n;

	    if (CDC_Ready) {
	        CDC_Transmit_HS(tx, idx);
	    }
	}

	void USB_Send_PCM_Interleaved_2CH(int16_t *pcm, uint16_t n_per_ch)
	{
	    uint16_t sync = 0xA55A;
	    uint16_t idx = 0;

	    memcpy(&usb_tx_buff[idx], &sync, 2);
	    idx += 2;

	    memcpy(&usb_tx_buff[idx], pcm, n_per_ch * 2 * sizeof(int16_t));
	    idx += n_per_ch * 2 * sizeof(int16_t);

	    uint8_t r = CDC_Transmit_HS(usb_tx_buff, idx);
	    if (r == USBD_OK) {
	        usb_ok++;
	    } else if (r == USBD_BUSY) {
	        usb_busy++;
	    } else {
	        usb_fail++;
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

  /* USER CODE BEGIN SysInit */
	  PeriphCommonClock_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  /* USER CODE BEGIN 2 */
	  //HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)pdm_buffer, 1024); // mic1, mic2, mic3, mic4....
	  if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*)pdm_buffer, 256) != HAL_OK)
	  {
		  Error_Handler();
	  }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		  /*
		  if (sai_pdm_half_ready) {
		      sai_pdm_half_ready = 0;


		      PDM_Filter(&pdm_buffer[0], (int16_t*)&pcm_buffer[0], &PDM_FilterHandler[0]);
		      PDM_Filter(&pdm_buffer[1], (int16_t*)&pcm_buffer[1], &PDM_FilterHandler[1]);

		      if (CDC_Ready) {
		          USB_Send_PCM_Interleaved_2CH(pcm_buffer, 16);
		      }
		  }

		  */

		  if (sai_pdm_full_ready) {
			  sai_pdm_full_ready = 0;
		  		      PDM_Filter(&(((uint8_t*)pdm_buffer)[0]), &(((int16_t*)pcm_buffer)[0]), &PDM_FilterHandler[0]);
		  		      PDM_Filter(&(((uint8_t*)pdm_buffer)[1]), &(((int16_t*)pcm_buffer)[0]), &PDM_FilterHandler[1]);

		  		      if (CDC_Ready) {
		  		          USB_Send_PCM_Interleaved_2CH(pcm_buffer, 16);
		  		      }
		  		  }

		  /*
		  if (sai_pdm_full_ready) {
			  sai_pdm_full_ready = 0;

			  static int16_t pcm_mic1[16];
			  static int16_t pcm_mic2[16];

	          PDM_Filter(pdm_buffer, pcm_buffer, &PDM1_filter_handler);
	          PCM_Deinterleave_2_mics(&pcm_buffer[0], pcm_mic1, pcm_mic2);

	          if (CDC_Ready) {
	        	  USB_Send_PCM_2CH(pcm_mic1, pcm_mic2, OUTPUT_SAMPLES);
	          		          }

		  }
		  */
		  /*
		      if (sai_pdm_full_ready) {
		          sai_pdm_full_ready = 0;

		          static uint8_t pdm_mic1[128];
		          static uint8_t pdm_mic2[128];

		          static int16_t pcm_mic1[16];
		          static int16_t pcm_mic2[16];


		          PDM_Deinterleave_2_mics(&pdm_buffer[0], pdm_mic1, pdm_mic2, 256);

		          PDM_Filter(pdm_mic1, pcm_mic1, &PDM1_filter_handler);
		          PDM_Filter(pdm_mic2, pcm_mic2, &PDM2_filter_handler);

		          if (CDC_Ready) {
		              USB_Send_PCM_2CH(pcm_mic1, pcm_mic2, OUTPUT_SAMPLES);
		          }
		      }
*/


		  /* TEST USB
		  uint16_t sync = 0xA55A;
		      uint8_t tx[2 + 16 * 4 * 2];   // sync + 16 samples * 4 ch * 2 bytes
		      uint16_t idx = 0;

		      int16_t mic1 = 100;
		      int16_t mic2 = 200;
		      int16_t mic3 = 300;
		      int16_t mic4 = 400;

		      memcpy(&tx[idx], &sync, 2);
		      idx += 2;

		      for (uint16_t i = 0; i < 16; i++)
		      {
		          memcpy(&tx[idx], &mic1, 2); idx += 2;
		          memcpy(&tx[idx], &mic2, 2); idx += 2;
		          memcpy(&tx[idx], &mic3, 2); idx += 2;
		          memcpy(&tx[idx], &mic4, 2); idx += 2;
		      }

		      if (CDC_Ready)
		      {
		          uint8_t r = CDC_Transmit_HS(tx, idx);
		          (void)r;
		      }

		      HAL_Delay(100);
*/
/*

		  if (sai_pdm_half_ready){
					sai_pdm_half_ready = 0;

					static  uint8_t pdm_mic1[128];
					static uint8_t pdm_mic2[128];
					static uint8_t pdm_mic3[128];
					static uint8_t pdm_mic4[128];

					static int16_t pcm_mic1[16];
					static int16_t pcm_mic2[16];
					static int16_t pcm_mic3[16];
					static int16_t pcm_mic4[16];

					PDM_Deinterleave_4_mics(&pdm_buffer[0], pdm_mic1,pdm_mic2,pdm_mic3,pdm_mic4, 512); // 512 to make sure that only the half of the buffer is being deinterleaved

					PDM_Filter(pdm_buffer[0], pcm_mic1, &PDM1_filter_handler);
					PDM_Filter(pdm_mic2, pcm_mic2, &PDM2_filter_handler);
					PDM_Filter(pdm_mic3, pcm_mic3, &PDM3_filter_handler);
					PDM_Filter(pdm_mic4, pcm_mic4, &PDM4_filter_handler);


					if (CDC_Ready){
						USB_Send_PCM_4CH(pcm_mic1, pcm_mic2, pcm_mic3, pcm_mic4,16);
						USB_Send_PCM_1CH(pcm_mic1, 16);
					}
			  }
			  if (sai_pdm_full_ready){
					  sai_pdm_full_ready= 0;

					static  uint8_t pdm_mic1[128];
					static uint8_t pdm_mic2[128];
					static uint8_t pdm_mic3[128];
					static uint8_t pdm_mic4[128];

					static int16_t pcm_mic1[16];
					static int16_t pcm_mic2[16];
					static int16_t pcm_mic3[16];
					static int16_t pcm_mic4[16];

					PDM_Deinterleave_4_mics(&pdm_buffer[512], pdm_mic1,pdm_mic2,pdm_mic3,pdm_mic4, 512);

					PDM_Filter(pdm_mic1, pcm_mic1, &PDM1_filter_handler);
					PDM_Filter(pdm_mic2, pcm_mic2, &PDM2_filter_handler);
					PDM_Filter(pdm_mic3, pcm_mic3, &PDM3_filter_handler);
					PDM_Filter(pdm_mic4, pcm_mic4, &PDM4_filter_handler);



					  if (CDC_Ready){
					  USB_Send_PCM_4CH(pcm_mic1, pcm_mic2, pcm_mic3, pcm_mic4, 16);
						USB_Send_PCM_1CH(pcm_mic1, 16);

					  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
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
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.PdmInit.Activation = ENABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK2_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 16;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000001;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
volatile uint32_t half_cnt = 0;
volatile uint32_t full_cnt = 0;

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
    if (hsai == &hsai_BlockA1){
        sai_pdm_half_ready = 1;
        half_cnt++;
    }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
    if (hsai == &hsai_BlockA1){
        sai_pdm_full_ready = 1;
        full_cnt++;
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
