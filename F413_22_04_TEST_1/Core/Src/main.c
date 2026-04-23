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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DFSDM_Filter_HandleTypeDef hdfsdm2_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm2_filter1;
DFSDM_Filter_HandleTypeDef hdfsdm2_filter2;
DFSDM_Filter_HandleTypeDef hdfsdm2_filter3;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel0;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel2;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel3;
DMA_HandleTypeDef hdma_dfsdm2_flt0;
DMA_HandleTypeDef hdma_dfsdm2_flt1;
DMA_HandleTypeDef hdma_dfsdm2_flt2;
DMA_HandleTypeDef hdma_dfsdm2_flt3;

/* USER CODE BEGIN PV */
#define PCM_SAMPLES_PER_HALF  16
#define DFSDM_DMA_SAMPLES     (PCM_SAMPLES_PER_HALF * 2) //(PCM_SAMPLES_PER_HALF * 2)
#define USB_SYNC_0            0x5A
#define USB_SYNC_1            0xA5

static int32_t pcm0_dma[DFSDM_DMA_SAMPLES];
static int32_t pcm1_dma[DFSDM_DMA_SAMPLES];
static int32_t pcm2_dma[DFSDM_DMA_SAMPLES];
static int32_t pcm3_dma[DFSDM_DMA_SAMPLES];

static volatile uint8_t flt0_half_ready = 0;
static volatile uint8_t flt0_full_ready = 0;
static volatile uint8_t flt1_half_ready = 0;
static volatile uint8_t flt1_full_ready = 0;
static volatile uint8_t flt2_half_ready = 0;
static volatile uint8_t flt2_full_ready = 0;
static volatile uint8_t flt3_half_ready = 0;
static volatile uint8_t flt3_full_ready = 0;

static uint8_t usb_tx_busy = 0;

/* sync + 16 samples mic0 + 16 samples mic1, interleaved int16 */
static uint8_t usb_frame[2 + PCM_SAMPLES_PER_HALF * 4 * 2]; //usb_frame[2 + PCM_SAMPLES_PER_HALF * 4 * 2];


/* test */
static volatile uint32_t cb_half0 = 0;
static volatile uint32_t cb_full0 = 0;
static volatile uint32_t cb_half1 = 0;
static volatile uint32_t cb_full1 = 0;
static volatile uint32_t cb_half2 = 0;
static volatile uint32_t cb_full2 = 0;
static volatile uint32_t cb_half3 = 0;
static volatile uint32_t cb_full3 = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_DFSDM2_Init(void);
/* USER CODE BEGIN PFP */
static void PackAndSendFrame(uint8_t half_index);
static int16_t DFSDM32_to_PCM16(int32_t x);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int16_t DFSDM32_to_PCM16(int32_t x)
{
    x >>= 8;

    if (x > 32767)  x = 32767;
    if (x < -32768) x = -32768;
    return (int16_t)x;
}

// used for troubleshooting

static volatile uint32_t usb_ok_cnt = 0;
static volatile uint32_t usb_busy_cnt = 0;
static volatile uint32_t usb_fail_cnt = 0;
static volatile uint8_t last_tx_ret = 0;

static void PackAndSendFrame(uint8_t half_index)
{
    uint32_t start = (half_index == 0) ? 0 : PCM_SAMPLES_PER_HALF;

    usb_frame[0] = USB_SYNC_0;
    usb_frame[1] = USB_SYNC_1;

    uint32_t j = 2;
    for (uint32_t i = 0; i < PCM_SAMPLES_PER_HALF; i++)
    {
        int16_t mic0 = DFSDM32_to_PCM16(pcm0_dma[start + i]);
        int16_t mic1 = DFSDM32_to_PCM16(pcm1_dma[start + i]);
        int16_t mic2 = DFSDM32_to_PCM16(pcm2_dma[start + i]);
        int16_t mic3 = DFSDM32_to_PCM16(pcm3_dma[start + i]);


        usb_frame[j++] = (uint8_t)(mic0 & 0xFF);
        usb_frame[j++] = (uint8_t)((mic0 >> 8) & 0xFF);
        usb_frame[j++] = (uint8_t)(mic1 & 0xFF);
        usb_frame[j++] = (uint8_t)((mic1 >> 8) & 0xFF);
        usb_frame[j++] = (uint8_t)(mic2 & 0xFF);
        usb_frame[j++] = (uint8_t)((mic2 >> 8) & 0xFF);
        usb_frame[j++] = (uint8_t)(mic3 & 0xFF);
        usb_frame[j++] = (uint8_t)((mic3 >> 8) & 0xFF);
    }

    CDC_Transmit_FS(usb_frame, sizeof(usb_frame));
}


void CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void)Buf;
    (void)Len;
    (void)epnum;
    usb_tx_busy = 0;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_DFSDM2_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter0,
                                       DFSDM_CHANNEL_0,
                                       DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter1,
                                       DFSDM_CHANNEL_1,
                                       DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter2,
                                         DFSDM_CHANNEL_2,
                                         DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter3,
                                         DFSDM_CHANNEL_3,
                                         DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
    {
        Error_Handler();
    }

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm2_filter0, pcm0_dma, DFSDM_DMA_SAMPLES) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm2_filter1, pcm1_dma, DFSDM_DMA_SAMPLES) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm2_filter2, pcm2_dma, DFSDM_DMA_SAMPLES) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm2_filter3, pcm3_dma, DFSDM_DMA_SAMPLES) != HAL_OK)
    {
        Error_Handler();
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	      if (flt0_half_ready && flt1_half_ready && flt2_half_ready && flt3_half_ready)
	      {
	          flt0_half_ready = 0;
	          flt1_half_ready = 0;
	          flt2_half_ready = 0;
	          flt3_half_ready = 0;
	          PackAndSendFrame(0);
	      }

	      if (flt0_full_ready && flt1_full_ready && flt2_full_ready && flt3_full_ready)
	      {
	    	  flt0_full_ready = 0;
	    	  flt1_full_ready = 0;
	    	  flt2_full_ready = 0;
	    	  flt3_full_ready = 0;
	          PackAndSendFrame(1);
	      }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DFSDM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM2_Init(void)
{

  /* USER CODE BEGIN DFSDM2_Init 0 */

  /* USER CODE END DFSDM2_Init 0 */

  /* USER CODE BEGIN DFSDM2_Init 1 */

  /* USER CODE END DFSDM2_Init 1 */
  hdfsdm2_filter0.Instance = DFSDM2_Filter0;
  hdfsdm2_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm2_filter0.Init.RegularParam.FastMode = DISABLE;
  hdfsdm2_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm2_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm2_filter0.Init.FilterParam.Oversampling = 32;
  hdfsdm2_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm2_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_filter1.Instance = DFSDM2_Filter1;
  hdfsdm2_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm2_filter1.Init.RegularParam.FastMode = DISABLE;
  hdfsdm2_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm2_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm2_filter1.Init.FilterParam.Oversampling = 32;
  hdfsdm2_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm2_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_filter2.Instance = DFSDM2_Filter2;
  hdfsdm2_filter2.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm2_filter2.Init.RegularParam.FastMode = DISABLE;
  hdfsdm2_filter2.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm2_filter2.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm2_filter2.Init.FilterParam.Oversampling = 32;
  hdfsdm2_filter2.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm2_filter2) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_filter3.Instance = DFSDM2_Filter3;
  hdfsdm2_filter3.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm2_filter3.Init.RegularParam.FastMode = DISABLE;
  hdfsdm2_filter3.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm2_filter3.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm2_filter3.Init.FilterParam.Oversampling = 32;
  hdfsdm2_filter3.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm2_filter3) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_channel0.Instance = DFSDM2_Channel0;
  hdfsdm2_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel0.Init.OutputClock.Divider = 8;
  hdfsdm2_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm2_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm2_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel0.Init.Awd.Oversampling = 1;
  hdfsdm2_channel0.Init.Offset = 0;
  hdfsdm2_channel0.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_channel1.Instance = DFSDM2_Channel1;
  hdfsdm2_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel1.Init.OutputClock.Divider = 8;
  hdfsdm2_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm2_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm2_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel1.Init.Awd.Oversampling = 1;
  hdfsdm2_channel1.Init.Offset = 0;
  hdfsdm2_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_channel2.Instance = DFSDM2_Channel2;
  hdfsdm2_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel2.Init.OutputClock.Divider = 8;
  hdfsdm2_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm2_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm2_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel2.Init.Awd.Oversampling = 1;
  hdfsdm2_channel2.Init.Offset = 0;
  hdfsdm2_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_channel3.Instance = DFSDM2_Channel3;
  hdfsdm2_channel3.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel3.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel3.Init.OutputClock.Divider = 8;
  hdfsdm2_channel3.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel3.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel3.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel3.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm2_channel3.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm2_channel3.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel3.Init.Awd.Oversampling = 1;
  hdfsdm2_channel3.Init.Offset = 0;
  hdfsdm2_channel3.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter1, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter2, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm2_filter3, DFSDM_CHANNEL_3, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM2_Init 2 */

  /* USER CODE END DFSDM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter->Instance == DFSDM2_Filter0) { flt0_half_ready = 1; cb_half0++; }
    if (hdfsdm_filter->Instance == DFSDM2_Filter1) { flt1_half_ready = 1; cb_half1++; }
    if (hdfsdm_filter->Instance == DFSDM2_Filter2) { flt2_half_ready = 1; cb_half2++; }
    if (hdfsdm_filter->Instance == DFSDM2_Filter3) { flt3_half_ready = 1; cb_half3++; }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter->Instance == DFSDM2_Filter0) { flt0_full_ready = 1; cb_full0++; }
    if (hdfsdm_filter->Instance == DFSDM2_Filter1) { flt1_full_ready = 1; cb_full1++; }
    if (hdfsdm_filter->Instance == DFSDM2_Filter2) { flt2_full_ready = 1; cb_full2++; }
    if (hdfsdm_filter->Instance == DFSDM2_Filter3) { flt3_full_ready = 1; cb_full3++; }
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
