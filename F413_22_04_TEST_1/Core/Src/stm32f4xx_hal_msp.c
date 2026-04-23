/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_dfsdm2_flt0;

extern DMA_HandleTypeDef hdma_dfsdm2_flt1;

extern DMA_HandleTypeDef hdma_dfsdm2_flt2;

extern DMA_HandleTypeDef hdma_dfsdm2_flt3;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief CRC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
    /* USER CODE BEGIN CRC_MspInit 0 */

    /* USER CODE END CRC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
    /* USER CODE BEGIN CRC_MspInit 1 */

    /* USER CODE END CRC_MspInit 1 */

  }

}

/**
  * @brief CRC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
    /* USER CODE BEGIN CRC_MspDeInit 0 */

    /* USER CODE END CRC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
    /* USER CODE BEGIN CRC_MspDeInit 1 */

    /* USER CODE END CRC_MspDeInit 1 */
  }

}

static uint32_t HAL_RCC_DFSDM2_CLK_ENABLED=0;

static uint32_t DFSDM2_Init = 0;
/**
  * @brief DFSDM_Filter MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdfsdm_filter: DFSDM_Filter handle pointer
  * @retval None
  */
void HAL_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(!(IS_DFSDM1_FILTER_INSTANCE(hdfsdm_filter->Instance))&&(DFSDM2_Init == 0))
  {
    /* USER CODE BEGIN DFSDM2_MspInit 0 */

    /* USER CODE END DFSDM2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DFSDM2;
    PeriphClkInitStruct.Dfsdm2ClockSelection = RCC_DFSDM2CLKSOURCE_APB2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_DFSDM2_CLK_ENABLED++;
    if(HAL_RCC_DFSDM2_CLK_ENABLED==1){
      __HAL_RCC_DFSDM2_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**DFSDM2 GPIO Configuration
    PA7     ------> DFSDM2_DATIN1
    PC5     ------> DFSDM2_DATIN2
    PE10     ------> DFSDM2_DATIN0
    PB10     ------> DFSDM2_CKOUT
    PC9     ------> DFSDM2_DATIN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_DFSDM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM2;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_DFSDM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_DFSDM2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN DFSDM2_MspInit 1 */

    /* USER CODE END DFSDM2_MspInit 1 */

  DFSDM2_Init++;
  }

    /* DFSDM2 DMA Init */
    /* DFSDM2_FLT0 Init */
  if(hdfsdm_filter->Instance == DFSDM2_Filter0){
    hdma_dfsdm2_flt0.Instance = DMA2_Stream0;
    hdma_dfsdm2_flt0.Init.Channel = DMA_CHANNEL_8;
    hdma_dfsdm2_flt0.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dfsdm2_flt0.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dfsdm2_flt0.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dfsdm2_flt0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dfsdm2_flt0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dfsdm2_flt0.Init.Mode = DMA_CIRCULAR;
    hdma_dfsdm2_flt0.Init.Priority = DMA_PRIORITY_LOW;
    hdma_dfsdm2_flt0.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dfsdm2_flt0) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm2_flt0);
    __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm2_flt0);
  }

    /* DFSDM2_FLT1 Init */
  if(hdfsdm_filter->Instance == DFSDM2_Filter1){
    hdma_dfsdm2_flt1.Instance = DMA2_Stream1;
    hdma_dfsdm2_flt1.Init.Channel = DMA_CHANNEL_8;
    hdma_dfsdm2_flt1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dfsdm2_flt1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dfsdm2_flt1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dfsdm2_flt1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dfsdm2_flt1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dfsdm2_flt1.Init.Mode = DMA_CIRCULAR;
    hdma_dfsdm2_flt1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_dfsdm2_flt1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dfsdm2_flt1) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm2_flt1);
    __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm2_flt1);
  }

    /* DFSDM2_FLT2 Init */
  if(hdfsdm_filter->Instance == DFSDM2_Filter2){
    hdma_dfsdm2_flt2.Instance = DMA2_Stream2;
    hdma_dfsdm2_flt2.Init.Channel = DMA_CHANNEL_8;
    hdma_dfsdm2_flt2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dfsdm2_flt2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dfsdm2_flt2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dfsdm2_flt2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dfsdm2_flt2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dfsdm2_flt2.Init.Mode = DMA_CIRCULAR;
    hdma_dfsdm2_flt2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_dfsdm2_flt2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dfsdm2_flt2) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm2_flt2);
    __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm2_flt2);
  }

    /* DFSDM2_FLT3 Init */
  if(hdfsdm_filter->Instance == DFSDM2_Filter3){
    hdma_dfsdm2_flt3.Instance = DMA2_Stream3;
    hdma_dfsdm2_flt3.Init.Channel = DMA_CHANNEL_8;
    hdma_dfsdm2_flt3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dfsdm2_flt3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dfsdm2_flt3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dfsdm2_flt3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dfsdm2_flt3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dfsdm2_flt3.Init.Mode = DMA_CIRCULAR;
    hdma_dfsdm2_flt3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_dfsdm2_flt3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dfsdm2_flt3) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm2_flt3);
    __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm2_flt3);
  }

}

/**
  * @brief DFSDM_Channel MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdfsdm_channel: DFSDM_Channel handle pointer
  * @retval None
  */
void HAL_DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(!(IS_DFSDM1_CHANNEL_INSTANCE(hdfsdm_channel->Instance))&&(DFSDM2_Init == 0))
  {
    /* USER CODE BEGIN DFSDM2_MspInit 0 */

    /* USER CODE END DFSDM2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DFSDM2;
    PeriphClkInitStruct.Dfsdm2ClockSelection = RCC_DFSDM2CLKSOURCE_APB2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_DFSDM2_CLK_ENABLED++;
    if(HAL_RCC_DFSDM2_CLK_ENABLED==1){
      __HAL_RCC_DFSDM2_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**DFSDM2 GPIO Configuration
    PA7     ------> DFSDM2_DATIN1
    PC5     ------> DFSDM2_DATIN2
    PE10     ------> DFSDM2_DATIN0
    PB10     ------> DFSDM2_CKOUT
    PC9     ------> DFSDM2_DATIN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_DFSDM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM2;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_DFSDM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_DFSDM2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN DFSDM2_MspInit 1 */

    /* USER CODE END DFSDM2_MspInit 1 */

  DFSDM2_Init++;
  }

}

/**
  * @brief DFSDM_Filter MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdfsdm_filter: DFSDM_Filter handle pointer
  * @retval None
  */
void HAL_DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  if(!(IS_DFSDM1_FILTER_INSTANCE(hdfsdm_filter->Instance)))
  {
    DFSDM2_Init-- ;
    if((DFSDM2_Init == 0))
    {
    /* USER CODE BEGIN DFSDM2_MspDeInit 0 */

    /* USER CODE END DFSDM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DFSDM2_CLK_DISABLE();

    /**DFSDM2 GPIO Configuration
    PA7     ------> DFSDM2_DATIN1
    PC5     ------> DFSDM2_DATIN2
    PE10     ------> DFSDM2_DATIN0
    PB10     ------> DFSDM2_CKOUT
    PC9     ------> DFSDM2_DATIN3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5|GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* DFSDM2 DMA DeInit */
    HAL_DMA_DeInit(hdfsdm_filter->hdmaInj);
    HAL_DMA_DeInit(hdfsdm_filter->hdmaReg);
    /* USER CODE BEGIN DFSDM2_MspDeInit 1 */

    /* USER CODE END DFSDM2_MspDeInit 1 */
    }
  }

}

/**
  * @brief DFSDM_Channel MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdfsdm_channel: DFSDM_Channel handle pointer
  * @retval None
  */
void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{
  if(!(IS_DFSDM1_CHANNEL_INSTANCE(hdfsdm_channel->Instance)))
  {
    DFSDM2_Init-- ;
    if((DFSDM2_Init == 0))
    {
    /* USER CODE BEGIN DFSDM2_MspDeInit 0 */

    /* USER CODE END DFSDM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DFSDM2_CLK_DISABLE();

    /**DFSDM2 GPIO Configuration
    PA7     ------> DFSDM2_DATIN1
    PC5     ------> DFSDM2_DATIN2
    PE10     ------> DFSDM2_DATIN0
    PB10     ------> DFSDM2_CKOUT
    PC9     ------> DFSDM2_DATIN3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5|GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* USER CODE BEGIN DFSDM2_MspDeInit 1 */

    /* USER CODE END DFSDM2_MspDeInit 1 */
    }
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
