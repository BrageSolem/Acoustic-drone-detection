/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : pdm2pcm.c
  * Description        : This file provides code for the configuration
  *                      of the pdm2pcm instances.
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
#include "pdm2pcm.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Global variables ---------------------------------------------------------*/
PDM_Filter_Handler_t PDM1_filter_handler; // for each microphone
PDM_Filter_Handler_t PDM2_filter_handler;
PDM_Filter_Handler_t PDM3_filter_handler;
PDM_Filter_Handler_t PDM4_filter_handler;

PDM_Filter_Config_t PDM1_filter_config;
PDM_Filter_Config_t PDM2_filter_config;
PDM_Filter_Config_t PDM3_filter_config;
PDM_Filter_Config_t PDM4_filter_config;

PDM_Filter_Handler_t* handlers[] = {&PDM1_filter_handler,&PDM2_filter_handler,&PDM3_filter_handler,&PDM4_filter_handler};
PDM_Filter_Config_t* configs[] = {&PDM1_filter_config, &PDM2_filter_config, &PDM3_filter_config, &PDM4_filter_config};

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/* PDM2PCM init function */
void MX_PDM2PCM_Init(void)
{
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

   /**
  */
  for(int i = 0; i<4; i++){
	  handlers[i]->bit_order = PDM_FILTER_BIT_ORDER_MSB;
	  handlers[i]->endianness = PDM_FILTER_ENDIANNESS_BE;
	  handlers[i]->high_pass_tap = 2104533974;
	  handlers[i]->in_ptr_channels= 2;
	  handlers[i]->out_ptr_channels = 1;
	  //handlers[i]->channel =  (i%2);
	  PDM_Filter_Init(handlers[i]);

	  configs[i]->decimation_factor= PDM_FILTER_DEC_FACTOR_128;
	  configs[i]->output_samples_number = 16;
	  configs[i]->mic_gain = 0;
	  PDM_Filter_setConfig(handlers[i], configs[i]);

	  /*
  PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_MSB;
  PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_BE;
  PDM1_filter_handler.high_pass_tap = 2104533974;
  PDM1_filter_handler.in_ptr_channels = 1;
  PDM1_filter_handler.out_ptr_channels = 1;
  PDM_Filter_Init(&PDM1_filter_handler);

  PDM1_filter_config.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
  PDM1_filter_config.output_samples_number = 16;
  PDM1_filter_config.mic_gain = 0;
  PDM_Filter_setConfig(&PDM1_filter_handler, &PDM1_filter_config);
*/

  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */

/*  process function */
uint8_t MX_PDM2PCM_Process(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
  /*
  uint8_t BSP_AUDIO_IN_PDMToPCM(uint16_t * PDMBuf, uint16_t * PCMBuf)

  Converts audio format from PDM to PCM.
  Parameters:
    PDMBuf : Pointer to PDM buffer data
    PCMBuf : Pointer to PCM buffer data
  Return values:
    AUDIO_OK in case of success, AUDIO_ERROR otherwise
  */
  /* this example return the default status AUDIO_ERROR */
  return (uint8_t) 1;
}

/* USER CODE END 4 */

/**
  * @}
  */
