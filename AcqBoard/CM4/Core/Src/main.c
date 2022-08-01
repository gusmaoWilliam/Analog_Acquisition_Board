/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct shared_data
{
	uint16_t Ts_time_elapsed_from4to7;
	uint16_t adc_average_from4to7;
  uint8_t pwm_from4to7;
  uint8_t setpoint_from7to4;
  uint8_t flagR_from7to4;

  uint16_t ADC1value_from4to7;
  uint16_t ADC2value_from4to7;
  uint16_t ADC3value_from4to7;

};

volatile struct shared_data * const shared_data_ptr = (struct shared_data *)0x38001000;

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

/* USER CODE BEGIN PV */



typedef struct 
{
  uint16_t adc_buffer[100];
  uint16_t adc_index;
  uint16_t adc_average;
  uint32_t adc_sum_temp;
} Estrutura_AD;

Estrutura_AD MyADC1, MyADC2, MyADC3;

uint8_t Ts_time_elapsed = 0;

uint16_t debug = 0;
uint16_t debug_buffer[100];
uint16_t i = 0;

uint8_t setpoint = 0;
uint8_t pwm_out = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == &hadc1)
  {
    MyADC1.adc_buffer[MyADC1.adc_index] = HAL_ADC_GetValue(&hadc1);
    MyADC1.adc_index++;
  }
  else if(hadc->Instance == &hadc2)
  {
    MyADC2.adc_buffer[MyADC2.adc_index] = HAL_ADC_GetValue(&hadc2);
    MyADC2.adc_index++;
  }
  else if(hadc->Instance == &hadc3)
  {
    MyADC3.adc_buffer[MyADC3.adc_index] = HAL_ADC_GetValue(&hadc3);
    MyADC3.adc_index++;
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    Ts_time_elapsed = 1;

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim15);
  HAL_TIM_Base_Start(&htim4);

  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start_IT(&hadc3);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint16_t local_adc = 0;
  int16_t y =0;
  uint8_t  value_test = 0;  
  uint32_t atualiza_pwm_test;
  uint8_t flag2 = 0;
  atualiza_pwm_test =  HAL_GetTick();
  shared_data_ptr->flagR_from7to4 = 0;
  double e_i = 0;

  double u_p = 0;
  double u_i = 0;
  double u_i_ant = 0;



  PWM_SetValue(10);

  while (1)
  {
    

    if(shared_data_ptr->flagR_from7to4 == 1)
    {

    }
    if(Ts_time_elapsed)
    {
      Ts_time_elapsed = 0;

      MyADC1.adc_sum_temp = 0;
      MyADC2.adc_sum_temp = 0;
      MyADC3.adc_sum_temp = 0;
      for(uint8_t i = 0; i < MyADC1.adc_index; i++)
      {
        MyADC1.adc_sum_temp += MyADC1.adc_buffer[i];
      }
      for(uint8_t i = 0; i < MyADC2.adc_index; i++)
      {
        MyADC2.adc_sum_temp += MyADC2.adc_buffer[i];
      }
      for(uint8_t i = 0; i < MyADC3.adc_index; i++)
      {
        MyADC3.adc_sum_temp += MyADC3.adc_buffer[i];
      }
      shared_data_ptr->ADC1value_from4to7 = MyADC1.adc_average = MyADC1.adc_sum_temp/MyADC1.adc_index;
      shared_data_ptr->ADC2value_from4to7 = MyADC2.adc_average = MyADC1.adc_sum_temp/MyADC1.adc_index;
      shared_data_ptr->ADC3value_from4to7 = MyADC3.adc_average = MyADC1.adc_sum_temp/MyADC1.adc_index;
      
      MyADC1.adc_index = 0;
      MyADC2.adc_index = 0;
      MyADC3.adc_index = 0;
      
      shared_data_ptr->Ts_time_elapsed_from4to7 = 1;
      
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
