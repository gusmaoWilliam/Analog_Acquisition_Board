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
#include "eth.h"
#include "usart.h"
#include "usb_otg.h"
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



typedef struct
{
  uint16_t max_mV;
  uint16_t min_mV;
  uint16_t max_value;
  uint16_t min_value;
  uint8_t on_off;
  uint16_t curr_mV;
} ADC_PAR;

void calc_mV(ADC_PAR *AD, uint16_t val);

#define MAX 1
#define MIN 0
#define ON 1
#define OFF 0
enum
{
  COM,
  ADC_CH,
  MAX_MIN,
  ON_OFF = MAX_MIN,
  VALUE_mV
};


ADC_PAR mADC1, mADC2, mADC3;
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
uint8_t usart_rxbuffer[255];
uint8_t usart_txbuffer[255];
uint8_t rx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  rx=1;
  HAL_UART_Receive_IT(&huart3, usart_rxbuffer, 8);
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, usart_rxbuffer, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart3, (uint8_t*)"Iniciando...\r\n", 16, 10);

  shared_data_ptr->Ts_time_elapsed_from4to7=0;
  shared_data_ptr->adc_average_from4to7=0;
  shared_data_ptr->pwm_from4to7=0;
  shared_data_ptr->setpoint_from7to4=0;

  while (1)
  {
    if(shared_data_ptr->Ts_time_elapsed_from4to7)
    {

      if(mADC1.on_off == ON)
        calc_mV(&mADC1, shared_data_ptr->ADC1value_from4to7);
      else
        mADC1.curr_mV = 0;

      if(mADC2.on_off == ON)
        calc_mV(&mADC2, shared_data_ptr->ADC2value_from4to7);
      else
        mADC2.curr_mV = 0;

      if(mADC3.on_off == ON)
        calc_mV(&mADC3, shared_data_ptr->ADC3value_from4to7);
      else
        mADC3.curr_mV = 0;

      shared_data_ptr->Ts_time_elapsed_from4to7 = 0;
      sprintf(usart_txbuffer, "%u,%u,%u\n", mADC1.curr_mV, mADC2.curr_mV, mADC3.curr_mV);
      HAL_UART_Transmit(&huart3, (uint8_t*)usart_txbuffer, strlen(usart_txbuffer), 10);
    }
    if(rx)
    {
	  char *p;
      rx=0;
      switch(usart_rxbuffer[COM])
      {
        case 'C':

          switch (usart_rxbuffer[ADC_CH])
          {
            case '1':
              
              p = usart_rxbuffer[VALUE_mV];
              if(usart_rxbuffer[MAX_MIN] == MAX)
              {
                mADC1.max_mV = atoi((uint8_t*)p);
                mADC1.max_value = 0;
              }
              else
              {
                mADC1.min_mV = 0;
                mADC1.min_value = shared_data_ptr->ADC1value_from4to7;
              }
              break;
            case '2':
              p = usart_rxbuffer[VALUE_mV];
              if(usart_rxbuffer[MAX_MIN] == MAX)
              {
                mADC2.max_mV = atoi((uint8_t*)p);
                mADC2.max_value = 0;
              }
              else
              {
                mADC2.min_mV = 0;
                mADC2.min_value = shared_data_ptr->ADC2value_from4to7;
              }
              break;
            case '3':
              p = usart_rxbuffer[VALUE_mV];
              if(usart_rxbuffer[MAX_MIN] == MAX)
              {
                mADC3.max_mV = atoi((uint8_t*)p);
                mADC3.max_value = 0;
              }
              else
              {
                mADC3.min_mV = 0;
                mADC3.min_value = shared_data_ptr->ADC3value_from4to7;
              }
              break;
            default:
              break;
          }
          break;
        case 'T':
          usart_rxbuffer[0] = '0';
          switch (usart_rxbuffer[ADC_CH])
          {
          case '1':
            mADC1.on_off = usart_rxbuffer[ON_OFF];
            break;
          case '2':
            mADC2.on_off = usart_rxbuffer[ON_OFF];
            break;
          case '3':
            mADC3.on_off = usart_rxbuffer[ON_OFF];
            break;
          default:
            break;
          }
        break;
        default:
        break;
      }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 120;
  PeriphClkInitStruct.PLL2.PLL2P = 6;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void calc_mV(ADC_PAR *AD, uint16_t val)
{
  double res;
  res = (val-AD->min_value)*AD->max_mV/AD->max_value;
  AD->curr_mV = res;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
