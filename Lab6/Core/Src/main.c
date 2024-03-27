/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef ledString = {GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9, //Initialize all LED's to output, low freq, no pull up pull down
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &ledString);
	
	//Configure the ADC input to PA0/ADC_IN0
	GPIOA -> MODER |= (1<<1) | (1<<0); // Analog mode
	GPIOA -> MODER &= ~(1<<1) | (1<<0); // No pull-up, pull-down
	
	//enable ADC in RCC
	__HAL_RCC_ADC1_CLK_ENABLE();
	ADC1 -> CFGR1 |= (1<<4); //Set 8-bit with 10 in bits 4:3
	ADC1 -> CFGR1 &= ~(1<<3); 
	ADC1 -> CFGR1 |= (1<<13); //Set continuous conversion with 1 in bit 13
	ADC1 -> CFGR1 &= ~((1<<11)|(1<<10)); //Disable hardware trigger with 00 in bits 11:10
	ADC1 -> CHSELR |= (1<<1); //Select channel associated with PAO with 1 in bit 1
	
	//ADC calibration
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN by setting ADDIS*/
	/* (3) Clear DMAEN */
	/* (4) Launch the calibration by setting ADCAL */
	/* (5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
	ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
	
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{
	
	}
	
	//ADC enable
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
	{
	ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
	{
	
	}
	
	//Start ADC
	ADC1 -> CR |= (1<<2);
												
	
	

  
  while (1)
  {
    int dataADC = ADC1 -> DR;
		if (dataADC > 0)
			GPIOC -> ODR |= (1<<6);
		else
			GPIOC -> ODR &= ~(1<<6);
		if (dataADC >= 64)
			GPIOC -> ODR |= (1<<9);
		else
			GPIOC -> ODR &= ~(1<<9);
		if (dataADC >= 128)
			GPIOC -> ODR |= (1<<7);
		else
			GPIOC -> ODR &= ~(1<<7);
		if (dataADC >= 192)
			GPIOC -> ODR |= (1<<8);
		else
			GPIOC -> ODR &= ~(1<<8);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
