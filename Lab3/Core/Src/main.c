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
  /* USER CODE BEGIN 2 */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	
	//Red and Blue LED setup (PC6 & PC7)
	GPIOC->MODER |= ((1<<15) | (1<<13));//AF mode
	GPIOC->MODER &= ~((1<<12) | (1<<14));
	GPIOC->AFR[0] &= ~((1<<24)| (1<<25)| (1<<26)| (1<<27)); //Set AF PC6
	GPIOC->AFR[0] &= ~((1<<28)| (1<<29)| (1<<30)| (1<<31)); //Set AF PC7
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7)); //Setting OTYPER bits to 0 for output push-pull
	GPIOC->OSPEEDR &= ~((1 <<	12) | (1 <<	13) | (1 <<	14) | (1 <<	15)); //Setting both OSPEEDR bits to 0 for low speed
	GPIOC->PUPDR &= ~((1 <<	12) | (1 <<	13) | (1 <<	14) | (1 <<	15)); //Setting both PUPDR bits 0 for no pull-up, pull-down
	
	//Orange and green LED setup (PC8 & PC9)
	GPIOC->MODER |= ((1 <<	16) | (1 <<	18)); //Setting second MODER bits to 1 for general-purpose output mode for PC8 and PC9
	GPIOC->MODER &= ~((1 <<	17) | (1 <<	19)); // Setting first MODER bits to 0 general-purpose output mode for PC8 and PC9
	GPIOC->OTYPER &= ~((1 << 8) | (1 << 9)); //Setting OTYPER bits to 0 for output push-pull
	GPIOC->OSPEEDR &= ~((1 <<	16) | (1 <<	17) | (1 <<	18) | (1 <<	19)); //Setting both OSPEEDR bits to 0 for low speed
	GPIOC->PUPDR &= ~((1 <<	16) | (1 <<	17) | (1 <<	18) | (1 <<	19)); //Setting both PUPDR bits 0 for no pull-up, pull-down
	
	
	GPIOC->ODR |= (1 << 6); //Set red LED high
	GPIOC->ODR |= (1 << 7); //Set blue LED high
	GPIOC->ODR &= ~((1 << 8)); //Set orange LED low
	GPIOC->ODR |= (1 << 9); //Set Green LED high

	
	//Timer 2 config
	TIM2->PSC = 3999; //PSC to 7999 to clk is 1kHz
	TIM2->ARR = 250; //4Hz
	TIM2->DIER |= (1<<0);// DEIR register set to enable update interrupt
	TIM2->CR1 |= (1<<0);//Timer 2 enable
	
	//Timer 3 config
	TIM3->PSC = 7; // Clk freq to 100kHz
	TIM3->ARR = 1250; // 800Hz
	/*Bit definitions for CCMR1 reg:
			Bit 3: Output compare preload ch1 enable
			Bits 4,5,6: Ch1 to PWM mode 2
			Bit 11: Output compare preload ch1 enable
			Bits 12,13,14: Ch2 to PWM mode 1
			*/
	TIM3->CCMR1 &= ~(/*(1<<0) | (1<<4) | (1<<1) | (1<<8) | (1<<9) |*/ (1<<12)); //Output ch1 to PWM mode 2, 
	TIM3->CCMR1 |= ((1<<3) | (1<<4) |(1<<5) | (1<<6) | (1<<11) | (1<<13) | (1<<14)); 
	TIM3->CCER |= ((1<<0) | (1<<4)); //Enable ch1 and ch2
	TIM3->CCR1 = 1250;//CCRx 20% ARR
	TIM3->CCR2 = 1250;
	TIM3 -> CR1 |= (1<<0);//Timer 3 enable
	
	NVIC_EnableIRQ(TIM2_IRQn);

		
	
	
  /* USER CODE END 2 */

  /* USER CODE END 2 */
	
	// Enable EXTI interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
	
	// Enable Timer 2
	TIM2->CR1 |= (1<<0);
	// Enable Timer 3
	TIM3->CR1 |= (1<<0);

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

void TIM2_IRQHandler(void)
{
	// Toggle PC8
  GPIOC->ODR ^= (1 << 8);
	// Toggle PC9
  GPIOC->ODR ^= (1 << 9);

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
	//Set flag that event occured
	TIM2->SR &= ~(1 << 0);
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
