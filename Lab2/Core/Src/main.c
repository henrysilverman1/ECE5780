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
volatile int exti0I;
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
		//Clock enables
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable peripheral clock to SYSCONFIG
	
		
		
		//Create handler
		 
		
	
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
	//LED config register
		GPIOC->MODER |= ((1 <<	12) | (1 <<	14) | (1 <<	16) | (1 <<	18)); //Setting second MODER bits to 1 for general-purpose output mode for PC6 and PC7
		GPIOC->MODER &= ~((1 <<	13) | (1 <<	15) | (1 <<	17) | (1 <<	19)); // Setting first MODER bits to 0 general-purpose output mode for PC6 and PC7
		GPIOC->OTYPER &= ~((1 << 6) | (1 << 7)| (1 << 8) | (1 << 9)); //Setting OTYPER bits to 0 for output push-pull
		GPIOC->OSPEEDR &= ~((1 <<	12) | (1 <<	14) | (1 <<	16) | (1 <<	18)); //Setting second OSPEEDR bits to 0 for low speed
		GPIOC->PUPDR &= ~((1 <<	12) | (1 <<	13) | (1 <<	14) | (1 <<	15) | (1 <<	16) | (1 <<	17) | (1 <<	18) | (1 <<	19)); //Setting both PUPDR bits 0 for no pull-up, pull-down
		GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8)); //Set blue, red, and green LED's low
		GPIOC->ODR |= ((1 << 9)); //Set Green LED high
	
		//Button config register
		GPIOA->MODER &= ~((1 << 0) | (1 << 1));
		GPIOA->OSPEEDR &= ~((1 << 0));
		GPIOA->PUPDR |= (1 << 1);
		GPIOA->PUPDR &= ~((1 << 0));
		
		//Syscfg pin set 
		SYSCFG->EXTICR[0] &= ~(0xb01111); //Selects PA0 for EXTI0
		
		//EXTI Congig
		EXTI->IMR |= (1<<0); //Unmasks interrupt request from line 0
		EXTI->RTSR |= (1<<0); //Rising edge trigger on line 0
		
		//Enable EXTI Interrupt
		NVIC_EnableIRQ(EXTI0_1_IRQn);
		//Set priority of EXTI Interrupt
		NVIC_SetPriority(EXTI0_1_IRQn, 2);
		//Set priority of SysTick Interrupt
		NVIC_SetPriority(SysTick_IRQn , 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// Toggle PC6
    GPIOC->ODR ^= (1 << 6);
		HAL_Delay(400); // Delay 400ms
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

void EXTI0_1_IRQHandler(void)
{
	// Toggle PC8
  GPIOC->ODR ^= (1 << 8);
	// Toggle PC9
  GPIOC->ODR ^= (1 << 9);
	exti0I = 0;
	while(exti0I<1500000)
	{
		exti0I ++;
	}
	
	// Toggle PC8
  GPIOC->ODR ^= (1 << 8);
	// Toggle PC9
  GPIOC->ODR ^= (1 << 9);

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
	//Set flag that event occured
	EXTI->PR |= (1 << 0);
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
