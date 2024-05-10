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
	
int receive_value = 0; //set intial receive value
int receive_flag = 0; //clear receive flag
int led_num = 0; //initialize LED number
int flag = 0; //
void trans_char(char c) //Function to transmit single character
{
	while(1) 
	{
		if ((USART3->ISR & (1<<7)) == (1<<7)) //Check stop bit
		{
			break;
		}
	}
	USART3->TDR = c; // Write character to UART register
}
	
void trans_string(const char* string) // Function to transmit string of charachters
{
	while(*string != '\0') //Check to see if end of string has been reached
	{
		trans_char(*string); //transmit current char of string
		string++; //Move to next char of string
	}
}

void USART3_4_IRQHandler(void)
{
	while (!(USART3->ISR & USART_ISR_RXNE)) {}//Check flags
	
	receive_value = (USART3->RDR) &= 0xff;
	
	switch (receive_value)//Upon getting character
	{
		case 'r': 
			trans_string("RED LED\n");
			led_num = 6;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			break;
		case 'b':
			trans_string("BLUE LED\n");
			led_num = 7;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
			break;
		case 'o':
			trans_string("ORANGE LED\n");
			led_num = 8;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			break;
		case 'g':
			trans_string("GREEN LED\n");
			led_num = 9;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			break;
		case '0':
			if (led_num != 0)
			{
				trans_string("COMMAND OFF\n");
				GPIOC->ODR &= ~(1<<led_num);
			}
			break;
		case '1':
			if (led_num != 0)
			{
				trans_string("COMMAND ON\n");
				GPIOC->ODR |= (1<<led_num);
			}
			break;
		case '2':
			if (led_num != 0)
			{
				trans_string("COMMAND TOGGLE\n");
				GPIOC->ODR ^= (1<<led_num);
			}
			break;
		default:
			trans_string("Error: Ivalid Command\n");
			led_num = 0;
	}
	receive_flag = 1; // Set received flag 
}
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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	
	/* Set PB10 as TX for USART3 */
	// Set to af
	GPIOB->MODER &= ~(1<<20);
	GPIOB->MODER |= (1<<21);
	// Set to af4
	GPIOB->AFR[1] &= ~(1<<11);
	GPIOB->AFR[1] |= (1<<10);
	GPIOB->AFR[1] &= ~(1<<9);
	GPIOB->AFR[1] &= ~(1<<8);
	
	/* Setup PB11 as RX for USART3 */
	// Set to af
	GPIOB->MODER &= ~(1<<22);
	GPIOB->MODER |= (1<<23);
	// Set to af4
	GPIOB->AFR[1] &= ~(1<<15);
	GPIOB->AFR[1] |= (1<<14);
	GPIOB->AFR[1] &= ~(1<<13);
	GPIOB->AFR[1] &= ~(1<<12);
	
	/* Setup for USART3*/
	USART3->CR1 |= (1<<2); // Enable transceiver and receiver
	USART3->CR1 |= (1<<3);
	USART3->CR1 |= (1<<5); // Enable interrupt
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600; // Set baud rate to 9600 bits/sec
	USART3->CR1 |= (1<<0); // Enable USART3
	
	NVIC_EnableIRQ(USART3_4_IRQn); // Enable interrupt for USART3
	NVIC_SetPriority(USART3_4_IRQn, 2); // Set priority of interrupt to 2
	
	
	GPIOC->MODER |= (1<<12);// Set PC6 (red) to general purpose output mode
	GPIOC->MODER &= ~(1<<13);
	GPIOC->OTYPER &= ~(1<<6); // Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<12); // Set to low speed
	GPIOC->OSPEEDR &= ~(1<<13);
	GPIOC->PUPDR &= ~(1<<12); // Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<13);
	GPIOC->ODR &= ~(1<<6); // Initialize to low
	
	
	GPIOC->MODER |= (1<<14); // Set PC7 (blue) to general purpose output mode
	GPIOC->MODER &= ~(1<<15);
	GPIOC->OTYPER &= ~(1<<7); // Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<14); // Set to low speed
	GPIOC->OSPEEDR &= ~(1<<15);
	GPIOC->PUPDR &= ~(1<<14); // Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<15);
	GPIOC->ODR &= ~(1<<7);// Set low
	

	GPIOC->MODER |= (1<<16); // Set PC8 (orange) to general purpose output mode
	GPIOC->MODER &= ~(1<<17);
	GPIOC->OTYPER &= ~(1<<8);// Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<16);// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<17);
	GPIOC->PUPDR &= ~(1<<16); // Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<17);
	GPIOC->ODR &= ~(1<<8); // Set low
	
	
	GPIOC->MODER |= (1<<18);// Set PC9 (green) to general purpose output mode
	GPIOC->MODER &= ~(1<<19);
	GPIOC->OTYPER &= ~(1<<9); // Set to push-pull mode
	GPIOC->OSPEEDR &= ~(1<<18); // Set to low speed
	GPIOC->OSPEEDR &= ~(1<<19);
	GPIOC->PUPDR &= ~(1<<18); // Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<19);
	GPIOC->ODR &= ~(1<<9); // Initialize to low
																
															
	GPIOB->AFR[1] |= 0x4400;//TX and RX line af select to 0100											
															
	int freq = HAL_RCC_GetHCLKFreq();
	int baud = 115200;
	USART3->BRR |= (freq/baud);
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_UE;
	
  /* USER CODE END 2 */													
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    {
		if(!flag) // Check if CMD? has already been sent
		{
			trans_string("CMD?\n");
			flag = 1;
		}
		if (receive_flag) // Reset values if receive flag is on
		{
			flag = 0;
			receive_flag = 0;
			receive_value = 0;
			led_num = 0;
		}
	}

		
		

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