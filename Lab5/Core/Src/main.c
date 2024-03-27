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
void blockWriteI2C(uint8_t slaveAddress,int transmission){
	I2C2->CR2 = 0;
	//Slave address set
	I2C2 -> CR2 |= (0x69 << 1);
	//Num bits to transmit															
	I2C2 -> CR2 |= (1 << 16);
	//Indicate write
	I2C2 -> CR2 &= ~(1 << 10);
	//Set start bit
	I2C2 -> CR2 |= (1 << 13);
	while(1) {
		if (I2C2 -> ISR & (1<<4)) { //Check NACKF flag
			//return;
		}
		else if (I2C2 -> ISR & (1<<1)) { //Check TXIS flag
			I2C2 -> TXDR = transmission; // Set TXDR to who am I
			break;
		}
	}
	while(1){
		if (I2C2 -> ISR & (1<<6)) { //Check TC flag
			break;
		}
	}
}
int blockReadI2C(uint8_t slaveAddress,int expectedValue){
	I2C2->CR2 = 0;
	//Slave address set
	I2C2 -> CR2 |= (0x69 << 1);
	//Num bits to transmit															
	I2C2 -> CR2 |= (1 << 16);
	//Indicate read
	I2C2 -> CR2 |= (1 << 10);
	//Set start bit
	I2C2 -> CR2 = (1 << 13);
	while(1) {
		if (I2C2 -> ISR & (1<<4)) { //Check NACKF flag
			return 2;
		}
		else if (I2C2 -> ISR & (1<<2)) { //Check RXNE flag
			break;
		}
	}
	int rxdr = I2C2 -> RXDR;
	while(1){
		if (I2C2 -> ISR & (1<<6)) { //Check TC flag
			break;
		}
	}
	if (rxdr == expectedValue) {
		return 1;
	}
	else{
		return 0;
	}
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();


  /* Initialize all configured peripherals */
	GPIOB -> MODER |= (1 << 23);
	GPIOB -> MODER &= ~(1 << 22); // set PB11 to alternate function mode
	GPIOB -> MODER |= (1 << 27);
	GPIOB -> MODER &= ~(1 << 26); // set PB13 to alternate function mode
	GPIOB -> MODER |= (1 << 28);
	GPIOB -> MODER &= ~(1 << 29); // set PB14 to output mode
	GPIOC -> MODER |= (1 << 0);
	GPIOC -> MODER &= ~(1 << 1); // set PC0 to output mode
	
	GPIOB -> OTYPER |= (1 << 11); // set PB11 to open-drain
	GPIOB -> OTYPER |= (1 << 13); // set PB13 to open-drain
	GPIOB -> OTYPER &= ~(1 << 14); // set PB14 to push-pull
	GPIOC -> OTYPER &= ~(1 << 0); // set PC0 to push-pull
	
	GPIOB -> AFR[1] |= (1 << 12); 
	GPIOB->AFR[1] &= (~(1<<13)| ~(1<<14)| ~(1<<15)); // select I2C2_SDA as PB11 alternate function
	
	GPIOB->AFR[1] &= (~(1<<21)| ~(1<<23));
	GPIOB->AFR[1] |= (1<<20)| (1<<22); // select I2C2_SCL as PB13 alternate function
	
	GPIOB -> ODR |= (1 << 14); // set PB14 to high 
	GPIOC -> ODR |= 1; // set PC0 to high
															
															
	I2C2->TIMINGR &= (~(1<<31) | ~(1<<30) | ~(1<<29)); 
	I2C2->TIMINGR |= (1<<28); //Set PRESC to 1													
	I2C2->TIMINGR |= (0x13); //SCLL set to 0x13
	I2C2->TIMINGR |= (0xF<<8); //SCLH set to 0xF											
	I2C2->TIMINGR |= (0x2<<16); //SDADEL set to 0x2												
	I2C2->TIMINGR |= (0x4<<20); //SCLDEL set to 0x4
	I2C2->CR1 |= 0x1; //I2C2 RC1 Enable
	
	//LED config with HAL
	GPIO_InitTypeDef ledString = {GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &ledString);
															
	/*Use I2C to write to a slave device
			blockWriteI2C(slave address, transmit bytes, transmit data)												
	*/
	blockWriteI2C(0x69, 0x0F);
	
	/*Use I2C to read from a slave device
			blockReadI2C(slave address, transmit bytes)												
	*/
	int RXDR = blockReadI2C(0x69, 0xD3);
															
	//Check value against expected WHO_AM_I register
	if (RXDR == 1)
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); //togle red LED
	else if (RXDR ==2)
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);//toggle blue LED
	else
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);//Toggle orange LED
	//Stop transaction
	I2C2 -> CR2 &= ~(1 << 14);
															

  while (1)
  {
		
	}

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
