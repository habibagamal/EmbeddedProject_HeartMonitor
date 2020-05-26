/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t adc_buffer[1] = {0};
uint16_t ADCValue;
#define COMMAND_SIZE 6
char command [COMMAND_SIZE];

int lastPeekTime = 0;
int BPM;
int time1 = 0;
int time2 = 0;
int BPMcalc = 0;
#define minRR 600
#define maxRR 1200
#define maxThreshold 3800

int lastEdge2 = 0, lastEdge3 = 0, lastEdge4 = 0;
int detectEdge(int thisEdge);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			//start ADC
			HAL_ADC_Start_IT(&hadc1);
//			//start timer 3 that is sampling trigger for ADC	
//			HAL_TIM_Base_Start(&htim3);
//			//start timer 3 that counts 1 minute
//			HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			HAL_UART_Receive_IT(&huart1,(uint8_t*) &command, sizeof(command));


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
////////////////////////////////////////
// called when ADC conversion is complete
////////////////////////////////////////
int detectEdge(int thisEdge){
	if (thisEdge > maxThreshold && thisEdge > lastEdge2 && lastEdge2 > lastEdge3 && lastEdge3 > lastEdge4){
		int currTime = __HAL_TIM_GET_COUNTER(& htim2);
		if((currTime - lastPeekTime) >= minRR){
			if (time1 == 0){
				time1 = currTime;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			}
			else if ((currTime - time1) > maxRR){
				time1 = currTime;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			}
			else if (time2 == 0){
				time2 = currTime;
				BPMcalc = 1;				
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			}
			lastPeekTime = currTime;
			return  1;
		}
	}
	return 0;
}

////////////////////////////////////////
// called when ADC conversion is complete
////////////////////////////////////////
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  /* This is called after the conversion is completed */
	UNUSED(hadc); 
	
	// read ADC converted value
	ADCValue = HAL_ADC_GetValue(&hadc1);
	
	if (BPMcalc == 0){
		if (__HAL_TIM_GET_COUNTER(&htim2) >= 12000){
			detectEdge(ADCValue);
		}
		lastEdge4 = lastEdge3;
		lastEdge3 = lastEdge2;
		lastEdge2 = ADCValue;
	}
	
	//convert to string and transmit over UART
	char buffer[10];
	int n = sprintf (buffer, "%d", ADCValue);
	buffer[n] = '\r';
	buffer[n+1] = '\n';
	for (int i = 0; i < (n+2); i++){
		HAL_UART_Transmit(&huart1,(uint8_t *) &buffer[i], sizeof(buffer[i]), 10);
	}
}
////////////////////////////////////////
// Called whenever UART receive IT is done
////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1){
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		//////////////////
		//command to start collecting 1 min worth of data
		//////////////////
		if(command[0] == 's' && command[1] == 't' && command[2] == 'a' && command[3] == 'r' && command[4] == 't'){	
			BPMcalc = 0;
			time1 = 0;
			time2 = 0;
			lastEdge2 = 0, lastEdge3 = 0, lastEdge4 = 0;

			__HAL_TIM_SET_COUNTER(&htim2, 0);
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			//start timer 3 that is sampling trigger for ADC	
			HAL_TIM_Base_Start(&htim3);
			//start timer 3 that counts 1 minute
			HAL_TIM_Base_Start_IT(&htim2);
		
			HAL_UART_Transmit(&huart1,(uint8_t *) command, sizeof(command), 100);
			char endLine[2] = "\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t *) endLine, sizeof(endLine), 100);

		} 
		//////////////////
		//command to set sampling rate
		//////////////////
		else if (command[0] == 'f' && command[1] == '='){
			//Get numerical value of freuqncy as string
			char freq[COMMAND_SIZE - 1]; 
			for (int i = 0; i < (COMMAND_SIZE - 2); i++){
				freq[i] = command[2+i];
			}
			freq[COMMAND_SIZE - 2] = '\0';
			//convert the frequency to integer
			const char * temp = freq; 
			int f;
			f = atoi(temp);
			//compute the time
			int time = 1000.0 / f;

			__HAL_TIM_SET_COUNTER(&htim2, 0);
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			//change the value of period register to time
			__HAL_TIM_SET_AUTORELOAD(&htim3, time);
			
			HAL_UART_Transmit(&huart1,(uint8_t *) command, sizeof(command), 100);
			char endLine[2] = "\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t *) endLine, sizeof(endLine), 100);
		} 
		//////////////////
		// command to get bpm
		//////////////////
		else if (command[0] == 'b' && command[1] == 'p' && command[2] == 'm'){
				//compute and UART transmit BPM
				
//				float numberOfSquares = (time2 - time1) / 200.0;
//			
//				float BPM = 300 / numberOfSquares;
				float BPM = 60 / ((time2 - time1)/1000.0);
			
				char c[4] = "bpm=";
				HAL_UART_Transmit(&huart1,(uint8_t *) c, sizeof(c), 100);
				
				char buffer [32]; 
				int n = sprintf (buffer, "%f", BPM);
				buffer[n] = '\r';
				buffer[n+1] = '\n';
				for (int i = 0; i < (n+2); i++){
					HAL_UART_Transmit(&huart1,(uint8_t *) &buffer[i], sizeof(buffer[i]), 10);
				}
		} 
		else {
			char buffer[12] = "no command\r\n";
			HAL_UART_Transmit(&huart1,(uint8_t *) buffer, sizeof(buffer), 100);
		}
	}
}
////////////////////////////////////////
// Called when every UART byte is received
////////////////////////////////////////
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
////////////////////////////////////////
// Called when timer period has elapsed
////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	
	// 1 min elapsed for timer 2
	if (htim->Instance == TIM2){
		if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET){
			// Stop timer 2
			HAL_TIM_Base_Stop(&htim2);
			// Stop timer 3
			HAL_TIM_Base_Stop(&htim3);
		}
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
