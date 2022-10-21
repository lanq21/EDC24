/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<string.h>
#include<stdio.h>
#include "pid.h"
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
int16_t lastCnt[4];
pid speedPid[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setSpeed(uint8_t idx, double speed);

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,HAL_MAX_DELAY);
	return ch;
}


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
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	
	HAL_TIM_Base_Start_IT(&htim7);
	
	pid_init(&speedPid[0],10.0f, 0.01f, 0.0f);
	pid_init(&speedPid[1],10.0f, 0.01f, 0.0f);
	pid_init(&speedPid[2],10.0f, 0.01f, 0.0f);
	pid_init(&speedPid[3],5.0f, 0.00f, 0.0f);
	speedPid[0].goal=0;
	speedPid[3].goal=4;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/*
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);

		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,200);
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);

		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,200);
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);

		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,200);
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,0);

		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,200);*/
	
		//setSpeed(1,20.0);
		//setSpeed(2,80.0);
		//setSpeed(3,20.0);
		//setSpeed(3,80.0);
		HAL_Delay(1000);
		//setSpeed(1,-20.0);
		//setSpeed(2,-80.0);
		//setSpeed(3,-20.0);
		//setSpeed(3,-80.0);
		HAL_Delay(1000);
		//char* ch="hello,world";
		//HAL_UART_Transmit(&huart2,(uint8_t*)ch,11,HAL_MAX_DELAY);
		//HAL_Delay(1000);
		
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void setSpeed(uint8_t idx, double speed){
	if(idx==1){
		if(speed>0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,0);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,1);
			speed=-speed;
		}
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,2000*speed/100);
	}else if(idx==2){
		if(speed>0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
			speed=-speed;
		}
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,2000*speed/100);
	}else if(idx==3){
		if(speed>0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
			speed=-speed;
		}
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,2000*speed/100);
	}else if(idx==4){
		if(speed>0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);
			speed=-speed;
		}
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,2000*speed/100);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM7){
		int16_t curCnt[4],speed[4];
		curCnt[0]=__HAL_TIM_GET_COUNTER(&htim1);
		curCnt[1]=__HAL_TIM_GET_COUNTER(&htim2);
		curCnt[2]=__HAL_TIM_GET_COUNTER(&htim3);
		curCnt[3]=__HAL_TIM_GET_COUNTER(&htim4);
		
		for(uint8_t i=0;i<4;++i){
			speed[i]=curCnt[i]-lastCnt[i];
			//if(i==2) speed[i]=-speed[1];
			lastCnt[i]=curCnt[i];
		}
	
		double tht1=pid_calculate(&speedPid[0], speed[2]);
		double tht2=pid_calculate(&speedPid[1], speed[1]);
		double tht3=pid_calculate(&speedPid[2], -speed[0]);
		double tht4=pid_calculate(&speedPid[3], -speed[3]);
		
		tht1=0;
		tht2=0;
		tht3=0;
		//tht4=0;
		
		if(tht1>70){tht1=70;}
		if(tht1<-70){tht1=-70;}
		setSpeed(1,tht1);
		
		if(tht2>70){tht2=70;}
		if(tht2<-70){tht2=-70;}
		setSpeed(2,-tht2);
		
		if(tht3>70){tht3=70;}
		if(tht3<-70){tht3=-70;}
		setSpeed(4,-tht3);
		
		
		if(tht4>70){tht4=70;}
		if(tht4<-70){tht4=-70;}printf("speed:%d,tht:%f\n", speed[3], tht4);
		//setSpeed(3,-tht4);
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
