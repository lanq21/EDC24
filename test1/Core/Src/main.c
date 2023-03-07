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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "pid.h"
#include "jy62.h"
#include "zigbee_edc24.h"
#include "map.h"
#include "drive.h"
#include "Dijkstra.h"
#include <math.h>
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
int32_t lastCnt[4], curCnt[4], speed[4];
pid speedPid[6];
float coff = 0.4f;

pid zAnglePid;

extern uint8_t jy62_buff[JY62_BUFFSIZE];
extern struct vec acc, angle, vel;
extern uint32_t height;
extern float g;

enum stateValue
{
	running,
	calibrating,
	stop
};
uint8_t state = stop;

extern uint8_t receive_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void setSpeed(uint8_t idx, double speed);

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
	MX_DMA_Init();
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

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start_IT(&htim7);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, jy62_buff, JY62_BUFFSIZE);

	state = calibrating;
	jy62_init(&huart3);
	// calibrate();
	HAL_Delay(500);
	initAngle();
	HAL_Delay(500);
	state = running;

	pid_init(&speedPid[0], 1.0f, 0.05f, 0.0f);
	pid_init(&speedPid[1], 1.0f, 0.05f, 0.0f);
	pid_init(&speedPid[2], 1.0f, 0.05f, 0.0f);
	pid_init(&speedPid[3], 1.0f, 0.05f, 0.0f);
	// speedPid[0].goal=10;
	// speedPid[1].goal=10;
	// speedPid[2].goal=10;
	// speedPid[3].goal=10;

	pid_init(&zAnglePid, 1.2f, 0.02f, 0.01f);
	zAnglePid.goal = 0;
	// u1_printf("hello");

	zigbee_Init(&huart2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t idx = 0;
	uint8_t built = 0;
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// setSpeed(1,10);
		// setSpeed(2,20);
		// setSpeed(3,30);
		// setSpeed(4,40);
		/*if(idx%200<100){
			setSpeed(3,20);
			setSpeed(1,20);
			setSpeed(2,20);
			setSpeed(4,20);
		}else{
			setSpeed(3,-20);
			setSpeed(1,-20);
			setSpeed(2,-20);
			setSpeed(4,-20);
		}
		idx++;*/
		if (receive_flag)
		{
			// u1_printf("6\n");
			// u1_printf("hello111");
			reqGameInfo();
			// u1_printf("hello222");
			zigbeeMessageRecord();
			// u1_printf("hello333");
			if (!built)
			{
				Barrier_edc24 b = getOneBarrier(0);
				if (b.pos_1.x != 0 && b.pos_1.y != 0 && b.pos_2.x != 0 && b.pos_2.y != 0)
				{
					built = 1;
					BuildMap();
					
					Set_Charge_Pile();
					// 放充电桩，应该还有 Bug

					// Position_edc24 tmppos=getVehiclePos();

					// uint16_t curNode=Get_Nearby_Node(tmppos.x, tmppos.y);
				}
			}
		}
		else
		{
		}
		if (built)
		{
			// Drive();
			Go_to(0, 0);

			// u1_printf("car:%d, %d\n", tmppos.x, tmppos.y);
			// uint16_t curNode=Get_Nearby_Node(tmppos.x, tmppos.y);
		}
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void setSpeed(uint8_t idx, double speed)
{
	if (idx == 1)
	{
		if (speed > 0)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
			speed = -speed;
		}
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 2000 * speed / 100);
	}
	else if (idx == 2)
	{
		if (speed > 0)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
			speed = -speed;
		}
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 2000 * speed / 100);
	}
	else if (idx == 3)
	{
		if (speed > 0)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
			speed = -speed;
		}
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 2000 * speed / 100);
	}
	else if (idx == 4)
	{
		if (speed > 0)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
			speed = -speed;
		}
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 2000 * speed / 100);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7)
	{
		//float vy = drive_velocity_goal;
		float vy=30;
		// zAnglePid.goal = drive_angle_goal;
		zAnglePid.goal = 0 ;
		if (zAnglePid.goal < -180.0f)
			zAnglePid.goal = 180;
		if (zAnglePid.goal > 180.0f)
			zAnglePid.goal = -180;

		float zAngleErr = angle.z - zAnglePid.goal;
		if (zAngleErr > 180.0f)
			zAngleErr -= 360.0f;
		if (zAngleErr < -180.0f)
			zAngleErr += 360.0f;

		double zAngleTht=pid_calculate(&zAnglePid,zAnglePid.goal+zAngleErr);
		if (zAngleTht > 60)
			zAngleTht = 60.0f;
		if (zAngleTht < -60)
			zAngleTht = -60.0f;

		speedPid[2].goal = vy + zAngleTht;
		speedPid[3].goal = vy + zAngleTht;
		speedPid[0].goal = vy - zAngleTht;
		speedPid[1].goal = vy - zAngleTht;

		int32_t tmp;
		tmp = __HAL_TIM_GET_COUNTER(&htim1);
		if (tmp >= 32768)
			tmp -= 65536;
		speed[0] = coff * (float)tmp + (1 - coff) * speed[0];

		tmp = __HAL_TIM_GET_COUNTER(&htim2);
		if (tmp >= 32768)
			tmp -= 65536;
		speed[1] = coff * (float)tmp + (1 - coff) * speed[1];

		tmp = __HAL_TIM_GET_COUNTER(&htim3);
		if (tmp >= 32768)
			tmp -= 65536;
		speed[2] = coff * (float)tmp + (1 - coff) * speed[2];

		tmp = __HAL_TIM_GET_COUNTER(&htim4);
		if (tmp >= 32768)
			tmp -= 65536;
		speed[3] = coff * (float)tmp + (1 - coff) * speed[3];

		for (uint8_t i = 0; i < 4; ++i)
		{
			if (speed[i] >= 32768)
				speed[i] -= 65536;
		}

		__HAL_TIM_SetCounter(&htim1, 0);
		__HAL_TIM_SetCounter(&htim2, 0);
		__HAL_TIM_SetCounter(&htim3, 0);
		__HAL_TIM_SetCounter(&htim4, 0);
		
		/*
		for(uint8_t i=0;i<4;++i){
			speed[i]=curCnt[i]-lastCnt[i];
			//if(i==2) speed[i]=-speed[1];
			lastCnt[i]=curCnt[i];
		}*/

		double tht1 = pid_calculate(&speedPid[0], -(float)speed[2]);
		double tht2 = pid_calculate(&speedPid[1], (float)speed[1]);
		double tht3 = pid_calculate(&speedPid[2], (float)speed[3]);
		double tht4 = pid_calculate(&speedPid[3], -(float)speed[0]);

		if (tht1 > 70)
			tht1 = 70;
		if (tht1 < -70)
			tht1 = -70;
		setSpeed(1, -tht1);

		if (tht2 > 70)
			tht2 = 70;
		if (tht2 < -70)
			tht2 = -70;
		setSpeed(2, -tht2);

		if (tht3 > 70)
			tht3 = 70;
		if (tht3 < -70)
			tht3 = -70;
		setSpeed(3, -tht3);

		if (tht4 > 70)
			tht4 = 70;
		if (tht4 < -70)
			tht4 = -70;
		setSpeed(4, -tht4);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	{
		if (huart->Instance == USART2)
		{
			// from zigbee_edc24
			receive_flag = 1;
		}
		else if (huart->Instance == USART3)
		{
			// from jy62

			// printf("%d\n", Size);
			// HAL_UART_Transmit(&huart1,buff,Size,0xFFFFFFFFU);
			// float pit;

			int idx = 0;
			int i = 0;
			while (idx < Size)
			{
				while (jy62_buff[idx] != 0x55)
					++idx;
				uint8_t sum = 0;
				for (i = 0; i < 10; ++i)
					sum += jy62_buff[idx + i];
				if (sum == jy62_buff[idx + 10])
				{
					if (jy62_buff[idx + 1] == 0x51)
					{
						int16_t tmp1, tmp2;
						tmp1 = jy62_buff[idx + 3], tmp2 = jy62_buff[idx + 2];
						acc.x = (float)((tmp1 << 8) + tmp2) * 16.0 * g / 32768.0;
						tmp1 = jy62_buff[idx + 5], tmp2 = jy62_buff[idx + 4];
						acc.y = (float)((tmp1 << 8) + tmp2) * 16.0 * g / 32768.0;
						tmp1 = jy62_buff[idx + 7], tmp2 = jy62_buff[idx + 6];
						acc.z = (float)((tmp1 << 8) + tmp2) * 16.0 * g / 32768.0;

						if (acc.x >= 16 * g)
							acc.x -= 32 * g;
						if (acc.y >= 16 * g)
							acc.y -= 32 * g;
						if (acc.z >= 16 * g)
							acc.z -= 32 * g;
					}
					else if (jy62_buff[idx + 1] == 0x52)
					{
						vel.x = (float)(((short)jy62_buff[idx + 3] << 8) + (short)jy62_buff[idx + 2]) * 2000.0 / 32768.0;
						vel.y = (float)(((short)jy62_buff[idx + 5] << 8) + (short)jy62_buff[idx + 4]) * 2000.0 / 32768.0;
						vel.z = (float)(((short)jy62_buff[idx + 7] << 8) + (short)jy62_buff[idx + 6]) * 2000.0 / 32768.0;
						if (vel.z > 2000)
							vel.z -= 4000;
					}
					else if (jy62_buff[idx + 1] == 0x53)
					{
						angle.x = (float)(((short)jy62_buff[idx + 3] << 8) + (short)jy62_buff[idx + 2]) * 180.0 / 32768.0;
						angle.y = (float)(((short)jy62_buff[idx + 5] << 8) + (short)jy62_buff[idx + 4]) * 180.0 / 32768.0;
						angle.z = (float)(((short)jy62_buff[idx + 7] << 8) + (short)jy62_buff[idx + 6]) * 180.0 / 32768.0;

						angle.x = angle.x > 180.0 ? angle.x - 360.0 : angle.x;
						angle.y = angle.y > 180.0 ? angle.y - 360.0 : angle.y;
						angle.z = angle.z > 180.0 ? angle.z - 360.0 : angle.z;
					}
					else if (jy62_buff[idx + 1] == 0x56)
					{
						height = ((uint32_t)jy62_buff[idx + 9] << 24) + ((uint32_t)jy62_buff[idx + 8] << 16) + ((uint32_t)jy62_buff[idx + 7] << 8) + (uint32_t)jy62_buff[idx + 6];
					}
					idx += 11;
				}
				else
				{
					idx++;
				}
				HAL_UARTEx_ReceiveToIdle_DMA(&huart3, jy62_buff, 200);
			}
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
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
