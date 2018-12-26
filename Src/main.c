
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myTask02uartHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
////	//	下面开始是	与 KYW 模组 的 串口 通讯协议	KYW00x系列模组和终端设备通信接口说明书V2.5（修改稿1806291423改）.docx

void huart1_receive_message_Handle(void);
volatile uint8_t Rxbuff_UART[100];
volatile uint8_t Rxbuff_UART1[100];
volatile uint8_t Rx_count_UART1=0; 
volatile uint8_t aaRxBuffer_UART1[1];
volatile uint8_t Rx_Num_UART1=0;
uint8_t power_on_FLAG=0;

volatile uint8_t RxBuffer_UART[1];
volatile uint8_t Rx_count_UART=0;
volatile uint16_t Rx_Num_UART=0;

//增加串口发送缓存部分
volatile uint8_t uart_send_buffer[500];
//uint8_t uart_send_buffer[800];
volatile uint8_t uart_send_buffer_data_size[5];
//volatile uint8_t uart_send_buffer_data_size[150];
volatile uint8_t uart_send_temp_buffer[100];
uint8_t uart_send_save_to_buffer(uint8_t *pData, uint16_t Size);
volatile uint8_t uart_send_buffer_available_number=5;		//	一开始5个都可用来存放，所以初始值为5
#define buffer_full_error 2
#define Size_error 3
#define save_OK 1
#define other_error 4
void load_from_buffer_and_send_uart_data_once(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02uart(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02uart */
  osThreadDef(myTask02uart, StartTask02uart, osPriorityNormal, 0, 64);
  myTask02uartHandle = osThreadCreate(osThread(myTask02uart), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD1_Pin|LD3_Pin|LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 1 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}
/* USER CODE END 1 */

//	下面开始是	与 KYW 模组 的 串口 通讯协议	KYW00x系列模组和终端设备通信接口说明书V2.5（修改稿1806291423改）.docx

//	现在换新架构
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
			Rxbuff_UART1[Rx_count_UART1] = aaRxBuffer_UART1[0];
			Rx_count_UART1++;
			//	增加一个判断条件：如果串口收到5个字节还没有检测出头码 AA，就将	Rx_count_UART1归零			这个待测试验证后用
//		UART_IT_IDLE
//			if((Rx_count_UART1==5)&&(Rxbuff_UART1[0]!=0xAA))
//			{
//				Rx_count_UART1=0;
//			}
			if((Rxbuff_UART1[0]==0xAA)&&(Rxbuff_UART1[1]==0x5A)&&(Rx_count_UART1==(12+Rxbuff_UART1[11])))
			{
				for(uint8_t i=0;i<Rx_count_UART1;i++)
				{
					Rxbuff_UART[i]=Rxbuff_UART1[i];			
				}
				huart1_receive_message_Handle();
				Rx_count_UART1=0;
			}
			HAL_UART_Receive_IT(&huart1,(uint8_t*)aaRxBuffer_UART1,1); 
	}
}

void huart1_receive_message_Handle(void)
{
	
	if((uart_send_buffer_available_number!=0)&&(uart_send_buffer_available_number<=5))
	{
		uart_send_save_to_buffer((uint8_t *)&Rxbuff_UART,Rx_count_UART1);
	}
}

uint8_t uart_send_save_to_buffer(uint8_t *pData, uint16_t Size)								//	改为递归或者循环方式，否则无法从5个扩展到100个
{
	//先判断发送缓存中的5个是否都已经用完，如果现在容量=5，则无法放入，返回出错值
//	如果5个都是空的，直接放[400]-[499]中
//	如果
	
	if(uart_send_buffer_available_number==0)
	{
		return buffer_full_error;
	}
	if(Size>100)
	{
		return Size_error;
	}
	if((uart_send_buffer_available_number>=1)&&(uart_send_buffer_available_number<=5))
	{

		for(uint8_t i=0;i<Size;i++)
		{
			uart_send_buffer[(i+(uart_send_buffer_available_number-1)*100)]=pData[i];
		}
		uart_send_buffer_available_number=(uart_send_buffer_available_number-1);
		uart_send_buffer_data_size[uart_send_buffer_available_number]=Size;		

		return save_OK;
		
	}
	else
	{
		return other_error;
	}
}

void load_from_buffer_and_send_uart_data_once(void)			//	改为递归或者循环方式，否则无法从5个扩展到100个
{
	uint8_t buffer_member_number=5;

	for(uint8_t i=0;i<100;i++)
	{
		uart_send_temp_buffer[i]=uart_send_buffer[(buffer_member_number-1)*100+i];
	}
	if(uart_send_buffer_available_number<=(buffer_member_number-1))
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&uart_send_temp_buffer,uart_send_buffer_data_size[(buffer_member_number-1)]);
	}
	
	//	发送后需要整理队列中的位置
			
	for(uint8_t j=0;j<(buffer_member_number-uart_send_buffer_available_number-1);j++)
	{
		for(uint8_t i=0;i<100;i++)
		{
			uart_send_buffer[i+(buffer_member_number-1-j)*100]=uart_send_buffer[i+(buffer_member_number-1-j-1)*100];
		}
		uart_send_buffer_data_size[buffer_member_number-1-j]=uart_send_buffer_data_size[buffer_member_number-1-j-1];				
	}

//	}
	//	取走1个，需要清空前面的
	for(uint8_t i=0;i<100;i++)
	{
		uart_send_buffer[i+uart_send_buffer_available_number*100]=0;
	}
	uart_send_buffer_data_size[uart_send_buffer_available_number]=0;
	
	//	可用队列数量更新
	uart_send_buffer_available_number=uart_send_buffer_available_number+1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	HAL_UART_Receive_IT(&huart1,(uint8_t*)aaRxBuffer_UART1,1);
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
		HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
		HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
		HAL_GPIO_TogglePin(LD4_GPIO_Port,LD4_Pin);
		HAL_GPIO_TogglePin(LD5_GPIO_Port,LD5_Pin);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02uart */
/**
* @brief Function implementing the myTask02uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02uart */
void StartTask02uart(void const * argument)
{
  /* USER CODE BEGIN StartTask02uart */
  /* Infinite loop */
  for(;;)
  {
//    osDelay(1);
    osDelay(600);

		if(uart_send_buffer_available_number!=5)
		{
			load_from_buffer_and_send_uart_data_once();
		}
  }
  /* USER CODE END StartTask02uart */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
