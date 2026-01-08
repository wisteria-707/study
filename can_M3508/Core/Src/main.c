/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "sbus.h"
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
 #define MAX_OUT 3000 
 // 最大输出限制
#define MAX_IOUT 1000 
//最大积分输出限制

#define MULT 1.0f

uint32_t Fifo[SBUS_RECV_MAX]={0};
//缓存数组



float mult=MULT;
//定义增益系数

int set_current[4]={0};
//当前电流值



int set_speed[4]={0};
//设置目标速度

int current_speed[4]={0};

CAN_RxHeaderTypeDef rx_header;
//CAN帧头结构体，作为getmessage函数的参数
	
const motor_measure_t *motor_data[4];
//定义一个指针，用来获取现在的电机的一些数值

pid_type_def motor_pid[4];
//储存配置的参数和数据，作为参数传入到初始化函数中，表示执行器是电机

 fp32 pid_params[3] = {8.0f, 0.0f, 0.0f}; 
 // Kp, Ki, Kd


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{ 
            memcpy(sbus_data2,sbus_data1, SBUS_RECV_MAX);
            SBUS_Parse_Data(sbus_data2); 
            HAL_UART_Receive_DMA(&huart3, sbus_data2, SBUS_RECV_MAX);
					
  }
}


int map(int x, float in_min)
{
    return (x - 992) * in_min;
	//in_min是增益系数
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	can_filter_init();
	//初始化CAN的过滤器
	


for(int i=0;i<4;i++)
{
	motor_data[i]=get_chassis_motor_measure_point(i);
	
	PID_init(&motor_pid[i], PID_POSITION, pid_params, MAX_OUT, MAX_IOUT);
	//初始化pid(获取数据，设置位模式为PID模式，KP，KI，KD三个参数，传入最大值）
}
	


	
HAL_UART_Receive_DMA(&huart3,sbus_data1,SBUS_RECV_MAX);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
		
		if(g_sbus_channels[2]>992 && g_sbus_channels[2]<1800)
    {
      set_speed[0]=map(g_sbus_channels[2],mult);
      set_speed[1]=map(g_sbus_channels[2],mult);
      set_speed[2]=-map(g_sbus_channels[2],mult);
      set_speed[3]=-map(g_sbus_channels[2],mult);
    }
    else if(g_sbus_channels[2]<992 && g_sbus_channels[2] > 100)
    {
      set_speed[0]=map(g_sbus_channels[2],mult);
      set_speed[1]=map(g_sbus_channels[2],mult);
      set_speed[2]=-map(g_sbus_channels[2],mult);
      set_speed[3]=-map(g_sbus_channels[2],mult);
    }
    
    // 读取现在的速度，速度定义在can_receive.h中
	for(int i=0;i<4;i++)
		{
     current_speed[i]= (fp32)motor_data[i]->speed_rpm;
    set_current[i] = (int)PID_calc(&motor_pid[i],current_speed[i], set_speed[i]);
		}
    
    
    // 发送数据到相应电机
    CAN_cmd_chassis(set_current[0] ,set_current[1], set_current[2], set_current[3]);
    
		HAL_Delay(10);
    
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
  RCC_OscInitStruct.PLL.PLLM = 6;
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
