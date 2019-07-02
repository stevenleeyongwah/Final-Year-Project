/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdbool.h> 
#include "NRF24L01.h"
#include "MPU6050.h"
#include "HCSR04.h"
#include "PID_Motor_Control.h"
#include "Robot_Motion_Control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t received_byte, received_com_byte, low_bat, activation, UART_rx;
uint8_t balancing=false,standing=false,sleeping=false;

uint32_t micros = 0, loop_timer = 0, rising_tick_1, rising_tick_3, echo1_pulse_width, echo3_pulse_width;
volatile uint32_t millis = 0;

int16_t throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int16_t throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int16_t battery_voltage, receive_counter;

float pid_i_mem, pid_output, self_balance_pid_setpoint, adcValue, angle_gyro;
float sleep=0;
char webpage[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t micro(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1)
		{
				if(UART_rx == 0x06)
				{
					HAL_UART_Transmit(&huart1,webpage,strlen(webpage),400);
				}
				if(standing == false && sleeping == false)
			  {
					if(UART_rx == 0x01 || UART_rx == 0x02 || UART_rx == 0x04 || UART_rx == 0x08 || UART_rx == 0x05)
					{
						received_com_byte = UART_rx;
					}
				}
				
				UART_rx = 0;
				HAL_UART_Receive_IT(&huart1, &UART_rx, 1);
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 if (htim ->Instance == TIM4)
	 {
			HAL_GPIO_TogglePin(GPIOC, LED_Pin);
			//Left motor pulse calculations
			throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
			if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
				throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
				throttle_left_motor_memory = throttle_left_motor; 
				//Load the next throttle_left_motor variable
				if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
					HAL_GPIO_WritePin(GPIOC, Left_DIR_Pin, GPIO_PIN_RESET);               //Set output 3 low to reverse the direction of the stepper controller
					throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
				}
				else HAL_GPIO_WritePin(GPIOC, Left_DIR_Pin, GPIO_PIN_SET);              //Set output 3 high for a forward direction of the stepper motor
			}
			else if(throttle_counter_left_motor == 1)HAL_GPIO_WritePin(GPIOC, Left_STEP_Pin, GPIO_PIN_SET);             //Set output 2 high to create a pulse for the stepper controller
			else if(throttle_counter_left_motor == 2)HAL_GPIO_WritePin(GPIOC, Left_STEP_Pin, GPIO_PIN_RESET);           //Set output 2 low because the pulse only has to last for 20us 
			
			//right motor pulse calculations
			throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
			if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
				throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
				throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
				
				if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
					HAL_GPIO_WritePin(GPIOB, Right_DIR_Pin, GPIO_PIN_RESET);              //Set output 5 low to reverse the direction of the stepper controller
					throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
				}
				else HAL_GPIO_WritePin(GPIOB, Right_DIR_Pin, GPIO_PIN_SET);           	//Set output 5 high for a forward direction of the stepper motor
			}
			else if(throttle_counter_right_motor == 1)HAL_GPIO_WritePin(GPIOB, Right_STEP_Pin, GPIO_PIN_SET);            //Set output 4 high to create a pulse for the stepper controller
			else if(throttle_counter_right_motor == 2)HAL_GPIO_WritePin(GPIOB, Right_STEP_Pin, GPIO_PIN_RESET);          //Set output 4 low because the pulse only has to last for 20us
	 }
}
/* USER CODE END 0 */

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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
	
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1); // Start timer 1 (Input capture for ultrasonic sensor)
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);	 // Start timer 2 (PWM for servo motor 1)
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);	 // Start timer 2 (PWM for servo motor 2)
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1); // Start timer 3 (Input capture for ultrasonic sensor)
	HAL_TIM_Base_Start_IT(&htim4);						 // Start timer 4 (Control stepper motors every 20us)
	HAL_UART_Receive_IT(&huart1, &UART_rx, 1); // Start UART (Communicate with ESP8266)

	htim2.Instance->CCR1 = 25;								 // Turn servo motor 1 to default position
	htim2.Instance->CCR2 = 25;								 // Turn servo motor 2 to default position
	HAL_Delay(200);														 // Wait some time for servo motor to return to default position									
	htim2.Instance->CCR1 = 0;									 // Stop holding servo motor 1 
	htim2.Instance->CCR2 = 0;									 // Stop holding servo motor 2
	
	NRF24_init();															 // Setup NRF24L01 configuration register
	
	MPU6050_config();													 // Setup MPU6050 configuration registers
	MPU6050_pitch_calibration();							 // Calirate MPU6050 pitch axis
	MPU6050_detect_initial_angle();						 // Detect initial angle of MPU6050 
	
	millis = 0;
	loop_timer = micro();											 // Start loop time. Loop time = 4ms								 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
			// Copy robot status (balancing, standing, sleeping) into webpage char array in order to send to ESP8266
			sprintf(webpage, "{ \"balancing\" : %d, \"standing\" : %d, \"sleeping\" : %d }\n", balancing, standing, sleeping);
			
			// Check received byte from remote control and computer to determine whether robot should be raised up or lowered down
			Raise_Or_Lower_Down_Robot(); 							
			
			// Calculate angle of MPU6050
			MPU6050_calculate_angle_gyro();
			
			// Raises up or lowers down the robot according to received byte
			Change_Robot_Position();
			
			// If robot enters vertical (upright) position, enable balancing mode
			Check_If_Robot_Enters_Vertical_Position();	
			
			// Rotate servo arm to vertical position once robot enters vertical position
			Rotate_Servo_Arm_To_Vertical_Position();
			
			// If robot tips over, deactivate balancing mode and reset parameters
			Check_If_Robot_Tips_Over();
			
			// If balancing mode is activated, activate PID controller calculations, control movement of robot, calculate pulses to control stepper motor
			Check_If_Balancing_Mode_Activated();
			
			received_com_byte = 0;							 // Reset received byte from computer every loop
			
			while((micro()- loop_timer) < 4000); // Make sure the loop is in 250Hz or 4ms.
			millis = 0; 												 // Reset millis
			loop_timer = micro();								 // Restart loop timer
		}
			
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65530;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1440-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65530;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 720;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|Left_DIR_Pin|Left_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CSN_Pin|Servo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Right_STEP_Pin|Right_DIR_Pin|TRIGGER1_Pin|TRIGGER3_Pin 
                          |motor_left_Pin|motor_right_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin Left_DIR_Pin Left_STEP_Pin */
  GPIO_InitStruct.Pin = LED_Pin|Left_DIR_Pin|Left_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin Servo_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin|Servo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_STEP_Pin Right_DIR_Pin TRIGGER1_Pin TRIGGER3_Pin 
                           motor_left_Pin motor_right_Pin */
  GPIO_InitStruct.Pin = Right_STEP_Pin|Right_DIR_Pin|TRIGGER1_Pin|TRIGGER3_Pin 
                          |motor_left_Pin|motor_right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
	if(htim ->Instance == TIM1)
	{
		TIM_IC_InitTypeDef sConfigIC;
		
		if(HAL_GPIO_ReadPin(GPIOA,ECHO1_Pin) == 1)
		{
				__HAL_TIM_SET_COUNTER(htim,0);
				rising_tick_1 = __HAL_TIM_GET_COUNTER(htim);
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);
				HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
		}
		else if(HAL_GPIO_ReadPin(GPIOA,ECHO1_Pin) == 0)
		{
				echo1_pulse_width = __HAL_TIM_GET_COUNTER(htim) - rising_tick_1;

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);
				HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
				
				__HAL_TIM_SET_COUNTER(htim,0);
		}
	}
	else if(htim ->Instance == TIM3)
	{
		TIM_IC_InitTypeDef sConfigIC;
		
		if(HAL_GPIO_ReadPin(GPIOB,ECHO3_Pin) == 1)
		{
				__HAL_TIM_SET_COUNTER(htim,0);
				rising_tick_3 = __HAL_TIM_GET_COUNTER(htim);
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
				HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
		}
		else if(HAL_GPIO_ReadPin(GPIOB,ECHO3_Pin) == 0)
		{
				echo3_pulse_width = __HAL_TIM_GET_COUNTER(htim) - rising_tick_3;

				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
				HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
				
				__HAL_TIM_SET_COUNTER(htim,0);
		}
	}
}

uint32_t micro()
{
	micros =  millis*1000 + 1000 - SysTick->VAL/72;
	return micros;
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	millis++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	
  /* USER CODE END SysTick_IRQn 1 */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
