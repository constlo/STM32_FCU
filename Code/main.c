/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "NRF24L01.h"
#include "MMA8452Q.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HALF_VALUE 32767
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define PACKET_START 204
#define PACKET_END 51
#define STICK_DEADZONE 512
#define THROTTLE_ADJUST 600
#define TWELVEBITCENTERVALUE 2047
#define HELPINGADJUST 165

#define ROLL_ADJUST		10
#define PITCH_ADJUST	10
#define YAW_ADJUST		10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
uint8_t receivedData[11];
uint16_t stickValues[4];
int throttle;
int32_t roll, pitch, yaw;
uint32_t mixingChannel_1, mixingChannel_2, mixingChannel_3, mixingChannel_4;
char Msg[70];
uint32_t PWM1, PWM2, PWM3, PWM4;
uint8_t x_Raw[2], y_Raw[2], z_Raw[2];
uint16_t x_Orientation, y_Orientation, z_Orientation;

//i2c variables
int16_t xPosition, yPosition, zPosition;
int32_t averaged_x, averaged_y, averaged_z;
uint8_t calibration_flag = 0;
int16_t xPositionDifference, yPositionDifference, zPositionDifference;
//calibration counter. The calibration sequence is executed at start, so there is no need for a timer-based calib.
uint8_t calibrationCounter = 0;
uint8_t initializationResult;

//mathematical variables
int16_t rollingBuffer_x[16];
int16_t rollingBuffer_y[16];
int16_t rollingBuffer_z[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
int16_t abs(int16_t val);
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
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //---------INITIALIZATIONS------------
  HAL_I2C_Init(&hi2c1);
  HAL_SPI_Init(&hspi2);
  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Init(&htim2);
  HAL_TIM_PWM_Init(&htim3);
  HAL_TIM_PWM_Init(&htim4);

  NRF24_Init();
  HAL_Delay(10);

  NRF24_RxMode(RxAddress, 10);
  HAL_Delay(10);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_Delay(30);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(30);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_Delay(30);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_Delay(30);

  initializationResult = MMA_init(SCALE_2G, ODR_12 );
  HAL_Delay(10);

  //we can't start the program if we cannot initialize the accelerometer
  while(initializationResult != 1)
  {
	  initializationResult = MMA_init(SCALE_2G, ODR_12 );
	  HAL_Delay(100);
  }

  //--------Start the PWM. The default value is 0, so the motors won't spin.----------
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//If the accelerometer has data ready, begin calibrating the flight controller.
	//we calibrate for calibrationcounter amount of iterations.
	if(MMA_available())
	  {
		  MMA_read(&xPosition, &yPosition, &zPosition);
		  if(calibrationCounter < 100 && calibration_flag == 0)
		  {
			  //add the current positional values to the average.
			  calibrationCounter++;
			  averaged_x += xPosition;
			  averaged_y += yPosition;
			  averaged_z += zPosition;
		  }
		  else if(calibration_flag == 0)
		  {
			  //Set calibration flag to 1 to stop calibrating, and calculate the average of measured variables.
			  //we use these values when piloting the drone to stabilize the device.
			  calibration_flag = 1;
			  averaged_x /= calibrationCounter;
			  averaged_y /= calibrationCounter;
			  averaged_z /= calibrationCounter;
		  }
	  }

	//if we have the averaged values, we can start receiving data from the transmitter.
	if(calibration_flag == 1)
	{
		//first, calculate the difference of the measured accelerometer data from the averages.
		xPositionDifference = (xPosition - averaged_x) / 8;
		yPositionDifference = (yPosition - averaged_y) / 8;
		zPositionDifference = (zPosition - averaged_z) / 8;
		//check data pipe 2 if there is any data available.
		if(isDataAvailable(2) == 1)
		{
			NRF24_Receive(receivedData);

			if(receivedData[0] == PACKET_START && receivedData[10] == PACKET_END)
			{
				//If we have a valid packet present, combine the received 6 bytes into 3 16-bit values.
				//the actual data size of the values sent is 12 bytes.

				combine_uint8_array_to_uint16_array(receivedData, stickValues);
				//1. adjust the 12-bit ADC values into signed 12-bit integers.
				//The stick values go from -11bit to +11bit. We therefore need to center the received data.
				//cast the roll, pitch and yaw to 32-bit signed integers.

				//The min_throttle value seems to be between 3000 and 4000.
				//1500 to arm, 3580 to spin properly.
				throttle = stickValues[2] / 3 + THROTTLE_ADJUST;
				roll = (int32_t)stickValues[3] - TWELVEBITCENTERVALUE;
				pitch = (int32_t)stickValues[0] - TWELVEBITCENTERVALUE;
				yaw = (int32_t)stickValues[1] - TWELVEBITCENTERVALUE;

				//inverse roll, yaw and pitch
				roll *= -1;
				pitch *= -1;
				yaw *= -1;

				//for stability purposes we don't actually map the controls directly to the motors.
				//the noise coming from the ADC-based sticks is too high to control the drone in a stable manner.
				//instead, outside of throttle we check for a deadzone from which we can control the drone.

				if(roll > STICK_DEADZONE)
				{
					roll = ROLL_ADJUST;
				}
				else if(roll <  -1* STICK_DEADZONE)
				{
					roll = -1 * ROLL_ADJUST;
				}
				else
				{
					roll = 0;
				}

				if(pitch >  STICK_DEADZONE)
				{
					pitch = PITCH_ADJUST;
				}
				else if(pitch < -1* STICK_DEADZONE)
				{
					pitch = -1 * PITCH_ADJUST;
				}
				else
				{
					pitch = 0;
				}

				if(yaw >STICK_DEADZONE)
				{
					yaw = YAW_ADJUST;
				}
				else if(yaw < -1* STICK_DEADZONE)
				{
					yaw = -1 * YAW_ADJUST;
				}
				else
				{
					yaw = 0;
				}

				//if the angle of the drone is too much, limit the motor speeds of that direction.
				/*if(abs(averaged_x - xPosition) < 20)
				{
					if(xPosition > 0)
					{
						roll += 10;
					}else
					{
						roll -= 10;
					}
				}
				if(abs(averaged_y - yPosition) < 20)
				{
					if(yPosition > 0)
					{
						pitch += 10;
					}else
					{
						pitch -= 10;
					}
				}*/

				//we assign the stick values into different mixing channels.
				mixingChannel_1 = (uint32_t)throttle + yaw - pitch - roll;
				mixingChannel_2 = (uint32_t)throttle - yaw + pitch - roll ;
				mixingChannel_3 = (uint32_t)throttle + yaw + pitch + roll;
				mixingChannel_4 = (uint32_t)throttle - yaw - pitch + roll ;

				//finally, we set the according timer CCR registers according to the mixing channel values.
				TIM1->CCR1 = mixingChannel_1;
				TIM2->CCR1 = mixingChannel_2;
				TIM3->CCR1 = mixingChannel_3;
				TIM4->CCR1 = mixingChannel_4;
			}
		}
		else
		{
			//----This else conditional is for safety. If we somehow lose connection to the flight controller,
			//lower the mixing channel values to slightly above arming speed. This will ensure that the drone will
			//fall down slowly and safely.
			if(mixingChannel_1 > 100)
			{
				mixingChannel_1--;
			}
			else if(mixingChannel_2 > 100)
			{
				mixingChannel_2--;
			}
			else if(mixingChannel_3 > 100)
			{
				mixingChannel_3--;
			}
			else if(mixingChannel_4 > 100)
			{
				mixingChannel_4--;
			}
			TIM2->CCR1 = mixingChannel_1;
			TIM2->CCR1 = mixingChannel_2;
			TIM2->CCR1 = mixingChannel_3;
			TIM2->CCR1 = mixingChannel_4;
			HAL_Delay(5);
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 12000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void combine_uint8_array_to_uint16_array(const uint8_t *srcArray, uint16_t *destArray) {
    for (int i = 0; i < 4; i++) {
        // Combine each pair of low and high bytes from the uint8_t array into uint16_t array.
        destArray[i] = ((uint16_t)srcArray[i * 2 + 3] << 8) | srcArray[i * 2 + 2];
    }
}

void HAL_TIM_PWM_PulseFinishedCallback 	( 	TIM_HandleTypeDef *  	htim	)
{
	//I had to implement PWM to basic GPIO pins which did not have PWM functionality in them. Luckily this can be
	//done quite easily with register control.
	/*if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{GPIOC->BRR = GPIO_PIN_0;}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{GPIOC->BRR = GPIO_PIN_1;}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{GPIOC->BRR = GPIO_PIN_2;}
	else
	{GPIOC->BRR = GPIO_PIN_3;}*/

	if(htim == &htim1)
		{GPIOC->BRR = GPIO_PIN_0;}
	else if(htim == &htim2)
		{GPIOC->BRR = GPIO_PIN_1;}
	else if(htim == &htim3)
		{GPIOC->BRR = GPIO_PIN_2;}
	else if(htim == &htim4)
		{GPIOC->BRR = GPIO_PIN_3;}
}

void HAL_TIM_PeriodElapsedCallback 	( 	TIM_HandleTypeDef *  	htim	)
{
	//Pulse start function.
	if(htim == &htim1)
		{GPIOC->BSRR = GPIO_PIN_0;}
	else if(htim == &htim2)
		{GPIOC->BSRR = GPIO_PIN_1;}
	else if(htim == &htim3)
		{GPIOC->BSRR = GPIO_PIN_2;}
	else if(htim == &htim4)
		{GPIOC->BSRR = GPIO_PIN_3;}
}

int16_t abs(int16_t val)
{
	if(val < 0)
	{
		return -1*val;
	}
	else
	{
		return val;
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
