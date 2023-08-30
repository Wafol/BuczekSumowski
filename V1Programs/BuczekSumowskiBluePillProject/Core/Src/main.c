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

#include <stdbool.h>
#include "vl53l0x_api.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum Rotation {
  CLOCKWISE,
  COUNTER_CLOCKWISE
};

enum Side {
	RIGHT,
	LEFT,
	STRAIGHT,
	BACK
};

enum IRSensorPosition {
	FRONT_RIGHT,
	FRONT_LEFT,
	BACK_MIDDLE
};

enum LaserSensorPosition {
	RIGHT_SIDE,
	RIGHT_FRONT,
	LEFT_SIDE,
	LEFT_FRONT
};

enum ActionState {
	FIND,
	ATTACK
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AIN1_PORT GPIOA
#define AIN1_PIN GPIO_PIN_8
#define AIN2_PORT GPIOB
#define AIN2_PIN GPIO_PIN_15

#define BIN1_PORT GPIOB
#define BIN1_PIN GPIO_PIN_13
#define BIN2_PORT GPIOB
#define BIN2_PIN GPIO_PIN_14

#define XSHUT_PORT GPIOB
GPIO_TypeDef *xshut_ports[] = {GPIOB, GPIOA, GPIOA, GPIOA};
uint16_t xshut_pins[] = {GPIO_PIN_7, GPIO_PIN_7, GPIO_PIN_5, GPIO_PIN_6};
//#define XSHUT_RIGHT_SIDE_PIN GPIO_PIN_4
//#define XSHUT_RIGHT_FRONT_PIN GPIO_PIN_5
//#define XSHUT_LEFT_SIDE_PIN GPIO_PIN_6
//#define XSHUT_LEFT_FRONT_PIN GPIO_PIN_7

#define IR_FRONT_RIGHT_PORT GPIOA
#define IR_FRONT_RIGHT_PIN GPIO_PIN_11
#define IR_FRONT_LEFT_PORT GPIOA
#define IR_FRONT_LEFT_PIN GPIO_PIN_12
#define IR_BACK_MIDDLE_PORT GPIOA
#define IR_BACK_MIDDLE_PIN GPIO_PIN_15



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void initLasers();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t vl53l0xs[4];
VL53L0X_DEV laser_devs[] = {&vl53l0xs[0], &vl53l0xs[1], &vl53l0xs[2], &vl53l0xs[3]};


enum ActionState action_state;

uint16_t laser_values[4];
bool ir_values[3];

uint16_t readLaserSensor(enum LaserSensorPosition laser_pos) {
	//VL53L0X_RangingMeasurementData_t RangingData;
	VL53L0X_PerformSingleRangingMeasurement(laser_devs[laser_pos], &RangingData);

	//while (RangingData.RangeStatus != 0)
	//	VL53L0X_PerformSingleRangingMeasurement(laser_devs[laser_pos], &RangingData);

	return RangingData.RangeMilliMeter;
}
void readFromAllLasers() {
	laser_values[RIGHT_SIDE] = readLaserSensor(RIGHT_SIDE);
	laser_values[RIGHT_FRONT] = readLaserSensor(RIGHT_FRONT);
	laser_values[LEFT_SIDE] = readLaserSensor(LEFT_SIDE);
	laser_values[LEFT_FRONT] = readLaserSensor(LEFT_FRONT);
}

bool readIRSensor(enum IRSensorPosition ir_sensor_pos) {
	switch (ir_sensor_pos) {
		case FRONT_RIGHT:
			return HAL_GPIO_ReadPin(IR_FRONT_RIGHT_PORT, IR_FRONT_RIGHT_PIN);
		case FRONT_LEFT:
			return HAL_GPIO_ReadPin(IR_FRONT_LEFT_PORT, IR_FRONT_LEFT_PIN);
		case BACK_MIDDLE:
			return HAL_GPIO_ReadPin(IR_BACK_MIDDLE_PORT, IR_BACK_MIDDLE_PIN);
	}

	return false;
}
void readFromAllIRs() {
	ir_values[FRONT_RIGHT] = readIRSensor(FRONT_RIGHT);
	ir_values[FRONT_LEFT] = readIRSensor(FRONT_LEFT);
	ir_values[BACK_MIDDLE] = readIRSensor(BACK_MIDDLE);
}

//speed = 0 to 255
void runMotor(enum Side motor_side, enum Rotation rot, uint8_t speed) {
  GPIO_TypeDef *controll_port1;
  int controll_pin1;
  GPIO_TypeDef *controll_port2;
  int controll_pin2;

  volatile uint32_t *pwm_channel;

  if (motor_side == RIGHT) {
	controll_port1 = BIN1_PORT;
	controll_pin1 = BIN1_PIN;
	controll_port2 = BIN2_PORT;
	controll_pin2 = BIN2_PIN;

	pwm_channel = &TIM1->CCR3;
  } else if (motor_side == LEFT) {
	controll_port1 = AIN2_PORT;
	controll_pin1 = AIN2_PIN;
	controll_port2 = AIN1_PORT;
	controll_pin2 = AIN1_PIN;

	pwm_channel = &TIM1->CCR2;
  }

  int value;

  if (rot == CLOCKWISE) {
    value = GPIO_PIN_RESET;
  } else if (rot == COUNTER_CLOCKWISE) {
    value = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(controll_port1, controll_pin1, value);
  HAL_GPIO_WritePin(controll_port2, controll_pin2, !value);

  //max pwm value: 65535
  int converted_speed = (((float) speed)*(65535/255));
  *pwm_channel = converted_speed;

}
void goDirection(enum Side side, uint8_t speed) {
  switch (side) {
    case RIGHT:
      runMotor(RIGHT, COUNTER_CLOCKWISE, speed);
      runMotor(LEFT, COUNTER_CLOCKWISE, speed);
      break;
    case LEFT:
      runMotor(RIGHT, CLOCKWISE, speed);
      runMotor(LEFT, CLOCKWISE, speed);
      break;
    case STRAIGHT:
      runMotor(RIGHT, CLOCKWISE, speed);
      runMotor(LEFT, COUNTER_CLOCKWISE, speed);
      break;
    case BACK:
      runMotor(RIGHT, COUNTER_CLOCKWISE, speed);
      runMotor(LEFT, CLOCKWISE, speed);
      break;
  }
}


uint16_t value1;
uint16_t value2;
uint16_t value3;


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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  	initLasers();


	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
		//HAL_GPIO_WritePin(xshut_ports[LEFT_SIDE], xshut_pins[LEFT_SIDE], GPIO_PIN_RESET);
	  //value1 = readLaserSensor(LEFT_FRONT);
	  //HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_RIGHT_SIDE_PIN, GPIO_PIN_RESET);
	  //HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_RIGHT_FRONT_PIN, GPIO_PIN_RESET);

	  //HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_LEFT_SIDE_PIN, GPIO_PIN_RESET);
	  //HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_LEFT_FRONT_PIN, GPIO_PIN_RESET);



	//VL53L0X_PerformSingleRangingMeasurement(laser_devs[RIGHT_SIDE], &RangingData);

	//while (RangingData.RangeStatus != 0)
		//VL53L0X_PerformSingleRangingMeasurement(laser_devs[LEFT_FRONT], &RangingData);
	//value1 = readLaserSensor(LEFT_SIDE);
	//value1 = RangingData.RangeMilliMeter;

	  //readFromAllLasers();
	//VL53L0X_PerformSingleRangingMeasurement(laser_devs[RIGHT_FRONT], &RangingData);

		//while (RangingData.RangeStatus != 0)
		//	VL53L0X_PerformSingleRangingMeasurement(laser_devs[LEFT_SIDE], &RangingData);

		//value2 = RangingData.RangeMilliMeter;
	  //value = readLaserSensor(LEFT_FRONT);
	  readFromAllLasers();
	  //readFromAllIRs();

		//VL53L0X_PerformSingleRangingMeasurement(laser_devs[LEFT_FRONT], &RangingData);

		//value3 = RangingData.RangeMilliMeter;
	  /*value = readLaserSensor(RIGHT_SIDE);

	  if (action_state == FIND) {

	  } else if (action_state == ATTACK) {

	  }*/
    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 PB4
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

void initLasers() {
	HAL_GPIO_WritePin(xshut_ports[RIGHT_SIDE], xshut_pins[RIGHT_SIDE], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(xshut_ports[RIGHT_FRONT], xshut_pins[RIGHT_FRONT], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(xshut_ports[LEFT_SIDE], xshut_pins[LEFT_SIDE], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(xshut_ports[LEFT_FRONT], xshut_pins[LEFT_FRONT], GPIO_PIN_RESET);

	uint8_t new_addresses[] = { 0x62, 0x64, 0x66, 0x68 };

	for (int dev_ind = 0; dev_ind < 4; dev_ind++) { //RIGHT_SIDE, RIGHT_FRONT, LEFT_SIDE, LEFT_FRONT
		laser_devs[dev_ind]->I2cHandle = &hi2c1;
		laser_devs[dev_ind]->I2cDevAddr = 0x52;

		HAL_GPIO_WritePin(xshut_ports[dev_ind], xshut_pins[dev_ind], GPIO_PIN_RESET); // Disable XSHUT
		HAL_Delay(20);
		HAL_GPIO_WritePin(xshut_ports[dev_ind], xshut_pins[dev_ind], GPIO_PIN_SET); // Enable XSHUT
		HAL_Delay(20);

		//HAL_GPIO_WritePin(xshut_ports[LEFT_SIDE], xshut_pins[LEFT_SIDE], GPIO_PIN_RESET);
		//
		// VL53L0X init for Single Measurement
		//

		VL53L0X_WaitDeviceBooted( laser_devs[dev_ind] );
		VL53L0X_DataInit( laser_devs[dev_ind] );
		VL53L0X_StaticInit( laser_devs[dev_ind]);
		VL53L0X_PerformRefCalibration(laser_devs[dev_ind], &VhvSettings, &PhaseCal);
		VL53L0X_PerformRefSpadManagement(laser_devs[dev_ind], &refSpadCount, &isApertureSpads);
		VL53L0X_SetDeviceMode(laser_devs[dev_ind], VL53L0X_DEVICEMODE_SINGLE_RANGING);

		// Enable/Disable Sigma and Signal check
		VL53L0X_SetLimitCheckEnable(laser_devs[dev_ind], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
		VL53L0X_SetLimitCheckEnable(laser_devs[dev_ind], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
		VL53L0X_SetLimitCheckValue(laser_devs[dev_ind], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
		VL53L0X_SetLimitCheckValue(laser_devs[dev_ind], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
		VL53L0X_SetMeasurementTimingBudgetMicroSeconds(laser_devs[dev_ind], 33000);
		VL53L0X_SetVcselPulsePeriod(laser_devs[dev_ind], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
		VL53L0X_SetVcselPulsePeriod(laser_devs[dev_ind], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

		VL53L0X_SetDeviceAddress(laser_devs[dev_ind], new_addresses[dev_ind]);
		laser_devs[dev_ind]->I2cDevAddr = new_addresses[dev_ind];
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
