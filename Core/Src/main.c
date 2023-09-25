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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	double limMin; // Minimum limit
	double limMax; // Maximum limit
	double T; // Sample time
	double proportional; // Proportional
	double integral; // Integral
	double prevError; // Previous error signal
	double out; // PWM out signal
} PIController; // Definition PI structure
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI_KP 0.0597*2 // Proportional constant
#define PI_KI 0.0896*2 // Integral constant
#define KPd 0.125*4 // Desired linear velocity constant
#define KPw 0.1*1 // Desired angular velocity constant
#define PI_LIM_MIN -500 // PI minimum limit
#define PI_LIM_MAX 500 // PI maximum limit
#define WHEEL_RADIUS 0.13 // Wheel ratio
//0.105
#define GEAR_RATIO 16 // Constant to get speed in RPM
#define WHEELS_DISTANCE 0.74 // Distance between wheels
//0.8128
#define PPR 1000 // Driver pulses/revolution
#define LOOP_RATE 20e-3 // Driver loop rate
#define FREQ_PSCCLOCK 3.2 // Prescaler clock frequency (MHz) (1000 < PWM < 2000) (3200 < CCR < 6400) (ARR = 64000)
#define FREQ_DATABASE 40 // RaspberryPi Tx frequency (/ms)
#define FREQ_MEASURE 50 // Wz frequency (/ms)
#define FREQ_BMS 100 // BMS frequency (/ms)
#define FREQ_CTRL 10 // Control frequency (/ms)
#define TIM6_TO_MILLIS 10 // TIM6 to ms ratio
#define DATABASE_SIZE 26 // Database size
#define TRAJECTORY_RESOLUTION 0.5 // Trajectory resolution (m)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
double speedRRPM; // Right wheel speed (RPM)
double speedLRPM; // Left wheel speed (RPM)
static const uint8_t I2C_SLAVE_ADDRESS_Measure = 64; // Measure MCU I2C address
static const uint8_t I2C_SLAVE_ADDRESS_BMS = 46; // BMS MCU I2C address
double Xref[1038]; // Defined trajectory X array
double Yref[1038]; // Defined trajectory Y array
double database[DATABASE_SIZE]; // Database array
int XYindex = 0; // Defined trajectory index
int k = 0; // Current point of the trajectory
int raspberry_flag = 0; // Raspberry Pi is ready -> Starting defined trajectory reception
int trajectory_flag = 0; // Receiving defined trajectory is done -> Starting while loop
int speed_flag = 1; // Receiving speed is done -> Get speed again
char encoders_string[20]; // String received from encoders
char position_string[100]; // String received from Measure MCU
char LidarControl = 'F'; // Robot Control using Lidar
char Umode = 'X'; // UART Mode (Trajectory X)
char Dmode = 'F'; // Database mode (OFF)
char Bmode = 'F'; // BMS mode (OFF)
char Mmode = 'F'; // Measure mode (OFF)
uint8_t uart2_Tx[50] = "Start\n"; // Tx buffer UART2
uint8_t uart2_Rx[1]; // Rx buffer UART2
uint8_t uart1_Tx[3] = "v\r"; // Tx buffer UART1
uint8_t uart1_Rx[1]; // Rx buffer UART1
uint8_t i2c1_Tx[20] = "Wz:"; // Tx buffer I2C1
uint8_t i2c1_Rx[20]; // Rx buffer I2C1
uint8_t data_buf[500];  // Database Tx buffer
uint16_t Mtimer1, Mtimer2, timer1, timer2, Dtimer1, Dtimer2, Btimer1, Btimer2, Ctimer = 0; // Timers (TIM6)
uint16_t BMSTime = 0; // BMS MCU execution time
uint32_t Beat, Acttimer = 0; // Heartbeat timer (SysTick)
double Voltage = 0; // BMS MCU voltage sensor (V)
double Current = 0; // BMS MCU current sensor (A)
double ControlTime = 0; // Control Time (s)
double Temperature = 0; // BMS MCU temperature sensor (°C)
double EstCharge = 0; // Battery estimated charge
double RealCharge = 0; // Battery real charge
double ExeTime = 0; // Execution time (µs)
double Wcc_z = 0; // Z axis angular speed (rad/s)
double speedL = 0; // Left wheel speed (RPM*GEAR_RATIO)
double speedR = 0; // Right wheel speed (RPM*GEAR_RATIO)
double Theta = 0; // Z axis angle (rad)
double X_dot = 0; // X axis speed (m/s)
double Y_dot = 0; // Y axis speed (m/s)
double X = 0; // X axis position (m)
double Y = 0; // Y axis position (m)
double Xlidar = 0; // X axis position using Lidar (m)
double Ylidar = 0; // Y axis position using Lidar (m)
double dx = 0; // X axis position error (m)
double dy = 0; // Y axis position error (m)
double distance = 0; // Global position error (m)
double ThetaDes = 0; // Z axis desired angle (rad)
double ThetaDes_N = 0; // Z axis desired angle normalized between 0 and 2 Pi (rad)
double Theta_N = 0; // Z axis angle normalized between 0 and 2 Pi (rad)
double Theta_Err = 0; // Z axis angle error (rad)
double W_des = 0; // Z axis desired angular speed (rad/s)
double V_des = 0; // Desired linear speed (m/s)
double speed_refL = 0; // Left wheel desired speed (RPM)
double speed_refR = 0; // Right wheel desired speed (RPM)
double Xrefd = 0; // X axis defined trajectory current position
double Yrefd = 0; // Y axis defined trajectory current position
double error_L = 0; // Left wheel speed error (RPM)
double error_R = 0; // Right wheel speed error (RPM)
double LeftPWM = 1500; // Left PWM signal (RPM) (Neutral value is 1500) (1000 < LeftPWM < 2000)
double RightPWM = 1500; // Right PWM signal (RPM) (Neutral value is 1500) (1000 < RightPWM < 2000)
double LinearActPWM = 1500; // Linear actuators PWM signal (Neutral value is 1500) (1000 < LinearActPWM < 2000)
PIController LeftController = {
    PI_LIM_MIN,
	PI_LIM_MAX,
    0}; // Left PI
PIController RightController = {
    PI_LIM_MIN,
	PI_LIM_MAX,
    0}; // Right PI
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c); // I2C Rx Callback
void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c); // I2C Tx Callback
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart); // UART Rx Callback
void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart); // UART Tx Callback
void PIController_Init(PIController *pi); // PI init
double PIController_Update(PIController *pi, double setpoint, double measurement); // PI update
void GetMeasureInfos(); // Get Z axis angular speed from IMU via Measure MCU
void GetBMS(); // Get BMS MCU info
void GetSpeed(); // Get speed from encoders
void ClearBuffer(uint8_t* buf, int size); // Clear buffer
void SendDatabase(); // Database Tx via RaspberryPi
void Heartbeat(); // LED Heartbeat
void LinAct(); // Activates linear actuators
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
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(5000);
  TIM1->CCR1 = 4800; // Left PWM neutral value
  TIM1->CCR4 = 4800; // Right PWM neutral value
  TIM16->CCR1 = 4800; // Linear actuators PWM neutral value
  HAL_TIM_Base_Start(&htim6); // Start TIM6 related timers
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // Start TIM16 CH1 (Linear actuators))
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Start TIM1 CH4 (Right PWM)
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // Start TIM1 CH1N (Left PWM)
  PIController_Init(&LeftController); // Left PI init
  PIController_Init(&RightController); // Right PI init
  while (raspberry_flag == 0){
	  HAL_UART_Receive(&huart2, uart2_Rx, 1, 10000);
	  if (uart2_Rx[0] == 'R'){
		  raspberry_flag = 1;
		  HAL_UART_Transmit_IT(&huart2, uart2_Tx, 50);
	  }
  } // Raspberry Pi is ready -> Starting defined trajectory reception
  while (trajectory_flag == 0){
  } // Defined trajectory received -> Starting while loop
  timer1 = __HAL_TIM_GET_COUNTER(&htim6);
  Btimer1 = __HAL_TIM_GET_COUNTER(&htim6);
  Mtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
  Dtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
  Ctimer = __HAL_TIM_GET_COUNTER(&htim6);// Start timers
  HAL_Delay(1000); // Waiting for init before starting while loop
  Acttimer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Heartbeat();

	  LinAct();

	  Mtimer2 = __HAL_TIM_GET_COUNTER(&htim6); // Updating Wz timer
	  if (Mtimer2<Mtimer1){
		  Mtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
		  Mtimer2 = __HAL_TIM_GET_COUNTER(&htim6);
	  } // Overflow -> Resetting Wz timer
	  if ((Mtimer2 - Mtimer1 > TIM6_TO_MILLIS*FREQ_MEASURE || Mmode == 'F') && (Bmode == 'F')){
		  GetMeasureInfos();
	  } // Getting Z axis angular speed from IMU via Measure MCU each 50 ms*/

	  Btimer2 = __HAL_TIM_GET_COUNTER(&htim6); // Updating BMS timer
	  if (Btimer2<Btimer1){
		  Btimer1 = __HAL_TIM_GET_COUNTER(&htim6);
		  Btimer2 = __HAL_TIM_GET_COUNTER(&htim6);
	  } // Overflow -> Resetting BMS timer
	  if ((Btimer2 - Btimer1 > TIM6_TO_MILLIS*FREQ_BMS && Bmode == 'F') && (Mmode == 'F')){
		  GetBMS();
	  } // Getting BMS MCU info each 100 ms

	  if (speed_flag == 1){
		  GetSpeed();
	  } // Receiving speed is done -> Get speed again

	  if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - Ctimer) > TIM6_TO_MILLIS*FREQ_CTRL){
		  ControlTime = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - Ctimer)/10000.0; // Control time (s)
		  Theta += Wcc_z*ControlTime*M_PI/180; // Z axis angle (rad)
		  X_dot = (speedR+speedL)*(WHEEL_RADIUS/(2*GEAR_RATIO))*cos(Theta)*((2*M_PI)/60); // X axis speed (m/s)
		  Y_dot = (speedR+speedL)*(WHEEL_RADIUS/(2*GEAR_RATIO))*sin(Theta)*((2*M_PI)/60); // Y axis speed (m/s)
		  X += X_dot*ControlTime; // X axis position (m)
		  Y += Y_dot*ControlTime; // Y axis position (m)
		  if (LidarControl == 'T'){
			  dx = Xref[k] - Xlidar; // X axis position error using Lidar (m)
			  dy = Yref[k] - Ylidar; // Y axis position error using Lidar (m)
		  }
		  else{
			  dx = Xref[k] - X; // X axis position error (m)
			  dy = Yref[k] - Y; // Y axis position error (m)
		  }
		  distance = sqrt(pow(dx,2)+pow(dy,2)); // Global position error (m)
		  ThetaDes = atan2(dy,dx); // Z axis desired angle (rad)
		  ThetaDes_N = fmod((ThetaDes + 2*M_PI), (2*M_PI)); // Z axis desired angle normalization between 0 and 2 Pi (rad)
		  Theta_N = fmod((Theta + 2*M_PI), (2*M_PI)); // Z axis angle normalization between 0 and 2 Pi (rad)
		  if (((ThetaDes_N-Theta_N) < -M_PI ) && (ThetaDes_N < (M_PI/3))){
			  Theta_Err = ThetaDes_N - Theta_N + (2*M_PI);
		  }
		  else if (((ThetaDes_N-Theta_N) > M_PI ) && (ThetaDes_N > (5*M_PI/3)) && (Theta_N < (M_PI/3))){
			  Theta_Err = ThetaDes_N - Theta_N - (2*M_PI);
		  }
		  else{
			  Theta_Err = ThetaDes_N - Theta_N;
		  } // Z axis angle error (rad)
		  W_des = KPw*Theta_Err; // Z axis desired angular speed (rad/s)
		  V_des = KPd*distance*cos(Theta_Err); // Desired linear speed (m/s)
		  speed_refL = (V_des-W_des*WHEELS_DISTANCE)*(GEAR_RATIO/WHEEL_RADIUS)*(60/(2*M_PI)); // Left wheel desired speed (RPM)
		  speed_refR = (V_des+W_des*WHEELS_DISTANCE)*(GEAR_RATIO/WHEEL_RADIUS)*(60/(2*M_PI)); // Right wheel desired speed (RPM)

		  PIController_Update(&LeftController, speed_refL, speedL); // Updating left PI
		  PIController_Update(&RightController, speed_refR, speedR); // Updating right PI

		  LeftPWM = 1500 + LeftController.out; // Updating left PWM
		  RightPWM = 1500 + RightController.out; // Updating right PWM

		  TIM1->CCR1 = LeftPWM*FREQ_PSCCLOCK; // Generating Left PWM signal
		  TIM1->CCR4 = RightPWM*FREQ_PSCCLOCK; // Generating Right PWM signal

		  error_L = LeftController.proportional + LeftController.integral; // Left wheel speed error (RPM)
		  error_R = RightController.proportional + RightController.integral; // Right wheel speed error (RPM)

		  LeftController.T = ControlTime; // Updating left PI sample time
		  RightController.T = ControlTime; // Updating right PI sample time

		  if ((distance < TRAJECTORY_RESOLUTION)){
			  Xrefd = Xref[k]; // Updating X axis trajectory point
			  Yrefd = Yref[k]; // Updating Y axis trajectory point
			  if (k >= 1037){
				  k = 0;
			  } // Defined trajectory ended -> Starting a new one
			  else{
				  k++; // Aiming for the next point
				  }
		  } // Updating trajectory points each
		  Ctimer = __HAL_TIM_GET_COUNTER(&htim6);
	  }

	  timer2 = timer1;
	  timer1 = __HAL_TIM_GET_COUNTER(&htim6); // Updating while loop timers
	  ExeTime = ((uint16_t)((timer1 - timer2)))*100.0; // Execution time (µs)

	  Dtimer2 = __HAL_TIM_GET_COUNTER(&htim6); // Updating database Tx timer
	  if (Dtimer2<Dtimer1){
		  Dtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
		  Dtimer2 = __HAL_TIM_GET_COUNTER(&htim6);
	  } // Overflow -> Resetting database Tx timer
	  if (Dtimer2 - Dtimer1 > TIM6_TO_MILLIS*FREQ_DATABASE){
		  database[0] = Wcc_z;
	  	  database[1] = Theta_N;
	  	  database[2] = X;
	  	  database[3] = Y;
	  	  database[4] = Xrefd;
	 	  database[5] = Yrefd;
	 	  database[6] = speedR;
	 	  database[7] = speedL;
	 	  database[8] = speed_refL;
	  	  database[9] = speed_refR;
	  	  database[10] = error_R;
	  	  database[11] = error_L;
	  	  database[12] = W_des;
	 	  database[13] = V_des;
	 	  database[14] = k;
	 	  database[15] = Theta_Err;
	 	  database[16] = ExeTime;
	  	  database[17] = Voltage;
	  	  database[18] = Current;
	  	  database[19] = Temperature;
	  	  database[20] = EstCharge;
	 	  database[21] = RealCharge;
	 	  database[22] = BMSTime;
	 	  database[23] = ThetaDes_N;
	 	  database[24] =  Xlidar;
	 	  database[25] =  Ylidar;
		  ClearBuffer(data_buf, 500);
		  strcpy((char*)data_buf, "D:");
		  for(int i=0; i<DATABASE_SIZE; i++){
			  sprintf((char*)uart2_Tx, "%f|", database[i]);
			  strcat((char*)data_buf, (char*)uart2_Tx);
			  ClearBuffer(uart2_Tx, 50);
		  }
		  strcat((char*)data_buf, "\n"); // Building database Tx buffer
		  if (huart2.gState == HAL_UART_STATE_READY){
			  SendDatabase();
			  Dtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
			  Dtimer2 = __HAL_TIM_GET_COUNTER(&htim6);
		  } // UART2 is ready -> Sending to RaspberryPi and resetting database Tx timers
	  } // Sending to database via RaspberryPi each 40 ms
  }
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  htim1.Init.Prescaler = 25 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 64000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65536 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 25 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 64000 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c){ // I2C Rx callback
	double Wcc_z_temp;
	double Xtemp;
	double Ytemp;
	if (Mmode == 'T' && i2c1_Rx[0] == 'W'){
		Wcc_z_temp = (double)((i2c1_Rx[1] << 8) | (i2c1_Rx[2] & 0x00FF)); // Z axis angular speed (rad/s)
		Wcc_z_temp /= 32768.0;
		Wcc_z_temp *= 2000.0;
		if (Wcc_z_temp >= 2000){
			Wcc_z = (double)(Wcc_z_temp) - 2*2000.0;
		} // -2000 < Wz < 2000
		else{
			Wcc_z = Wcc_z_temp;
		}
		Xtemp = (double)((i2c1_Rx[4] << 8) | (i2c1_Rx[5] & 0x00FF));
		Xlidar = Xtemp/1000.0;
		Ytemp = (double)((i2c1_Rx[6] << 8) | (i2c1_Rx[7] & 0x00FF));
		Ylidar = Ytemp/1000.0;
		ClearBuffer(i2c1_Rx, 20);
		Mmode = 'F'; // Measure mode OFF
	}
	else if (Bmode == 'T'){
		Current = ((int16_t)(i2c1_Rx[0] << 8) | i2c1_Rx[1])/1000.0;
		Voltage = ((int16_t)(i2c1_Rx[2] << 8) | i2c1_Rx[3])/1000.0;
		Temperature = ((int16_t)(i2c1_Rx[4] << 8) | i2c1_Rx[5])/100.0;
		EstCharge = ((int16_t)(i2c1_Rx[6] << 8) | i2c1_Rx[7])/1000.0;
		RealCharge = ((int16_t)(i2c1_Rx[8] << 8) | i2c1_Rx[9])/1000.0;
		BMSTime = (uint16_t)((i2c1_Rx[10] << 8) | i2c1_Rx[11]);
		Btimer1 = __HAL_TIM_GET_COUNTER(&htim6);
		Btimer2 = __HAL_TIM_GET_COUNTER(&htim6);
		Bmode = 'F'; // BMS mode OFF
	}
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c){ // I2C Tx Callback
	if (Mmode == 'T'){ // I2C Measure MCU mode
		Mtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
		Mtimer2 = __HAL_TIM_GET_COUNTER(&htim6);
		HAL_I2C_Master_Seq_Receive_IT(&hi2c1, I2C_SLAVE_ADDRESS_Measure, i2c1_Rx, sizeof(i2c1_Rx), I2C_FIRST_FRAME); // Transmission OK -> Lancement reception Wz
	}
	else if (Bmode == 'T'){ // I2C BMS MCU mode
		Btimer1 = __HAL_TIM_GET_COUNTER(&htim6);
		Btimer2 = __HAL_TIM_GET_COUNTER(&htim6);
		HAL_I2C_Master_Seq_Receive_IT(&hi2c1, I2C_SLAVE_ADDRESS_BMS, i2c1_Rx, sizeof(i2c1_Rx), I2C_FIRST_FRAME);
	}
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart){ // UART Rx Callback
	uint8_t speed_string_L[50];
	uint8_t speed_string_R[50];
	uint8_t stringL[50];
	uint8_t stringR[50];
	double speedLtemp;
	double speedRtemp;
	char *token;
	double position_number;
	int indexenc = 0;
	if ((huart->Instance == USART2) && (Umode == 'X')){
		if (uart2_Rx[0] == 'X' || uart2_Rx[0] == ':' || uart2_Rx[0] == 'R'){
			ClearBuffer(uart2_Rx, 1);
			HAL_UART_Receive_IT(&huart2, uart2_Rx, 1);
		}
		else if (uart2_Rx[0] == 'Y'){
			position_number = strtod((char*) position_string, NULL);
			ClearBuffer((uint8_t*)position_string, 100);
			Xref[XYindex] = position_number;
			XYindex = 0;
			Umode = 'Y';
			HAL_UART_Receive_IT(&huart2, uart2_Rx, 1);
		}
		else if (uart2_Rx[0] == '|'){
			position_number = strtod((char*)position_string, NULL);
			ClearBuffer((uint8_t*)position_string, 100);
			Xref[XYindex] = position_number;
			XYindex++;
			HAL_UART_Receive_IT(&huart2, uart2_Rx, 1);
		}
		else{
			strcat(position_string, (char*)uart2_Rx);
			HAL_UART_Receive_IT(&huart2, uart2_Rx, 1); // Receive until end of line
		}
	}
	else if ((huart->Instance == USART2) && (Umode == 'Y')){
		if (uart2_Rx[0] == 'Y' || uart2_Rx[0] == ':'){
			HAL_UART_Receive_IT(&huart2, uart2_Rx, 1);
		}
		else if (uart2_Rx[0] == 'F'){
			position_number = strtod((char*) position_string, NULL);
			ClearBuffer((uint8_t*)position_string, 100);
			Yref[XYindex] = position_number;
			Xref[1] = 0.6016000000000001;
			strcpy((char*)uart2_Tx, "BXY\n");
			HAL_UART_Transmit_IT(&huart2, uart2_Tx, sizeof(uart2_Tx)); // Receiving trajectory is done -> Sending BXY
		}
		else if (uart2_Rx[0] == '|'){
			position_number = strtod((char*) position_string, NULL);
			ClearBuffer((uint8_t*)position_string, 100);
			Yref[XYindex] = position_number;
			XYindex++;
			HAL_UART_Receive_IT(&huart2, uart2_Rx, 1); // Receive next number
		}
		else{
			strcat(position_string, (char*) uart2_Rx);
			HAL_UART_Receive_IT(&huart2, uart2_Rx, 1); // Receive until end of number
		}
	}
	else if (huart->Instance == USART1){ // Getting speed
		if (uart1_Rx[0] != '\n') {
			strcat(encoders_string, (char*) uart1_Rx);
			HAL_UART_Receive_IT(&huart1, uart1_Rx, 1); // Receive until end of line
		}
		else {
			speed_flag = 1; // Line is received entirely
		}
		if (speed_flag == 1){
			if ((encoders_string[0] == '?') || (encoders_string[1] == '?')){
				HAL_UART_Transmit_IT(&huart1, uart1_Tx, sizeof(uart1_Tx)-1);
			} // Received data is incorrect
			else{
				token = strtok((char*)encoders_string, "R");
				if (encoders_string[0] == '>'){
					indexenc = 1;
				}
				strncpy((char*)speed_string_L, token + 2 + indexenc, strlen(token));
				strncpy((char*)speed_string_R, (char*)encoders_string + strlen(token) + 2, strlen(token));
				int z = 0;
				for (int i=0; i<sizeof(speed_string_L); i++){
					if (speed_string_L[i] != ' '){
						stringL[z] = speed_string_L[i];
						z++;
					}
				}
				speedLtemp = strtod((char*)stringL, NULL);
				speedL = -speedLtemp*60/(2*PPR*LOOP_RATE);
				speedLRPM = speedR/GEAR_RATIO;
				z = 0;
				for (int i=0; i<sizeof(speed_string_R); i++){
					if (speed_string_R[i] != ' '){
						stringR[z] = speed_string_R[i];
						z++;
					}
				}
				speedRtemp = strtod((char*)stringR, NULL);
				speedR = speedRtemp*60/(2*PPR*LOOP_RATE); // Formatting buffer to extract speed values
				speedRRPM = speedR/GEAR_RATIO;
			}
			ClearBuffer((uint8_t*)encoders_string, 20);
		}
	}
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart){ // UART Tx Callback
	if ((huart->Instance == USART2) && (Umode == 'X')){
		HAL_UART_Receive_IT(&huart2, uart2_Rx, 1);
	}
	else if ((huart->Instance == USART2) && (Umode == 'Y')){
		if (uart2_Tx[2] == 'Y'){
			ClearBuffer(uart2_Tx, 50);
			Umode = 'F';
			trajectory_flag = 1; // BXY is transmitted -> Starting while loop
		}
	} // Defined trajectory received -> Starting while loop
	else if ((huart->Instance == USART2) && (Dmode == 'T')){
		Dmode = 'F'; // Database mode OFF
		Dtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
		Dtimer2 = __HAL_TIM_GET_COUNTER(&htim6);
	}
	else if (huart->Instance == USART1){
		speed_flag = 0; // Speed is being received
		HAL_UART_Receive_IT(&huart1, uart1_Rx, 1);
	}
}


void GetMeasureInfos(){ // Get Z axis angular speed from IMU  and X and Y from Lidar via Measure MCU
	int ThetaInt = Theta_N*1000;
	Mmode = 'T'; // Wz mode ON
	Mtimer1 = __HAL_TIM_GET_COUNTER(&htim6);
	Mtimer2 = __HAL_TIM_GET_COUNTER(&htim6); // Resetting Wz timer
	strcpy((char*)i2c1_Tx, "WL:");
	i2c1_Tx[3] = ThetaInt >> 8;
	i2c1_Rx[4] = ThetaInt & 0x00FF; // Transmit Theta in two bytes
	HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, I2C_SLAVE_ADDRESS_Measure, i2c1_Tx, sizeof(i2c1_Tx), I2C_FIRST_FRAME);
}

void GetBMS(){ // Get BMS MCU info
	Bmode = 'T'; // BMS MCU mode ON
	Btimer1 = __HAL_TIM_GET_COUNTER(&htim6);
	Btimer2 = __HAL_TIM_GET_COUNTER(&htim6); // Resetting BMS timer
	strcpy((char*)i2c1_Tx, "BMS:");
	HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, I2C_SLAVE_ADDRESS_BMS, i2c1_Tx, sizeof(i2c1_Tx), I2C_FIRST_FRAME);
}

void GetSpeed(){ // Get encoders speed
	speed_flag = 0; // Speed is being received
	HAL_UART_Transmit_IT(&huart1, uart1_Tx, sizeof(uart1_Tx)-1);
}

void SendDatabase(){ // Database Tx via RaspberryPi
	Dmode = 'T'; // Database mode ON
	HAL_UART_Transmit_IT(&huart2, data_buf, sizeof(data_buf));
}

void ClearBuffer(uint8_t* buffer, int size){ // Clear buffer
    for(int i=0; i<size; i++){
    	buffer[i]='\0';
    }
}

void Heartbeat(){ // LED Heartbeat
	if (HAL_GetTick() - Beat > 1000) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		Beat = HAL_GetTick();
	}
}

void LinAct(){ // Activates linear actuators
	if (k == 20){
		Acttimer = HAL_GetTick(); // Reboot linear actuators cycle
	}
	else if (k == 290){
		Acttimer = HAL_GetTick(); // Reboot linear actuators cycle
	}
	else if (k == 550){
		Acttimer = HAL_GetTick(); // Reboot linear actuators cycle
	}
	else if (k == 850){
		Acttimer = HAL_GetTick(); // Reboot linear actuators cycle
	}

	if (k>=20){
		if (HAL_GetTick() - Acttimer <= 5000){
			LinearActPWM = 1100; // Linear actuators go up
		}
		else if ((HAL_GetTick() - Acttimer > 5000) && (HAL_GetTick() - Acttimer <= 7000)){
			LinearActPWM = 1700; // Linear actuators go down
		}
		else if ((HAL_GetTick() - Acttimer > 7000) && (HAL_GetTick() - Acttimer <= 12000)){
			LinearActPWM = 1100; // Linear actuators go up
		}
		else if ((HAL_GetTick() - Acttimer > 12000) && (HAL_GetTick() - Acttimer <= 27000)){
			LinearActPWM = 1500; // Linear actuators stay straight
		}
		TIM16->CCR1 = LinearActPWM*FREQ_PSCCLOCK; // Generating LinearAct PWM signal
	}
}

void PIController_Init(PIController *pi){ // PI init
	pi->proportional = 0;
	pi->integral = 0;
	pi->prevError  = 0;
	pi->out = 0;
}

double PIController_Update(PIController *pi, double setpoint, double measurement){ // PI update
	double error = setpoint - measurement; // Error signal
	pi->proportional = PI_KP * error; // Proportional
    pi->integral = pi->integral + 0.5 * PI_KI * pi->T * (error + pi->prevError); // Integral
    pi->out = pi->proportional + pi->integral; // Out
    if (pi->out > pi->limMax){
        pi->out = pi->limMax;
    }
    else if (pi->out < pi->limMin){
        pi->out = pi->limMin;
    } // Applying PI limits
    pi->prevError = error; // Keeping error for the next update
    return pi->out;
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
