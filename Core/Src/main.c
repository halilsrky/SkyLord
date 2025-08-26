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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Standard library includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Project configuration */
#include "configuration.h"

/* Sensor drivers */
#include "bme280.h"                    // BME280 barometric pressure sensor
#include "bmi088.h"                    // BMI088 6-axis IMU sensor

/* Algorithm and processing modules */
#include "queternion.h"                // Quaternion math operations
#include "sensor_fusion.h"             // Sensor fusion algorithms (Mahony, Kalman)
#include "flight_algorithm.h"          // Flight state detection and control

/* Communication modules */
#include "uart_handler.h"              // UART command processing
#include "e22_lib.h"                   // LoRa wireless communication
#include "packet.h"                    // Telemetry packet handling
#include "l86_gnss.h"                  // L86 GPS/GNSS module
#include "data_logger.h"

/* Test and diagnostic modules */
#include "test_modes.h"                // System test modes (SIT, SUT)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Timer configuration */
#define TIMER_PERIOD_MS                10      // Timer interrupt period in milliseconds

/* ADC buffer sizes */
#define ADC_BUFFER_SIZE                1       // Single ADC reading per channel

/* UART buffer sizes */
#define UART_RX_BUFFER_SIZE            36      // UART4 receive buffer size
#define UART_TX_BUFFER_SIZE            128     // General UART transmit buffer size

/* Flight algorithm parameters */
#define FLIGHT_ACCEL_THRESHOLD         15.0f   // Acceleration threshold for launch detection (m/s²)
#define FLIGHT_MAX_ALTITUDE            2000.0f // Maximum expected altitude (m)
#define FLIGHT_MIN_ALTITUDE            500.0f  // Minimum altitude for descent detection (m)
#define FLIGHT_MAX_VELOCITY            60.0f   // Maximum expected velocity (m/s)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Hardware configuration macros */
#define IMU_I2C_HNDLR                  hi2c3   // I2C handler for IMU communication

/* GPIO helper macros */
#define LED_ON                         GPIO_PIN_SET
#define LED_OFF                        GPIO_PIN_RESET
#define LORA_MODE_CONFIG               RESET   // LoRa M0 pin state for configuration mode
#define LORA_MODE_ACTIVE               SET     // LoRa M1 pin state for active mode

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* Structures */
static BME_280_t BME280_sensor;         // BME280 barometric pressure sensor
bmi088_struct_t BMI_sensor;             // BMI088 IMU sensor (accelerometer + gyroscope)
sensor_fusion_t sensor_output;          // Sensor fusion output data
BME_parameters_t bme_params;             // BME280 calibration parameters
gps_data_t gnss_data;                  // L86 GNSS receiver data
static e22_conf_struct_t lora_1;

/* Communication buffers and structures */
uint8_t usart4_rx_buffer[UART_RX_BUFFER_SIZE];  // UART4 receive buffer
static char uart_buffer[UART_TX_BUFFER_SIZE];   // General purpose UART buffer for formatted strings
extern unsigned char normal_paket[62];  // Normal mode telemetry packet

/* ADC buffers for voltage and current measurement */
float current_mA = 0.0f;               // Processed current value in mA
float voltage_V = 0.0f;                // Processed voltage value in V

/* System status flags */
int is_BME_ok = 0;                     // BME280 initialization status
int is_BMI_ok = 0;                     // BMI088 initialization status
int bmi_status_ok = 0;                 // BMI088 operational status
uint32_t lastUpdate = 0;               // Last sensor update timestamp

/* External variables from other modules */
extern uint8_t Gain;                   // Kalman filter gain parameter
extern uint8_t gyroOnlyMode;           // Gyroscope-only mode flag

/* Interrupt and communication flags */
volatile uint8_t usart4_packet_ready = 0;  // UART4 packet received flag
volatile uint16_t usart4_packet_size = 0;  // Size of received UART4 packet
volatile uint8_t tx_timer_flag_100ms = 0;        // Timer-based transmission flag
volatile uint8_t tx_timer_flag_1s = 0;        // Timer-based transmission flag
volatile uint8_t usart4_tx_busy = 0;       // UART4 transmission busy flag
volatile uint8_t usart2_tx_busy = 0;       // UART4 transmission busy flag

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* Sensor initialization functions */
static void bme280_begin(void);              // Initialize BME280 barometric sensor
uint8_t bmi_imu_init(void);                  // Initialize BMI088 IMU sensor
void lora_init(void);                 // Initialize LoRa E22 module


/* Utility and debug functions */
void IMU_visual(void);                       // Send IMU data for visualization
void read_ADC(void);                         // Read and display ADC values
void HSD_StatusCheck(void);                  // Check high-speed data status
void uart4_send_packet_dma(uint8_t *data, uint16_t size);  // Send packet via UART4 DMA
void lora_send_packet_dma(uint8_t *data, uint16_t size);

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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	/* ==== TIMER AND INTERRUPT CONFIGURATION ==== */
	// Configure Timer 2 for periodic operations (10ms interval)
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	// Configure external interrupt priorities
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
	
	// Enable external interrupts for IMU data ready signals
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* ==== SENSOR INITIALIZATION ==== */
	// Initialize BME280 barometric pressure sensor
	bme280_begin();
	HAL_Delay(1000);
	bme280_config();
	bme280_update();

	// Initialize BMI088 IMU sensor (accelerometer + gyroscope)
	bmi_imu_init();
	bmi088_config(&BMI_sensor);
	get_offset(&BMI_sensor);

	// Get initial quaternion for orientation
	getInitialQuaternion();

	/* ==== LORA COMMUNICATION SETUP ==== */
    e22_config_mode(&lora_1);
    HAL_Delay(20);
	lora_init();
    HAL_Delay(20);
	e22_transmit_mode(&lora_1);

	/* ==== SENSOR FUSION AND ALGORITHMS ==== */
	// Initialize sensor fusion system
	sensor_fusion_init();
	
	// Configure flight algorithm parameters (accel_threshold, max_altitude, min_altitude, max_velocity)
	flight_algorithm_set_parameters(FLIGHT_ACCEL_THRESHOLD, FLIGHT_MAX_ALTITUDE, FLIGHT_MIN_ALTITUDE, FLIGHT_MAX_VELOCITY);
	flight_algorithm_init();

	/* ==== UART AND COMMUNICATION SETUP ==== */
	// Initialize UART handler for command processing
	uart_handler_init();

	// Start UART4 DMA reception for incoming commands
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, usart4_rx_buffer, UART_RX_BUFFER_SIZE);

	/* ==== GPS/GNSS INITIALIZATION ==== */
	// Initialize UART5 and DMA for GPS communication
	HAL_UART_Transmit(&huart5, (uint8_t*)"$PMTK251,57600*2C\r\n", 19, 100);
    HAL_UART_DeInit(&huart5);
    huart5.Init.BaudRate = 57600;
    HAL_UART_Init(&huart5);
	HAL_DMA_Init(&hdma_uart5_rx);
	L86_GNSS_Init(&huart5);

	data_logger_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		/*CONTINUOUS SENSOR UPDATES*/
		bmi088_update(&BMI_sensor);		// Update IMU sensor data (accelerometer + gyroscope) - High frequency sampling
		bme280_update(); 		// Update barometric pressure sensor data for altitude estimation

		/*COMMAND PROCESSING*/
		uart_handler_process_packets();		// Process incoming UART packets and handle system mode changes

		// Handle command processing and system state transitions
		if (uart_handler_command_ready()) {
			uart_handler_clear_command_flag();
			
			// Reset flight algorithm state when switching to NORMAL operational mode
			if (uart_handler_get_mode() == MODE_NORMAL) {
				flight_algorithm_reset();
			}
		}
		


		/*PERIODIC OPERATIONS (100ms Timer)*/
		if (tx_timer_flag_100ms) {
			tx_timer_flag_100ms = 0;
			
			// Update GPS/GNSS data for position and velocity tracking
			L86_GNSS_Update(&gnss_data);
			//L86_GNSS_Print_Info(&gnss_data, &huart1);



			// Check high-speed data acquisition status
			HSD_StatusCheck();

			/* Optional debug and monitoring functions (currently disabled for performance) */
			//L86_GNSS_Print_Info(&gnss_data, &huart4);     // Transmit GPS information for debugging
			//IMU_visual();                                  // Send IMU data for external visualization tools

			
			/*MODE-SPECIFIC OPERATIONS*/
			SystemMode_t current_mode = uart_handler_get_mode();
			
			switch (current_mode) {
				case MODE_NORMAL:
					/* Full operational flight mode with complete sensor fusion and flight control */
					
					// Enhanced sensor fusion using Kalman filter for precise state estimation
					sensor_fusion_update_kalman(&BME280_sensor, &BMI_sensor, &sensor_output);
					
					// Execute flight algorithm for launch detection, apogee detection, and recovery deployment
					flight_algorithm_update(&BME280_sensor, &BMI_sensor, &sensor_output);
					
					// Package all sensor data into telemetry packet for ground station transmission
					addDataPacketNormal(&BME280_sensor, &BMI_sensor, &gnss_data, &sensor_output, voltage_V, current_mA);
					HAL_UART_Transmit(&huart1, (uint8_t*)normal_paket, 62, 100);

					log_normal_packet_data(normal_paket);
					/* Optional real-time telemetry transmission (disabled to reduce latency) */
					//uint16_t status_bits = flight_algorithm_get_status_bits();
					//uart_handler_send_status(status_bits);
					break;

				case MODE_SIT:
					/* SIT Mode: Tests sensor integration, data flow, and core system operations with real-time hardware */
					test_modes_handle_sit(&BME280_sensor, &BMI_sensor);
					break;

				case MODE_SUT:
					/* SUT Mode: Validates specific algorithm and calibration logic with synthetic flight data (no real-time sensor input) */
					algorithm_update_sut();
					break;
					
				default:
					/* Unknown mode - Log error or switch to safe mode */
					// TODO: Add error handling for unknown system modes
					break;
			}
		}
		if(tx_timer_flag_1s >= 10){
			tx_timer_flag_1s = 0;
			read_ADC();
			lora_send_packet_dma((uint8_t*)normal_paket, 62);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 8999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_M0_Pin|RF_M1_Pin|SD_CS_Pin|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SGU_LED2_Pin|SGU_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|MCU_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STATUS1_Pin STATUS2_Pin */
  GPIO_InitStruct.Pin = STATUS1_Pin|STATUS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_M0_Pin RF_M1_Pin SD_CS_Pin PA11 */
  GPIO_InitStruct.Pin = RF_M0_Pin|RF_M1_Pin|SD_CS_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SGU_LED2_Pin SGU_LED1_Pin PB14 */
  GPIO_InitStruct.Pin = SGU_LED2_Pin|SGU_LED1_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 MCU_LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|MCU_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(50);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initialize BME280 barometric pressure sensor
 * @note Configures BME280 with optimal settings for flight applications
 */
void bme280_begin()
{
	BME280_sensor.device_config.bme280_filter = BME280_FILTER_8;
	BME280_sensor.device_config.bme280_mode = BME280_MODE_NORMAL;
	BME280_sensor.device_config.bme280_output_speed = BME280_OS_8;
	BME280_sensor.device_config.bme280_standby_time = BME280_STBY_20;
	bme280_init(&BME280_sensor, &hi2c1);
}

/**
 * @brief Initialize BMI088 IMU sensor (accelerometer + gyroscope)
 * @return Initialization status (0 = success, non-zero = error)
 * @note Configures both accelerometer and gyroscope with appropriate ranges and data rates
 */
uint8_t bmi_imu_init(void)
{
	// Accelerometer configuration
	BMI_sensor.device_config.acc_bandwith = ACC_BWP_OSR4;
	BMI_sensor.device_config.acc_outputDateRate = ACC_ODR_200;
	BMI_sensor.device_config.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
	BMI_sensor.device_config.acc_range = ACC_RANGE_24G;

	// Gyroscope configuration
	BMI_sensor.device_config.gyro_bandWidth = GYRO_BW_116;
	BMI_sensor.device_config.gyro_range = GYRO_RANGE_2000;
	BMI_sensor.device_config.gyro_powerMode = GYRO_LPM_NORMAL;

	// Interrupt and communication configuration
	BMI_sensor.device_config.acc_IRQ = EXTI15_10_IRQn;
	BMI_sensor.device_config.gyro_IRQ = EXTI15_10_IRQn;
	BMI_sensor.device_config.BMI_I2c = &IMU_I2C_HNDLR;
	BMI_sensor.device_config.offsets = NULL;  // Offset data stored in backup SRAM

	return bmi088_init(&BMI_sensor);
}

/**
 * @brief Initialize LoRa E22 module
 * @note Configures LoRa module for telemetry transmission
 */
void lora_init(void)
{
	lora_1.baud_rate 		= 	E22_BAUD_RATE_115200;
	lora_1.parity_bit		=	E22_PARITY_8N1;
	lora_1.air_rate			=	E22_AIR_DATA_RATE_38400;
	lora_1.packet_size		=	E22_PACKET_SIZE_64;
	lora_1.rssi_noise		=	E22_RSSI_NOISE_DISABLE;
	lora_1.power			=	E22_TRANSMITTING_POWER_22;
	lora_1.rssi_enable		=	E22_ENABLE_RSSI_DISABLE;
	lora_1.mode				= 	E22_TRANSMISSION_MODE_TRANSPARENT;
	lora_1.repeater_func	=	E22_REPEATER_FUNC_DISABLE;
	lora_1.lbt				=	E22_LBT_DISABLE;
	lora_1.wor				=	E22_WOR_RECEIVER;
	lora_1.wor_cycle		=	E22_WOR_CYCLE_1000;
	lora_1.channel			=	25;

	e22_init(&lora_1, &huart2);

	HAL_UART_DeInit(&huart2);
	HAL_Delay(20);
	huart2.Init.BaudRate = 115200;
	HAL_Delay(20);
	HAL_UART_Init(&huart2);

}

/**
 * @brief GPIO external interrupt callback
 * @param GPIO_Pin Pin number that triggered the interrupt
 * @note Handles IMU data ready interrupts
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_12) {
		bmi088_set_accel_INT(&BMI_sensor);
	}
	if (GPIO_Pin == GPIO_PIN_13) {
		bmi088_set_gyro_INT(&BMI_sensor);
	}
}

/**
 * @brief UART receive event callback
 * @param huart UART handle
 * @param Size Number of bytes received
 * @note Handles UART4 packet reception with DMA
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == UART4) {
		usart4_packet_ready = 1;
		usart4_packet_size = Size;
		// Restart RX DMA for next packet
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, usart4_rx_buffer, sizeof(usart4_rx_buffer));
		__HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT); // Disable half-transfer interrupt
	}
}

/**
 * @brief Timer period elapsed callback
 * @param htim Timer handle
 * @note Triggers periodic telemetry transmission (10ms interval)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
	    tx_timer_flag_100ms = 1;   // 100ms flag
	    tx_timer_flag_1s++;      // 1s flag (counts to 10)
	}
}

/**
 * @brief UART transmission complete callback
 * @param huart UART handle
 * @note Clears transmission busy flag
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4) {
		usart4_tx_busy = 0;
	}
	if (huart->Instance == USART2) {
		usart2_tx_busy = 0;
	}
}

/**
 * @brief Send packet via UART4 using DMA
 * @param data Pointer to data buffer
 * @param size Size of data to send
 * @note Non-blocking transmission using DMA
 */
void uart4_send_packet_dma(uint8_t *data, uint16_t size)
{
	if (!usart4_tx_busy) {
		usart4_tx_busy = 1;
		HAL_UART_Transmit_DMA(&huart4, data, size);
	}
}

/**
 * @brief Send packet with LORA using DMA
 * @param data Pointer to data buffer
 * @param size Size of data to send
 * @note Non-blocking transmission using DMA
 */
void lora_send_packet_dma(uint8_t *data, uint16_t size)
{
	if (!usart2_tx_busy) {
		usart2_tx_busy = 1;
		HAL_UART_Transmit_DMA(&huart2, data, size);
	}
}

/**
 * @brief Send IMU visualization data via UART
 * @note Transmits Euler angles and system parameters for visualization
 */
void IMU_visual()
{
	float yaw = BMI_sensor.datas.angle_y;
	float pitch = BMI_sensor.datas.angle_z;
	float roll = BMI_sensor.datas.angle_x;
	float yaw1 = BMI_sensor.datas.yaw1;
	float pitch1 = BMI_sensor.datas.pitch1;
	float roll1 = BMI_sensor.datas.roll1;

	sprintf(uart_buffer, "A1 %.2f %.2f %.2f\r", yaw, pitch, roll);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

	sprintf(uart_buffer, "A2 %.2f %.2f %.2f\r\n", yaw1, pitch1, roll1);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

	sprintf(uart_buffer, "G %d\r", Gain);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

	sprintf(uart_buffer, "M %d\r", gyroOnlyMode);
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

/**
 * @brief Read ADC values for voltage and current monitoring (optimized)
 * @note Fast polling mode - reads both ADC1 and ADC2 efficiently
 */
void read_ADC()
{
    static uint16_t adc1_raw = 0;  // ADC1 değeri (Channel 9)
    static uint16_t adc2_raw = 0;  // ADC2 değeri (Channel 8)

    // ADC1 okuma
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        adc1_raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // ADC2 okuma
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 5) == HAL_OK) {
        adc2_raw = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);

    // Kalibrasyonlu değerleri hesapla
    voltage_V = (adc1_raw * 13.2f) / 4096.0f;  // 3.3V referans, 12-bit ADC
    current_mA = (adc2_raw * 3.3f) / 4096.0f; // Gerekirse akım sensörüne göre kalibre edin

	if(voltage_V < 7.0){
		e22_sleep_mode(&lora_1);

	}else if(voltage_V >= 7.0){
		e22_transmit_mode(&lora_1);
	}
}

/**
 * @brief Check High Speed Data (HSD) status and control status LEDs
 * @note Monitors STATUS1 and STATUS2 pins and controls corresponding LEDs
 */
void HSD_StatusCheck()
{
	// Check STATUS1 pin (PC0) and control LED1
	GPIO_PinState pc0_state = HAL_GPIO_ReadPin(STATUS1_GPIO_Port, STATUS1_Pin);
	if (pc0_state == GPIO_PIN_RESET) {
		HAL_GPIO_WritePin(SGU_LED1_GPIO_Port, SGU_LED1_Pin, LED_ON);
	} else {
		HAL_GPIO_WritePin(SGU_LED1_GPIO_Port, SGU_LED1_Pin, LED_OFF);
	}

	// Check STATUS2 pin (PC5) and control LED2
	GPIO_PinState pc5_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
	if (pc5_state == GPIO_PIN_RESET) {
		HAL_GPIO_WritePin(SGU_LED2_GPIO_Port, SGU_LED2_Pin, LED_ON);
	} else {
		HAL_GPIO_WritePin(SGU_LED2_GPIO_Port, SGU_LED2_Pin, LED_OFF);
	}
}

/**
 * @brief I2C Memory read complete callback (DMA)
 * @param hi2c I2C handle
 * @note Handles BMI088 sensor data DMA transfer completion
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3) {
        if (hi2c->Devaddress == ACC_I2C_ADD) {
            // Accelerometer data received (9 bytes: XYZ + sensor time)
            bmi088_accel_dma_complete_callback(&BMI_sensor);
        }
        else if (hi2c->Devaddress == GYRO_I2C_ADD) {
            // Gyroscope data received (6 bytes: XYZ)
            bmi088_gyro_dma_complete_callback(&BMI_sensor);
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
