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
#include "MY_NRF24.h"
#include "test.h"
#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

HRTIM_HandleTypeDef hhrtim1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
void converterFail(void);
void idle(void);
void startCharging(void);
void startDischarging(void);

volatile float FILTEREDVOUT; 
volatile float balancerVOUT;
volatile float FILTEREDVIN;
volatile float FILTEREDCURRENT;
volatile float balancerCURRENT;

volatile float maxCurrent = 0.1;
volatile float maxVoltage = 16.8;
volatile float maxTemperature = 105;
volatile float maxVin = 32.0;
volatile float minVin = 19.0;
volatile float maxVout = 18.0;
volatile float currentTemperature = 0;

volatile uint16_t adc_value[16] = {0};

volatile float powerMeasurement = 0;
volatile long long timeCounter = 0;
volatile int statusMeasurement = 0;
int stuckCounter = 0;

uint64_t TxpipeAddrs = 0xFFFFFFFF01;
char myTxData[32] = "0xFFFFFFFF01";

HAL_StatusTypeDef result;
volatile int i2cMessageCounter = -1;
volatile int i2cFailedMessageCounter = 0;
volatile int displayResetCounter = 0;

volatile int setCharge = 30;
volatile int chargerNumber = 1;
volatile uint32_t setAlarmMessage = 0;
volatile uint32_t setFaultMessage = 0;


uint8_t i2cBufferSend[14] = {0, 6, 8, 9, 10, 13, 14, 16, 17, 22, 24, 26, 20, 21};
uint8_t i2cBufferStart[3] = {42, 1, 0};
uint8_t i2cBufferStop[3] = {42, 0, 0};
uint8_t i2cBufferReceive[56] = {0};

volatile uint8_t check;
volatile uint8_t data[10];
volatile uint8_t data1[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
volatile long temp = 0;
char displayString[50] = "";
volatile int statusMessage = 0;


volatile int controlState = CHARGE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
/* Direct printf to output somewhere */
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	//H1 off, L1 on
	SMALLBATTERY = REALMAXDUTY;
	
	//H2 off, L2 on
	BIGBATTERY = 0;
	
	//L1 L2 on
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, L2 | L1);
	
	//Start-Up Delay
	HAL_Delay(10);
	
  	//initialize OLED screen
	SSD1306_Init (); // initialize the diaply  
	
	//activate CURRENT OFFSET
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	
	//start dual simultaneous ADC sampling
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_value, 8);
	
	//initialize all LED
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 | HRTIM_OUTPUT_TD1);
	HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);
	
	//NRF24_begin(GPIOB, ChipSelect_Pin, ChipEnable_Pin, hspi1);
	//nrf24_DebugUART_Init(huart2);
	
	//NRF24_stopListening();
	//NRF24_openWritingPipe(TxpipeAddrs);
	//NRF24_setAutoAck(true);
	//NRF24_setChannel(125);
	//NRF24_setPayloadSize(32);
	
	//Start-Up Delay
	//HAL_Delay(10);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {		
		//reset WTD
		HAL_IWDG_Refresh(&hiwdg);
			
		//change setCharge depending on Switch state
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) ? (setCharge = 30) : (setCharge = 99);
		
		/*if (RFMessageDelayCounter > 500) {
			sprintf(myTxData, "%2d %5.2f %5.2f %5.2f %3d", chargerNumber, FILTEREDVIN, FILTEREDVOUT, FILTEREDCURRENT, CURRENTCHARGE);
			if (!NRF24_write(myTxData, 32)) {
				statusMessage = 1;  //RF
			}
			RFMessageDelayCounter = 0;
			displayResetCounter++;
		}*/
		
		if (displayResetCounter > 10000) {
			//Reset Display
			SSD1306_Init (); 
			displayResetCounter = 0;
		}
		
		i2cMessageCounter >= 13 ? i2cMessageCounter = 0 : i2cMessageCounter++;
			
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
		if (HAL_I2C_Master_Transmit(&hi2c1, BATTERY_ADDRESS, i2cBufferSend + i2cMessageCounter, 1, 100) != HAL_OK) {
			//reset I2C BUS
			I2C1->CR1 |= I2C_CR1_SWRST;
			statusMessage = 2; //TX
			i2cMessageCounter = -1;
			i2cFailedMessageCounter++;			
		} else {
			// Read 2 bytes
			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
			if ( HAL_I2C_Master_Receive(&hi2c1, BATTERY_ADDRESS, i2cBufferReceive + i2cMessageCounter * 2, 2, 100) != HAL_OK) {
				//reset I2C BUS
				I2C1->CR1 |= I2C_CR1_SWRST;
				statusMessage = 3; //RX
				i2cMessageCounter = -1;
				i2cFailedMessageCounter++;
			} else {
				
				//detect offSet Failure
				if (controlState == IDLE && FILTEREDCURRENT < -1.5) while (1) converterFail();
				
				if (setCharge == 99) {
						if (CURRENTCHARGE < setCharge && currentTemperature < SETTEMPERATURE && setAlarmMessage < 32768) {
								startCharging();
						} else {
								idle();
						}
				} else if (setCharge == 30) {
						if (currentTemperature < SETTEMPERATURE && CURRENTCHARGE > setCharge) {
								startDischarging();
						} else if (setAlarmMessage < 32768 && currentTemperature < SETTEMPERATURE && CURRENTCHARGE < setCharge) {
								startCharging();
						} else {
							idle();
						}
				}
				
				//overtemperature protection
				if (i2cMessageCounter == 2) {
					currentTemperature = ((i2cBufferReceive[5] * 256 + i2cBufferReceive[4]) / 10.0 -273.15) * 9 / 5 + 32;
					if (currentTemperature > maxTemperature) {
						statusMessage = 8; //overtemperature
						converterFail(); 
						setFaultMessage |= 1 << 4; 
					} else {
						setFaultMessage &= ~(1 << 4);
					}
				}
				
				// checks battery remaining life >70%, compares relative and absolute charge states
				if (i2cMessageCounter == 6) {
					if (CURRENTCHARGE >= 10 && (CURRENTCHARGE * 7 >= i2cBufferReceive[12] * 10)) {
						statusMessage = 4; //70% less charge
						converterFail();
						setFaultMessage |= 1 << 3; 
					} else {
						setFaultMessage &= ~(1 << 3);
					}
				}
				
				if (i2cMessageCounter == 9) {
					setAlarmMessage = i2cBufferReceive[19] * 256 + i2cBufferReceive[18];
				}
				
				if (i2cMessageCounter == 12) {
					if ((i2cBufferReceive[25] * 256 + i2cBufferReceive[24]) / 1000.0 != 0) {
						if ((i2cBufferReceive[25] * 256 + i2cBufferReceive[24]) / 1111.0 < 1.2 && (i2cBufferReceive[25] * 256 + i2cBufferReceive[24]) / 1111.0 > 0.5) {
							maxCurrent = (i2cBufferReceive[25] * 256 + i2cBufferReceive[24]) / 1111.0;
						} else if ((i2cBufferReceive[25] * 256 + i2cBufferReceive[24]) / 1111.0 >= 1.2) {
							maxCurrent = 1.2;
						} else {
							maxCurrent = 0.1;
						}
					}
				}
				
				if (i2cMessageCounter == 13) {
					if ((i2cBufferReceive[27] * 256 + i2cBufferReceive[26]) / 1000.0 != 0)
						maxVoltage = (i2cBufferReceive[27] * 256 + i2cBufferReceive[26]) / 1000.0;
				}
				//self calibration Process
				if (i2cMessageCounter > 0 && controlState != IDLE) {
					if (FILTEREDVOUT > (i2cBufferReceive[7] * 256 + i2cBufferReceive[6]) / 1000.0) {
						balancerVOUT -= 0.01f;
					} else {
						balancerVOUT += 0.01f;
					}
					if (i2cBufferReceive[9] * 256 + i2cBufferReceive[8] > 32000) {
						//discharging
						if (FILTEREDCURRENT > (i2cBufferReceive[9] * 256.0 + i2cBufferReceive[8] - 65536.0) / 1000.0) {
							balancerCURRENT -= 0.001f;
						} else {
							balancerCURRENT += 0.001f;
						}
					} else {
						//charging
						if (FILTEREDCURRENT > (i2cBufferReceive[9] * 256.0 + i2cBufferReceive[8]) / 1000.0) {
							balancerCURRENT -= 0.001f;
						} else {
							balancerCURRENT += 0.001f;
						}
					}
				}				
				i2cFailedMessageCounter = 0;
			}
		} 
		
		if (controlState == FAILED && setFaultMessage == 0) idle();
		
		//if 5 consecutive battery communication fails reset controller to IDLE state
		if (i2cFailedMessageCounter > 5) {
			if (controlState != IDLE) idle();
			BLUE = LEDOFF;
			RED = LEDOFF;
			GREEN =LEDOFF;
			CURRENTCHARGE = 0;
			setAlarmMessage = 0;
			stuckCounter = 0;
		}
		//calculate current temperature
		SSD1306_GotoXY (0, 0);
		//sprintf(displayString, "%4d %4d %4d %4d", i2cBufferReceive[0], i2cBufferReceive[1], i2cBufferReceive[2], i2cBufferReceive[3]);
		sprintf(displayString, "I:%5.2f %4.2f %5.0f", FILTEREDCURRENT, maxCurrent, timeCounter / 500.0);
		SSD1306_Puts (displayString, &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_GotoXY (0, 9);
		//sprintf(displayString, "%4d %4d %4d %4d", i2cBufferReceive[4], i2cBufferReceive[5], i2cBufferReceive[6], i2cBufferReceive[7]);
		sprintf(displayString, "Vi:%4.1f %4d %5d", FILTEREDVIN, statusMessage, i2cBufferReceive[12]);
		SSD1306_Puts (displayString, &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_GotoXY (0, 18);
		//sprintf(displayString, "%4d %4d %4d %4d", i2cBufferReceive[8], i2cBufferReceive[9], i2cBufferReceive[10], i2cBufferReceive[11]);
		sprintf(displayString, "Vo:%4.1f %4.1f %5d", FILTEREDVOUT, maxVoltage, setAlarmMessage);
		SSD1306_Puts (displayString, &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_GotoXY (0, 27); 
		//sprintf(displayString, "%4d %4d %4d %4d", i2cBufferReceive[12], i2cBufferReceive[13], i2cBufferReceive[14], i2cBufferReceive[15]);
		sprintf(displayString, "T:%5.0f %4.1fW%2d", currentTemperature, powerMeasurement / 1800000.0f, statusMessage);
		SSD1306_Puts (displayString, &Font_7x10, SSD1306_COLOR_WHITE); 
		SSD1306_GotoXY (0, 37); 
		sprintf(displayString, "%3d %3d", CURRENTCHARGE, setCharge);
		SSD1306_Puts (displayString, &Font_16x26, SSD1306_COLOR_WHITE); 
		SSD1306_UpdateScreen();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONVHRTIM_TRG1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERD_CMP2;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 15360;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 6825;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 5000;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 3;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  pDeadTimeCfg.RisingValue = 200;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 200;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMPER;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 1000;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 36000;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV4;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 10000;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 15000;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

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
  hi2c1.Init.Timing = 0x00200E1D;
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
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 15) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 0;
  hiwdg.Init.Reload = 150;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|ChipSelect_Pin|ChipEnable_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : Switch_Pin */
  GPIO_InitStruct.Pin = Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 ChipSelect_Pin ChipEnable_Pin PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|ChipSelect_Pin|ChipEnable_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void startDischarging() {
	if (stuckCounter > 50) {
			BLUE = LEDOFF;
			RED = LEDON;
			GREEN =LEDOFF;
	} else {
			//turn on all LED
			BLUE = 0;
			RED = 0;
			GREEN = 0;
	}

	
	//send start charging command to battery
	if (setAlarmMessage == 0 || setAlarmMessage > 16384) {
			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
			if (HAL_I2C_Master_Transmit(&hi2c1, BATTERY_ADDRESS, i2cBufferStart, 3, 25) != HAL_OK) {
					statusMessage = 5; //START
			}
			stuckCounter++;
			return;
	}
	
	stuckCounter = 0;
	
	if (controlState == DISCHARGE) return;	
	controlState = DISCHARGE;
	
			//buck-boost transaction added coil discharge cycle 12/1/2020
			
			//All Mosfets OFF
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1 | H2 | L1 | L2);
			
			//H1 off, L1 on
			SMALLBATTERY = REALMAXDUTY;
			
			//H2 off, L2 on
			BIGBATTERY = 0;
			
			//turn off top 2 mosfet and turn on bottom 2 mosfet to discharge inductor, if you don't discharge it there will be voltage spike and mosfet will be damaged
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1 | H2);
			HAL_HRTIM_WaveformOutputStart(&hhrtim1, L1 | L2);
			
			for (int i = 0; i < 100000; i++);		//delay
	
	//reset calibration coefficent
	balancerCURRENT = 0;
	balancerVOUT = 0;
	
	//start power measurement
	timeCounter = 0;
	powerMeasurement = 0;
	statusMeasurement = SET;
	
	//All Mosfets OFF
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1 | H2 | L1 | L2);
	
	//initial duty cycle set for discharge MAXDUTY means low start
	SMALLBATTERY = MAXDUTY;
	
	//H2 on, L2 off
	BIGBATTERY = REALMAXDUTY;
	
	//only turn off H1
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, H2 | L1 | L2);
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1);
	
	
	//initialize OLED screen
	SSD1306_Init (); // initialize the diaply 
}
void startCharging() {
	if (setCharge == 99 && CURRENTCHARGE >= 98) {
			//turn on green LED
			BLUE = LEDOFF;
			RED = LEDOFF;
			GREEN =LEDON;
	} else if (stuckCounter > 50){
			BLUE = LEDOFF;
			RED = LEDON;
			GREEN =LEDOFF;
	} else {
			//turn on blue LED
			BLUE = LEDON;
			RED = LEDOFF;
			GREEN =LEDOFF;
	}
	
	if (setAlarmMessage == 0 || setAlarmMessage > 16384) {
			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
			if (HAL_I2C_Master_Transmit(&hi2c1, BATTERY_ADDRESS, i2cBufferStart, 3, 25) != HAL_OK) {
									statusMessage = 6; //STOP
			}
			stuckCounter++;
			return;
	}
	
	stuckCounter = 0;
	
	if (controlState == CHARGE) return;
	controlState = CHARGE;
	
	//reset calibration coefficent
	balancerCURRENT = 0;
	balancerVOUT = 0;
	
	//start power measurement
	powerMeasurement = 0;
	timeCounter = 0;
	statusMeasurement = SET;
	
	//All Mosfets OFF
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1 | H2 | L1 | L2);
	
	//initial duty cycle set
	SMALLBATTERY = MINDUTY;
	
	//H2 on, L2 off
	BIGBATTERY = REALMAXDUTY;
	
	//only turn off H2
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, H1 | L1 | L2);
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H2);

	//initialize OLED screen
	SSD1306_Init (); // initialize the diaply 
}
void idle(void) {
	//turn on GREEN LED if target charge achieved, if not turn off all LED
	if (setCharge == 99 && CURRENTCHARGE >= 98) {
			BLUE = LEDOFF;
			RED = LEDOFF;
			GREEN = LEDON;
	} else if (setCharge == 30 && CURRENTCHARGE == 30 && setAlarmMessage > 16384) {
			BLUE = LEDOFF;
			RED = LEDOFF;
			GREEN = LEDON;
	} else {
			BLUE = LEDOFF;
			RED = LEDOFF;
			GREEN = LEDOFF;
	}
	
	if (setCharge == 30 && CURRENTCHARGE == 30 && setAlarmMessage < 16384) {
			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
			if (HAL_I2C_Master_Transmit(&hi2c1, BATTERY_ADDRESS, i2cBufferStop, 3, 25) != HAL_OK) {
									statusMessage = 6; //STOP
			}
	}
	 	
	if (controlState == IDLE) return;
	controlState = IDLE;
		
	setAlarmMessage = 0;
	currentTemperature = 0;
	//stop power measurement
	statusMeasurement = RESET;
	
	//All Mosfets OFF
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1 | H2 | L1 | L2);
	
	//initial duty cycle set
	SMALLBATTERY = MINDUTY;
	//H2 on, L2 off
	BIGBATTERY = REALMAXDUTY;
	
	//only turn off H2
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, H1 | L1 | L2);
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H2);
	
	//reset charge current
	maxCurrent = 0.1;
	
	//initialize OLED screen
	//SSD1306_Init (); // initialize the diaply 
}

void converterFail(void) {
	controlState = FAILED;
	
	//All Mosfets OFF
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1 | H2 | L1 | L2);
	
	//H1 off, L1 on
	SMALLBATTERY = REALMAXDUTY;
	
	//H2 off, L2 on
	BIGBATTERY = 0;
	
	//turn off top 2 mosfet and turn on bottom 2 mosfet to discharge inductor, if you don't discharge it there will be voltage spike and mosfet will be damaged
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, H1 | H2);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, L1 | L2);
	
	//turn on RED LED
	BLUE = LEDOFF;
	RED = LEDON;
	GREEN =LEDOFF;
	for (int i = 0; i < 10000; i++);
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /*if (hi2c->Instance==hi2c1.Instance)
    {
        HAL_DMA_Abort_IT(hi2c->hdmatx);
    }*/
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /*if (hi2c->Instance==hi2c1.Instance)
    {
        HAL_DMA_Abort_IT(hi2c->hdmarx);
    }*/
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
