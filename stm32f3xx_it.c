/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void powerControl(int input);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
/* USER CODE BEGIN EV */
extern void idle(void);
extern void converterFail(void);
extern HRTIM_HandleTypeDef hhrtim1;

volatile uint16_t CURRENT;
volatile uint16_t VIN;
volatile uint16_t VOUT;
volatile int arrayCounter;
volatile int arrayCurrent[QuantitySample];
volatile int arrayInputVoltage[QuantitySample];
volatile int arrayOutputVoltage[QuantitySample];
volatile long sumCurrent;
volatile long sumInputVoltage;
volatile long sumOutputVoltage;

int bankFlag = 1;

extern volatile uint16_t adc_value[16];
extern volatile uint32_t setFaultMessage;
extern volatile int controlState;
extern volatile int displayResetCounter;
extern volatile int statusMessage;
extern volatile int statusMeasurement;
extern volatile float maxCurrent;
extern volatile float maxVoltage;
extern volatile float maxVin;
extern volatile float minVin;
extern volatile float maxVout;
extern volatile float balancerCURRENT;
extern volatile float balancerVOUT;
extern volatile float FILTEREDVOUT; 
extern volatile float FILTEREDVIN;
extern volatile float FILTEREDCURRENT;
extern volatile float powerMeasurement;
extern volatile long long timeCounter;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	if (bankFlag == 1) {
			//First Half of Buffer Filled
			//Overcurrent protection: First thing we have to do is check overcurrent
			if (adc_value[0] > 4000 || adc_value[4] > 4000 || adc_value[0] < 100 || adc_value[4] < 100) {
					statusMessage = 7; //over current
					converterFail();
					setFaultMessage |= 1 << 0;
			} else {
					setFaultMessage &= ~(1 << 0);
			}
			CURRENT = (adc_value[0] + adc_value[4]) >> 1;
			VIN = (adc_value[1] + adc_value[5]) >> 1;
			VOUT = (adc_value[2] + adc_value[6]) >> 1;
		
			bankFlag = 0;
	} else {
			//Second Half of Buffer Filled
			//Overcurrent protection: First thing we have to do is check overcurrent
			if (adc_value[8] > 4000 || adc_value[12] > 4000 || adc_value[8] < 100 || adc_value[12] < 100) {
					statusMessage = 7; //over current
					converterFail();
					setFaultMessage |= 1 << 0;
			} else {
					setFaultMessage &= ~(1 << 0);
			}
			CURRENT = (adc_value[8] + adc_value[12]) >> 1;
			VIN = (adc_value[9] + adc_value[13]) >> 1;
			VOUT = (adc_value[10] + adc_value[14]) >> 1;
		
			bankFlag = 1;
	}
	
	displayResetCounter++;
	
	arrayCurrent[arrayCounter] = CURRENT;
	arrayInputVoltage[arrayCounter] = VIN;
  arrayOutputVoltage[arrayCounter] = VOUT;
	
	sumCurrent += CURRENT;
	sumInputVoltage += VIN;
    sumOutputVoltage += VOUT;
	
	if (arrayCounter == QuantitySample - 1) {
        sumCurrent -= arrayCurrent[0];
        sumInputVoltage -= arrayInputVoltage[0];
        sumOutputVoltage -= arrayOutputVoltage[0];
		arrayCounter = 0;		
	} else {
        sumCurrent -= arrayCurrent[arrayCounter + 1];
        sumInputVoltage -= arrayInputVoltage[arrayCounter + 1];
        sumOutputVoltage -= arrayOutputVoltage[arrayCounter + 1];
		arrayCounter++;
	}
	
	FILTEREDCURRENT = ((sumCurrent / (QuantitySample - 1)) * 0.00102 - 2.11049 + balancerCURRENT);
	FILTEREDVOUT = ((sumOutputVoltage / (QuantitySample - 1)) * 0.02410 + 1.569 + balancerVOUT);
	FILTEREDVIN = (sumInputVoltage / (QuantitySample - 1) / 80.01);
	
	if (controlState == CHARGE) {
		if (FILTEREDCURRENT < maxCurrent * 0.99f) {
			powerControl(1);
		}
		else if (FILTEREDCURRENT > maxCurrent * 1.05f) {
			powerControl(-1);
		} else {
			powerControl(0);
		}
	} else if (controlState == DISCHARGE) {
		if (FILTEREDCURRENT < -SETDISCHARGERATE * 1.03f) {
			powerControl(1);
		} else if (FILTEREDCURRENT > -SETDISCHARGERATE * 0.99f && (FILTEREDVIN < 25.0f)) {
			powerControl(-1);
		} else {
			powerControl(0);
		}
	} else if (controlState == IDLE) {
		if (FILTEREDVOUT < 5) {
			powerControl(1);
		} else if (FILTEREDVOUT > 6){
			powerControl(-1);
		} else {
			powerControl(0);
		}
	} else if (controlState == FAILED) {
		powerControl(0);
	}

	if (statusMeasurement == SET) {
		powerMeasurement += FILTEREDVOUT * FILTEREDCURRENT;
		timeCounter++;
	}
	
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	statusMessage = 0;
	//for (int i = 0; i < 1000; i++);
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void powerControl(int input) {      
    //Input voltage under and overvoltage check (19-32V)
    if (FILTEREDVIN < minVin || FILTEREDVIN > maxVin) {
		statusMessage = 9; //input overvoltage
		converterFail();
		setFaultMessage |= 1 << 1;
    } else {
		setFaultMessage &= ~(1 << 1);
	}
    
    //Output voltage overvoltage check (18V)
    if (FILTEREDVOUT > maxVout) {
		statusMessage = 10; //output overvoltage
		converterFail();
        setFaultMessage |= 1 << 2;    
    } else {
		setFaultMessage &= ~(1 << 2);
	}
   
    //Output Voltage limiter at maxVoltage 
	if (controlState == CHARGE && FILTEREDVOUT > maxVoltage+ 0.1f && SMALLBATTERY > MINDUTY) {
			SMALLBATTERY--;
			BLUE = LEDOFF;
			RED = LEDON;
			GREEN = LEDOFF;
			statusMessage = 11; //output overvoltage
		return;
	}
	//InputVoltage limiter at 25V
	if (controlState == DISCHARGE && FILTEREDVIN > 25.0f && (SMALLBATTERY < 667.82f * FILTEREDVOUT) && SMALLBATTERY > MAXDUTY) {
		SMALLBATTERY++;
		return;
	} 
	if (controlState == CHARGE && FILTEREDVIN < 20 && SMALLBATTERY > MINDUTY) {
		SMALLBATTERY--;
		return;
	}
	
    //max duty cycle 100% -> 15360 * 1 = 15360
	if (input > 0 && SMALLBATTERY < MAXDUTY) {
		SMALLBATTERY++;  
	} else if (input < 0 && SMALLBATTERY > MINDUTY){
		SMALLBATTERY--;
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
