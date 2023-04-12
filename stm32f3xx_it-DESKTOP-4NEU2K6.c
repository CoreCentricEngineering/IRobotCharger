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
#define adcValueCurrent adc_value[0]
#define adcValueInputVoltage adc_value[1]
#define adcValueOutputVoltage adc_value[2]
#define QuantitySample 50

#define SETCHARGE 100
#define SETTEMPERATURE 110.0
#define SETCURRENT 288

#define CURRENT sumCurrent / (QuantitySample - 1)
#define CURRENTTEMPERATURE ((message[1][2] * 256 + message[2][2]) / 10.0 -273.15) * 9 / 5 + 32
#define CURRENTCHARGE message[2][5]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern int chargeState;
extern uint8_t message[3][14];
extern volatile uint16_t adc_value[8];
extern int offSetCurrent;

volatile int arrayCurrent[QuantitySample] = {0};
volatile int arrayInputVoltage[QuantitySample] = {0};
volatile int arrayOutputVoltage[QuantitySample] = {0};

volatile long sumCurrent = 0;
volatile long sumInputVoltage = 0;
volatile long sumOutputVoltage = 0;

volatile int arrayCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void BoostControl(int input);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

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
	
	arrayCurrent[arrayCounter] = adcValueCurrent;
    arrayInputVoltage[arrayCounter] = adcValueInputVoltage;
    arrayOutputVoltage[arrayCounter] = adcValueOutputVoltage;
	
	sumCurrent += adcValueCurrent;
    sumInputVoltage += adcValueInputVoltage;
    sumOutputVoltage += adcValueOutputVoltage;
	
	if (arrayCounter == QuantitySample - 1) {
		sumCurrent -= arrayCurrent[0];
		sumInputVoltage -= arrayInputVoltage[0];
		sumOutputVoltage -= arrayOutputVoltage[0];

		arrayCounter = 0;
		
		if (CURRENTCHARGE != SETCHARGE) {
			if (SETCURRENT * 0.98  > CURRENT - offSetCurrent) {
				BoostControl(1);
			} else if (SETCURRENT * 1.03 < CURRENT - offSetCurrent) {
				BoostControl(-1);
			}
		}
	} else {
        sumCurrent -= arrayCurrent[arrayCounter + 1];
        sumInputVoltage -= arrayInputVoltage[arrayCounter + 1];
        sumOutputVoltage -= arrayOutputVoltage[arrayCounter + 1];
        
        arrayCounter++;
    }
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void BoostControl(int input) {
   /* if (CURRENTTEMPERATURE > SETTEMPERATURE) {
        OCR1A = 0;
        OCR1B = 1023;
        chargeState = RESET;
        return;
    }
    
    //if battery communication check
    if (errorCode != 0) {
        //OCR1A = 0;
        CURRENTCHARGE = 0;
        chargeState = RESET;
    }
    
    //Input voltage under and overvoltage check (23-25V)
    if (adcValueInputVoltage < 602 || adcValueInputVoltage > 655) {
        OCR1A = 0;
        OCR1B = 1023;
        CURRENTCHARGE = 0;
        chargeState = RESET;
        return;    
    } 
    
    //Output voltage under and overvoltage check (18V)
    if (adcValueOutputVoltage > 471) {
        OCR1A = 0;
        OCR1B = 1023;
        CURRENTCHARGE = 0;
        chargeState = RESET;
        return;    
    }

    if (chargeState == 1) digitalWrite(GREEN, !digitalRead(GREEN));
    
    //max duty cycle 100% -> 1024 * 1 = 1024
    //Output Voltage limiter at 16.8V (16.8 * 26.2 = 440)
    if (chargeState == SET) {
        if (input > 0 && OCR1A < 1021 && OUTPUTVOLTAGE < 440) {
            OCR1A++;
            if (CURRENT - offSetCurrent > 70) {
                OCR1B = OCR1A + 2;
            } else {
                OCR1B = 1023;
            }    
        } else if (input < 0 && OCR1A > 0){
            if (CURRENT - offSetCurrent > 70) {
                OCR1B = OCR1A + 1;    
            } else {
                OCR1B = 1023;
            }
            OCR1A--;
        }
    } else {
        if (OUTPUTVOLTAGE < 262 && OCR1A < 1022) {
            OCR1A++;
            OCR1B = 1023;
        } else if (OUTPUTVOLTAGE > 262 && OCR1A > 0) {
            OCR1B = 1023;
            OCR1A--;
        }
    }*/
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
