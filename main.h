/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define H2 HRTIM_OUTPUT_TA1
#define L2 HRTIM_OUTPUT_TA2
#define H1 HRTIM_OUTPUT_TB1
#define L1 HRTIM_OUTPUT_TB2

#define MINDUTY 200
#define MAXDUTY (15360 - MINDUTY)
#define REALMAXDUTY 15360

#define LEDON 18000
#define LEDOFF 36000
#define LEDBRIGHTNESSFACTOR 36000

#define SET 1
#define RESET 0
#define STOP 0
#define ON 1
#define OFF 0

#define CHARGE 1
#define IDLE 0
#define DISCHARGE -1
#define FAILED -2

#define BLUE HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR
#define RED HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP2xR
#define GREEN HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR

#define SMALLBATTERY HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR
#define BIGBATTERY HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR

#define QuantitySample 10
#define adcValueCurrent adc_value[0]
#define adcValueInputVoltage adc_value[1]
#define adcValueOutputVoltage adc_value[2]

#define BATTERY_ADDRESS         0x16

//#define FILTEREDCURRENT ((sumCurrent / (QuantitySample - 1)) * 0.00102 - 2.12049)
//#define FILTEREDVOUT ((sumOutputVoltage / (QuantitySample - 1)) * 0.02410 + 1.569)
//#define FILTEREDVIN (sumInputVoltage / (QuantitySample - 1) / 80.01)

//#define FILTEREDCURRENT ((sumCurrent / (QuantitySample - 1)) * 0.001614 - 3.283)



#define SETTEMPERATURE 95.0f
#define SETDISCHARGERATE 1.0f

#define CURRENTCHARGE i2cBufferReceive[10]

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Switch_Pin GPIO_PIN_0
#define Switch_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_12
#define BLUE_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_13
#define RED_GPIO_Port GPIOB
#define GREEN_Pin GPIO_PIN_14
#define GREEN_GPIO_Port GPIOB
#define H2_Pin GPIO_PIN_8
#define H2_GPIO_Port GPIOA
#define L2_Pin GPIO_PIN_9
#define L2_GPIO_Port GPIOA
#define H1_Pin GPIO_PIN_10
#define H1_GPIO_Port GPIOA
#define L1_Pin GPIO_PIN_11
#define L1_GPIO_Port GPIOA
#define ChipSelect_Pin GPIO_PIN_6
#define ChipSelect_GPIO_Port GPIOB
#define ChipEnable_Pin GPIO_PIN_7
#define ChipEnable_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
