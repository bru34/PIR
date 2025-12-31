/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef enum {
    SIM800L_SUCCESS,
    SIM800L_FAILED
} SIM800L_Status;

#define TIMEOUT_RESET_SIM800 20000

typedef enum {
	MODEM_OK = 0,
	ERR_NOT_INITIALIZED,
	ERR_AT_SYNC,
	ERR_ATE0,
	ERR_CMEE,
	ERR_CPIN,
	ERR_CREG,
	ERR_CTZU,
	ERR_SMS_FORMAT,
	ERR_SMS_NUMBER,
	ERR_SMS_BODY,
	ERR_SETBAUD,
	ERR_WRITEFLASH,
	ERR_NOT_ALIVE,
	ERR_SLEEPMODE,
	ERR_IFC,
} ModemStatus;

typedef enum {
	SLEEP_MODE_DISABLE = 0,
	SLEEP_MODE_DTR,
	SLEEP_MODE_AUTO,
} SleepMode;

#define PIN_NUMBER "667234"
#define AT_PIN_CMD "AT+CPIN=\"" PIN_NUMBER "\"\r"
#define PHONE_NUMBER "+33626031205"
#define CLE_API "1CX4AjmP2zlIZe"  // Cle API pour envoi SMS via modem A7670G sur Free mobile
#define USER_FREE "50349591"

ModemStatus Modem_Init(void);
ModemStatus Modem_Init_Sequence(void);
ModemStatus Modem_Send_SMS(char*, char* );
ModemStatus Modem_Send_AT_Wait(char*, char*, uint32_t);
ModemStatus Modem_Check_Alive(void);
int Modem_Get_Signal_Quality(void);
void gpio_Wakeup(void);
void gpio_Sleep(void);
void A7670_Free_Send_Notif(UART_HandleTypeDef *huart, char *user, char *pass, char *msg);
void A7670_Free_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define MODEM_SLEEP_Pin GPIO_PIN_0
#define MODEM_SLEEP_GPIO_Port GPIOC
#define ALARM_Pin GPIO_PIN_0
#define ALARM_GPIO_Port GPIOA
#define ALARM_EXTI_IRQn EXTI0_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
