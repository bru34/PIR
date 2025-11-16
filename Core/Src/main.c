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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stm32f4xx_ll_usart.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA
#define SENSOR_PIN GPIO_PIN_11
#define SENSOR_PORT GPIOC
void SIM800L_Diagnostic(void);
void SIM800L_ConnectNetwork(void);
SIM800L_Status SIM800L_SendCommandTimeOut(char *command, int timeout);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osMutexId_t mutexSIM800Send;
osSemaphoreId_t mySemaphoreAlarm;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
char response[100] = {0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Redirection de printf vers ITM (SWO)
int _write(int file, char *ptr, int len)
{
	for (int i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}

// Initialise le module SIM800Lvoid SIM800L_Init(void) {

void SIM800L_Diagnostic(void) {

	// Vérification radio
	SIM800L_SendCommand("AT+CFUN=1,1");
	printf("CFUN: %s\n", response);
	HAL_Delay(60000);

	// Vérification bande
	SIM800L_SendCommand("AT+CBAND?");
	printf("CBAND: %s\n", response);

	// Vérification SIM
	SIM800L_SendCommand("AT+CPIN?");
	printf("CPIN: %s\n", response);

	while(1)
	{
		// Vérification signal
		SIM800L_SendCommand("AT+CSQ");
		printf("CSQ: %s\n", response);
		HAL_Delay(500);
	}
	// Vérification enregistrement réseau
	SIM800L_SendCommand("AT+CREG?");
	printf("CREG: %s\n", response);
}

#if 0
SIM800L_Status SIM800L_Init(void) {
	// Reset et démarrage du module
	HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(5000); // Temps de démarrage du SIM800L

	// Vérification du module
	if (SIM800L_SendCommand("AT") == SIM800L_FAILED) {
		return SIM800L_FAILED;
	}

	// Vérification de la carte SIM
	if (SIM800L_SendCommand("AT+CSMINS?") == SIM800L_FAILED) {
		printf("Erreur : Aucune carte SIM détectée !\n");
		return SIM800L_FAILED;
	}

	if (SIM800L_SendCommand("AT+CPIN=\"667234\"") == SIM800L_FAILED) {
		printf("Erreur : mauvais PIN !\n");
		return SIM800L_FAILED;
	}
	HAL_Delay(10000); // attendre que la SIM soit validée

	if (SIM800L_SendCommand("AT+CPIN?") == SIM800L_FAILED) {
		printf("Erreur : mauvais PIN !\n");
		return SIM800L_FAILED;
	}

	if (SIM800L_SendCommand("AT+CSQ") == SIM800L_FAILED) {
		printf("Erreur : mauvais PIN !\n");
		return SIM800L_FAILED;
	}

	// Attente active de l'enregistrement réseau
	int networkReady = 0;
	for (int i = 0; i < 10; i++) { // 10 essais max
		if (SIM800L_SendCommand("AT+CREG?") == SIM800L_SUCCESS) {
			networkReady = 1;
			break;
		}
		HAL_Delay(3000);
	}

	if (!networkReady) {
		printf("Erreur : Échec de l'enregistrement réseau !\n");
		return SIM800L_FAILED;
	}

	// Vérification du signal
	SIM800L_SendCommand("AT+CSQ");

	// Mode texte pour les SMS
	SIM800L_SendCommand("AT+CMGF=1");

	// Activation des notifications SMS, envoie direct sur le port serie les SMS recus
	SIM800L_SendCommand("AT+CNMI=2,2,0,0,0");

	// 2 = Activation de l'économie d'énergie, sans DTR
	// 2 n est pas dans le datasheet mais ne renvoie pas d'erreur ...
	SIM800L_SendCommand("AT+CSCLK=2");

	printf("SIM800L prêt et connecté au réseau !\n");
	return SIM800L_SUCCESS;
}
#else
SIM800L_Status SIM800L_Init(void) {


	printf("START INIT\r\n");
	HAL_Delay(500);

	// Reset et démarrage du module
	HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(5000); // Temps de démarrage du SIM800L

	//SIM800L_Diagnostic();

	// Désactiver l’écho
	SIM800L_SendCommand("ATE0");

	// Mode complet
	SIM800L_SendCommand("AT+CFUN=1");
	HAL_Delay(5000);

	// Vérification SIM
	SIM800L_SendCommand("AT+CSMINS?");
	SIM800L_SendCommand("AT+CPIN?");

	// Entrée du PIN
	SIM800L_SendCommand("AT+CPIN=\"667234\"");
	HAL_Delay(10000); // attendre que la SIM soit validée

	// Vérification signal
	SIM800L_SendCommand("AT+CSQ");

	SIM800L_SendCommandTimeOut("AT+COPS=?", 60000);
	printf("OPERATEUR: %s",response);

	/*	// Enregistrement réseau
	SIM800L_SendCommand("AT+CREG=2");
	for (int i = 0; i < 20; i++) {
		SIM800L_SendCommand("AT+CREG?");
		if (strstr(response, "+CREG: 0,1") || strstr(response, "+CREG: 0,5")) {
			printf("Enregistré sur le réseau\n");
			break;
		}
		printf("Echec d enregistrement reseau %s\n", response);
		SIM800L_SendCommand("AT+CSQ");
		printf("%s",response);
		HAL_Delay(5000);
	}*/

	SIM800L_SendCommand("AT+CREG=2");
	HAL_Delay(1000);



	while((!strstr(response, "+CREG: 0,1") && !strstr(response, "+CREG: 0,5")))
	{
		SIM800L_SendCommand("AT+CSQ");
		printf("SIGNAL: %s",response);

		SIM800L_SendCommand("AT+CREG?");
		HAL_Delay(2000);
		printf("%s",response);
	}

	printf("Enregistré sur le réseau\n");
	// Config SMS
	SIM800L_SendCommand("AT+CMGF=1");
	SIM800L_SendCommand("AT+CNMI=2,2,0,0,0");
	SIM800L_SendCommand("AT+CSCLK=0");

	return SIM800L_SUCCESS;
}
#endif

// Se connecte au réseau et vérifie l'état
void SIM800L_ConnectNetwork(void) {
	// Vérification de l'enregistrement réseau
	SIM800L_SendCommand("AT+CREG?");
	// Vérification de la qualité du signal
	SIM800L_SendCommand("AT+CSQ");

	// Activation des fonctions radio (si nécessaire)
	SIM800L_SendCommand("AT+CFUN=1");

	// Vérification de l'opérateur
	SIM800L_SendCommand("AT+COPS?");
}

// Envoi SMS
SIM800L_Status SIM800L_SendSMS(char *phoneNumber, char *message) {
	SIM800L_Status result = SIM800L_FAILED;

	if (osMutexAcquire(mutexSIM800Send, TIMEOUT_RESET_SIM800) != osOK) {
		// Mutex non obtenu on reset le stm32
		NVIC_SystemReset();
	}

	// Reveil du SIM800
	//SIM800L_SendCommand("AT+CSCLK=0");

	do {
		// Vérifier l'enregistrement réseau
		if (SIM800L_SendCommand("AT+CREG?") == SIM800L_FAILED) {
			printf("Erreur : Le module n'est pas enregistré sur le réseau.\n");
			break;
		}

		// Vérifier le signal réseau
		if (SIM800L_SendCommand("AT+CSQ") == SIM800L_FAILED) {
			printf("Erreur : Signal réseau insuffisant.\n");
			break;
		}

		// Activer le mode texte pour les SMS
		if (SIM800L_SendCommand("AT+CMGF=1") == SIM800L_FAILED) {
			printf("Erreur : Impossible d'activer le mode texte SMS.\n");
			break;
		}

		// Construire la commande d'envoi du SMS
		char command[30];
		snprintf(command, sizeof(command), "AT+CMGS=\"%s\"", phoneNumber);
		if (SIM800L_SendCommand(command) == SIM800L_FAILED) {
			printf("Erreur : Problème avec la commande AT+CMGS.\n");
			break;
		}

		// Envoyer le message texte
		HAL_StatusTypeDef txStatus = HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
		if (txStatus != HAL_OK) {
			printf("Erreur lors de l'envoi du message : %d\n", txStatus);
			break;
		}

		// Envoyer CTRL+Z (0x1A) pour valider l'envoi du SMS
		HAL_StatusTypeDef endStatus = HAL_UART_Transmit(&huart1, (uint8_t*)"\x1A", 1, HAL_MAX_DELAY);
		if (endStatus != HAL_OK) {
			printf("Erreur lors de la validation de l'envoi du SMS.\n");
			break;
		}

		HAL_Delay(5000); // Attente de la confirmation !! TODO  on doit verifier le status !!!
		result = SIM800L_SUCCESS;
		printf("SMS envoyé avec succès à %s !\n", phoneNumber);

	} while (0);

	//SIM en sommeil
	//SIM800L_SendCommand("AT+CSCLK=1");

	osMutexRelease(mutexSIM800Send);

	// met le stm32 en mode STOP ...
	//	HAL_SuspendTick();
	//	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	//	SystemClock_Config();  // ⚠️ doit restaurer les clocks
	//	HAL_ResumeTick();

	return result;
}

SIM800L_Status SIM800L_SendCommandTimeOut(char *command, int timeout)
{
	// Taille = longueur de la commande + 2 pour \r\n + 1 pour le \0
	size_t length = strlen(command) + 3;
	char fullCommand[length];

	// Construit la commande complète avec \r\n
	snprintf(fullCommand, length, "%s\r\n", command);

	// Vide le buffer de réponse
	memset(response, 0, sizeof(response));

	// Envoi de la commande AT
	HAL_UART_Transmit(&huart1, (uint8_t*)fullCommand, strlen(fullCommand), HAL_MAX_DELAY);

	// Lecture de la réponse
	HAL_UART_Receive(&huart1, (uint8_t*) response, sizeof(response) - 1, timeout);

	// Vérifie si "OK" est présent
	if (strstr((char*)response, "OK") != NULL) {
		return SIM800L_SUCCESS;
	} else {
		return SIM800L_FAILED;
	}
}

SIM800L_Status SIM800L_SendCommand(char *command)
{
	return SIM800L_SendCommandTimeOut(command, 3000);
}

void ThreadAlarm(void *argument)
{
	for(;;)
	{
		osSemaphoreAcquire(mySemaphoreAlarm, osWaitForever);
		SIM800L_SendSMS("+33626031205", "Detection sur STM32 !");
		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // Etteint la LED
	}
}

void ThreadReception(void *argument)
{
	for(;;)
	{
		SIM800L_SendSMS("+33626031205", "Echo SMS");
	}
}
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
	MX_RTC_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	SIM800L_Init();  // Initialisation SIM800L
	SIM800L_ConnectNetwork(); // Connexion au réseau
	HAL_Delay(100);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	// Création du mutex avec héritage de priorité
	const osMutexAttr_t mutexAttr = {
			.name = "ProtectB",
			.attr_bits = osMutexPrioInherit
	};
	mutexSIM800Send = osMutexNew(&mutexAttr);

	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	// Création des threads avec priorités différentes
	const osThreadAttr_t highAttr = {
			.name = "HighThread",
			.priority = osPriorityHigh
	};
	const osThreadAttr_t lowAttr = {
			.name = "LowThread",
			.priority = osPriorityBelowNormal
	};

	osThreadNew(ThreadAlarm, NULL, &highAttr);
	osThreadNew(ThreadReception, NULL, &lowAttr);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// 💤 Entrée initiale en mode Stop
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	SystemClock_Config();
	HAL_ResumeTick();

	while (1)
	{
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	RTC_AlarmTypeDef sAlarm = {0};

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable the Alarm A
	 */
	sAlarm.AlarmTime.Hours = 0x0;
	sAlarm.AlarmTime.Minutes = 0x0;
	sAlarm.AlarmTime.Seconds = 0x0;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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
	huart1.Init.BaudRate = 9600;
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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ALARM_Pin */
	GPIO_InitStruct.Pin = ALARM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SIM_RST_Pin */
	GPIO_InitStruct.Pin = SIM_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SIM_RST_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// WKUP pin : detection ou RTC ...
	if (GPIO_Pin == GPIO_PIN_0)
	{
		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // Allume la LED
		osSemaphoreRelease(mySemaphoreAlarm);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
