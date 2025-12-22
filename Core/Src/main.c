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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Statuts de l'initialisation du modem
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
} ModemStatus;

#define PIN_NUMBER "667234"
#define AT_PIN_CMD "AT+CPIN=\"" PIN_NUMBER "\"\r"
#define PHONE_NUMBER "+33626031205"

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
char modem_buffer[128] = {0}; // Buffer rception modem
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int Modem_Get_Signal_Quality(void);
ModemStatus Modem_Init(void);
ModemStatus Modem_Init_Sequence(void);
ModemStatus Modem_Send_SMS(char*, char* );
ModemStatus Modem_SendWait(char*, char*, uint32_t);

// Redirection de printf vers ITM (SWO)
int _write(int file, char *ptr, int len)
{
	for (int i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}

// Traitement de l'alarme dans un thread dédié
void ThreadAlarm(void *argument)
{
	// On garde en mémoire l'heure du dernier envoi
	static uint32_t last_sms_tick = 0;
	// On définit un temps de pause (ex: 10 secondes = 10000 ms)
	const uint32_t SMS_COOLDOWN = 10000;

	for(;;)
	{
		// 1. On attend qu'un jeton soit dispo (Vraie alarme OU parasite)
		osSemaphoreAcquire(mySemaphoreAlarm, osWaitForever);

		// 2. LE FILTRE TEMPOREL
		// On vérifie si suffisamment de temps s'est écoulé depuis le dernier SMS
		if (HAL_GetTick() - last_sms_tick > SMS_COOLDOWN)
		{
			printf("Alarme VALIDE\n");

			Modem_Send_SMS(PHONE_NUMBER, "ALARME DETECTEE !");

			// On met à jour l'heure du dernier envoi
			/* Note : ce cpt n'est pas init au demarrage de l'appli, donc on ne filtre pas a la 1ere emmission.
			 * C'est tres bien, comme ca on recoit un SMS au demarrage pour confirmer que tout fonctionne !
			 */
			last_sms_tick = HAL_GetTick();

			osDelay(500);
		}
		else
		{
			// C'est un parasite arrivé trop tôt après le précédent envoi !
			printf("Alarme ignoree (Cooldown actif)\n");
		}

		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
		osSemaphoreAcquire(mySemaphoreAlarm, 0);
	}
}

// Thread de réception des données modem (SMS entrants, etc.)
void ThreadReception(void *argument)
{
	for(;;)
	{
		// envoi d un SMS en echo
		osDelay(10); // Laisse respirer le système
	}
}

// Fonction utilitaire pour envoyer une commande et attendre une réponse (simplifiée)
//ModemStatus Modem_SendWait(char* cmd, char* expected_resp, uint32_t timeout) {
//	memset(modem_buffer, 0, sizeof(modem_buffer));
//
//	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
//	HAL_UART_Receive(&huart1, (uint8_t*)modem_buffer, sizeof(modem_buffer) - 1, timeout);
//	// printf("\r\nRecu: [%s]", modem_buffer); // <--- Ajoute ça pour voir ce qui arrive vraiment !
//
//	if (strstr(modem_buffer, expected_resp))
//		return MODEM_OK;
//	else
//		return ERR_NOT_INITIALIZED;
//}
ModemStatus Modem_SendWait(char* cmd, char* expected_resp, uint32_t timeout) {

	// 1. Nettoyage du buffer et Envoi de la commande
	memset(modem_buffer, 0, sizeof(modem_buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);

	// 2. Réception intelligente (byte par byte)
	uint32_t tickstart = HAL_GetTick();
	int index = 0;

	// Tant qu'on n'a pas dépassé le temps imparti...
	while ((HAL_GetTick() - tickstart) < timeout) {

		uint8_t received_char = 0;

		// On essaie de lire 1 seul caractère avec un tout petit timeout (10ms)
		if (HAL_UART_Receive(&huart1, &received_char, 1, 10) == HAL_OK) {

			// On stocke le caractère si on a de la place
			if (index < sizeof(modem_buffer) - 1) {
				modem_buffer[index++] = received_char;
				modem_buffer[index] = '\0'; // Toujours fermer la chaîne pour strstr
			}

			// On vérifie tout de suite si on a reçu la réponse attendue !
			if (strstr(modem_buffer, expected_resp) != NULL) {
				return MODEM_OK; // Gagné ! On sort tout de suite (pas d'attente inutile)
			}
		}
	}

	// Si on sort de la boucle while, c'est que le timeout global a expiré
	return ERR_NOT_INITIALIZED; // Ou TIMEOUT
}


//	Séquence d'initialisation logicielle du modem A7670G
ModemStatus Modem_Init_Sequence(void) {

	int retry = 0;
	ModemStatus retVal = ERR_NOT_INITIALIZED;

	memset(modem_buffer, 0, sizeof(modem_buffer));

	// 1. Sync Baudrate
	printf("\tSync baudrate...");

	while(Modem_SendWait("AT\r", "OK", 1000) != MODEM_OK) {
		retry++;
		if(retry > 5) {
			printf(" FAIL\n");
			return ERR_AT_SYNC;
		}
		HAL_Delay(500);
	}
	printf(" OK\n");

	// 2. Echo Off
	if (Modem_SendWait("ATE0\r", "OK", 1000) != MODEM_OK) {
		printf("Erreur ATE0\n");
		return ERR_ATE0;
	}

	// 3. Error Message Format
	if (Modem_SendWait("AT+CMEE=2\r", "OK", 1000) != MODEM_OK) {
		printf("Erreur AT+CMEE\n");
		return ERR_CMEE;
	}

	// 4. SIM Check & PIN Management
	printf("\tVerif SIM...");

	// Étape 4.1 : On demande l'état de la SIM
	if (Modem_SendWait("AT+CPIN?\r", "+CPIN: READY", 500) != MODEM_OK) {


		// Étape 4.2 : Si pas prête, est-ce qu'elle demande le PIN ?
		if (Modem_SendWait("AT+CPIN?\r", "+CPIN: SIM PIN", 500) == MODEM_OK) {

			// On envoie le code PIN
			if (Modem_SendWait(AT_PIN_CMD, "OK", 2000) != MODEM_OK) {
				return ERR_CPIN; // Le modem n'a pas accepté la commande (ou timeout)
			}

			// IMPORTANT : Après le PIN, la SIM met quelques secondes à passer en READY
			HAL_Delay(3000);

			// Étape 3 : Vérification finale obligatoire
			if (Modem_SendWait("AT+CPIN?\r", "+CPIN: READY", 1000) != MODEM_OK) {
				return ERR_CPIN;
			}
		}
	}
	printf(" OK\n");

	// 5. Network Check
	printf("\tVerif Reseau... ");
	// On laisse un peu plus de temps (5s) car l'enregistrement peut prendre du temps
	if (Modem_SendWait("AT+CREG?\r", "OK", 5000) == MODEM_OK) {

		// 2. On vérifie si le buffer contient l'un des statuts d'enregistrement valides
		if (strstr(modem_buffer, "+CREG: 0,1") != NULL ||  // Enregistré (Réseau domestique)
				strstr(modem_buffer, "+CREG: 0,5") != NULL ||  // Enregistré (Roaming)
				strstr(modem_buffer, "+CREG: 0,6") != NULL ||  // SMS uniquement (Réseau domestique)
				strstr(modem_buffer, "+CREG: 0,7") != NULL)    // SMS uniquement (Roaming)
		{
			int niveau_signal;
			// Succès : Le modem est enregistré et prêt
			niveau_signal = Modem_Get_Signal_Quality();
			printf("OK [signal: %d/31]\n", niveau_signal);
		}
		else {
			printf(" FAIL (Not registered)\n");
			return ERR_CREG;
		}
	}

	// 6. Timezone
	if (Modem_SendWait("AT+CTZU=1\r", "OK", 1000) != MODEM_OK) {
		printf("Erreur AT+CTZU\n");
		return ERR_CTZU;
	}

	retVal = Modem_Send_SMS(PHONE_NUMBER, "Big Brother is watching you...");

	return retVal;
}

ModemStatus Modem_Send_SMS(char* phone_number, char* message) {
	char cmd[64];

	// 1. Passage en mode Texte (indispensable)
	if (Modem_SendWait("AT+CMGF=1\r", "OK", 1000) != MODEM_OK) {
		printf("Erreur: Impossible de passer en mode texte\n");
		return ERR_SMS_FORMAT;
	}

	// 2. Envoi du numéro de téléphone
	// Le modem va répondre par le caractère '>' pour dire qu'il attend le texte
	sprintf(cmd, "AT+CMGS=\"%s\"\r", phone_number);
	printf("\tEnvoi au %s de { %s } ... ", phone_number, message);

	if (Modem_SendWait(cmd, ">", 2000) != MODEM_OK) {
		printf("Erreur: Le modem n'attend pas le texte\n");
		return ERR_SMS_NUMBER;
	}

	// 3. Envoi du corps du message + CTRL+Z (ASCII 26)
	// On envoie le texte brut sans \r\n à la fin du texte lui-même
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);

	// On termine par CTRL+Z pour valider l'envoi
	uint8_t ctrlz = 26;
	HAL_UART_Transmit(&huart1, &ctrlz, 1, 100);

	// 4. Attente de la confirmation d'envoi (peut prendre du temps)
	// Le modem répond "+CMGS: <id>" puis "OK"
	if (Modem_SendWait("", "OK", 10000) != MODEM_OK) {
		printf(" FAIL (Pas de confirmation reseau)\n");
		return ERR_SMS_BODY;
	}

	printf("OK\n");
	return MODEM_OK;
}

int Modem_Get_Signal_Quality(void) {
	char* ptr;
	int rssi = -1;

	// 1. On envoie la commande
	//if (Modem_SendWait("AT+CSQ\r", "+CSQ:", 2000) == MODEM_OK) {
	if (Modem_SendWait("AT+CSQ\r", "OK", 2000) == MODEM_OK) {

		// Le buffer contient un truc genre: "\r\n+CSQ: 23,0\r\nOK\r\n"
		// 2. On cherche le début de la réponse utile
		ptr = strstr(modem_buffer, "+CSQ: ");
		if (ptr != NULL) {
			// 3. On extrait le nombre juste après "+CSQ: "
			// On avance le pointeur de 6 cases (longueur de "+CSQ: ")
			sscanf(ptr + 6, "%d", &rssi);
		}
	}

	return rssi; // Retourne entre 0 et 31, ou 99/ -1 si erreur
}

// FIXME : si modem non alimenté, on boucle indéfiniment ici ???
// Initialisation complète du modem avec réessais
ModemStatus Modem_Init(void) {

	ModemStatus status = ERR_NOT_INITIALIZED;
	int tentative = 0;
	printf("Demarrage Modem:\n");

	// 1. Allumage électrique (Reset + PowerKey)
	//Modem_Hard_Init();

	// 2. Configuration logicielle (Baudrate, PIN, Réseau...) avec réessais
	do {
		tentative++;
		printf("   Initialisation Modem (Tentative %d/3)...\n", tentative);

		status = Modem_Init_Sequence();

		if (status == MODEM_OK) {
			break; // Succès, on sort de la boucle immédiatement
		}

		if (tentative < 3) {
			printf("\tEchec (Code: %d). Tentative de Soft Reset...\n", status);

			// 1. On envoie la commande de reset
			// On ne checke pas trop le retour, car le modem va couper la com pour rebooter
			Modem_SendWait("AT+CRESET\r", "OK", 1000);

			// 2. IMPORTANT : On attend que le modem redémarre
			// Un reboot prend du temps (bootloader + recherche réseau)
			// 5 à 10 secondes sont recommandées
			printf("Redemarrage en cours, patientez 5s...\n");
			HAL_Delay(5000);
		}
	} while (tentative < 3);

	// Verdict final après la boucle
	if (status != MODEM_OK) {
		printf("!! Echec critique de l'initialisation Modem apres 3 essais (Dernier Code: %d) !! \n", status);
		// Blocage ou LED d'erreur
		Error_Handler();
	}
	else {
		printf("Systeme fonctionnel\n\n");
	}
	return status;
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
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	// Création du sémaphore binaire pour l’alarme
	const osSemaphoreAttr_t mySemaphoreAlarm_attributes = {
			.name = "mySemaphoreAlarm"
	};
	mySemaphoreAlarm = osSemaphoreNew(1, 0, &mySemaphoreAlarm_attributes);

	/*
	 * 1 = nombre max de jetons (sémaphore binaire)
	 * 0 = valeur initiale (donc ThreadAlarm attendra un release)
	 */
	if (mySemaphoreAlarm == NULL) {
		Error_Handler(); // gestion d’erreur si allocation échoue
	}

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

	Modem_Init();

	osThreadNew(ThreadAlarm, NULL, &highAttr);
	osThreadNew(ThreadReception, NULL, &lowAttr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
  HAL_GPIO_WritePin(PWRKEY_GPIO_Port, PWRKEY_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PWRKEY_Pin */
  GPIO_InitStruct.Pin = PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWRKEY_GPIO_Port, &GPIO_InitStruct);

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
	osThreadTerminate(NULL);
	/* Infinite loop */
	for(;;)
	{

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
#ifdef USE_FULL_ASSERT
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
