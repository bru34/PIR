/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - Gestion Modem A7670G (Mode CSCLK=2)
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
#include <stm32f446xx.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA
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
osSemaphoreId_t mySemaphoreAlarm; // Semaphore alarme
char modem_buffer[512] = {0}; // Buffer réception modem
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void ThreadAlarm(void *argument);
void ThreadReception(void *argument);
ModemStatus Modem_Set_Sleep_Mode(int mode);
ModemStatus Modem_Check_Alive(void);
ModemStatus Modem_Send_AT_Wait(char* cmd, char* expected_resp, uint32_t timeout);
ModemStatus Modem_Init_Sequence(void);
ModemStatus Modem_Send_SMS(char* phone_number, char* message);
void Modem_Free_Send_Notif(UART_HandleTypeDef *huart, char *user, char *pass, char *msg);
void Modem_Free_Init(void);
int Modem_Get_Signal_Quality(void);
ModemStatus Modem_Init(void);
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

/* Synchronisation de l'heure via NTP (Internet) */
ModemStatus Modem_Sync_Time_NTP(void) {
	printf("\tSync Heure NTP...");

	// 1. Configurer le serveur NTP
	// "fr.pool.ntp.org" = Serveur français
	// 4 = GMT+1 (4 quarts d'heure). Mets 8 pour l'été (GMT+2).
	if (Modem_Send_AT_Wait("AT+CNTP=\"fr.pool.ntp.org\",0\r", "OK", 2000) != MODEM_OK) {
		printf(" FAIL (Config)\n");
		return ERR_NTP; // Code erreur générique, tu peux en créer un spécifique
	}

	// 2. Lancer la synchro
	// Le modem répond d'abord "OK", puis quelques secondes après "+CNTP: 0" (Succès)
	// On met un timeout long (15s) pour laisser le temps au réseau
	if (Modem_Send_AT_Wait("AT+CNTP\r", "+CNTP: 0", 15000) != MODEM_OK) {
		printf(" FAIL (Timeout ou Erreur Reseau)\n");
		return ERR_NTP;
	}

	printf(" OK\n");

	// 3. Lire l'heure fraîchement acquise et mettre à jour le STM32
	// Une fois CNTP ok, l'horloge interne du modem (CCLK) est à jour.
	// On la récupère pour régler le RTC du microcontrôleur.
	memset(modem_buffer, 0, sizeof(modem_buffer));
	if (Modem_Send_AT_Wait("AT+CCLK?\r", "OK", 1000) == MODEM_OK) {

		// Format reçu : +CCLK: "yy/MM/dd,hh:mm:ss+zz"
		char *ptr = strstr(modem_buffer, "+CCLK: \"");
		if (ptr != NULL) {
			RTC_TimeTypeDef sTime = {0};
			RTC_DateTypeDef sDate = {0};

			ptr += 8; // On saute +CCLK: "

			// Parsing rapide (Attention : sscanf dépend de ta lib standard, sinon faire manuellement)
			int yy, MM, dd, hh, mm, ss;
			sscanf(ptr, "%d/%d/%d,%d:%d:%d", &yy, &MM, &dd, &hh, &mm, &ss);

			sTime.Hours = hh;
			sTime.Minutes = mm;
			sTime.Seconds = ss;
			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

			sDate.Year = yy; // Le RTC STM32 prend l'année sur 2 digits (20xx)
			sDate.Month = MM;
			sDate.Date = dd;
			// Note: Le WeekDay n'est pas calculé ici, mais c'est pas critique pour toi
			HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			printf("\tRTC STM32 mis a jour: %02d:%02d:%02d\n", hh, mm, ss);
		}
	}

	return MODEM_OK;
}

/* Fonction d'encodage URL simple (remplace espaces par %20) */
void url_encode(char *dest, const char *src) {
	while (*src) {
		if (*src == ' ') {
			// On remplace l'espace par %20
			*dest++ = '%';
			*dest++ = '2';
			*dest++ = '0';
		} else {
			// On recopie le caractère tel quel
			*dest++ = *src;
		}
		src++;
	}
	*dest = '\0'; // On termine la chaîne de caractères
}

/* Récupération de l'heure réseau brute (format YY/MM/DD,HH:MM) */
void Get_Network_Time_Raw(char *dest_time) {
	strcpy(dest_time, "Date_Inconnue");

	// Nettoyage buffer
	memset(modem_buffer, 0, sizeof(modem_buffer));

	if (Modem_Send_AT_Wait("AT+CCLK?\r", "OK", 1000) == MODEM_OK) {
		osDelay(100); // Laisse l'UART finir la réception
		char *ptr = strstr(modem_buffer, "\"");
		if (ptr != NULL) {
			ptr++;
			strncpy(dest_time, ptr, 14); // Copie "YY/MM/DD,HH:MM"
			dest_time[14] = '\0';
		}
	}
}

/* Gestion GPIO Modem Sleep */
void gpio_Wakeup(void) {
	HAL_GPIO_WritePin(MODEM_SLEEP_GPIO_Port, MODEM_SLEEP_Pin, GPIO_PIN_SET);
	osDelay(50);
	printf("Modem reveille.\n");
}

/* Gestion GPIO Modem Sleep */
void gpio_Sleep(void) {
	osDelay(50);
	HAL_GPIO_WritePin(MODEM_SLEEP_GPIO_Port, MODEM_SLEEP_Pin, GPIO_PIN_RESET);
	printf("Modem en veille.\n");
}

/* Gestion LED */
void LED_ON(void)
{
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}
/* Gestion LED */
void LED_OFF(void) {
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

/* Configuration mode veille automatique CSCLK */
ModemStatus Modem_Set_Sleep_Mode(int mode) {

	char cmd[32];

	ModemStatus status = ERR_SLEEPMODE;
	sprintf(cmd, "AT+CSCLK=%d\r", mode);
	status = Modem_Send_AT_Wait(cmd, "OK", 1000);
	if (status != MODEM_OK) {
		printf("Erreur activation CSCLK=%i\n",mode);
	}
	else {
		printf("Mode CSCLK=%i actif.\n",mode);
	}
	return status;
}

/* Vérification si le modem répond */
ModemStatus Modem_Check_Alive() {
	int essais_max = 20;

	// On nettoie préventivement l'UART (ORE)
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		volatile uint32_t tmpreg = huart1.Instance->DR; (void)tmpreg;
	}

	for (int i = 0; i < essais_max; i++) {
		printf("Ping modem (%d/%d)...\n", i+1, essais_max);

		// Si tu utilises ta fonction qui attend la réponse :
		if (Modem_Send_AT_Wait("AT\r", "OK", 200) == MODEM_OK) { // Timeout court (200ms)

			// 2. VICTOIRE ! On a eu "OK".
			// On ne spamme plus, on sort immédiatement.
			printf(" -> Modem repond ! On arrete le spam.\n");

			// Petit délai de sécurité pour vider les buffers si besoin
			osDelay(50);
			return MODEM_OK; // Succès
		}

		osDelay(50);
	}

	return ERR_NOT_ALIVE; // Le modem est mort ou sourd
}

/* Envoi d'une commande AT et attente d'une réponse spécifique */
ModemStatus Modem_Send_AT_Wait(char* cmd, char* expected_resp, uint32_t timeout) {

	// 1. Nettoyage PRÉVENTIF (Si un Overrun traîne, on le vire)
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		volatile uint32_t tmpreg = huart1.Instance->DR; (void)tmpreg;
	}

	// 2. Envoi
	memset(modem_buffer, 0, sizeof(modem_buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);

	// 3. Réception
	uint32_t tickstart = HAL_GetTick();
	int index = 0;

	while ((HAL_GetTick() - tickstart) < timeout) {
		uint8_t received_char = 0;

		// On récupère le statut de la lecture
		HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, &received_char, 1, 10);

		if (status == HAL_OK) {
			// Lecture réussie
			if (index < sizeof(modem_buffer) - 1) {
				modem_buffer[index++] = received_char;
				modem_buffer[index] = '\0';
			}
			// Vérification de la réponse
			if (strstr(modem_buffer, expected_resp) != NULL) {
				return MODEM_OK;
			}
		}
		else if (status == HAL_ERROR) {
			// <<< CORRECTION OVERRUN >>>
			// Si le STM32 détecte une erreur (bruit, overrun), il faut l'acquitter
			if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
				__HAL_UART_CLEAR_OREFLAG(&huart1);
				// Lecture fictive du DR pour valider le clear (F4/F1)
				volatile uint32_t tmpreg = huart1.Instance->DR;
				(void)tmpreg;
			}
		}
	}
	return ERR_NOT_INITIALIZED; // Timeout
}

/* Thread alarm */
void ThreadAlarm(void *argument)
{
	static uint32_t last_sms_tick = 0;
	const uint32_t SMS_COOLDOWN = 12000;
	static uint32_t cptAlarm = 0;
	char buffer_msg[64]={0};

	for(;;)
	{
		osSemaphoreAcquire(mySemaphoreAlarm, osWaitForever);
		LED_ON();

		if (HAL_GetTick() - last_sms_tick > SMS_COOLDOWN)
		{
			printf("Alarme VALIDE\n");
			gpio_Wakeup();

			cptAlarm++;

			if (Modem_Check_Alive() == MODEM_OK)
			{
				snprintf(buffer_msg, sizeof(buffer_msg), "@ %lu", cptAlarm);
				Modem_Free_Send_Notif(&huart1, USER_FREE, CLE_API, buffer_msg);
			}

			last_sms_tick = HAL_GetTick();

			printf("Attente transmission radio (10s)...\n");
			osDelay(10000);
		}
		else
		{
			printf("Alarme ignoree (Cooldown actif)\n");
		}

		gpio_Sleep();
		LED_OFF();
		osSemaphoreAcquire(mySemaphoreAlarm, 0);
	}
}

// Thread réception (Echo simple pour l'instant)
void ThreadReception(void *argument)
{
	for(;;)
	{
		osDelay(10);
	}
}

/* Séquence d'initialisation du modem */
ModemStatus Modem_Init_Sequence(void) {
	int retry = 0;
	ModemStatus retVal = ERR_NOT_INITIALIZED;

	memset(modem_buffer, 0, sizeof(modem_buffer));

	if ( Modem_Check_Alive() != MODEM_OK ) {
		printf("Modem non joignable apres 10 essais.\n");
		return ERR_NOT_INITIALIZED;
	}

	// 1. Sync Baudrate
	printf("\tSync baudrate...");
	while(Modem_Send_AT_Wait("AT\r", "OK", 500) != MODEM_OK) {
		retry++;
		if(retry > 10) {
			printf(" FAIL\n");
			return ERR_AT_SYNC;
		}
		osDelay(200);
	}
	printf(" OK\n");

	osDelay(50);

	// 3. Carte SIM
	printf("\tVerif SIM...");
	if (Modem_Send_AT_Wait("AT+CPIN?\r", "+CPIN: READY", 500) != MODEM_OK) {
		if (Modem_Send_AT_Wait("AT+CPIN?\r", "+CPIN: SIM PIN", 500) == MODEM_OK) {
			if (Modem_Send_AT_Wait(AT_PIN_CMD, "OK", 2000) != MODEM_OK) return ERR_CPIN;
			osDelay(3000);
			if (Modem_Send_AT_Wait("AT+CPIN?\r", "+CPIN: READY", 1000) != MODEM_OK) return ERR_CPIN;
		}
	}
	printf(" OK\n");

	// 4. Réseau
	printf("\tVerif Reseau... ");
	if (Modem_Send_AT_Wait("AT+CEREG?\r", "OK", 5000) == MODEM_OK) {
		// Accepte Home(1), Roaming(5), SMS Home(6), SMS Roaming(7)
		if (strstr(modem_buffer, "+CEREG: 0,1") || strstr(modem_buffer, "+CEREG: 0,5") ||
				strstr(modem_buffer, "+CEREG: 0,6") || strstr(modem_buffer, "+CEREG: 0,7"))
		{
			int niveau = Modem_Get_Signal_Quality();
			printf("OK [signal: %d/31]\n", niveau);
			retVal = MODEM_OK;
		} else {
			printf(" FAIL (Not registered on LTE)\n");
			return ERR_CREG;
		}
	}

	Modem_Free_Init();

	// Mise a jour de l heure par NTP
	Modem_Sync_Time_NTP();

	return retVal;
}

/* Envoi d'un SMS via le modem A7670G */
ModemStatus Modem_Send_SMS(char* phone_number, char* message) {
	char cmd[64];
	uint8_t ctrlz = 26;

	// on n utilise plus pour le moment ...
	// FIXME : a utiliser si fail de l API
	printf("\tNE PAS UTILISER - Envoi SMS direct via Modem...\n");
	return MODEM_OK;

	// 1. Passage en mode Texte
	if (Modem_Send_AT_Wait("AT+CMGF=1\r", "OK", 1000) != MODEM_OK) {
		return ERR_SMS_FORMAT;
	}

	// 2. Numéro
	sprintf(cmd, "AT+CMGS=\"%s\"\r", phone_number);
	printf("\tEnvoi SMS <");
	printf("%s", message);

	if (Modem_Send_AT_Wait(cmd, ">>", 2000) != MODEM_OK) {
		printf("Erreur Prompt >\n");
		return ERR_SMS_NUMBER;
	}

	// 3. Corps + CTRL-Z
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 1000);
	HAL_UART_Transmit(&huart1, &ctrlz, 1, 100);

	// 4. Confirmation (Timeout long)
	if (Modem_Send_AT_Wait("", ">> OK", 10000) != MODEM_OK) {
		printf(" FAIL (Pas de confirmation)\n");
		//FIXME : si le modem est eteint puis rallume, il faut le reinitialiser, puis renvoyer le SMS : a traiter

	} else {
		printf("OK\n");
	}

	return MODEM_OK;
}

/* * Fonction pour envoyer une notif Free Mobile via A7670 (Data/HTTPS)
 * huart : pointeur vers ton UART (ex: &huart1)
 * user  : ton identifiant Free (ex: "12345678")
 * pass  : ta clé API (ex: "AbCdEfGhIjK")
 * msg   : le message (ATTENTION: Pas d'espaces, utilise des %20 ou des underscores)
 */
void Modem_Free_Send_Notif(UART_HandleTypeDef *huart, char *user, char *pass, char *msg) {
	char time_now[20];
	char raw_full_msg[200];
	char encoded_full_msg[512];
	char http_cmd[600];

	// 1. On récupère l'heure (YY/MM/DD,HH:MM)
	Get_Network_Time_Raw(time_now);

	// 2. MODIFICATION ICI : On insère " GMT" dans les crochets
	sprintf(raw_full_msg, "%s GMT %s", time_now, msg);

	// 3. Encodage URL
	url_encode(encoded_full_msg, raw_full_msg);

	printf("SMS Final : %s\n", raw_full_msg);

	// ... La suite de la fonction reste identique (HTTPTERM, HTTPINIT, etc.) ...
	Modem_Send_AT_Wait("AT+HTTPTERM\r\n", "OK", 500);
	if (Modem_Send_AT_Wait("AT+HTTPINIT\r\n", "OK", 1000) != MODEM_OK){
		printf("\tProbleme de requette HTTP");
		// TODO : gérer l'erreur car le modem ne repond plus, par exemple suite a un redemarrage
		// suite pb alimentation ... le code se bloque a ce moment du code suite redemarrage Modem
		Error_Handler();
		return;
	}

	sprintf(http_cmd, "AT+HTTPPARA=\"URL\",\"https://smsapi.free-mobile.fr/sendmsg?user=%s&pass=%s&msg=%s\"\r\n",
			user, pass, encoded_full_msg);

	Modem_Send_AT_Wait(http_cmd, "OK", 1000);

	if (Modem_Send_AT_Wait("AT+HTTPACTION=0\r\n", "+HTTPACTION: 0,200", 15000) == MODEM_OK) {
		printf("SMS envoye\n");
	} else {
		printf("Echec envoi SMS\n");
	}
	Modem_Send_AT_Wait("AT+HTTPTERM\r\n", "OK", 1000);
}

/* Initialisation spécifique Free Mobile (APN, PDP) */
void Modem_Free_Init(void) {
    // On utilise un buffer local pour éviter de compter les caractères à la main
    char cmd[64];

    printf("--- Config APN Free ---\n");

    // 1. Configurer l'APN
    // On prépare la commande proprement
    sprintf(cmd, "AT+CGDCONT=1,\"IP\",\"free\"\r");

    // On utilise TA fonction standard (qui gère les timeouts, le nettoyage buffer, etc.)
    if (Modem_Send_AT_Wait(cmd, "OK", 2000) != MODEM_OK) {
        printf("Erreur Config APN\n");
        // Optionnel : return; si c'est critique
    }

    // 2. Activer la Data
    printf("Activation DATA...\n");
    // Si c'est déjà activé, le modem peut répondre ERROR ou OK, on gère les deux cas si besoin
    Modem_Send_AT_Wait("AT+CGACT=1,1\r", "OK", 3000);

    // Petit temps de respiration pour le modem
    osDelay(2000);

    printf("--- Config Done ---\n");
}

/* Récupération de la qualité du signal (RSSI) */
int Modem_Get_Signal_Quality(void) {
	char* ptr;
	int rssi = -1;

	memset(modem_buffer, 0, sizeof(modem_buffer));

	if (Modem_Send_AT_Wait("AT+CSQ\r", "OK", 2000) == MODEM_OK) {
		ptr = strstr(modem_buffer, "+CSQ: ");
		if (ptr != NULL) sscanf(ptr + 6, "%d", &rssi);
	}
	return rssi;
}

/* Séquence complète d'initialisation du modem avec retries */
ModemStatus Modem_Init(void) {
	ModemStatus status = ERR_NOT_INITIALIZED;
	int tentative = 0;

	printf("Demarrage Modem:\n");
	HAL_Delay(5000);

	// 1. reveil par DTR
	gpio_Wakeup();

	// 2. Boucle d'initialisation
	do {
		tentative++;
		printf("   Initialisation Modem (Tentative %d/3)...\n", tentative);

		status = Modem_Init_Sequence();

		if (status == MODEM_OK) break;

		if (tentative < 3) {
			printf("\tEchec. Retry dans 5s...\n");
			// On ne peut pas hard-reset via PWRKEY, donc on attend juste.
			// On peut essayer un soft reset si l'UART répondait un peu
			HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CRESET\r", 10, 100);
			osDelay(5000);
		}
	} while (tentative < 3);

	if (status != MODEM_OK) {
		gpio_Sleep();
		printf("!! Echec critique !!\n");
		Error_Handler();
	}
	else {
		printf("Systeme fonctionnel.\n");
		Modem_Free_Send_Notif(&huart1, USER_FREE, CLE_API, "On ...");
		osDelay(10000);
		status = Modem_Set_Sleep_Mode(SLEEP_MODE_DTR);
		gpio_Sleep();
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
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	const osSemaphoreAttr_t mySemaphoreAlarm_attributes = { .name = "mySemaphoreAlarm" };
	mySemaphoreAlarm = osSemaphoreNew(1, 0, &mySemaphoreAlarm_attributes);

	if (mySemaphoreAlarm == NULL) Error_Handler();
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
	const osThreadAttr_t highAttr = { .name = "HighThread", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityHigh };
	const osThreadAttr_t lowAttr = { .name = "LowThread", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityBelowNormal };

	Modem_Init();

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
  HAL_GPIO_WritePin(MODEM_SLEEP_GPIO_Port, MODEM_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MODEM_SLEEP_Pin */
  GPIO_InitStruct.Pin = MODEM_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MODEM_SLEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
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
	if (GPIO_Pin == GPIO_PIN_0) {
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
		printf("ERREUR CRITIQUE, REDEMARRAGE...\n");
		// on a plus qu a reset le systeme ... ;(
		NVIC_SystemReset();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
