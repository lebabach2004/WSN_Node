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
#include "aht20.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FLASH_CFG_ADDR  0x0800FC00
#define RX_BUF_SIZE 128
#define CFG_MAGIC  0xA5A55A5A
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char node_id[] = "0001"; 
volatile uint8_t  rx_byte;                    
volatile uint8_t  rx_buf[RX_BUF_SIZE];        
volatile uint16_t rx_idx = 0;
volatile bool line_ready = false;   
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
float t = 20.0f;
float h = 50.3f;
typedef struct {
    float Temp_Th;
    float Hum_Th;
    uint32_t magic;   
} node_config_t;
node_config_t node_config ;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Flash_SaveConfig(node_config_t *cfg){
	HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = FLASH_CFG_ADDR;
  erase.NbPages = 1;
  HAL_FLASHEx_Erase(&erase, &page_error);
	uint32_t addr = FLASH_CFG_ADDR;
	const uint32_t *p = (const uint32_t*)cfg;
	for (uint32_t i = 0; i < sizeof(node_config_t)/4; i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, p[i]);
		addr += 4;
	}
	HAL_FLASH_Lock();
}
bool Flash_LoadConfig(node_config_t *cfg)
{
	const node_config_t *flash_cfg = (const node_config_t*)FLASH_CFG_ADDR;
	if (flash_cfg->magic != CFG_MAGIC)
		return false;   
	memcpy(cfg, flash_cfg, sizeof(node_config_t));
  return true;
}
void Config_Init(void)
{
	if (!Flash_LoadConfig(&node_config)){
		node_config.Temp_Th = 30.0f;
		node_config.Hum_Th  = 40.0f;
		node_config.magic     = CFG_MAGIC;
		Flash_SaveConfig(&node_config);
   }
}

volatile uint8_t dem=0;
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
   dem++;
}
void RTC_SetAlarm_AfterSecond(uint32_t sec){
	RTC_TimeTypeDef nowTime;
	RTC_AlarmTypeDef sAlarm;
	uint32_t now, then;
	HAL_RTC_GetTime(&hrtc, &nowTime, RTC_FORMAT_BIN);
  now  = nowTime.Hours  * 3600UL + nowTime.Minutes * 60UL + nowTime.Seconds;
	then = (now + sec) % 86400UL;
	sAlarm.AlarmTime.Hours   = then / 3600UL;
  sAlarm.AlarmTime.Minutes = (then % 3600UL) / 60UL;
  sAlarm.AlarmTime.Seconds = then % 60UL;
	sAlarm.Alarm = RTC_ALARM_A;

  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
		Error_Handler();
  }
}
void EnterStop(void){
	RTC_SetAlarm_AfterSecond(20);
	HAL_SuspendTick();                
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	HAL_ResumeTick();      
  SystemClock_Config();
}
void LoRa_UART_Send(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1 )
    {
			uint8_t ch = rx_byte;
			if (!line_ready)  
			{
				if (rx_idx < RX_BUF_SIZE - 1)
				{
					rx_buf[rx_idx++] = ch;
					if (ch == '\n')   
					{
						rx_buf[rx_idx] = '\0';
						line_ready = true;
					}
				}
				else
				{
					rx_idx = 0;
				}
			}
			HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
    }
}
// Cho 1 dong moi tu RX (toi '\n') voi timeout_ms
// copy vào buf (size maxlen), return: so byte (0 = timeout)
int LoRa_WaitLine(char *buf, uint16_t maxlen, uint32_t timeout_ms)
{
		uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
			if (line_ready)
			{
				// copy safely
				uint16_t len = rx_idx;
				if (len >= maxlen) len = maxlen - 1;
				memcpy(buf, (const char*)rx_buf, len);
				buf[len]='\0';
				// reset trang thái cho dong sau
				rx_idx = 0;
				line_ready = false;
				return len;
			}
    }
    return 0;  // timeout
}
#define MAX_SEND_RETRIES        50
#define SEND_RESP_TIMEOUT_MS    2000  // 2s cho OK
bool request_send_permission(void)
{
    char line[64];

    for (int attempt = 0; attempt < MAX_SEND_RETRIES; attempt++)
    {
			// 1) Gui SEND|id
			char msg[32];
			snprintf(msg, sizeof(msg), "SEND|%s\r\n", node_id);
			LoRa_UART_Send(msg);
			// 2) Cho OK|id
			int len = LoRa_WaitLine(line, sizeof(line), SEND_RESP_TIMEOUT_MS);
			HAL_UART_Transmit(&huart2, (uint8_t*)line, strlen(line), 1000);
			if (len > 0)
			{
				// vi du: "OK|0001\r\n"
				if (strncmp(line, "OK|", 3) == 0)
				{
					char id[8] = {0};
					if (sscanf(line + 3, "%7s", id) == 1){
						if (strcmp(id, node_id) == 0){
							// Duoc phep gui Data
							return true;
						}
					}
				}
			}
      HAL_Delay(200);
    }
    return false; 
}
#define MAX_DATA_RETRIES        50
#define DATA_ACK_TIMEOUT_MS     2000  // 2s cho ACK

bool send_data_with_ack(float hum, float temp)
{
    char line[64];
    for (int attempt = 0; attempt < MAX_DATA_RETRIES; attempt++)
    {
			// 1) Gui DATA|id|Hum: .. Tmp: ..
			char msg[64];
			snprintf(msg, sizeof(msg), "DATA|%s|Hum: %.1f Tmp: %.1f\r\n", node_id, hum, temp);
			LoRa_UART_Send(msg);
			// 2) Cho ACK|id
			int len = LoRa_WaitLine(line, sizeof(line), DATA_ACK_TIMEOUT_MS);
			HAL_UART_Transmit(&huart2, (uint8_t*)line, strlen(line), 1000);
			if (len > 0)
      {
				// vi du: "ACK|0001\r\n"
				if (strncmp(line, "ACK|", 4) == 0){
					char id[8] = {0};
					if (sscanf(line + 4, "%7s", id) == 1){
						if (strcmp(id, node_id) == 0){
							return true;  // gui ok
						}
					}
				}
			}
        HAL_Delay(200); // retry
    }
    return false;
}
#define CFG_RESP_TIMEOUT_MS  2000
#define MAX_CFG_RETRIES      5
bool request_config_update(float *temp_th,float *hum_th){
	char line[64];
	for (int attempt=0 ; attempt < MAX_CFG_RETRIES; attempt++)
	{
		// Gui CONFIG?|id
		char msg[32];
		snprintf(msg, sizeof(msg),"CONFIG?|%s\r\n",node_id);
		LoRa_UART_Send(msg);
		int len = LoRa_WaitLine(line, sizeof(line),CFG_RESP_TIMEOUT_MS);
		if(len>0){
			if(strncmp(line,"CFG|",4)==0){
				char id[8] = {0};
				if (sscanf(line, "CFG|%7[^|]|TempTh: %f HumTh: %f", id, temp_th, hum_th) == 3){
					if (strcmp(id, node_id) == 0)
						return true; // có config m?i
				}
			}
			if(strncmp(line, "NO|", 3) == 0)
			{
				char id[8];
				if (sscanf(line, "NO|%7s", id) == 1)
				{
					if (strcmp(id, node_id) == 0)
						return false; // không có config
					}
				}
		}
		HAL_Delay(200);
	}
	return false;
}
float new_TempTh,new_HumTh;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	Config_Init();
	AHT20_Init(&hi2c1);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		AHT20_Read(&hi2c1,&t,&h);
		// 1) Xin quyen gui
		bool can_send = request_send_permission();
		if(can_send){
			// 2) Gui DATA & cho ACK
			bool sent_ok = send_data_with_ack(h, t);
			if(sent_ok){
//				float new_TempTh,new_HumTh;
				bool has_cfg=request_config_update(&new_TempTh,&new_HumTh);
				if(has_cfg){
					node_config.Temp_Th = new_TempTh;
          node_config.Hum_Th = new_HumTh;
					node_config.magic= CFG_MAGIC;
					Flash_SaveConfig(&node_config);
				}
			}
		}
//		HAL_Delay(10000);
		EnterStop();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
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

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
