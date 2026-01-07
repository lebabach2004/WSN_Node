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
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char node_id[] = "0002"; 
volatile uint8_t  rx_byte;                    
volatile uint8_t  rx_buf[RX_BUF_SIZE];        
volatile uint16_t rx_idx = 0;
volatile bool line_ready = false;   
#define CMD_SEND_REQUEST   0x01  
#define CMD_SEND_OK        0x02  
#define CMD_SENSOR_DATA    0x03  
#define CMD_DATA_ACK       0x04  
#define CMD_CFG_REQUEST    0x05  
#define CMD_CFG_DATA       0x07  
#define CMD_CFG_NOCHANGE   0x06 
#define HEADER_BYTE        0xAA
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static float t = 20.0f;
static float h = 50.3f;
static float soil = 30.0f;
typedef struct {
    float Temp_Th;
    float Hum_Th;
		float Soil_Th;
		int period_sec;
    uint32_t magic;   
} node_config_t;
node_config_t node_config ;

float new_TempTh,new_HumTh,new_Soil;
int period ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float ReadSoil_ADC(void) {
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 20);
  uint16_t tmp= (uint16_t)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	float soil_hum = (4095 - tmp) * 100.0f / (4095 - 0);
	return soil_hum;
}

void LoRa_Sleep(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_SET);
}
void LoRa_Wakeup_And_Stabilize(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_RESET);   
	HAL_Delay(100);
}

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
		node_config.Hum_Th  = 50.0f;
		node_config.Soil_Th  = 30.0f;
		node_config.period_sec = 30;
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
	LoRa_Sleep();
	RTC_SetAlarm_AfterSecond(node_config.period_sec);
	HAL_SuspendTick();                
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	HAL_ResumeTick();      
  SystemClock_Config();
	LoRa_Wakeup_And_Stabilize();
}
void LoRa_UART_Send(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, buf, len, 1000);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_idx < RX_BUF_SIZE)
        {
            rx_buf[rx_idx++] = rx_byte;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
    }
}
// Cho 1 dong moi tu RX (toi '\n') voi timeout_ms
// copy vào buf (size maxlen), return: so byte (0 = timeout)
int LoRa_WaitLine(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
		uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
			  if (rx_idx >= len)
        {
         memcpy(buf, (const uint8_t*)rx_buf, len);
         rx_idx = 0;
         return len;
        }
    }
    return 0;  // timeout
}
#define MAX_SEND_RETRIES        50
#define SEND_RESP_TIMEOUT_MS    2000  // 2s cho OK
bool request_send_permission(void) { 
	char line[20]; 
	uint8_t p[4]; 
		p[0] = 0xAA; 
		uint16_t id = (uint16_t)strtol(node_id, NULL, 16); 
		p[1] = (id >> 8) & 0xFF; 
		p[2] = id & 0xFF;
		p[3] = CMD_SEND_REQUEST ;
	for (int attempt = 0; attempt < MAX_SEND_RETRIES; attempt++) 
	{ 
		LoRa_UART_Send(p,4); 
		uint8_t resp[4]; 
		int len= LoRa_WaitLine(resp,sizeof(resp),SEND_RESP_TIMEOUT_MS);
		if(len ==4){
			if( resp[0] == 0xAA && resp[3] == CMD_SEND_OK && resp[1] == p[1] && resp[2] == p[2]){
				return true;
			}			
		}
		HAL_Delay(200);
	}
	return false;
}
#define MAX_DATA_RETRIES        50
#define DATA_ACK_TIMEOUT_MS     2000  // 2s cho ACK

bool send_data_with_ack(float hum, float temp, float soil)
{
    char line[64];
		uint8_t frame[16];
    uint16_t id = (uint16_t)strtol(node_id, NULL, 16);
    frame[0] = 0xAA;
    frame[1] = (id >> 8) & 0xFF;
    frame[2] = id & 0xFF;
    frame[3] = CMD_SENSOR_DATA;
    memcpy((void*)&frame[4],  &hum,  4);
    memcpy((void*)&frame[8],  &temp, 4);
    memcpy((void*)&frame[12], &soil, 4);
    for (int attempt = 0; attempt < MAX_DATA_RETRIES; attempt++)
    {
			LoRa_UART_Send(frame,16);
			// 2) Cho ACK|id
			uint8_t r[4];
			int len = LoRa_WaitLine(r, 4, DATA_ACK_TIMEOUT_MS);
			if (len == 4)
      {
				if (r[0] == HEADER_BYTE && r[3] == CMD_DATA_ACK && r[1] == frame[1] && r[2] == frame[2]){
					return true;
				}
			}
        HAL_Delay(200); // retry
    }
    return false;
}
#define CFG_RESP_TIMEOUT_MS  2000
#define MAX_CFG_RETRIES      5
bool request_config_update(float *temp_th,float *hum_th,float *soil_th, int *period_sec){
	uint8_t q[4];
	uint16_t id = (uint16_t)strtol(node_id, NULL, 16);
  q[0] = HEADER_BYTE;
  q[1] = (id >> 8) & 0xFF;
  q[2] = id & 0xFF;
  q[3] = CMD_CFG_REQUEST;
	for (int attempt=0 ; attempt < MAX_CFG_RETRIES; attempt++)
	{
		LoRa_UART_Send(q,4);
		uint8_t r4[4];
		int len4 = LoRa_WaitLine(r4, 4 ,CFG_RESP_TIMEOUT_MS);
		if (len4 == 4 && r4[0] == HEADER_BYTE && r4[1] == q[1] && r4[2] == q[2]){
        if (r4[3] == CMD_CFG_NOCHANGE){
					return false; 
         }
         if (r4[3] == CMD_CFG_DATA){
						uint8_t r20[20];
            r20[0] = r4[0];
            r20[1] = r4[1];
            r20[2] = r4[2];
						r20[3] = r4[3];
            int len20 = LoRa_WaitLine(&r20[4], 16, CFG_RESP_TIMEOUT_MS);
						if (len20 == 16)
						{
								memcpy(temp_th,    &r20[4],  4);
								memcpy(hum_th,     &r20[8],  4);
								memcpy(soil_th,    &r20[12], 4);
								memcpy(period_sec, &r20[16], 4);
								return true;
						}
			}
		}
//		if(len>0){
//			if (r[0] == HEADER_BYTE && r[3] == CMD_CFG_NOCHANGE)
//				return false;
//			if (r[0] == HEADER_BYTE && r[3] == CMD_CFG_DATA && r[1] == q[1] && r[2] == q[2] ){
//				memcpy(temp_th, &r[4], 4);
//        memcpy(hum_th, &r[8], 4);
//        memcpy(soil_th, &r[12], 4);
//				memcpy(period_sec, &r[16], 4);
//				return true;
//			}
//		}
		HAL_Delay(200);
	}
	return false;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	Config_Init();
	AHT20_Init(&hi2c1);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte, 1);
	HAL_ADCEx_Calibration_Start(&hadc1); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		AHT20_Read(&hi2c1,&t,&h);
		soil = ReadSoil_ADC();
		// 1) Xin quyen gui
		bool can_send = request_send_permission();
		if(can_send){
			// 2) Gui DATA & cho ACK
			bool sent_ok = send_data_with_ack(h, t, soil);
			if(sent_ok){
//				float new_TempTh,new_HumTh;
				bool has_cfg=request_config_update(&new_TempTh,&new_HumTh,&new_Soil,&period);
				if(has_cfg){
					node_config.Temp_Th = new_TempTh;
          node_config.Hum_Th = new_HumTh;
					node_config.Soil_Th = new_Soil;
					node_config.period_sec = period;
					node_config.magic= CFG_MAGIC;
					Flash_SaveConfig(&node_config);
				}
			}
		}
		if( h >= node_config.Hum_Th || t>= node_config.Temp_Th ){
			HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin, GPIO_PIN_SET);			
			HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin, GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(GREEN_GPIO_Port,GREEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin, GPIO_PIN_RESET);
		}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_3|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA3 GREEN_Pin RED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GREEN_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
