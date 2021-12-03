/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint8_t READ			=	0x03;
const uint8_t CHIP_ERASE	=	0x60;
const uint8_t BYTE_PROG		=	0x02;
const uint8_t RDSR			=	0x05;
const uint8_t EWSR			=	0x50;
const uint8_t WRSR			=	0x01;
const uint8_t WREN			=	0x06;
const uint8_t WRDI			=	0x04;
const uint8_t AAI			=	0xAD;
const uint8_t EBSY			=	0x70;
const uint8_t DBSY			=	0x80;
#define STR_NUM		10
#define STR_LEN		100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
enum {
	Read,
	Write
};

enum {
	ON,
	OFF
};

uint32_t StartAddress;

uint8_t time_capsule[STR_NUM][STR_LEN] = {
		"From: Artem Shyshko, mr.artemshishko18@gmail.com\r",
		"Mentor: Daniil Ruban, daniil.ruban@globallogic.com\r",
		"Date: 03.12.21\r",
		"TIME CAPSULE              \r",
		"Through the darkness      \r",
		"Of future past            \r",
		"The magician longs to see \r",
		"One chance out            \r",
		"Between two worlds        \r",
		"Fire walk with me         \r",
};

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, 10);
  return len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void CE(_Bool State);
void WriteStatusRegister(void);
void WriteEnable(void);
void WriteDisable(void);
void EraseMem(void);
void AAIEnable(void);
void AAIDisable(void);
uint8_t ReadStatusRegister(void);
uint8_t AddressByte(uint8_t byte, uint32_t adr);
void WaitForBuisyBit(void);
void WriteStringAAI(uint32_t adr, uint8_t string[], uint16_t lenght);
void WriteTimeCapsule(uint8_t info[STR_NUM][STR_LEN]);
void ReadTimeCapsule(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CE(_Bool State){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, State);
}

void WriteStatusRegister(){
	CE(ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EWSR, 1, 100);
	CE(OFF);

	uint8_t tx[2] = {WRSR, 0x00};
	CE(ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &tx, 2, 100);
	CE(OFF);
}

void WriteEnable(){
	CE(ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WREN, 1, 100);
	CE(OFF);
}

void WriteDisable(){
	CE(ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WRDI, 1, 100);
	CE(OFF);
}

void EraseMem(void){
	CE(OFF);

	WriteStatusRegister();

	WriteEnable();

	CE(ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &CHIP_ERASE, 1, 100);
	CE(OFF);

	WriteDisable();
}

void AAIEnable(){
	WriteStatusRegister();

	CE(ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &EBSY, 1, 100);
	CE(OFF);

	WriteEnable();
}

void AAIDisable(){
	CE(ON);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &DBSY, 1, 100);
	CE(OFF);
}

uint8_t ReadStatusRegister(void){
  uint8_t status[2];
  uint8_t buffer[2] = {RDSR, 0x00};
  CE(ON);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &buffer, (uint8_t*) &status, 2, 100);
  CE(OFF);
  return status[1];
}

uint8_t AddressByte(uint8_t byte, uint32_t adr){
  uint8_t temp;
  switch(byte)
  {
    case 1:
      temp = (uint8_t)(adr >> 16);
      return temp;
      break;
    case 2:
      temp = (uint8_t)(adr >> 8);
      return temp;
      break;
    case 3:
      temp = (uint8_t)adr;
      return temp;
      break;
  }
  return 0;
}

void WaitForBuisyBit(void){
  CE(ON);
  while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
  CE(OFF);
}

void WriteStringAAI(uint32_t adr, uint8_t string[], uint16_t lenght){
  AAIEnable();

  CE(ON);
  uint8_t tx[] = { AAI, AddressByte(1, adr), AddressByte(2, adr), AddressByte(3, adr), string[0], string[1]};
  HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), 100);
  CE(OFF);
  WaitForBuisyBit();

  for (uint8_t i = 2; i <= lenght; i += 2)
  {
    CE(ON);
    uint8_t tx[] = { AAI, string[i], string[i+1]};
    HAL_SPI_Transmit(&hspi1, (uint8_t*) &tx, sizeof(tx), 100);
    CE(OFF);

    WaitForBuisyBit();
  }

  WriteDisable();

  AAIDisable();
}

void WriteTimeCapsule(uint8_t info[STR_NUM][STR_LEN]){

	for(uint8_t i = 0; i < STR_NUM; i++)
    	{
			uint8_t CurrentStringBuffer[STR_LEN] = {0};
			strcpy( (char*)  CurrentStringBuffer,(char*)  &info[i][0]);
			WriteStringAAI(StartAddress, (uint8_t*) &CurrentStringBuffer, STR_LEN);
			StartAddress += 4096;
    	}

	StartAddress = 0x00;
}

void ReadTimeCapsule(void){
  uint8_t Buff[STR_LEN] = {0};
  for(uint8_t i = 0; i < STR_NUM; i++)
  {
    uint8_t ReadCommand[4] = {READ, AddressByte(1, StartAddress), AddressByte(2, StartAddress), AddressByte(3, StartAddress)};
    CE(ON);
    HAL_SPI_Transmit(&hspi1, ReadCommand, sizeof(ReadCommand), 100);
    HAL_SPI_Receive(&hspi1, Buff, sizeof(Buff), 100);
    CE(OFF);
    while (ReadStatusRegister() & 0x01) {};
    for(uint8_t j = 0; j < STR_LEN; j++)
      if(Buff[j] == 0xFF)	Buff[j] = 0x00;
    HAL_UART_Transmit(&huart3, Buff, sizeof(Buff), 100);
    HAL_UART_Transmit(&huart3, (uint8_t*) "\n\r", 2, 50);
    StartAddress += 4096;
  }
  StartAddress = 0x00;
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_StatusTypeDef status ;
  uint16_t val;

  while (1)
  {
	  status = HAL_UART_Receive(&huart3, (uint8_t*) &val, 1, 10);
	  if (status == HAL_OK) {
	          switch (val) {
	            case '1':
	              EraseMem();
	              HAL_UART_Transmit(&huart3, (uint8_t*) "Entire memory was erased\r\n\n", 27, 50);
	              break;
	            case '2':
	              EraseMem();
	                WriteTimeCapsule(time_capsule);
	                HAL_UART_Transmit(&huart3, (uint8_t*) "The time capsule was recorded\r\n\n",32, 50);
	              break;
	            case '3':
	              HAL_UART_Transmit(&huart3, (uint8_t*) "The time capsule:\r\n",19, 50);
	              ReadTimeCapsule();
	              HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n",2, 50);
	              break;
	            default:
	              break;
	            }
	          }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
