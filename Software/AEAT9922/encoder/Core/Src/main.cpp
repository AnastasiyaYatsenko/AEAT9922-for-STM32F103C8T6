/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t READ =0x40; // read flag in command
uint8_t WRITE=0x00; // write flag in command

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

union Reg8Config {
	struct {
		uint8_t res :4;
		uint8_t dir :1;
		uint8_t hyst :3;
	};
	uint8_t bits;
};

Reg8Config reg8;

union Data16 {
	struct {
		uint8_t high;
		uint8_t low;
	};
	uint8_t bits;
};

union Data24 {
	struct {
		uint8_t high;
		uint8_t mid;
		uint8_t low;
	};
	uint8_t bits;
};


unsigned int error_flag = 0;
unsigned int error_parity = 0; // читання з пристрою пройшло з битою парністю
unsigned int error_reg = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned int parity(unsigned int n) {
	unsigned int b;
	b = n ^ (n >> 1);
	b = b ^ (b >> 2);
	b = b ^ (b >> 4);
	b = b ^ (b >> 8);
	b = b ^ (b >> 16);
	return b & 1;
}

unsigned long int spi_transfer16(uint8_t reg, uint8_t RW) {
	uint8_t high = 0;
	uint8_t low = 0;
   	uint8_t header = parity(reg|RW)<<7 | RW; //  в контрольну суму повинно входити ВСЕ! окрім біта контрольної суми
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // 1us
    HAL_SPI_TransmitReceive(&hspi1, &header, &high, 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi1, &reg, &low, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    return (high&0xff)<<8 | (low&0xff);
}

// Глючить і косячить :(
unsigned long int spi_transfer16_2(uint8_t reg, uint8_t RW) {
	uint8_t txdata16[2];
	uint8_t rxdata16[2];
    txdata16[0] = parity(reg|RW)<<7 | RW; //  в контрольну суму повинно входити ВСЕ! окрім біта контрольної суми
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // 1us
    HAL_SPI_TransmitReceive(&hspi1, txdata16, rxdata16, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    return (rxdata16[0]<<8) | rxdata16[1];
}


unsigned int spi_transfer24(uint8_t reg, uint8_t RW) {
	uint8_t high = 0;
	uint8_t mid  = 0;
	uint8_t low  = 0;
	uint8_t header = (parity(reg | RW) << 7) | RW;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
//	DWT_Delay_us(1);

	HAL_SPI_TransmitReceive(&hspi1, &header, &high, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1, &reg, &mid, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1, &header, &low, 1, HAL_MAX_DELAY);

//	DWT_Delay_us(1);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//	DWT_Delay_us(1);

	return (high&0xff)<<16 | (mid&0xff)<<8 | (low&0xff);
}


int setup_spi(){
	//HAL_SPI_Abort(&hspi1);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_MSEL_GPIO_Port, SPI1_MSEL_Pin, GPIO_PIN_SET);

    return 0;
}

unsigned long int  spi_read(uint8_t reg) {
    unsigned int data = 0;
    unsigned long int raw_data = 0;

//    printf("SPI READ %d\n",reg);
//    HAL_HalfDuplex_EnableTransmitter(&huart2);

//    HAL_UART_Transmit(&huart2, (uint8_t *)&reg, 1, 0xFFFF);
//    HAL_UART_Transmit(&huart2, &reg, 1, 10);
    HAL_UART_Transmit(&huart3, &reg, 1, 10);

    spi_transfer16(reg,READ);

    if (reg != 0x3f) { // єдиний регістр, який потребує 24-бітне читання
        raw_data = spi_transfer16(0x3f,READ); // сире читання, тут дані + прапор помилок + парність
        error_flag = raw_data&0x4000?1:0; // зберігаємо біт прапору помилок глобально
        // підраховуємо парність прийнятих даних включно з бітом парності і зберігаємо глобально
        error_parity = parity(raw_data); // зберігаємо біт прапору помилок глобально
        data = raw_data&0x3fff;

      } else {

        // відповідь 24-бітна
        raw_data = spi_transfer24(0x3f,READ); // сире читання, тут дані + прапор помилок + парність
        error_flag = raw_data&0x400000?1:0; // відокремлюємо біт прапору помилок
        error_parity = parity(raw_data);
        data = (raw_data&0x3fffff)>>4; // обрізаємо верхні 2 біти і зміщуємо на 4 біта вниз, отримуємо 24-2-4=18 біт розрядності
      }

      if (error_parity != 0) {      }
      if (error_flag != 0) {      }

	return data;
}


unsigned long int spi_write(uint8_t reg, uint8_t data) {
	//setup_spi();
    unsigned long int raw_data = 0;

	spi_transfer16(0x10, WRITE); // 80 10
	spi_transfer16(0xAB, WRITE); // 80 ab
	spi_transfer16(reg, WRITE); // 80/00 <reg>
	raw_data = spi_transfer16(data, WRITE); 	 // це дані для запису  80/00 <data>
	spi_transfer16(0x10, WRITE); // LOCK write 80 10
	spi_transfer16(0x00, WRITE); // 00 00
	error_parity = parity(raw_data); // рахуємо parity разом з отриманим бітом парності, воно ПОВ�?ННО дорівнювати нулю, інакше це і є збій контроля парності
	return raw_data & 0x3fff;
}

uint8_t write_resolution(uint8_t data) {
	reg8.bits = spi_read(0x08);
	if (data > 18 || data < 10)
		return(1);

	reg8.res = 18 - data; // значення роздільності наведено до бітової комбінація 0000..1000
	spi_write(0x08, reg8.bits);
	if (spi_read(0x08) != reg8.bits)
		return(0); // ERROR!     while (!write_resolution(r)) { delay_ms(1); }
    return(1); // OK
}

int esc_neutral() {

	//1.5 ms

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Stop_IT(&htim1);

	htim1.Instance->PSC = 83;
	htim1.Instance->ARR = 1500;
	htim1.Instance->CCR1 = 750;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);

	return 0;
}

int esc_forward(int speed) {

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Stop_IT(&htim1);

	htim1.Instance->PSC = 83;
	htim1.Instance->ARR = speed;
	htim1.Instance->CCR1 = speed / 2;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);

	return 0;
}

int esc_backward(int speed) {

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Stop_IT(&htim1);

	htim1.Instance->PSC = 83;
	htim1.Instance->ARR = speed;
	htim1.Instance->CCR1 = speed / 2;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);

	return 0;
}

unsigned int read_resolution() {
	reg8.bits = spi_read(8);
	return 18 - reg8.res; // бітова комбінація 0000..1000 наведена до значення роздільності
}

unsigned int set_zero() {
	unsigned int data = 0;
	spi_write(0x12, 0x00); // НЕДОКУМЕНТОВАНА фіча! Треба спочатку занулити регістр, а потім щось писати!
	// про всякий випадок
	HAL_Delay(50);
	uint8_t cde_prev = (spi_read(0x0c) & 0xff) << 10
			| (spi_read(0x0d) & 0xff) << 2 | (spi_read(0x0e) & 0xc0) >> 6;
	spi_write(0x12, 0x08);
	HAL_Delay(50); // Memory busy bit[8] address 0x22 will flag high for 40ms.
	data = spi_read(0x22);
	data = (data >> 2) & 0x03;
	uint8_t cde_new = (spi_read(0x0c) & 0xff) << 10
			| (spi_read(0x0d) & 0xff) << 2 | (spi_read(0x0e) & 0xc0) >> 6;
	if (data == 2) {
		if (cde_prev != cde_new) { // Порівнюємо значення регістру Zero Reset перед та після виконанням запису до регістру 0x12, якщо не змінився - то failed
			//  Serial.printf("Set zero is SUCCESSFULL\n");
			return 0; // OK
		} else {
			//  Serial.printf("Register 0x22 returns OK, but set zero is FAILED\n");
			return 2;
		}
	} else if (data == 3) {
		// Serial.printf("Set zero is FAILED\n");
		return 3;
	} else {
		//  Serial.printf("Set zero is STRANGE: %d\n",data);
		return 4;
	}
	return 1; // Not OK
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	DWT_Delay_Init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
    SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    setup_spi(); //спочатку піни потім ініціалізація сука
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_MSEL_GPIO_Port, SPI1_MSEL_Pin, GPIO_PIN_SET);

	//	write_resolution(18);
//	dataResAs=read_resolution();
//	HAL_Delay(1000);

//	  esc_neutral();
//	  DWT_Delay_ms(1000);
//
//	  esc_forward(1000);
//	  DWT_Delay_ms(500);
//
//	  esc_backward(2000);
//	  DWT_Delay_ms(500);

	float angle[9];
//	for (int i = 0; i < 20; i++) arr[i] = 0;
	// unsigned int dataResAs=0;
	//   set_zero();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

//		unsigned int res = read_resolution();
//        res = res+0;
/*	for(uint8_t r=11; r<=18; r++){
		write_resolution(r);
		DWT_Delay_us(100);
		unsigned long int dataResAs=read_resolution();
		DWT_Delay_us(200);
	}
*/
		uint8_t rr[9];
		for (int i = 0; i <= 8; i++) { // 0..8 => 10..18 bits
			write_resolution(i+10);
//			DWT_Delay_us(100);
			rr[i] = read_resolution();
//			arr[i] = rr;
			angle[i] = spi_read(0x3f)*360.0/262144.0;
			DWT_Delay_us(100);
		}
/*
		for (int i = 1; i < 10; i++) arr[i-1] = arr[i];
		arr[9] = spi_read(0x3f); //* 			360.0 / 262144.0
		for (int i = 0; i < 10; i++) {
//		HAL_Delay(1);
			arr[i] = spi_read(0x3f); //* 			360.0 / 262144.0
			HAL_Delay(10);
//			spi_read(0x15); //* 			360.0 / 262144.0
		}
*/
		HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
	DWT_Delay_ms(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_HalfDuplex_EnableTransmitter(&huart2);
  /* USER CODE END USART2_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_MSEL_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_MSEL_Pin */
  GPIO_InitStruct.Pin = SPI1_MSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_MSEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_Pin */
  GPIO_InitStruct.Pin = TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

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
