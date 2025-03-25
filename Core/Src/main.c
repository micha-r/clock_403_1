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

int fl = 0;

struct time {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;

	int s1, s2;
	int m1, m2;
	int h1, h2;
} time;

const uint8_t cif_clear = 0b11111111;

const uint8_t cif[10] = { 0b10000100, 0b11110101, 0b01001100, 0b01100100, 0b00110101, 0b00100110, 0b00000110, 0b11110100, 0b00000100, 0b00100100 }; // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
const uint8_t cif_dp[10] = { 0b10000000, 0b11110001, 0b01001000, 0b01100000, 0b00110001, 0b00100010, 0b00000010, 0b11110000, 0b00000000, 0b00100000 }; // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

const uint8_t cif_invers[10] = { 0b10000100, 0b11110101, 0b11001000, 0b11100000, 0b10110001, 0b10100010, 0b10000010, 0b11110100, 0b10000000, 0b10100000 }; // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
const uint8_t cif_invers_dp[10] = { 0b00000100, 0b01110101, 0b01001000, 0b01100000, 0b00110001, 0b00100010, 0b00000010, 0b01110100, 0b00000000, 0b00100000 }; // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

uint8_t disp[6] = { 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111 };

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

#define DS3231_ADDRESS 0xD0

#define cs_reset() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define cs_set() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define cs_strob() cs_reset();cs_set()

#define disp_clear() HAL_SPI_Transmit(&hspi1, (uint8_t*) &cif_clear, 1, 100); cs_strob();

#define data_reset() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define data_set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define data_strob() data_reset();data_set()

#define clock_reset() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define clock_set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define clock_strob() clock_reset();clock_set()

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void write_data() {
	cs_reset();
	HAL_SPI_Transmit(&hspi1, (uint8_t*) disp, sizeof(disp), HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	clock_strob();
	cs_set();
	HAL_Delay(1000);
}

// Function to convert normal decimal numbers to binary-coded decimal (BCD)
uint8_t decToBcd(int val) {
	return (uint8_t) ((val / 10 * 16) + (val % 10));
}

// Function to convert binary-coded decimal (BCD) to normal decimal numbers
int bcdToDec(uint8_t val) {
	return (int) ((val / 16 * 10) + (val % 16));
}

// Function to set the time in DS3231
void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom,
		uint8_t month, uint8_t year) {
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

int lenNumLL(int value) {
	int len = 0;

	do {
		value /= 10;
		len++;
	} while (value);

	return len;
}

int get_data_1(int n) {

	int d1 = 0;
	int d2 = 0;

	if (lenNumLL(n) == 2) {
		d2 = n % 10;
		d1 = (n - d2) / 10;
	} else {
		d1 = 0;
	}

	return d1;
}

int get_data_2(int n) {

	int d2 = 0;

	if (lenNumLL(n) == 2) {
		d2 = n % 10;
	} else {
		d2 = n;
	}

	return d2;
}

// Function to get the time from DS3231
void Get_Time(void) {
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);

	time.h1 = get_data_1(time.hour);
	time.h2 = get_data_2(time.hour);

	time.m1 = get_data_1(time.minutes);
	time.m2 = get_data_2(time.minutes);

	time.s1 = get_data_1(time.seconds);
	time.s2 = get_data_2(time.seconds);

}

// Function to get the temperature from DS3231
float Get_Temp(void) {
	uint8_t temp[2];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x11, 1, temp, 2, 1000);
	return ((temp[0]) + (temp[1] >> 6) / 4.0);
}

// Function to force temperature conversion in DS3231
void force_temp_conv(void) {
	uint8_t status = 0;
	uint8_t control = 0;
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x0F, 1, &status, 1, 100); // Read status register
	if (!(status & 0x04))  // Check if BSY bit is not set
	{
		HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100); // Read control register
		HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x0E, 1,
				(uint8_t*) (control | (0x20)), 1, 100); // Write modified control register with CONV bit as '1'
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	Get_Time();

	disp[0] = cif[time.s2]; //6

	if (fl == 1) {

		disp[1] = cif_invers_dp[time.s1]; //5
		disp[2] = cif_dp[time.m2];        //4

		disp[3] = cif_invers_dp[time.m1]; //3
		disp[4] = cif_dp[time.h2];        //2
		fl = 0;

	} else {

		disp[1] = cif_invers[time.s1]; //5
		disp[2] = cif[time.m2];        //4

		disp[3] = cif_invers[time.m1]; //3
		disp[4] = cif[time.h2];        //2
		fl = 1;
	}

	disp[5] = cif[time.h1];           //1

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	/* USER CODE BEGIN 2 */

	cs_reset();
//	int cf = 9;
//	disp[5] = cif[cf];
//
//	disp[1] = cif_invers[cf];   //4
//	disp[3] = cif_invers[cf];   //2
//	write_data();

	//disp_clear();

	//Set_Time(00, 04, 13, 5, 3, 1, 19);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		write_data();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
