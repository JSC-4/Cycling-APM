/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

#include "fatfs_sd.h"
#include "dfr_no2_co.h"
#include "particulate_matter.h"
#include "sht40.h"
#include "ds3231.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t warmUpTimer; //delete
char str1[80];  // delete

FATFS fs;	// File system
FIL fil;	// File
FRESULT fresult; // To store results
char buffer[1024]; // To store data

UINT br, bw; // File read/write count

/* Capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/* Registers for interfacing with the sensor */

void send_uart(char *string) {
	uint8_t len = strlen(string);

	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 2000); // Transmit in blocking mode
}

/* To find the size of the data in the buffer */
int bufSize(char *buf) {
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

/* To clear the buffer */
void bufClear(void) {

	for (int i = 0; i < 1024; i++) {
		buffer[i] = 0;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	dfr_data dfr_s;
	pm_data pm_s;
	sht40_data sht40_s;
	ds3231_data ds3231_s;

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM16_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */
	dfr_initialise(&dfr_s, &hi2c1);
	pm_initialise(&pm_s, &hi2c1);
	sht40_initialise(&sht40_s, &hi2c1);
	ds3231_initialise(&ds3231_s, &hi2c1);

//	dfrWakeUp(&dfr_s);

	HAL_StatusTypeDef status;
	char str[80];

	// Start Timer
//	HAL_TIM_Base_Start_IT(&htim16);
//	while (warmUpTimer <= 10)
//		;
//	HAL_TIM_Base_Stop_IT(&htim16);
	/* Mount SD Card */

	fresult = f_mount(&fs, "/", 1);
	if (fresult == FR_OK) {
		send_uart("SD card mounted successfully ...\n\r");
	} else {
		send_uart("error in mounting sd card ... \n\r");
	}

	/***** Card Capacity Details ********/

	/* Check free space */
	f_getfree("", &free_space, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "SD Card Total Size: \t%lu\n\r", total);
	send_uart(buffer);
	bufClear();
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	send_uart(buffer);

//	/* Open a file to write / create a file if it doesn't exist */
//	fresult = f_open(&fil, "Sensor.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//
//	/* Writing text */
//	fresult = f_puts("This data is from the First FILE\n\r", &fil);
//
//	/* Close file */
//	f_close(&fil);
//
//	send_uart("File.txt created and the data is written \n\r");
//
//	/* Open file to read */
//	fresult = f_open(&fil, "file1.txt", FA_READ);
//
//	/* Read string from the file */
//	f_gets(buffer, f_size(&fil), &fil);
//
//	send_uart(buffer);
//
//	/* Close file */
//	f_close(&fil);
//
//	bufClear();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		status = getTime(&ds3231_s);
		if (status == HAL_ERROR) {
			send_uart("ERROR\n\r");
		}
			else{
				sprintf(str, "%02d:%02d:%02d\r\n", ds3231_s.minute, ds3231_s.hour, ds3231_s.second);
				send_uart(str);
			}


//		getGasData(&dfr_s);
//		pm_read(&pm_s);
//		status = sht40_read(&sht40_s);
//		if (status == HAL_TIMEOUT) {
//			send_uart("Timeout\n\r");
//		} else if (status == HAL_ERROR) {
//			send_uart("Error\n\r");
//		} else if (status == HAL_OK) {
//			sprintf(str, "PM2.5: %d\t dfr: %d\t T: %lf\t H: %lf\r\n",
//					pm_s.pm25_env, dfr_s.r0_ox, sht40_s.temperature,
//					sht40_s.humidity);
//			send_uart(str);
//		}
//		HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
//		  /* Open a file to write */
//		  fresult = f_open(&fil, "sensor.txt", FA_OPEN_ALWAYS | FA_WRITE);
//
//		  fresult = f_lseek(&fil, f_size(&fil));
//
//		  /* Writing text */
//		  fresult = f_puts(str, &fil);
//
//		  /* Close file */
//		  f_close(&fil);

			HAL_Delay(1000);

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

		/** Configure the main internal regulator output voltage
		 */
		if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
				!= HAL_OK) {
			Error_Handler();
		}

		/** Configure LSE Drive Capability
		 */
		HAL_PWR_EnableBkUpAccess();
		__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

		/** Initializes the RCC Oscillators according to the specified parameters
		 * in the RCC_OscInitTypeDef structure.
		 */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
				| RCC_OSCILLATORTYPE_MSI;
		RCC_OscInitStruct.LSEState = RCC_LSE_ON;
		RCC_OscInitStruct.MSIState = RCC_MSI_ON;
		RCC_OscInitStruct.MSICalibrationValue = 0;
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
		RCC_OscInitStruct.PLL.PLLM = 1;
		RCC_OscInitStruct.PLL.PLLN = 40;
		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
		RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
		RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/** Initializes the CPU, AHB and APB buses clocks
		 */
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
				| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4)
				!= HAL_OK) {
			Error_Handler();
		}

		/** Enable MSI Auto calibration
		 */
		HAL_RCCEx_EnableMSIPLLMode();
	}

	/**
	 * @brief I2C1 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_I2C1_Init(void) {

		/* USER CODE BEGIN I2C1_Init 0 */

		/* USER CODE END I2C1_Init 0 */

		/* USER CODE BEGIN I2C1_Init 1 */

		/* USER CODE END I2C1_Init 1 */
		hi2c1.Instance = I2C1;
		hi2c1.Init.Timing = 0x10909CEC;
		hi2c1.Init.OwnAddress1 = 0;
		hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c1.Init.OwnAddress2 = 0;
		hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
			Error_Handler();
		}

		/** Configure Analogue filter
		 */
		if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
				!= HAL_OK) {
			Error_Handler();
		}

		/** Configure Digital filter
		 */
		if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN I2C1_Init 2 */

		/* USER CODE END I2C1_Init 2 */

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
		hspi1.Init.Direction = SPI_DIRECTION_2LINES;
		hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi1.Init.NSS = SPI_NSS_SOFT;
		hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
		hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi1.Init.CRCPolynomial = 7;
		hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
		if (HAL_SPI_Init(&hspi1) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN SPI1_Init 2 */

		/* USER CODE END SPI1_Init 2 */

	}

	/**
	 * @brief TIM16 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM16_Init(void) {

		/* USER CODE BEGIN TIM16_Init 0 */

		/* USER CODE END TIM16_Init 0 */

		/* USER CODE BEGIN TIM16_Init 1 */

		/* USER CODE END TIM16_Init 1 */
		htim16.Instance = TIM16;
		htim16.Init.Prescaler = 8000 - 1;
		htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim16.Init.Period = 10000 - 1;
		htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim16.Init.RepetitionCounter = 0;
		htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM16_Init 2 */

		/* USER CODE END TIM16_Init 2 */

	}

	/**
	 * @brief USART2 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_USART2_UART_Init(void) {

		/* USER CODE BEGIN USART2_Init 0 */

		/* USER CODE END USART2_Init 0 */

		/* USER CODE BEGIN USART2_Init 1 */

		/* USER CODE END USART2_Init 1 */
		huart2.Instance = USART2;
		huart2.Init.BaudRate = 115200;
		huart2.Init.WordLength = UART_WORDLENGTH_8B;
		huart2.Init.StopBits = UART_STOPBITS_1;
		huart2.Init.Parity = UART_PARITY_NONE;
		huart2.Init.Mode = UART_MODE_TX_RX;
		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(&huart2) != HAL_OK) {
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
	static void MX_GPIO_Init(void) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pin : SD_CS_Pin */
		GPIO_InitStruct.Pin = SD_CS_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : LD3_Pin */
		GPIO_InitStruct.Pin = LD3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	}

	/* USER CODE BEGIN 4 */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if (htim == &htim16) {
			sprintf(str1, "Seconds passed: %d\r\n", warmUpTimer++);
			send_uart(str1);

		}
	}
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
