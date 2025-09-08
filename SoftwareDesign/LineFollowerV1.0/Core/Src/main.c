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
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 	1
#define FALSE 	0
// NeoPixel configuration
#define NUM_PIXELS 8
#define PIXEL_BYTES 3  // RGB
#define SPI_BYTES_PER_BIT 3  // Each bit encoded as 3 SPI bytes
#define TOTAL_BYTES (NUM_PIXELS * PIXEL_BYTES * 8 * SPI_BYTES_PER_BIT)
#define RESET_BYTES 50  // Reset pulse (>50μs)
#define CODE0 0b11100000
#define CODE1 0b1111000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint16_t IR_buff[4][8]={0};
volatile uint8_t IR_Readed = FALSE;
volatile uint8_t IR_DigValue = 0;
volatile uint64_t pressTime=0;

 //Color structure
typedef struct {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} RGB_Color;
uint8_t spi_buffer[TOTAL_BYTES + RESET_BYTES];
RGB_Color pixels[NUM_PIXELS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Forwrad(uint16_t RSpeed,uint16_t LSpeed);
uint32_t Flash_Write_Data (uint32_t StartSectorAddress, volatile uint16_t *Data);
void NeoPixel_SetPixel(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue);
void NeoPixel_Show(void);
void NeoPixel_Clear(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Forwrad(uint16_t RSpeed,uint16_t LSpeed)
{
	  if((LSpeed <1200) && (RSpeed <1200))
	  {
		  TIM1->CCR1 = 0;
		  TIM1->CCR3 = 0;
		  TIM1->CCR2 = RSpeed;
		  TIM1->CCR4 = LSpeed;
		  //Turn OFF the Blue LED
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	  }
	  else
	  {
		  //Turn ON the Blue LED
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  }
}
void LineFollowerRun(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)IR_buff, 8);
	while(IR_Readed != TRUE);
	IR_DigValue = ((IR_buff[0][7]>IR_buff[3][7])<<7)|((IR_buff[0][6]>IR_buff[3][6])<<6)|((IR_buff[0][5]>IR_buff[3][5])<<5)|((IR_buff[0][4]>IR_buff[3][4])<<4)|((IR_buff[0][3]>IR_buff[3][3])<<3)|((IR_buff[0][2]>IR_buff[3][2])<<2)|((IR_buff[0][1]>IR_buff[3][1])<<1)|(IR_buff[0][0]>IR_buff[3][0]);
	__NOP();
	switch(IR_DigValue)
	{
	case 0b11111111:
		//LineFollowerStop();
	}
}
void LineFollowerCalibration(void)
{
	  uint16_t i=0;
	  IR_buff[2][0]= 4095;
	  IR_buff[2][1]= 4095;
	  IR_buff[2][2]= 4095;
	  IR_buff[2][3]= 4095;
	  IR_buff[2][4]= 4095;
	  IR_buff[2][5]= 4095;
	  IR_buff[2][6]= 4095;
	  IR_buff[2][7]= 4095;
	  for(i =0 ;i<10; i++)
	  {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  HAL_Delay(100);
	  }
	  Forwrad(1000,0);
	  for(i =0 ;i<65535; i++)
	  {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  IR_Readed = FALSE;
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)IR_buff, 8);
		  while(IR_Readed != TRUE);
		  //Get the maximum ADC values
		  IR_buff[1][0]= (IR_buff[0][0]>IR_buff[1][0])? IR_buff[0][0] : IR_buff[1][0];
		  IR_buff[1][1]= (IR_buff[0][1]>IR_buff[1][1])? IR_buff[0][1] : IR_buff[1][1];
		  IR_buff[1][2]= (IR_buff[0][2]>IR_buff[1][2])? IR_buff[0][2] : IR_buff[1][2];
		  IR_buff[1][3]= (IR_buff[0][3]>IR_buff[1][3])? IR_buff[0][3] : IR_buff[1][3];
		  IR_buff[1][4]= (IR_buff[0][4]>IR_buff[1][4])? IR_buff[0][4] : IR_buff[1][4];
		  IR_buff[1][5]= (IR_buff[0][5]>IR_buff[1][5])? IR_buff[0][5] : IR_buff[1][5];
		  IR_buff[1][6]= (IR_buff[0][6]>IR_buff[1][6])? IR_buff[0][6] : IR_buff[1][6];
		  IR_buff[1][7]= (IR_buff[0][7]>IR_buff[1][7])? IR_buff[0][7] : IR_buff[1][7];
		  //Get the minimum ADC values
		  IR_buff[2][0]= (IR_buff[0][0]<IR_buff[2][0])? IR_buff[0][0] : IR_buff[2][0];
		  IR_buff[2][1]= (IR_buff[0][1]<IR_buff[2][1])? IR_buff[0][1] : IR_buff[2][1];
		  IR_buff[2][2]= (IR_buff[0][2]<IR_buff[2][2])? IR_buff[0][2] : IR_buff[2][2];
		  IR_buff[2][3]= (IR_buff[0][3]<IR_buff[2][3])? IR_buff[0][3] : IR_buff[2][3];
		  IR_buff[2][4]= (IR_buff[0][4]<IR_buff[2][4])? IR_buff[0][4] : IR_buff[2][4];
		  IR_buff[2][5]= (IR_buff[0][5]<IR_buff[2][5])? IR_buff[0][5] : IR_buff[2][5];
		  IR_buff[2][6]= (IR_buff[0][6]<IR_buff[2][6])? IR_buff[0][6] : IR_buff[2][6];
		  IR_buff[2][7]= (IR_buff[0][7]<IR_buff[2][7])? IR_buff[0][7] : IR_buff[2][7];
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }
	  IR_buff[3][0]= IR_buff[2][0]+(IR_buff[1][0]-IR_buff[2][0])/2;
	  IR_buff[3][1]= IR_buff[2][1]+(IR_buff[1][1]-IR_buff[2][1])/2;
	  IR_buff[3][2]= IR_buff[2][2]+(IR_buff[1][2]-IR_buff[2][2])/2;
	  IR_buff[3][3]= IR_buff[2][3]+(IR_buff[1][3]-IR_buff[2][3])/2;
	  IR_buff[3][4]= IR_buff[2][4]+(IR_buff[1][4]-IR_buff[2][4])/2;
	  IR_buff[3][5]= IR_buff[2][5]+(IR_buff[1][5]-IR_buff[2][5])/2;
	  IR_buff[3][6]= IR_buff[2][6]+(IR_buff[1][6]-IR_buff[2][6])/2;
	  IR_buff[3][7]= IR_buff[2][7]+(IR_buff[1][7]-IR_buff[2][7])/2;
	  Flash_Write_Data(0x08020000,&IR_buff[3][0]);
	  Forwrad(0,0);
}

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, volatile uint16_t *Data)
{

	int sofar=0;
	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();
	  /* Erase the user Flash area */
	  FLASH_Erase_Sector(5U, FLASH_VOLTAGE_RANGE_3);

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	   while (sofar<8)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, StartSectorAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartSectorAddress += 2;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}
void Flash_Read_Data (uint32_t StartSectorAddress,volatile uint16_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{
		*RxBuf = *(__IO uint16_t *)StartSectorAddress;
		StartSectorAddress += 2;
		RxBuf++;
		if (!(numberofwords--)) break;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//  while(1)
//  {
//  HAL_GPIO_TogglePin(IR_ON_GPIO_Port, IR_ON_Pin);
//  HAL_Delay(1000);
//  }
  Flash_Read_Data (0x08020000, &IR_buff[3][0], 8);

  HAL_GPIO_WritePin(IR_ON_GPIO_Port, IR_ON_Pin,1);
  /* Start ADC conversion with DMA */
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)IR_buff, 8);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  NeoPixel_SetPixel(0, 255, 0, 0);
  NeoPixel_Show();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0)
	  {
		  HAL_Delay(300);
		  pressTime =HAL_GetTick()+3000;
		  while(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0)
		  {
			  if(pressTime < HAL_GetTick())
				  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
		  }
		  if(pressTime < HAL_GetTick())
		  {
			  LineFollowerCalibration();
		  }
		  else
		  {
			  LineFollowerRun();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 8;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_ON_GPIO_Port, IR_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EEP_GPIO_Port, EEP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_Pin IR_ON_Pin */
  GPIO_InitStruct.Pin = LED_Pin|IR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EEP_Pin */
  GPIO_InitStruct.Pin = EEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULT_Pin */
  GPIO_InitStruct.Pin = ULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ULT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void NeoPixel_SetPixel(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue)
{
    if (pixel >= NUM_PIXELS) return;

    pixels[pixel].red = red;
    pixels[pixel].green = green;
    pixels[pixel].blue = blue;
}

void NeoPixel_Show(void)
{
    uint16_t buffer_idx = 0;

    // Convert pixel data to SPI timing patterns
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
        uint8_t color_bytes[3] = {pixels[pixel].green, pixels[pixel].red, pixels[pixel].blue};

        for (int byte = 0; byte < 3; byte++) {
            for (int bit = 7; bit >= 0; bit--) {
                if (color_bytes[byte] & (1 << bit)) {
                    // Bit is 1
                    spi_buffer[buffer_idx++] = CODE1;
                } else {
                    // Bit is 0
                    spi_buffer[buffer_idx++] = CODE0;
                }
            }
        }
    }

    // Add reset pulse (low for >50μs)
    for (int i = 0; i < RESET_BYTES; i++) {
        spi_buffer[buffer_idx++] = 0x00;
    }

    // Transmit via SPI with DMA
    HAL_SPI_Transmit_DMA(&hspi2, spi_buffer, buffer_idx);

    // Wait for transmission to complete
    while (hspi2.State == HAL_SPI_STATE_BUSY_TX) {
        // Could add timeout here
    }
}

void NeoPixel_Clear(void)
{
    for (int i = 0; i < NUM_PIXELS; i++) {
        pixels[i].red = 0;
        pixels[i].green = 0;
        pixels[i].blue = 0;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
__NOP();
//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)IR_buff, 8);
IR_Readed = TRUE;
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
