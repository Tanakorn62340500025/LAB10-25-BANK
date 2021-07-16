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
#include <math.h>
#include <stdio.h>   //sprintf
#include <string.h>   //strlen
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ADCin = 0;
uint64_t _micro = 0;
float bitvoltuint = 0;
float bitvoltuint1 = 0;
//12bit ของ dac
uint16_t dataOut = 0;
uint8_t DACConfig = 0b0011;
uint64_t timestamp = 0;
uint64_t timestamp1 = 0;
uint64_t timestamp2 = 0;
float a = 0;
float dutycycle = 0;
float squarecheck = 0;

float AngleInput =0;
float SineOutput =0;
double ChangeRate =0.0015;
float OutputAmp = 0;


float bitvolt = 0;
float hz = 1;
float sawtooth = 0;
float vhigh = 3.3;
float vlow = 0;
float bitvhigh = 0;
float bitvlow = 0;
float slopeup = 1;
float slopedown = 0;
float sinewave = 1;
float squarewave = 0;
float hzzero = 0;

char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };

enum state
{
	mainmenu = 0,
	mainmenuwait,
	sawtoothmenu,
	sawtoothmenuwait,
	sinewavemenu,
	sinewavemenuwait,
	squarewavemenu,
	squarewavemenuwait,
	frequencymenu,
	frequencymenuwait,
	vhighmenu,
	vhighmenuwait,
	vlowmenu,
	vlowmenuwait,
	slopemenu,
	slopemenuwait,
	dutycyclemenu,
	dutycyclemenuwait,


};

uint8_t state = mainmenu;
//ใน datasheet สิ่งที่เปลี่ยนคือ d0-d12
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();
void UARTRecieveAndResponsePolling();
int16_t UARTRecieveIT();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start(&htim3);
  	HAL_TIM_Base_Start_IT(&htim11);
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);
//เอาไว้ตรงนี้เพื่อให้มันทำงาน
  	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  	HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

  		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
  		{

  			sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
  			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
  		}


		bitvhigh = ((4095*vhigh)/3.3);
	    bitvlow = ((4095*vlow)/3.3);
	    if(sawtooth == 1)
	    {
				if(hz > 0)
				{

						//static uint64_t timestamp = 0;
						if (micros() - timestamp > 100) //100us = 10khz
						{
								timestamp = micros();
								a += 0.0001;
								if(slopeup == 1)
								{

										 bitvolt+=((bitvhigh-bitvlow)*hz)/10000;
										 if(bitvolt >= bitvhigh)
										 {
											 bitvolt = bitvlow;
										 }
									//	 bitvoltuint%=4096;
										 bitvoltuint = bitvolt;
										 dataOut = bitvoltuint;
										 if (hspi3.State == HAL_SPI_STATE_READY
													 && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
															 == GPIO_PIN_SET)
										 {
												 MCP4922SetOutput(DACConfig, dataOut);
										 }
								  }
								  else if(slopedown == 1)
								  {
									  	  bitvolt-=((bitvhigh-bitvlow)*hz)/10000;
										  if(bitvolt <= bitvlow)
										  {
											  bitvolt = bitvhigh;
										  }
										  bitvoltuint = bitvolt;
										  dataOut = bitvoltuint;
										  if (hspi3.State == HAL_SPI_STATE_READY
														 && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
																 == GPIO_PIN_SET)
										  {
												 MCP4922SetOutput(DACConfig, dataOut);
										  }
								  }


						 }
				}


	    }
	    else if(sinewave == 1)
	    {
				if(hz > 0)
	    		{
	    				if (micros() - timestamp1 >  100 )
						{
							  timestamp1 = micros();
							  //t = micros();
							 // A = 4095/2;
							 //dcgain = (4095/2);
							  OutputAmp = (bitvhigh-bitvlow)/2;
							  AngleInput += ChangeRate;
							  if (AngleInput > 2*M_PI)
							  {
									  AngleInput -= 2*M_PI;
							  }
							  else if (AngleInput < 0)
							  {
									  AngleInput += 2*M_PI;
							  }


							  SineOutput = OutputAmp*sin(2*(M_PI)*hz*(timestamp1-timestamp)/1000000.0)+((bitvhigh+bitvlow)/2);
							//  bitvolt = A*sin(2*(M_PI)*hz*t)+dcgain;
							//  bitvoltuint = bitvolt;
							  bitvoltuint1 = SineOutput;

							  dataOut = bitvoltuint1;
							  if (hspi3.State == HAL_SPI_STATE_READY
											 && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
													 == GPIO_PIN_SET)
							  {
									 MCP4922SetOutput(DACConfig, dataOut);
							  }
						}
	    		}


	    }
	    else if(squarewave == 1)
	    {
	    		if(hz > 0)
	    		{
	    					if (micros() - timestamp2 > (1/hz)*(dutycycle/100.0)*1000000 && squarecheck == 1)
							{
									timestamp2 = micros();
									dataOut = bitvlow;
									squarecheck = 0;
							}
							else if (micros() - timestamp2 > (1/hz)*((100-dutycycle)/100.0)*1000000 && squarecheck == 0)
							{
									timestamp2 = micros();
									dataOut = bitvhigh;
									squarecheck = 1;
							}
							  if (hspi3.State == HAL_SPI_STATE_READY
											 && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
													 == GPIO_PIN_SET)
							  {
									 MCP4922SetOutput(DACConfig, dataOut);
							  }

	    		}

	    }



	    switch(state)
	    {
	    		case mainmenu:
	  					sprintf(TxDataBuffer, "Main Menu\r\n0:Sawtooth\r\n1:Sine wave\r\n2:Square wave\r\nx:back\r\n");
	  					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = mainmenuwait;
	    				break;


	    		case mainmenuwait:
						switch(inputchar)
						{
	  							case '0':
	  								sawtooth = 1;
	  								sinewave = 0;
	  								squarewave = 0;
	  								state = sawtoothmenu;
	  								break;
	  							case '1':
	  								sawtooth = 0;
	  								sinewave = 1;
	  								squarewave = 0;
	  								timestamp = micros();   //เวลาเเรกสุดของการ gen sine
	  								state = sinewavemenu;
	  								break;
	  							case '2':
	  								sawtooth = 0;
	  								sinewave = 0;
	  								squarewave = 1;
	  								timestamp2 = micros();
	  								state = squarewavemenu;
	  								break;
	  							case 'x':
	  								state = mainmenu;
	  								break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

				case sawtoothmenu:
						sprintf(TxDataBuffer, "sawtooth\r\nf:frequency\r\nh:vhigh\r\nl:vlow\r\ns:slopeup/down\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = sawtoothmenuwait;
						break;

				case sawtoothmenuwait:
						switch(inputchar)
						{
								case 'f':
									state = frequencymenu;
									break;
								case 'h':
									state = vhighmenu;
									break;
								case 'l':
									state = vlowmenu;
									break;
								case 's':
									state = slopemenu;
									break;
								case 'x':
									state = mainmenu;
									break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

				case sinewavemenu:
						sprintf(TxDataBuffer, "sinewave\r\nf:frequency\r\nh:vhigh\r\nl:vlow\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = sinewavemenuwait;
						break;
				case sinewavemenuwait:
						switch(inputchar)
						{
								case 'f':
									state = frequencymenu;
									break;
								case 'h':
									state = vhighmenu;
									break;
								case 'l':
									state = vlowmenu;
									break;
								case 'x':
									state = mainmenu;
									break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

				case squarewavemenu:
						sprintf(TxDataBuffer, "square\r\nf:frequency\r\nh:vhigh\r\nl:vlow\r\nd:dutycycle\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = squarewavemenuwait;
						break;

				case squarewavemenuwait:
						switch(inputchar)
						{
								case 'f':
									state = frequencymenu;
									break;
								case 'h':
									state = vhighmenu;
									break;
								case 'l':
									state = vlowmenu;
									break;
								case 'd':
									state = dutycyclemenu;
									break;
								case 'x':
									state = mainmenu;
									break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

				case frequencymenu:
						sprintf(TxDataBuffer, "frequency\r\n+:add\r\n-:minus\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = frequencymenuwait;
						break;

				case frequencymenuwait:
						switch(inputchar)
						{
								case '+':
									if(hz < 10)
									{
										hz += 0.1;
									}
									break;
								case '-':
									if(hz > 0)
									{
										hz -= 0.1;
									}
									break;
								case 'x':
									state = mainmenu;
									break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

				case vhighmenu:
						sprintf(TxDataBuffer, "vhigh\r\n+:add\r\n-:minus\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = vhighmenuwait;
						break;
				case vhighmenuwait:
						switch(inputchar)
						{
								case '+':
									if(vhigh < 3.3)
									{
										vhigh += 0.1;
									}
									if(vhigh > 3.3)
									{
										vhigh = 3.3;
									}
									break;
								case '-':
									if(vhigh >= vlow)
									{
										if(vhigh > 0)
										{
											vhigh -= 0.1;
										}
									}
									break;
								case 'x':
									state = mainmenu;
									break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

				case vlowmenu:
						sprintf(TxDataBuffer, "vlow\r\n+:add\r\n-:minus\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = vlowmenuwait;
						break;
				case vlowmenuwait:
						switch(inputchar)
						{
										case '+':
											if(vlow <= vhigh)
											{
												if(vlow < 3.3)
												{
													vlow += 0.1;
												}
											}
											break;
										case '-':
											if(vlow > 0)
											{
												vlow -= 0.1;
											}
											if(vlow < 0)
											{
												vlow = 0;
											}
											break;
										case 'x':
											state = mainmenu;
											break;
										case -1:
											break;
										default:
											sprintf(TxDataBuffer, "Wrong\r\n");
											HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
											break;
						}
						break;

				case slopemenu:
						sprintf(TxDataBuffer, "slope\r\n0:slopedown\r\n1:slopeup\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = slopemenuwait;
						break;

				case slopemenuwait:
						switch(inputchar)
						{
								case '1':
									slopedown = 0;
									slopeup = 1;
									break;
								case '0':
									slopeup = 0;
									slopedown = 1;
									break;
								case 'x':
									state = mainmenu;
									break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

				case dutycyclemenu:
						sprintf(TxDataBuffer, "dutycycle\r\n+:add\r\n-:minus\r\nx:back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						state = dutycyclemenuwait;
						break;
				case dutycyclemenuwait:
						switch(inputchar)
						{
								case '+':
									if(dutycycle < 100)
									{
										dutycycle += 10;
									}
									break;
								case '-':
									if(dutycycle > 0)
									{
										dutycycle -= 10;
									}
									break;
								case 'x':
									state = mainmenu;
									break;
								case -1:
									break;
								default:
									sprintf(TxDataBuffer, "Wrong\r\n");
									HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
									break;
						}
						break;

//						case vhighmenu:




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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	//config คือ 4 bit บน เเละ dacoutput คือ 12 bit ล่าง
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}


void UARTRecieveAndResponsePolling() //function ส่งอะไรมาเเล้วรับอะไรไป
{
	char Recieve[32]={0};
//Received ตามไฟ�?ระพริบของ LD2ด้วย ไฟติด received ไฟดับ received เเละใน received สามารถ�?ดเเป้น key พิมพ์ได้
	HAL_UART_Receive(&huart2, (uint8_t*)Recieve, 4, 1000);
//ส่ง�?ลับไป
	//ถ้า�?ำหนดหลัง receive เป็น 4 เเปลว่าถ้าเรา�?ดเเป้นพิมครบ 4 ตัวมันจะออ�?จา�? function ทันทีเเละเเต่ละอันที่มัน received จะไม่มีทางเ�?ิน 4 ถ้ายัง�?ดไม่ครบ 4 ตัว มันจะ
	//receive ความเร็วตามป�?ติ เนื่องจา�?มันยังรอตัวอั�?ษรที่ป้อนเข้าไป เเต่ ถ้าครบเเล้วมันจะออ�?จา�? function ทันที มันจะขึ้น received เร็วมา�?
	sprintf(TxDataBuffer, "Received:[%s]\r\n", Recieve);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
//คือตัว printf มัน print ลงในตัวเเปรซั�?ตัวเเปรนึง
	//print ไว้ใน databuffer ซึ่งมันเป็น global
}


int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	//ประ�?าศให้ตัวเเปรนี้รั�?ษาค่าเดิมเอาไว้
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		//เพื่อให้ได้ตำเเหน่งของข้อมูล
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
	//HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer));
	//functionนี้คือจะไม่ขึ้น receive จน�?ว่าจะใส่ข้อมูลครบ ถ้าเรา�?ำหนดขนาด 32  มันจะ received �?็ต่อเมื่อมันครบ 32
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11)
	{
		_micro += 65535;
	}
}

inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
