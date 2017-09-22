/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
//#include <tcp_com.h>
#include <MPU6050.h>
#include <rover_gps.h>
#include <tm_stm32f4_bmp180.h>


#define INDENT_SPACES "  "

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//IMU VARS
imu_data_t data_pack;
bool imu_data_ready = false;
uint_fast32_t g_imu_tick = 0;
uint_fast32_t g_mag_tick = 0;
extern imu_data_t data_buffer;
int g_imu_condition = 1;

//GPS VARS
int i=0;
char buffer[100];
int len;
char Rx_indx, Rx_data[2], Rx_Buffer[100], Transfer_cplt, flag;
TM_GPS_t g_GPS_Data;
TM_GPS_Result_t result, current;
TM_GPS_Float_t GPS_Float;
TM_GPS_Distance_t GPS_Distance;
extern TM_GPS_Data_t TM_GPS_INT_Data;
extern TM_GPS_t GPS_Data;

int uart_listen = 1;
int uart_available = 0;

//BMP VARS
TM_BMP180_t bmp_data_pack;
uint_fast32_t g_bmp_tick = 0;
uint_fast32_t g_bmp_TR = 0;
uint_fast32_t g_bmp_PR = 0;
uint_fast8_t g_bmp_pos = 0;
uint32_t bmp_test[] = {0,0,0};
uint32_t adc_data[2];
char Rx_indx2, Rx_data2[2], Rx_Buffer2[30], Transfer_cplt2;
float probe_temp;
float probe_hum;

//UDP VARS
uint_fast32_t g_udp_tick = 0;

uint_fast8_t g_i2c_busy = 0;
unsigned int LED_POS = 0;
int cnt = 0;

//Auto vars
uint_fast32_t g_auto_tick = 0;
float desiredHeading = 0;
float errorHeading = 0;
float errorDistance = 10;
float KH = 3;
int controlDir = 0;
int controlSpeed = 0;

int new_gps_data_occured = 0;

float desiredCoordinate[10][2];
float fsCoordinate[10][2];

int coordinatesIndex = 0;
int fs_coordinatesIndex = 0;

char Rx_indx3, Rx_data3[2], Rx_Buffer3[30], Transfer_cplt3;
float lat3;
float lon3;

uint32_t fs_timer;
uint8_t fs_preactive_flag = 0;

uint8_t printf_buffer[] = {"B100000E-"};

bool auto_active_flag = false;
bool fs_active_flag = false;
bool auto_delete_flag = false;
bool auto_mission_flag = false;
uint8_t mission_indicator = 0;
int mission_heading_buffer = 0;
bool turnaround_flag = false;
bool target_locked_flag = false;
int target_direction = 0;

uint32_t g_adc_tick = 0;

float fs_distance = 0;
uint32_t g_fs_tick = 0;

uint32_t batt_raw;
float batt;

float initial_coordinates[] = {38.00,-111.00};

uint8_t sc_mode = 'd';
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void send_message_to_mc(void);
bool coor_check(float latitude, float longitude);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t led_timer = 0;
uint8_t led_counter = 0;
uint8_t overshoot_counter = 0;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	char data_str;
	int returnValue = 10;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	imu_begin(&hi2c1, 20u, IMU_GYRO_RANGE_250, IMU_ACC_RANGE_2);
	TM_BMP180_Init(&hi2c1 , &bmp_data_pack);
	
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);   //activate uart rx interrupt avery time receiving 1 byte
	HAL_UART_Receive_IT(&huart2, Rx_data2, 1);
	HAL_UART_Receive_IT(&huart3, Rx_data3, 1);
	


	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		////////////////////////////////////AUTONOMUS///////////////////////////////////////////
		
		if(HAL_GetTick() >= led_timer)
		{
			led_timer = HAL_GetTick() + 100;
			if(turnaround_flag)
			{
				sc_mode = 't';
				if((led_counter & 2) == 0)
				{
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				}
				else
				{
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				}
			}
			else if(auto_active_flag)
			{
				sc_mode = 'a';
				switch(led_counter)
				{
					case 0:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
						break;
					case 1:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
						break;
				}
			}
			else if(fs_active_flag)
			{
				sc_mode = 'f';
				switch(led_counter)
				{
					case 0:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
						break;
					case 1:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
						break;
					case 2:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
						break;
					case 3:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
						break;
				}
			}
			else
			{
				sc_mode = 'd';
				switch(led_counter)
				{
					case 0:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
						break;
					case 5:
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
						break;
				}
			}
					
			if(led_counter >= 9)
			{
				led_counter = 0;
			}
			else
			{
				led_counter++;
			}
		}
		
		
		if (((HAL_GetTick() - g_auto_tick) > 100) && (auto_active_flag || fs_active_flag))
		{
			g_auto_tick = HAL_GetTick();		
//			if(target_locked_flag)
//			{
//				desiredHeading = target_direction + (int)data_buffer.imu_data_Z;
//				turnaround_flag = false;
//			}
//			else if(turnaround_flag)
//			{
//				desiredHeading = (int)(data_buffer.imu_data_Z + 90) % 360;
//				int heading_error =  mission_heading_buffer - data_buffer.imu_data_Z;
//				if((heading_error > -15) && (heading_error < -5))
//				{
//					turnaround_flag = false;
//					mission_indicator = 0;
//				}
//			}			
			if(auto_active_flag)
			{
				if((new_gps_data_occured == 1) && (coor_check(TM_GPS_INT_Data.Latitude, TM_GPS_INT_Data.Longitude)))
				{
					new_gps_data_occured = 0;
					desiredHeading = 360 + atan2(desiredCoordinate[0][1] - TM_GPS_INT_Data.Longitude, desiredCoordinate[0][0] - TM_GPS_INT_Data.Latitude) * (180/3.1416); 
					errorDistance = sqrt(((TM_GPS_INT_Data.Latitude - desiredCoordinate[0][0]) * (TM_GPS_INT_Data.Latitude - desiredCoordinate[0][0])) + ((TM_GPS_INT_Data.Longitude - desiredCoordinate[0][1]) * (TM_GPS_INT_Data.Longitude - desiredCoordinate[0][1]))) * 111000;
				
					if(errorDistance < 3)
					{
						if(auto_mission_flag)
						{
							mission_indicator = 1;
							turnaround_flag = true;
							mission_heading_buffer = (int)data_buffer.imu_data_Z;
						}
						for(int l = 1; l < 9 ; l++)
						{
							
							desiredCoordinate[l - 1][1] = desiredCoordinate[l][1];
							desiredCoordinate[l - 1][0] = desiredCoordinate[l][0];
						}
						if(coordinatesIndex > 0)
						{
							coordinatesIndex--;
						}
						else if(coordinatesIndex == 0)
						{
							auto_active_flag = false;
						}
					}
				}
				else if(new_gps_data_occured == 1)
				{
					new_gps_data_occured = 0;
				}
			}
			
			
			else if(fs_active_flag)
			{
				if((new_gps_data_occured == 1) && (coor_check(TM_GPS_INT_Data.Latitude, TM_GPS_INT_Data.Longitude)))
				{ 
					new_gps_data_occured = 0;
					desiredHeading = 360 + atan2(fsCoordinate[0][1] - TM_GPS_INT_Data.Longitude, fsCoordinate[0][0] - TM_GPS_INT_Data.Latitude) * (180/3.1416); 
					errorDistance = sqrt(((TM_GPS_INT_Data.Latitude - fsCoordinate[0][0]) * (TM_GPS_INT_Data.Latitude - fsCoordinate[0][0])) + ((TM_GPS_INT_Data.Longitude - fsCoordinate[0][1]) * (TM_GPS_INT_Data.Longitude - fsCoordinate[0][1]))) * 111000;
				
					if(errorDistance < 2)
					{
						for(int l = 1; l < 9 ; l++)
						{
							fsCoordinate[l - 1][1] = fsCoordinate[l][1];
							fsCoordinate[l - 1][0] = fsCoordinate[l][0];
						}
						if(fs_coordinatesIndex > 0)
						{
							fs_coordinatesIndex--;
						}
						else if(fs_coordinatesIndex == 0)
						{
							fs_active_flag = false;
						}
					}
				}
				else if(new_gps_data_occured == 1)
				{
					new_gps_data_occured = 0;
				}
			}
			
			
			errorHeading = desiredHeading - data_buffer.imu_data_Z;
			if(errorHeading > 180)
			{
				errorHeading = errorHeading - 360;
			}
			controlDir = KH * errorHeading;
		
			controlSpeed = 100 - abs((int)(errorHeading * 5)); 
			
			//CONSTRAIN DIR
			if(controlDir > 99)
			{
				controlDir = 99;
			}
			else if(controlDir < -99)
			{
				controlDir = -99;
			}
			
			//CONSTARIN SPEED
			if(controlSpeed > 99)
			{
				controlSpeed = 99;
			}
			else if(controlSpeed < 0)
			{
				controlSpeed = 0; 
			}
			
			if(auto_active_flag && coordinatesIndex == 0)
			{
				auto_active_flag = false;
			}
			if(fs_active_flag && fs_coordinatesIndex == 0)
			{
				//fs_active_flag = false;
			}
			
			if(auto_active_flag ||  fs_active_flag)
			{
				//target_locked_flag=false;
				send_message_to_mc();
			}
		}
		
//	if ((HAL_GetTick() - g_imu_tick) > 2)
//	{
//		g_imu_tick = HAL_GetTick();
//		if(adc_data[0] > 2000)
//		{
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);	
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);	
//		}
//		//imu_read_data();
//		//send_message_to_mc();
//	}
		
		if (((HAL_GetTick() - g_mag_tick) > 20))
		{
			g_mag_tick = HAL_GetTick();
			magnet_read_data();
		}	
		

		
		if (((HAL_GetTick() - g_bmp_tick) > 300) && (g_bmp_pos == 0))
		{
			bmp_test[0] = HAL_GetTick();
			TM_BMP180_StartTemperature(&bmp_data_pack);
			g_bmp_tick = HAL_GetTick();
			g_bmp_TR = HAL_GetTick() + 7;
			g_bmp_pos = 1;


		}
		if ((HAL_GetTick() >= g_bmp_TR) && (g_bmp_pos == 1))
		{
			bmp_test[1] = HAL_GetTick();
			TM_BMP180_ReadTemperature(&bmp_data_pack);
			TM_BMP180_StartPressure(&bmp_data_pack, TM_BMP180_Oversampling_UltraHighResolution);
			g_bmp_PR = HAL_GetTick() + 27;
			g_bmp_pos = 2;
		}	
		if ((HAL_GetTick() >= g_bmp_PR) && (g_bmp_pos == 2))
		{
			bmp_test[2] = HAL_GetTick();
			TM_BMP180_ReadPressure(&bmp_data_pack);
			g_bmp_pos = 0;
		}
		
		if ((HAL_GetTick() - g_udp_tick) > 1000)
		{
			uint8_t print_buffer[100];
			g_udp_tick = HAL_GetTick();
			
//			HAL_ADC_Start(&hadc1);
//			HAL_ADC_PollForConversion(&hadc1, 100);
//			batt_raw = HAL_ADC_GetValue(&hadc1);
//			batt = ((float)batt_raw / 4096.00) * 33; 
//			HAL_ADC_Stop(&hadc1);
			
			int print_size = sprintf(print_buffer, "SC,%d,%3.2f,%d,%3.2f,%3.2f,%f,%f,%3.2f,%3.2f,%3.2f,%3.2f,ITU\n",\
			bmp_data_pack.Pressure, bmp_data_pack.Temperature, mission_indicator, data_buffer.imu_data_Y, \
			data_buffer.imu_data_Z, TM_GPS_INT_Data.Latitude, TM_GPS_INT_Data.Longitude, TM_GPS_INT_Data.VDOP, \
			TM_GPS_INT_Data.Altitude, TM_GPS_INT_Data.Speed, batt);	
			HAL_UART_Transmit(&huart3, print_buffer, print_size, 100);
			print_size = sprintf(print_buffer, "%c,%d,%f3.2,%f,%f\r\n", sc_mode,coordinatesIndex,errorDistance, TM_GPS_INT_Data.Latitude, TM_GPS_INT_Data.Longitude);
			HAL_UART_Transmit(&huart2, print_buffer, print_size, 100);				
		}
		
		if ((HAL_GetTick() - g_fs_tick) > 2000)
		{
			g_fs_tick = HAL_GetTick();
			if(coor_check(TM_GPS_INT_Data.Latitude, TM_GPS_INT_Data.Longitude))
			{
				fs_distance = sqrt(((TM_GPS_INT_Data.Latitude - fsCoordinate[0][0]) * (TM_GPS_INT_Data.Latitude - fsCoordinate[0][0])) + ((TM_GPS_INT_Data.Longitude - fsCoordinate[0][1]) * (TM_GPS_INT_Data.Longitude - fsCoordinate[0][1]))) * 111000;
				if((fs_distance > 15) && (!fs_active_flag))
				{
					int m = 0;
					for(int m = 8; m >= 0; m--)
					{
						fsCoordinate[m + 1][1] = fsCoordinate[m][1];
						fsCoordinate[m + 1][0] = fsCoordinate[m][0];
					}
					fsCoordinate[0][0] = TM_GPS_INT_Data.Latitude;
					fsCoordinate[0][1] = TM_GPS_INT_Data.Longitude;
					if(fs_coordinatesIndex < 9)
					{
						fs_coordinatesIndex++;
					}
				}
			}
		}

	
				if (Transfer_cplt)
        {
						Rx_Buffer[3] = 'P';
            read_gps_data(Transfer_cplt, Rx_Buffer ,buffer,GPS_Float);
            Transfer_cplt=0;
						new_gps_data_occured = 1;
        }
				
				/////////LORA/////////////
				if (Transfer_cplt2)
        {
          Transfer_cplt2 = 0;
					switch(Rx_Buffer2[0])
					{
						case 's':
							auto_active_flag = false;
							break;
						case 'g':
							auto_active_flag = true;
							break;
						case 'd':
 							coordinatesIndex = 0;
							break;
						case 'a':
							fs_active_flag = false;
							fs_preactive_flag = 0;
							break;
						default:
							lon3 = atof(Rx_Buffer2);
							for (int l = 0; l < 25; l++)
							{
								if (Rx_Buffer2[l] == ',')
								{
									lat3 = atof(&Rx_Buffer2[l+1]);
									break;
								}
							}
							break;
					}
					if(coor_check(lat3, lon3))
					{
						if(coordinatesIndex == 0)
						{
							desiredCoordinate[0][0] = lat3;
							desiredCoordinate[0][1] = lon3;
							coordinatesIndex++;
						}
						else if((coordinatesIndex <= 9) && (lon3 != desiredCoordinate[coordinatesIndex - 1][0]) && (lat3 != desiredCoordinate[coordinatesIndex - 1][1]))
						{
							desiredCoordinate[coordinatesIndex][0] = lat3;
							desiredCoordinate[coordinatesIndex][1] = lon3;
							coordinatesIndex++;
						}
					}
					lat3 = 0;
					lon3 = 0;
        }
				
				//////////TCP////////////////
				if(Transfer_cplt3)
				{
 					Transfer_cplt3 = 0;
					
					switch(Rx_Buffer3[0])
					{
						case 's':
							auto_active_flag = false;
							break;
						case 'g':
							auto_active_flag = true;
							break;
						case 'd':
							coordinatesIndex = 0;
							break;
						case 'l':
							if(Rx_Buffer3[1] == 'o')
							{
								fs_preactive_flag = 1;
								fs_timer = HAL_GetTick() + 5000;
							}
							break;
						case 'o':
							fs_preactive_flag = 0;
							fs_active_flag = false;
							break;
						case 'T':
							target_direction = atoi(&Rx_Buffer3[1]);
							target_locked_flag = true;
							break;
						default:
							lon3 = atof(Rx_Buffer3);
 							for (int l = 0; l < 25; l++)
							{
								if (Rx_Buffer3[l] == ',')
								{
									lat3 = atof(&Rx_Buffer3[l+1]);
									break;
								}
							}
							break;
					}
				
					
					if(coor_check(lat3, lon3))
					{
						if(coordinatesIndex == 0)
						{
							desiredCoordinate[0][0] = lat3;
							desiredCoordinate[0][1] = lon3;
							coordinatesIndex++;
						}
						else if((coordinatesIndex <= 9) && (lon3 != desiredCoordinate[coordinatesIndex - 1][0]) && (lat3 != desiredCoordinate[coordinatesIndex - 1][1]))
						{
							desiredCoordinate[coordinatesIndex][0] = lat3;
							desiredCoordinate[coordinatesIndex][1] = lon3;
							coordinatesIndex++;
						}
					}
					lat3 = 0;
					lon3 = 0;
        }
				
					if((fs_preactive_flag == 1) && (HAL_GetTick() >= fs_timer))
					{
						HAL_UART_Transmit(&huart2, "com_c\r\n", 7, 100);
						fs_timer = fs_timer + 3000;
						fs_preactive_flag = 2;
					}
					else if(fs_preactive_flag == 2 && (HAL_GetTick() > fs_timer))
					{
						fs_active_flag = true;
						fs_preactive_flag = 0;
					}
			
					
 				


		
  }
	
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void send_message_to_mc(void)
{		
		
		printf_buffer[8] = 0x0A;
		
		if(controlSpeed == 0)
		{
			sprintf(&printf_buffer[2], "00");
		}
		else if(controlSpeed < 10 && controlSpeed > -10)
		{
			sprintf(&printf_buffer[2], "0%d", abs(controlSpeed));
		}
		else
		{
			sprintf(&printf_buffer[2], "%d", abs(controlSpeed));
		}
		 
		if(controlDir < 0)
		{
			sprintf(&printf_buffer[4], "1");
		}
		else
		{
			sprintf(&printf_buffer[4], "0");
		}
		
		if(controlDir == 0)
		{
			sprintf(&printf_buffer[5], "00");
		}
		else if(controlDir < 10 && controlDir > -10)
		{
			sprintf(&printf_buffer[5], "0%d", abs(controlDir));
		}
		else
		{
			sprintf(&printf_buffer[5], "%d", abs(controlDir));
		}
		printf_buffer[7] = 'E';
		HAL_UART_Transmit_IT(&huart1, printf_buffer, 9);
	
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//imu_rx_cplt_callback();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i;
	flag=1;
	if (huart->Instance == USART1)   //current UART
			{
			if (Rx_data[0]!=13) //if received data different from ascii 13 (enter)
					{
					Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
					}
			else            //if received data = 13
					{
					Rx_indx=0;
					Transfer_cplt=1;//transfer complete, data is ready to read
					}
			HAL_UART_Receive_IT(&huart1, Rx_data, 1);
			}
			
			if (huart->Instance == USART2)   //current UART
			{
			if (Rx_data2[0]!= 0x0A) //if received data different from ascii 13 (enter)
					{
					Rx_Buffer2[Rx_indx2++]=Rx_data2[0];    //add data to Rx_Buffer
					}
			else            //if received data = 13
					{
					Rx_indx2++;
					//Rx_Buffer2[Rx_indx2] = ',';
					Rx_indx2=0;
					Transfer_cplt2=1;//transfer complete, data is ready to read
					}
				HAL_UART_Receive_IT(&huart2, Rx_data2, 1);
				}
			
			
			if (huart->Instance == USART3)   //current UART
			{
			if (Rx_data3[0]!= 0x0A) //if received data different from ascii 13 (enter)
					{
					Rx_Buffer3[Rx_indx3++]=Rx_data3[0];    //add data to Rx_Buffer
					}
			else            //if received data = 13
					{
					Rx_indx3++;
					Rx_Buffer3[Rx_indx3 - 1] = ',';
					Rx_indx3=0;
					Transfer_cplt3=1;//transfer complete, data is ready to read
					}
			HAL_UART_Receive_IT(&huart3, Rx_data3, 1);
			}
}

void imu_data_ready_callback(imu_data_t data)
{
//	imu_data_ready = true;
//	data_pack = data;
}

bool coor_check(float latitude, float longitude)
{
	if((latitude < initial_coordinates[0] + 3) && (latitude > initial_coordinates[0] - 3)\
		&& (longitude < initial_coordinates[1] + 3) && (longitude > initial_coordinates[1] - 3))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
