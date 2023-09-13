/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include<stdio.h>
#include<string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t id_16[1],ctrl_meas_16[10];
uint8_t add=0x76<<1,id[8],reset[1],value[3],status[8],ctrl_meas[10],ctrl[1],config[1];
uint8_t press_lsb[7],press_msb[1],press_xlsb[3],press[2],calib[1];
uint8_t temp_lsb[7],temp_xlsb[7],temp_msb[7] = {0};
int32_t avg_temp[100],pressure[100];
float avg_temp_final=0,avg_press_final=0;
unsigned int temp_raw,press_raw;
int32_t t_fine;
double temp_final,final_press;
uint8_t calibdata[5],calibdata_press[20];
unsigned short dig_T1,dig_P1;
signed short dig_T2,dig_T3,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
unsigned int i=0;
HAL_StatusTypeDef ret;
char r[100];
int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
double bmp280_compensate_T_double( uint32_t adc_T)
{
double var1, var2, T;
var1 = (((double)adc_T)/16384.0 -((double)dig_T1)/1024.0) * ((double)dig_T2);
var2 = ((((double)adc_T)/131072.0 -((double)dig_T1)/8192.0) *(((double)adc_T)/131072.0 -((double) dig_T1)/8192.0)) * ((double)dig_T3);
t_fine = (int32_t)(var1 + var2);
T = (var1 + var2) / 5120.0;
return T;
}
double bmp280_compensate_P_double(uint32_t adc_P) {
double var1, var2, p;
var1 = ((double)t_fine/2.0)-64000.0;
var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
var2 = var2 + var1 * ((double)dig_P5) * 2.0;
var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
if (var1 == 0.0)
{
return 0; // avoid exception caused by division by zero
}
p = 1048576.0 -(double)adc_P;
p = (p -(var2 / 4096.0)) * 6250.0 / var1;
var1 = ((double)dig_P9) * p * p / 2147483648.0;
var2 = p * ((double)dig_P8) / 32768.0;
p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
return p;
}


void compute_temperature(unsigned int Raw_Temperature )
{
	signed long  var1=0,var2=0;
	signed int T=0;
	signed long t_fine=0;

	var1 = (((Raw_Temperature>>3) -((signed int )dig_T1<<1))) ;
	var1 = ((var1 *((signed int)dig_T2)) >> 11) ;

    var2 = ((Raw_Temperature/16) -((signed int)dig_T1));
	var2 =  (( (var2 * var2) /4096 ) * (((signed int)dig_T3) ) / 16384) ;
//    printf("v1 %d v2 %d | ",var1, var2 );

    t_fine = var1 +var2;
    T = ((t_fine * 5) + 128) / 256;
    avg_temp[i]=T;
    avg_temp_final+=avg_temp[i];
    i++;
    if(i==99){
    	avg_temp_final/=10000;
    	if(avg_temp_final==-40){
    		printf("Under Limit\n\r");
    	}
    	else if(avg_temp_final==85){
    		printf("Over Limit\n\r");
    	}
    	else{
    		printf("Final_Temp:%2.2f°c\n\r",avg_temp_final);
    	}
    	    i=0;
    }

}
void configuration(void){
	ret= HAL_I2C_Master_Transmit(&hi2c1, add , id, 1, HAL_MAX_DELAY);
			 if(ret != HAL_OK){
				 printf("Error %d\n\r",ret);
			 }
			 else{
				  ret=HAL_I2C_Master_Receive(&hi2c1, add , id , 1 , HAL_MAX_DELAY);
		//		  HAL_UART_Transmit(&huart2, id, 1, HAL_MAX_DELAY);
				  printf("Device Tested\n\r");
				  if (id[0]==0x58){
					  status[0]=0xF3; // to check status
					  HAL_I2C_Master_Transmit(&hi2c1, add , id, 1, HAL_MAX_DELAY);
					  HAL_I2C_Master_Receive(&hi2c1, add , status , 4 , HAL_MAX_DELAY);
			//		  HAL_UART_Transmit(&huart2, status, 4, HAL_MAX_DELAY);
				  }
				  if(status[0]==0&&status[3]==0){	//status data is processed and copied
					  ctrl_meas[0]=0xF4;
					  ctrl[0]=0x27;
					  HAL_I2C_Mem_Write(&hi2c1, add, ctrl_meas[0], I2C_MEMADD_SIZE_8BIT,(&ctrl[0]),1, HAL_MAX_DELAY);
			//		  HAL_I2C_Master_Transmit(&hi2c1, add , ctrl, 1, HAL_MAX_DELAY);
					  HAL_Delay(1000);
					  HAL_I2C_Master_Receive(&hi2c1, add , ctrl_meas, 1, HAL_MAX_DELAY);
					  printf("ctrl_meas Set\n\r");
			//		  HAL_UART_Transmit(&huart2, ctrl_meas, 1, HAL_MAX_DELAY);
				  }
				  if(ctrl_meas[0]==0x27){
					  config[0]=0xF5;
					  config[1]=0x00;
					  HAL_I2C_Mem_Write(&hi2c1, add, config[0], I2C_MEMADD_SIZE_8BIT,(&config[1]),1, HAL_MAX_DELAY);
					  HAL_Delay(1000);
					  HAL_I2C_Master_Receive(&hi2c1, add , config, 1, HAL_MAX_DELAY);
					  printf("Config Set\n\r");
			//		  HAL_UART_Transmit(&huart2, config, 1, HAL_MAX_DELAY);
				  }
			 }
			 }
void temp_calc (void){

			  if(config[0]==0x00){
				  calibdata[0]=0x88;
		//		  HAL_I2C_Master_Receive(&hi2c1, add , calibdata, 6, HAL_MAX_DELAY);
				  HAL_I2C_Mem_Read(&hi2c1,add,calibdata[0],I2C_MEMADD_SIZE_8BIT,(&calibdata[0]),6,HAL_MAX_DELAY);
		//		  HAL_UART_Transmit(&huart2, calib, 6, HAL_MAX_DELAY);
				  dig_T1=(calibdata[1]<<8)| calibdata[0];
				  dig_T2=(calibdata[3]<<8)| calibdata[2];
				  dig_T3=(calibdata[5]<<8)| calibdata[4];
		//		  dig_T1=27504;
		//		  dig_T2=26435;
		//		  dig_T3=-1000;
		//		  printf("dig_T1:%d dig_T2:%d dig_T3:%d\n\r",dig_T1,dig_T2,dig_T3);
//				  temp_lsb[0]=0xFB;
//				  temp_msb[0]=0xFA;
//				  temp_xlsb[0]=0xFC;

				  if(status[0]==0&&status[3]==0){
	//			  temp_lsb[0]=0x91;
	//			  temp_xlsb[0]=0x2<<4;
	//			  temp_msb[0]=0x73;
	//			  temp_raw=(temp_xlsb[0]>>4)|(temp_lsb[0]<<4)|(temp_msb[0]<<12);
		//		  temp_raw=519888;
		//		  compute_temperature(temp_raw);
		//		  temp_final=bmp280_compensate_T_double(temp_raw);
		//		  printf("Temp_final%lf\n\r",temp_final);
						  temp_lsb[0]=0xFB;
						  temp_msb[0]=0xFA;
						  temp_xlsb[0]=0xFC;
						  HAL_I2C_Mem_Read(&hi2c1,add,temp_msb[0],I2C_MEMADD_SIZE_8BIT,(&temp_msb[0]),3,HAL_MAX_DELAY);
						  temp_lsb[0]=temp_msb[1];
						  temp_xlsb[0]=temp_msb[2];
		//	 	 		  temp_lsb[0]=0x91;
		//	 	 		  temp_xlsb[0]=0x2<<4;
		//	 	 		  temp_msb[0]=0x73;
		//	 	 		  printf("lsb:%d xlsb:%d,msb:%d\n\r",temp_lsb[0],temp_xlsb[0],temp_msb[0]);
						  temp_raw=(temp_xlsb[0]>>4)|(temp_lsb[0]<<4)|(temp_msb[0]<<12);
		//		 		  temp_raw=519888;
						  compute_temperature(temp_raw);
		//	 		 	  temp_final=bmp280_compensate_T_double(temp_raw);
		//	 		      printf("Temp_final%lf\n\r",temp_final);
						  HAL_Delay(10);
				  }
			  }
}


void press_calc(void){
	static int i=0;
	if(config[0]==0x00){
	 calibdata_press[0]=0x8E;
	 				  HAL_I2C_Master_Receive(&hi2c1, add , calibdata, 18, HAL_MAX_DELAY);
//	 				  HAL_I2C_Mem_Read(&hi2c1,add,calibdata[0],I2C_MEMADD_SIZE_8BIT,(&calibdata[0]),18,HAL_MAX_DELAY);
	 		//		  HAL_UART_Transmit(&huart2, calib, 6, HAL_MAX_DELAY);
	 				  dig_P1=(calibdata_press[1]<<8)| calibdata_press[0];
	 				  dig_P2=(calibdata_press[3]<<8)| calibdata_press[2];
	 				  dig_P3=(calibdata_press[5]<<8)| calibdata_press[4];
	 				  dig_P4=(calibdata_press[7]<<8)| calibdata_press[6];
	 				  dig_P5=(calibdata_press[9]<<8)| calibdata_press[8];
	 				  dig_P6=(calibdata_press[11]<<8)| calibdata_press[10];
	 				  dig_P7=(calibdata_press[13]<<8)| calibdata_press[12];
	 				  dig_P8=(calibdata_press[15]<<8)| calibdata_press[14];
	 				  dig_P9=(calibdata_press[17]<<8)| calibdata_press[16];
//	 				  printf("P1:%d,P2:%d,P3:%d,P4:%d,P5:%d,P6:%d,p7:%d,P8:%d,P9:%d\n\r",dig_P1,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9);

	 if(status[0]==0&&status[3]==0){
	 	//			  temp_lsb[0]=0x91;
	 	//			  temp_xlsb[0]=0x2<<4;
	 	//			  temp_msb[0]=0x73;
	 	//			  temp_raw=(temp_xlsb[0]>>4)|(temp_lsb[0]<<4)|(temp_msb[0]<<12);
	 		//		  temp_raw=519888;
	 		//		  compute_temperature(temp_raw);
	 		//		  temp_final=bmp280_compensate_T_double(temp_raw);
	 		//		  printf("Temp_final%lf\n\r",temp_final);
	 						  press_lsb[0]=0xF8;
	 						  press_msb[0]=0xF7;
	 						  press_xlsb[0]=0xF9;
	 						  HAL_I2C_Mem_Read(&hi2c1,add,press_msb[0],I2C_MEMADD_SIZE_8BIT,(&press_msb[0]),3,HAL_MAX_DELAY);
	 						  press_lsb[0]=press_msb[1];
	 						  press_xlsb[0]=press_msb[2];
	 		//	 	 		  temp_lsb[0]=0x91;
	 		//	 	 		  temp_xlsb[0]=0x2<<4;
	 		//	 	 		  temp_msb[0]=0x73;
	 		//	 	 		  printf("lsb:%d xlsb:%d,msb:%d\n\r",temp_lsb[0],temp_xlsb[0],temp_msb[0]);
	 						  press_raw=(press_xlsb[0]>>4)|(press_lsb[0]<<4)|(press_msb[0]<<12);
	 		//		 		  temp_raw=519888;
	 						 final_press=bmp280_compensate_P_double(press_raw)/1000;
	 						 pressure[i]=final_press;
 							 avg_press_final+=pressure[i];
 							 i++;
	 						 if(i == 99){
	 							 avg_press_final=avg_press_final/100;
	 							 avg_press_final-=150;
	 	 						printf("Press_final:%lfpa\n\r",avg_press_final);
	 	 						  HAL_Delay(10);
	 	 						  i=0;
	 						 }
	 		//	 		 	  temp_final=bmp280_compensate_T_double(temp_raw);
	 		//	 		      printf("Temp_final%lf\n\r",temp_final);

	 				  }
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
//	HAL_StatusTypeDef buf,bufr;
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
  /* USER CODE BEGIN 2 */
//  sprintf(buf,"");
  /* USER CODE END 2 */
 id[0]= 0xD0;
 id_16[0]= 0xD0;
 configuration();
// reset[0]=0xE0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  temp_calc();
	  press_calc();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 1000000;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
