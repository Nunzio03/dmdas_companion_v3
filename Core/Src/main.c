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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "si5351.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DMILLIM_USTEP_CONSTANT 0.012528906

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*HCSR04 variables*/

uint8_t rised_hc=0;
uint16_t dist_time1=0;
uint32_t distance=0;

/*PLC COMMUNICATION Variables*/

uint8_t rised_plc=0;
uint8_t first_wave_rec=0;
uint16_t dur1=0;
uint16_t dur2=0;
uint16_t dur_time1=0;
uint16_t magic_number=0;

/* STEPPER MOTOR VARIABLES*/
uint8_t moving = 0;
int16_t open_loop_motor_position = 0;
uint8_t move_mot = 0;
uint8_t motor_moved = 0;
uint8_t direction = 0;
uint8_t startup_movement = 0;
uint16_t tare_counter = 0;
uint8_t back_movement = 0;

/* UART Variables*/

volatile uint8_t cmd_char = 0;
int distance_avg = 0;
char buffer[32] = "";
int dim=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while(__HAL_TIM_GET_COUNTER(&htim3)<time);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	/* HC_SR04 INTERRUPT*/
	if(htim->Instance == TIM3){
		uint16_t val = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
		if(!rised_hc){
			dist_time1 = val;
			rised_hc = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}else{ //falling
			if(val >= dist_time1){
				distance = (uint32_t)((val - dist_time1)*100/58.0); // decimi di millimetro
			}else{
				distance = (uint32_t)(((0xffff-dist_time1)+val)*100/58.0);
			}
			rised_hc = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		}

	}
	/*CUSTOM PROTOCOL INTERRUPT*/
	if(htim->Instance == TIM4){

		uint16_t val = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
		if(first_wave_rec==0){

			if(!rised_plc){

				dur_time1 = val;
				rised_plc = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				}else{

					if(val>=dur_time1){
						dur1= val-dur_time1;
					}else{
						dur1= (0xffff-dur_time1)+val;
					}
					rised_plc=0;
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
					first_wave_rec=1;
				}

			}else{

				if(!rised_plc){


					dur_time1 = val;
					rised_plc = 1;
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
					}else{

						if(val>=dur_time1){
							dur2= val-dur_time1;
						}else{
							dur2= (0xffff-dur_time1)+val;
						}
						rised_plc=0;
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
						first_wave_rec=0;
						dur1 = (dur1/15)-15;
						dur2 = (dur2/15)-15;
						magic_number = (dur1<<5)|dur2;

					}


		}


	}

}

void HCSR04_trigger(){
	HAL_GPIO_WritePin(HCSR_TRIG_GPIO_Port, HCSR_TRIG_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(HCSR_TRIG_GPIO_Port, HCSR_TRIG_Pin, GPIO_PIN_RESET);

}


void move_stepper(uint32_t steps){
	HAL_GPIO_WritePin(MOTOR_DIRECTION_GPIO_Port, MOTOR_DIRECTION_Pin,direction );
	HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, 0);
	 for (int i = 0; i <steps ; i++)
	  {

		 HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, 1);
		 delay_us(30);
		 HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, 0);
		 delay_us(30);
	  }
	 HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, 1);

}

void move_stepper_dec_mm(uint32_t dec_mm){
	moving = 1;
	if(direction){
		open_loop_motor_position += dec_mm;
	}else{

		open_loop_motor_position -= dec_mm;

	}
	move_stepper((uint32_t)(dec_mm/DMILLIM_USTEP_CONSTANT));
	moving = 0;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);


  HAL_UART_Receive_IT(&huart2, &cmd_char, 1);
  si5351_Init();
  si5351_setupPLLInt(SI5351_PLL_A, 32);
  si5351_setupMultisynth(0, SI5351_PLL_A, 4, 1000-4, 1);
  si5351_setupRdiv(0, SI5351_R_DIV_8);
  si5351_enableOutputs(0xFF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(100);
	  HCSR04_trigger();
	  move_mot = !HAL_GPIO_ReadPin(PLC_EN_MOT_GPIO_Port, PLC_EN_MOT_Pin);
	  direction = HAL_GPIO_ReadPin(PLC_DIRECTION_GPIO_Port, PLC_DIRECTION_Pin);
	  if(move_mot&&(!motor_moved)){

		  move_stepper_dec_mm(magic_number);
		  motor_moved = 1;
	  }

	  if(startup_movement){
		  direction =0;
		  move_stepper(600);
	  }

	  if(back_movement){
		  direction = 1;
		  move_stepper(600);
		  tare_counter += 600;
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch(cmd_char){
	case 'a':

		for(int k=0; k<10;k++){
			distance_avg += distance;
		}
		buffer[32] = "";
		dim = sprintf(buffer,"%d\n",(int)(distance_avg/10));
		HAL_UART_Transmit_IT(&huart2, buffer,dim);
		//HAL_TIM_Base_Stop_IT(&htim11);
		break;
	case 'b':
		buffer[32] = "";
		dim = sprintf(buffer,"%d\n",open_loop_motor_position);
		HAL_UART_Transmit_IT(&huart2, buffer,dim);
		break;
	case 'c':
		buffer[32] = "";
		dim = sprintf(buffer,"%d\n",moving);
		HAL_UART_Transmit_IT(&huart2, buffer,dim);
		break;
	case 'd':
		buffer[32] = "";
		dim = sprintf(buffer,"%d\n",magic_number);
		HAL_UART_Transmit_IT(&huart2, buffer,dim);
		motor_moved = 0;
		break;
	case 't':
		HAL_UART_Transmit_IT(&huart2, "moving\n",7);
		startup_movement = 1;
		break;
	case 'g':
		buffer[32] = "";
		dim = sprintf(buffer,"%d\n",tare_counter*DMILLIM_USTEP_CONSTANT);
		HAL_UART_Transmit_IT(&huart2, buffer,dim);
	default:

		HAL_UART_Transmit_IT(&huart2, "unknown char\n",13);


	}

	HAL_UART_Receive_IT(&huart2, &cmd_char, 1);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM11){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == END_STOPmax_Pin){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_UART_Transmit_IT(&huart2, "9\n",2);
		//TEST GITHUB
		back_movement = 1;

	}else if(GPIO_Pin==END_STOPmin_Pin){
		back_movement = 0;
		HAL_UART_Transmit_IT(&huart2, "0\n",2);
	}
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
