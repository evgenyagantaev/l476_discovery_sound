#include "main.h"
#include "dac.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "ssd1306.h"
#include "stdio.h"

#include "math.h"

void SystemClock_Config(void);

uint32_t tick_counter_100khz = 0;
uint32_t r_pulse_time_old_marker = 0;
uint32_t r_pulse_time_new_marker = 0;
int new_r_pulse_flag = 0;
uint32_t rr = 0;
uint32_t rr_array[] = {0, 0, 0};
uint32_t r_pick_counter = 0;

uint8_t Send_buf[8];

void Uart_SendCMD(uint8_t CMD ,uint8_t feedback , uint16_t dat)
{
	Send_buf[0] = 0x7e;
	Send_buf[1] = 0xff; 				// зарезервированные байты
	Send_buf [2] = 0x06; 				//длина
	Send_buf[3] = CMD; 					// директива управления
	Send_buf[4] = feedback; 			// нужна ли обратная связь
	Send_buf[5] = (uint8_t)(dat >> 8);	//datah
	Send_buf[6] = (uint8_t)(dat); 		//datal
	Send_buf[7] = 0xef;
	HAL_UART_Transmit(&huart2, Send_buf, 8, 500);
}


int main(void)
{

	HAL_Init();


	SystemClock_Config();
	/* Disable SysTick Interrupt */
	//SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;


	MX_GPIO_Init();
	MX_DAC1_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();

	HAL_Delay(30);
	Uart_SendCMD (0x06, 0, 0x15); // уровень громкости 21
	HAL_Delay(100);

	//****************************************************************
	/*
	ssd1306_set_i2c_port(&hi2c1, 1);
	ssd1306_Init();
	HAL_Delay(100);

	ssd1306_Fill(White);
	ssd1306_UpdateScreen();
	HAL_Delay(100);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	HAL_Delay(100);

	ssd1306_SetCursor(0,0);
	ssd1306_WriteString("VALS", Font_11x18, White);
	ssd1306_UpdateScreen();
	HAL_Delay(100);
	//*/
	//****************************************************************

	// debug********************************************
	/*
	HAL_Delay(30);
	Uart_SendCMD (0x06, 0, 0x15); // уровень громкости 21
	//HAL_Delay(10);
	Uart_SendCMD (0x03, 0, 0x09); // воспроизведение первой песни
	HAL_Delay(100);
	//*/

	/*
	int wait = 1;
	while(wait)
	{
		if ((GPIOE->IDR & GPIO_PIN_15) != 0x00u)
			wait = 0;
	}
	//*/


	// DEBUG STOP
	int wait = 1;
	while(wait)
	{
		if ((GPIOE->IDR & GPIO_PIN_15) == 0x00u)
		{
			Uart_SendCMD (0x03, 0, 0x09); // воспроизведение первой песни
			HAL_Delay(100);
			wait = 0;
		}
	}
	HAL_Delay(12000);

	while(1)
	{
		if ((GPIOE->IDR & GPIO_PIN_15) == 0x00u)
		{
			Uart_SendCMD (0x03, 0, 0x0e); // воспроизведение удара сердца
			HAL_Delay(100);
		}
	}
	//*/


	//Uart_SendCMD (0x06, 0, 0x1e); // уровень громкости
	//Uart_SendCMD (0x03, 0, 0x01); // воспроизведение первой песни
	//Uart_SendCMD (0x06, 0, 0x10);
	//Uart_SendCMD (0x06, 0, 0x0a);
	//Uart_SendCMD (0x06, 0, 0x13);
	//Uart_SendCMD (0x03, 0, 0x02); // воспроизведение второй песни
	//Uart_SendCMD (0x03, 0, 0x03); // воспроизведение третьей песни
	//Uart_SendCMD (0x03, 0, 0x04);
	//Uart_SendCMD (0x03, 0, 0x05);
	//Uart_SendCMD (0x03, 0, 0x06);
	//Uart_SendCMD (0x03, 0, 0x07);
	//Uart_SendCMD (0x03, 0, 0x08);
	//Uart_SendCMD (0x03, 0, 0x09);
	//Uart_SendCMD (0x03, 0, 0x0a);
	//Uart_SendCMD (0x03, 0, 0x0b);
	//Uart_SendCMD (0x03, 0, 0x0c);
	//Uart_SendCMD (0x03, 0, 0x0d);
	// debug********************************************

	/* Disable SysTick Interrupt */
	//SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;


	MX_TIM2_Init();

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	TIM2->DIER |= TIM_DIER_UIE;
	HAL_TIM_Base_Start(&htim2);

	uint32_t tick_counter_old = 0;

	uint32_t A = 0x00000fff/2;
	uint32_t base_T = 1000;
	//uint32_t base_T = 33;
	double time_masshtab = (double)(1.0/(double)base_T*2.0*3.14);


	uint32_t sin_table[1000];
	int i;
	for(i=0; i<base_T; i++)
	{
		double t = ((double)i)*time_masshtab;
		sin_table[i] = (uint32_t)(A + (int32_t)((double)A*sin(t)));
	}

	int period = 20; // period v tikah chastota tikov 100000 Hz

	double step = (double)base_T/(double)period;

	uint32_t dac_data = A;
	UNUSED(dac_data);

	UNUSED(step);
	UNUSED(sin_table);

	while (1)
	{

		if(tick_counter_old < tick_counter_100khz)
		{
			tick_counter_old = tick_counter_100khz;

			/*
			// debug sinus

			uint32_t T = (uint32_t)((tick_counter_100khz%period)*step);
			if(T>=base_T) T = base_T - 1;
			dac_data = sin_table[T];
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_data);
			//*/

			/*
			// debug rectangle

			int semi_period = period/2;

			if((tick_counter_100khz % semi_period) == 0)
			{
				if(dac_data == 0)
					dac_data = 2 * A;
				else
					dac_data = 0;

				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_data);
			}
			//*/


			// debug led blink
			if(tick_counter_100khz%50000 == 0)
			{
				HAL_GPIO_TogglePin(red_led_GPIO_Port, red_led_Pin);
				HAL_GPIO_TogglePin(green_led_GPIO_Port, green_led_Pin);

			}
		}// end if(tick_counter_old < tick_counter_100khz)

		if(new_r_pulse_flag)
		{
			new_r_pulse_flag = 0;

			Uart_SendCMD (0x03, 0, 0x0e);

			rr_array[0] = rr_array[1];
			rr_array[1] = rr_array[2];
			rr_array[2] = rr;

			if(rr_array[0]*rr_array[1]*rr_array[2] > 0)
			{
				uint32_t mean_rr = (rr_array[0] + rr_array[1] + rr_array[2]) / 3;
				uint32_t heart_rate = 60000 / mean_rr;

				r_pick_counter++;

				char aux_message[64];

				sprintf(aux_message, "R -> %u", (unsigned int)r_pick_counter);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString("           ", Font_11x18, White);
				ssd1306_SetCursor(0,0);
				ssd1306_WriteString(aux_message, Font_11x18, White);
				//ssd1306_UpdateScreen();

				sprintf(aux_message, "H -> %u", (unsigned int)heart_rate);
				ssd1306_SetCursor(0,22);
				ssd1306_WriteString("           ", Font_11x18, White);
				ssd1306_SetCursor(0,22);
				ssd1306_WriteString(aux_message, Font_11x18, White);
				ssd1306_UpdateScreen();
			}

		}


	}// end while

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
