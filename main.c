/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f0xx_hal.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "wwdg.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "CANopen.h"
#include "ds401.h"
#include "application.h"
#include "eeprom.h"

#define CO_FCY     48000

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint32_t CO_timer1ms = 0U; /* variable increments each millisecond */
volatile uint8_t digital_input[2] = {0x00, 0x00};
volatile uint8_t digital_output[2] = {0x00, 0x00};
CO_EE_t	CO_EEO;         /* Eeprom object */
CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
bool_t CO_Timer_EN = CO_false;
uint8_t I2C_State = 0;

uint32_t can_error = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void JumpersRead(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	CO_ReturnError_t eeStatus;
	uint16_t timer1msPrevious;
	uint16_t timer1msDiff;
	
	oneWire1.searchFlag = 0;
	oneWire1.found_devices = 0;
	 
	
	osKernelInitialize();                    // initialize CMSIS-RTOS
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();

  /* USER CODE BEGIN 2 */	
	
  MCP23S08_Init();
  		
  /* initialize EEPROM - part 1*/
  eeStatus = CO_EE_init_1(&CO_EEO, (uint8_t*) &CO_OD_EEPROM, sizeof(CO_OD_EEPROM), (uint8_t*) &CO_OD_ROM, sizeof(CO_OD_ROM));
  
  JumpersRead();
  
  /* Intit WWDG*/
  MX_WWDG_Init();
  
  /* Application interface */
  programStart();
	
  /* USER CODE END 2 */

   /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(reset != CO_RESET_APP)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		/* CANopen communication reset - initialize CANopen objects *******************/
		
		/* Communication Reset */
		communicationReset();
		
		/* initialize EEPROM - part 2 */
		CO_EE_init_2(&CO_EEO, eeStatus, CO->SDO[0], CO->em);
		
		/* initialize variables */
		timer1msPrevious = CO_timer1ms = 0;
		OD_performance[ODA_performance_mainCycleMaxTime] = 0;
		OD_performance[ODA_performance_timerCycleMaxTime] = 0;
		reset = CO_RESET_NOT;

		/* start Timer interupts*/
		NVIC_EnableIRQ(TIM17_IRQn);
		HAL_TIM_Base_Start_IT(&htim17);
		
		/*tid_TIMThread enable CAN receive interrupts */
		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
		
		/* start CAN */
    CO_CANsetNormalMode(CO->CANmodule[0]);
		
		while(reset == CO_RESET_NOT)
		{
			/* calculate cycle time for performance measurement */
			CO_DISABLE_INTERRUPTS();
			timer1msDiff = CO_timer1ms - timer1msPrevious;
			timer1msPrevious = CO_timer1ms;
			CO_ENABLE_INTERRUPTS();
			
			/* Application interface */
			programAsync(timer1msDiff);
			
			/* CANopen process */
			reset = CO_process(CO, timer1msDiff, NULL);
			
			/* Process EEPROM */
			CO_EE_process(&CO_EEO);
			
		}
	}

	/* program exit ***************************************************************/
	CO_DISABLE_INTERRUPTS();
	
	/* Application interface */
	programEnd();

	/* delete objects from memory */
	CO_delete(0/* CAN module address */);

	/* reset */
	NVIC_SystemReset();
	
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	CO_CANinterrupt_Tx(CO->CANmodule[0]);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	CO_CANinterrupt_Rx(CO->CANmodule[0]);
	
	/* enable CAN receive interrupts */
	HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}


/* For debug CAN Errors Status*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	can_error = HAL_CAN_GetError(hcan);
}


/* Read Jumper state from PCB */
void JumpersRead(void)
{
	uint8_t jspeed;
	uint8_t jaddress;
	
	jspeed = (HAL_GPIO_ReadPin( SBIT0_GPIO_Port, SBIT0_Pin)) | ((HAL_GPIO_ReadPin( SBIT1_GPIO_Port, SBIT1_Pin)) << 1);
	
	switch (jspeed)
  {
  	case 0:
			CO_OD_ROM.CANBitRate = 1000;
  		break;
  	case 1:
			CO_OD_ROM.CANBitRate = 500;
  		break;
		case 2:
			CO_OD_ROM.CANBitRate = 250;
  		break;
		case 3:
			CO_OD_ROM.CANBitRate = 125;
  		break;
  	default:
  		break;
  }
	
	/* Invers jumper bits state*/
	jaddress = (
		(~HAL_GPIO_ReadPin( BIT0_GPIO_Port, BIT0_Pin) & 1)
	| ((~HAL_GPIO_ReadPin( BIT1_GPIO_Port, BIT1_Pin)& 1) << 1)
	| ((~HAL_GPIO_ReadPin( BIT2_GPIO_Port, BIT2_Pin)& 1) << 2)
	| ((~HAL_GPIO_ReadPin( BIT3_GPIO_Port, BIT3_Pin)& 1) << 3)
	| ((~HAL_GPIO_ReadPin( BIT4_GPIO_Port, BIT4_Pin)& 1) << 4)
	| ((~HAL_GPIO_ReadPin( BIT5_GPIO_Port, BIT5_Pin)& 1) << 5)
);
	
	/* max address = 64*/
	CO_OD_ROM.CANNodeID = jaddress + 1;
	
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */
	uint16_t t;
/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM17)
	{
		CO_timer1ms++;

    /* Application interface */
    program1ms();
		
		/* calculate cycle time for performance measurement */
    t = htim17.Instance->CNT / (CO_FCY / 100);
		
    OD_performance[ODA_performance_timerCycleTime] = t;
    if(t > OD_performance[ODA_performance_timerCycleMaxTime])
		{
        OD_performance[ODA_performance_timerCycleMaxTime] = t;
		}
	}
/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
