/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t new_byte = 0;

uint8_t crc_new, crc_calc;
float new_left_speed, new_right_speed;

uint8_t transmition_state; // 1- have header, 2 have command, 3 have data, 0 - waiting data
uint8_t command;
uint8_t com_data;

uint8_t input_data[27];

union u8_to_float_
{
	uint8_t u8[4];
	float f;
}u8_to_float;

float target_speed_left;
float target_speed_right;
uint8_t led_state;

uint8_t type_msg; //0xA0 - Velocity 0x0F - PID parameters

float new_pid_parameters[6];

uint8_t new_parameters_available;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
  Name  : CRC-8
  Poly  : 0x31    x^8 + x^5 + x^4 + 1
  Init  : 0xFF
  Revert: false
  XorOut: 0x00
  Check : 0xF7 ("123456789")
  MaxLen: 15 байт(127 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned char Crc8(unsigned char *pcBlock, unsigned int len)
{
    unsigned char crc = 0xFF;
    unsigned int i;

    while (len--)
    {
        crc ^= *pcBlock++;

        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

void check_new_pid_parameters(void)
{
	if(new_parameters_available == 1)
		update_pid_parameters(new_pid_parameters[0], new_pid_parameters[1], new_pid_parameters[2], new_pid_parameters[3], new_pid_parameters[4], new_pid_parameters[5]);
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  track_update_position();
  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */
	HAL_I2C_DeInit(&hi2c2);

	HAL_I2C_Init(&hi2c2);
  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(USART1->SR & USART_SR_RXNE)
	{
		new_byte = USART1->DR;

		if(transmition_state > 1)
		{
			input_data[transmition_state] = new_byte;
			transmition_state++;

			if(type_msg == 0xA0) //velocity
			{
				if(transmition_state == 12)
				{
					u8_to_float.u8[0] = input_data[2];
					u8_to_float.u8[1] = input_data[3];
					u8_to_float.u8[2] = input_data[4];
					u8_to_float.u8[3] = input_data[5];

					new_left_speed = u8_to_float.f;

					u8_to_float.u8[0] = input_data[6];
					u8_to_float.u8[1] = input_data[7];
					u8_to_float.u8[2] = input_data[8];
					u8_to_float.u8[3] = input_data[9];

					new_right_speed = u8_to_float.f;

					led_state = input_data[10];

					crc_new = input_data[11];

					crc_calc = Crc8(input_data+1, 10);

					if(crc_new == crc_calc)
					{
						target_speed_left  = new_left_speed;
						target_speed_right = new_right_speed;
					}

					transmition_state = 0;
				}
			}

			if(type_msg == 0x0F) //PID parameters
			{
				if(transmition_state == 27)
				{
					u8_to_float.u8[0] = input_data[2];
					u8_to_float.u8[1] = input_data[3];
					u8_to_float.u8[2] = input_data[4];
					u8_to_float.u8[3] = input_data[5];

					new_pid_parameters[0] = u8_to_float.f;

					u8_to_float.u8[0] = input_data[6];
					u8_to_float.u8[1] = input_data[7];
					u8_to_float.u8[2] = input_data[8];
					u8_to_float.u8[3] = input_data[9];

					new_pid_parameters[1] = u8_to_float.f;

					u8_to_float.u8[0] = input_data[10];
					u8_to_float.u8[1] = input_data[11];
					u8_to_float.u8[2] = input_data[12];
					u8_to_float.u8[3] = input_data[13];

					new_pid_parameters[2] = u8_to_float.f;

					u8_to_float.u8[0] = input_data[14];
					u8_to_float.u8[1] = input_data[15];
					u8_to_float.u8[2] = input_data[16];
					u8_to_float.u8[3] = input_data[17];

					new_pid_parameters[3] = u8_to_float.f;

					u8_to_float.u8[0] = input_data[18];
					u8_to_float.u8[1] = input_data[19];
					u8_to_float.u8[2] = input_data[20];
					u8_to_float.u8[3] = input_data[21];

					new_pid_parameters[4] = u8_to_float.f;

					u8_to_float.u8[0] = input_data[22];
					u8_to_float.u8[1] = input_data[23];
					u8_to_float.u8[2] = input_data[24];
					u8_to_float.u8[3] = input_data[25];

					new_pid_parameters[5] = u8_to_float.f;

					crc_new = input_data[26];

					crc_calc = Crc8(input_data+1, 25);

					if(crc_new == crc_calc)
					{
						new_parameters_available = 1;
					}

					transmition_state = 0;
				}
			}

			if((type_msg != 0x0F) && (type_msg != 0xA0))
				transmition_state = 0;
		}



		if(transmition_state == 1)
		{
			transmition_state = 2;
			type_msg = new_byte;
			input_data[1] = new_byte;
		}

		if(transmition_state == 0 && new_byte == 0x7E)
		{
			transmition_state = 1;
			input_data[0] = new_byte;
		}
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

	if(USART2->SR & USART_SR_RXNE)
	{
		new_byte = USART2->DR;

		if(transmition_state == 2)
		{
			com_data = new_byte;
			transmition_state = 3;
		}

		if(transmition_state == 1)
		{
			command = new_byte;
			transmition_state = 2;
		}

		if(transmition_state == 0 && new_byte == 0x7E)
			transmition_state = 1;
	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
