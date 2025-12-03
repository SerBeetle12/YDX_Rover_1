/*
 * encoder.c
 *
 *  Created on: Oct 20, 2025
 *      Author: danil
 */
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

float 	track_left_position 		= 0;
float 	track_left_old_position 	= 0;
float 	track_left_speed 			= 0;
uint8_t track_left_direction = 0;

float 	track_right_position 		= 0;
float 	track_right_old_position 	= 0;
float 	track_right_speed 			= 0;
uint8_t track_right_direction = 0;

//test
uint16_t debug_pwm;

float RPM_right;
float Velocity_right;

float RPM_left;
float Velocity_left;

float track_get_absolute_position(uint8_t track_number)
{
	//HAL_Delay(1);
	if(track_number == 1)
		return track_left_position;
	if(track_number == 2)
		return track_right_position;
	return 0;
}

float get_raw_angle(uint8_t wheel)  // 1 - left, 2 -right
{
	uint8_t data[2] = {0, 0};
	float result = 0;

	if(wheel == 2)
		HAL_I2C_Mem_Read(&hi2c2, 0x0C, 0x03, 1, data, 2, 1);

	if(wheel == 1)
		HAL_I2C_Mem_Read(&hi2c1, 0x0C, 0x03, 1, data, 2, 1);

	result = (float)((uint16_t)data[0]<<6 | data[1]>>2)/16384.0f;

	return result;
}

void track_update_position(void)
{

#define HALF_ROTATE 0.5f
#define FULL_ROTATE 1.0f
#define TIME_DISCRETE_ROTATE 0.01f


	//Left track
	float x0 = track_left_old_position;
	float x1 = get_raw_angle(1);
	float delta = 0;

	if(x1 == x0)
	{
		track_left_speed = 0;
	}
	else
	{
		if(x1 < x0)
		{
			if((x0 - x1) > HALF_ROTATE)
			{
				delta = FULL_ROTATE - x0 + x1;
				track_left_position -= ((float)delta)/FULL_ROTATE;
				track_left_speed = -1.0f * delta/TIME_DISCRETE_ROTATE;
			}
			else
			{
				delta = x0 - x1;
				track_left_position += ((float)delta)/FULL_ROTATE;
				track_left_speed = delta/TIME_DISCRETE_ROTATE;
			}
		}
		else
		{
			if((x1 - x0) > HALF_ROTATE)
			{
				delta = FULL_ROTATE - x1 + x0;
				track_left_position += ((float)delta)/FULL_ROTATE;
				track_left_speed = delta/TIME_DISCRETE_ROTATE;
			}
			else
			{
				delta = x1 - x0;
				track_left_position -= ((float)delta)/FULL_ROTATE;
				track_left_speed = -1.0f * delta/TIME_DISCRETE_ROTATE;
			}
		}
	}

	track_left_old_position = x1;

	//Right track
	x0 = track_right_old_position;
	x1 = get_raw_angle(2);
	delta = 0;

	if(x1 == x0)
	{
		track_right_speed = 0;
	}
	else
	{
		if(x1 < x0)
		{
			if((x0 - x1) > HALF_ROTATE)
			{
				delta = FULL_ROTATE - x0 + x1;
				track_right_position += ((float)delta)/FULL_ROTATE;
				track_right_speed = delta/TIME_DISCRETE_ROTATE;
			}
			else
			{
				delta = x0 - x1;
				track_right_position -= ((float)delta)/FULL_ROTATE;
				track_right_speed = -1.0f * delta/TIME_DISCRETE_ROTATE;
			}
		}
		else
		{
			if((x1 - x0) > HALF_ROTATE)
			{
				delta = FULL_ROTATE - x1 + x0;
				track_right_position -= ((float)delta)/FULL_ROTATE;
				track_right_speed = -1.0f * delta/TIME_DISCRETE_ROTATE;
			}
			else
			{
				delta = x1 - x0;
				track_right_position += ((float)delta)/FULL_ROTATE;
				track_right_speed = delta/TIME_DISCRETE_ROTATE;
			}
		}
	}

	track_right_old_position = x1;

	RPM_left = track_left_position/0.3948f;
	Velocity_left = track_left_speed * 16.0;

	RPM_right = track_right_position/0.3948f;
	Velocity_right = track_right_speed * 16.2;
}

void set_start_position()
{
	track_left_old_position  = get_raw_angle(1);
	track_right_old_position = get_raw_angle(2);
}

void set_zero_position()
{
	track_left_position  = 0;
	track_left_speed 	 = 0;
	track_right_position = 0;
	track_right_speed 	 = 0;
}

void left_wheel(int8_t speed)
{
	//PWM channel TIM2->CH2 (PA1)
	//Reverse pin PB12
	if(speed == 0)
	{
		TIM2->CCR2 = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
		return;
	}

	if(speed > 0) //Движение вперёд
	{
		debug_pwm = (uint16_t)((float)speed * -1.111f + 121.111f);
		TIM2->CCR2 = debug_pwm;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	}
	else
	{
		debug_pwm = (uint16_t)((float)speed * -0.7778f + 42.222f);
		TIM2->CCR2 = debug_pwm;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	}
}

void right_wheel(int8_t speed)
{
	//PWM channel TIM2->CH1 (PA0)
	//Reverse pin PC15
	if(speed == 0)
	{
		TIM2->CCR1 = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0);
		return;
	}

	if(speed > 0) //Движение вперёд
	{
		debug_pwm = (uint16_t)((float)speed * -1.111f + 121.111f);

		TIM2->CCR1 = debug_pwm;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);
	}
	else
	{

		debug_pwm = (uint16_t)((float)speed * -0.7778f + 42.222f);
		TIM2->CCR1 = debug_pwm;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0);
	}
}

float kp_left  = 2;
float kp_right = 2;

float ki_left = 0.5;
float ki_right = 0.5;

float k_vel_rad_left = 1.6;
float k_vel_rad_right = 2.22916;

float integral_left;
float integral_right;


//temp
float reg_left, reg_right;

int start_engine_delay = 8;

void update_pid_parameters(float new_kpl, float new_kpr, float new_kil, float new_kir, float new_kl, float new_kr)
{
	//kp_left = new_kpl;
	//kp_right = new_kpr;

	//ki_left = new_kil;
	//ki_right = new_kir;

	//k_vel_rad_left = new_kl;
	//k_vel_rad_right = new_kr;
}

void go_to_velocity(float tvl, float tvr)
{

	integral_left += ki_left * (tvl - Velocity_left);

	if(tvl == 0)
		integral_left = 0;

	reg_left = tvl * k_vel_rad_left + kp_left * (tvl - Velocity_left) + integral_left;
	//reg_right = tvr;

	if(tvl < 0 && Velocity_left == 0)
	{
		left_wheel(-125);
		HAL_Delay(start_engine_delay);
		left_wheel(0);
	}
	//reg_left = tvl;

	if(reg_left > 126)
		reg_left = 126;
	if(reg_left < -126)
		reg_left = -126;

	integral_right += ki_right * (tvr - Velocity_right);

	if(tvr == 0)
		integral_right = 0;

	reg_right = tvr * k_vel_rad_right + kp_right * (tvr - Velocity_right) + integral_right;
	//reg_right = tvr;

	if(tvr < 0 && Velocity_right == 0)
	{
		right_wheel(-125);
		HAL_Delay(start_engine_delay);
		right_wheel(0);
	}

	if(reg_right > 126)
		reg_right = 126;
	if(reg_right < -126)
		reg_right = -126;

	left_wheel((int8_t)reg_left);
	right_wheel((int8_t)reg_right);
}

float track_get_error_left(float tp)
{
	float err = tp - track_get_absolute_position(1);
	if(err > 0)
		return err;
	return err*-1;
}

float track_get_error_right(float tp)
{
	float err = tp - track_get_absolute_position(2);
	if(err > 0)
		return err;
	return err*-1;
}
