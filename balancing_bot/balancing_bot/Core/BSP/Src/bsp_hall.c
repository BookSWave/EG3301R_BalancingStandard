/*
 * bsp_hall.c
 *
 *  Created on: Mar 21, 2024
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"
//to switch from active high to low whenever
extern motor_data_t g_can_motors[24];

static uint8_t prev_state = HALL_OFF;

uint8_t hall_state = HALL_OFF;


void hall_enable(){
	hall_state = HALL_ON;
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void hall_disable(){
	hall_state = HALL_OFF;
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}


void hall_int(){
	static int32_t prev_tick;
	uint8_t curr_state = (GPIOE->IDR & GPIO_PIN_11) ? 1 : 0;
	if (curr_state & GPIO_PIN_11){
		//current state and previous state are the same, do nothing
		return;
	}
	if (hall_state == HALL_ON){
	//assume hall sensor has changed state
		switch (curr_state){
		case HALL_OFF:
			//hall went from on to off
			//motor just brushed past
			prev_tick = g_can_motors[YAW_MOTOR_ID-1].angle_data.ticks;

			break;
		case HALL_ON:
			//hall went from off to on
			//yaw motor just let goo
			g_can_motors[YAW_MOTOR_ID-1].angle_data.ticks = (g_can_motors[YAW_MOTOR_ID-1].angle_data.ticks-prev_tick)/2 ;
			//sensor is done, turn it off
			hall_disable();
			break;

		default:
			prev_state = 1;
		}
		prev_state = curr_state;
		return;
	} else{
		hall_disable();
		return;
	}
}

