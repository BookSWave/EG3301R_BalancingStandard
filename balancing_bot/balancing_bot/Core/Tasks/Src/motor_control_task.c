/*
 * motor_control_task.c
 *
 *  Created on: Sep 26, 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_control.h"
#include "motor_control_task.h"

extern motor_data_t g_can_motors[24];
extern QueueHandle_t g_buzzing_task_msg;
extern remote_cmd_t g_remote_cmd;

extern uint8_t g_safety_toggle;
volatile uint32_t g_motor_control_time;
void motor_control_task(void *argument) {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box;
	int16_t temp_converter;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;
	uint32_t enabled_motors = 0;
	//initialise motor data
#ifdef LEFT_MOTOR_ID
	if (LEFT_MOTOR_ID < 25 && LEFT_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (LEFT_MOTOR_ID - 1);
	}
#endif

#ifdef RIGHT_MOTOR_ID
	if (RIGHT_MOTOR_ID < 25 && RIGHT_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (RIGHT_MOTOR_ID - 1);
	}
#endif
#ifdef FR_MOTOR_ID
	if (FR_MOTOR_ID < 25 && FR_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (FR_MOTOR_ID - 1);
	}
#endif

#ifdef FL_MOTOR_ID
	if (FL_MOTOR_ID < 25 && FL_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (FL_MOTOR_ID - 1);
	}
#endif

#ifdef BL_MOTOR_ID
	if (BL_MOTOR_ID < 25 && BL_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (BL_MOTOR_ID - 1);
	}
#endif

#ifdef BR_MOTOR_ID
	if (BR_MOTOR_ID < 25 && BR_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (BR_MOTOR_ID - 1);
	}
#endif

#ifdef LFRICTION_MOTOR_ID
	if (LFRICTION_MOTOR_ID < 25 && LFRICTION_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (LFRICTION_MOTOR_ID - 1);
	}
#endif

#ifdef RFRICTION_MOTOR_ID
	if (RFRICTION_MOTOR_ID < 25 && RFRICTION_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (RFRICTION_MOTOR_ID - 1);
	}
#endif

#ifdef FEEDER_MOTOR_ID
	if (FEEDER_MOTOR_ID < 25 && FEEDER_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (FEEDER_MOTOR_ID - 1);
	}
#endif

#ifdef PITCH_MOTOR_ID
	if (PITCH_MOTOR_ID < 25 && PITCH_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (PITCH_MOTOR_ID - 1);
	}
#endif

#ifdef YAW_MOTOR_ID
	if (YAW_MOTOR_ID < 25 && YAW_MOTOR_ID > 0) {
		enabled_motors = enabled_motors | 1 << (YAW_MOTOR_ID - 1);
	}
#endif
	TickType_t start_time;
//	uint32_t last_time;
	while (1) {
		start_time = xTaskGetTickCount();
		if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN){
			CAN_send_data[0] = 0;
			CAN_send_data[1] = 0;
			CAN_send_data[2] = 0;
			CAN_send_data[3] = 0;
			CAN_send_data[4] = 0;
			CAN_send_data[5] = 0;
			CAN_send_data[6] = 0;
			CAN_send_data[7] = 0;
			CAN_tx_message.StdId = 0x200;
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					&send_mail_box);;
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			vTaskDelay(1);
			CAN_tx_message.StdId = 0x1FF;
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					&send_mail_box);;
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			vTaskDelay(1);;
			CAN_tx_message.StdId = 0x2FF;
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					&send_mail_box);;
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			vTaskDelayUntil(&start_time, 5);
			continue;
		}
		TickType_t delay = 0;
		if (enabled_motors & 0x00000F) {
			CAN_tx_message.StdId = 0x200;
			temp_converter = g_can_motors[0x0].output;
			CAN_send_data[0] = temp_converter >> 8;
			CAN_send_data[1] = temp_converter;
			temp_converter = g_can_motors[0x1].output;
			CAN_send_data[2] = temp_converter >> 8;
			CAN_send_data[3] = temp_converter;
			temp_converter = g_can_motors[0x2].output;
			CAN_send_data[4] = temp_converter >> 8;
			CAN_send_data[5] = temp_converter;
			temp_converter = g_can_motors[0x3].output;
			CAN_send_data[6] = temp_converter >> 8;
			CAN_send_data[7] = temp_converter;
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			delay++;
		}
		if (enabled_motors & 0x00F000) {
			CAN_tx_message.StdId = 0x200;
			CAN_send_data[0] = g_can_motors[0x0 + 12].output >> 8;
			CAN_send_data[1] = g_can_motors[0x0 + 12].output;
			CAN_send_data[2] = g_can_motors[0x1 + 12].output >> 8;
			CAN_send_data[3] = g_can_motors[0x1 + 12].output;
			CAN_send_data[4] = g_can_motors[0x2 + 12].output >> 8;
			CAN_send_data[5] = g_can_motors[0x2 + 12].output;
			CAN_send_data[6] = g_can_motors[0x3 + 12].output >> 8;
			CAN_send_data[7] = g_can_motors[0x3 + 12].output;
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			delay++;
		}

		vTaskDelay(1);

		if (enabled_motors & 0x0000F0) {
			CAN_tx_message.StdId = 0x1FF;
			temp_converter = g_can_motors[0x4].output;
			CAN_send_data[0] = temp_converter >> 8;
			CAN_send_data[1] = temp_converter;
			temp_converter = g_can_motors[0x5].output;
			CAN_send_data[2] = temp_converter >> 8;
			CAN_send_data[3] = temp_converter;
			temp_converter = g_can_motors[0x6].output;
			CAN_send_data[4] = temp_converter >> 8;
			CAN_send_data[5] = temp_converter;
			temp_converter = g_can_motors[0x7].output;
			CAN_send_data[6] = temp_converter >> 8;
			CAN_send_data[7] = temp_converter;
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			delay++;
		}


		if (enabled_motors & 0x0F0000) {
			CAN_tx_message.StdId = 0x1FF;
			CAN_send_data[0] = g_can_motors[0x4 + 12].output >> 8;
			CAN_send_data[1] = g_can_motors[0x4 + 12].output;
			CAN_send_data[2] = g_can_motors[0x5 + 12].output >> 8;
			CAN_send_data[3] = g_can_motors[0x5 + 12].output;
			CAN_send_data[4] = g_can_motors[0x6 + 12].output >> 8;
			CAN_send_data[5] = g_can_motors[0x6 + 12].output;
			CAN_send_data[6] = g_can_motors[0x7 + 12].output >> 8;
			CAN_send_data[7] = g_can_motors[0x7 + 12].output;
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			delay++;
		}


		if (enabled_motors & 0x000F00) {
			CAN_tx_message.StdId = 0x2FF;
			temp_converter = g_can_motors[0x8].output;
			CAN_send_data[0] = temp_converter >> 8;
			CAN_send_data[1] = temp_converter;
			temp_converter = g_can_motors[0x9].output;
			CAN_send_data[2] = temp_converter >> 8;
			CAN_send_data[3] = temp_converter;
			temp_converter = g_can_motors[0xA].output;
			CAN_send_data[4] = temp_converter >> 8;
			CAN_send_data[5] = temp_converter;
			temp_converter = g_can_motors[0xB].output;
			CAN_send_data[6] = temp_converter >> 8;
			CAN_send_data[7] = temp_converter;
			HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			delay++;
		}

		if (enabled_motors & 0xF00000) {
			CAN_tx_message.StdId = 0x2FF;
			CAN_send_data[0] = g_can_motors[0x8 + 12].output >> 8;
			CAN_send_data[1] = g_can_motors[0x8 + 12].output;
			CAN_send_data[2] =  g_can_motors[0x9 + 12].output >> 8;
			CAN_send_data[3] =  g_can_motors[0x9 + 12].output;
			CAN_send_data[4] = g_can_motors[0xA + 12].output >> 8;
			CAN_send_data[5] = g_can_motors[0xA + 12].output;
			CAN_send_data[6] = g_can_motors[0xB + 12].output >> 8;
			CAN_send_data[7] = g_can_motors[0xB + 12].output;
			HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
					&send_mail_box);
			delay++;
		}
		delay = (delay > 5) ? delay : 5;
//		last_time = get_microseconds();
		vTaskDelayUntil(&start_time, 2);
//		vTaskDelay(1);



	}
}
