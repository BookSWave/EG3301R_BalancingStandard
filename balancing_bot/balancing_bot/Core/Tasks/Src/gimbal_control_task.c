/*
 * gimbal_control_task.c
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_control.h"
#include "motor_config.h"
#include "can_msg_processor.h"
#include "gimbal_control_task.h"

extern uint8_t aimbot_mode;

extern EventGroupHandle_t gimbal_event_group;
extern float g_chassis_yaw;
extern motor_data_t g_can_motors[24];
extern gimbal_control_t gimbal_ctrl_data;
extern orientation_data_t imu_heading;
extern QueueHandle_t telem_motor_queue;
extern chassis_control_t chassis_ctrl_data;
extern int32_t chassis_rpm;
static float rel_pitch_angle;
uint8_t g_gimbal_state = 0;

extern int g_spinspin_mode;
#ifdef YAW_FEEDFORWARD
static pid_data_t g_yaw_ff_pid = {
			.kp = YAW_FF_SPD_KP,
			.ki = YAW_FF_SPD_KI,
			.kd = YAW_FF_SPD_KD,
			.int_max = YAW_FF_INT_MAX,
			.max_out = YAW_FF_MAX_OUTPUT
};
#endif
float curr_rot;


uint8_t check_yaw(){
	if (HAL_GetTick()- g_can_motors[YAW_MOTOR_ID-1].last_time[0] < 500){
		return 1;
	} else {
		return 0;
	}
}

/**
 *
 * FreeRTOS task for gimbal controls
 * Has HIGH2 priority
 *
 */
void gimbal_control_task(void *argument) {
	TickType_t start_time;
	while (1) {
		xEventGroupWaitBits(gimbal_event_group, 0b11, pdTRUE, pdFALSE,
		portMAX_DELAY);
		start_time = xTaskGetTickCount();
		if (gimbal_ctrl_data.enabled) {
//			calc_chassis_rot(&g_can_motors[FL_MOTOR_ID - 1],
//					&g_can_motors[FR_MOTOR_ID - 1],
//					&g_can_motors[BR_MOTOR_ID - 1],
//					&g_can_motors[BL_MOTOR_ID - 1]);
#ifdef HALL_ZERO
			if (check_yaw()){
				g_gimbal_state = 1;
			}
#endif


			if (gimbal_ctrl_data.imu_mode) {
				gimbal_control(g_can_motors + PITCH_MOTOR_ID - 1,
						g_can_motors + YAW_MOTOR_ID - 1);
			} else {
				gimbal_angle_control(g_can_motors + PITCH_MOTOR_ID - 1,
						g_can_motors + YAW_MOTOR_ID - 1);
			}
		} else {
			g_can_motors[PITCH_MOTOR_ID - 1].output = 0;
			g_can_motors[YAW_MOTOR_ID - 1].output = 0;
		}
		status_led(2, off_led);
		xEventGroupClearBits(gimbal_event_group, 0b11);
		vTaskDelayUntil(&start_time, GIMBAL_DELAY);
	}
	//should not run here
}

/**
 * This function controls the gimbals based on IMU reading
 * @param 	pitch_motor		Pointer to pitch motor struct
 * 			yaw_motor		Pointer to yaw motor struct
 * @note both pitch and yaw are currently on CAN2 with ID5 and 6.
 * Need to check if having ID4 (i.e. 0x208) + having the launcher motors (ID 1-3, 0x201 to 0x203)
 * still provides a fast enough response
 */
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {

}

void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor) {

	if (gimbal_ctrl_data.pitch > pitch_motor->angle_data.max_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.pitch < pitch_motor->angle_data.min_ang) {
		gimbal_ctrl_data.pitch = pitch_motor->angle_data.min_ang;
	}

	if (gimbal_ctrl_data.yaw > yaw_motor->angle_data.max_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.max_ang;
	}
	if (gimbal_ctrl_data.yaw < yaw_motor->angle_data.min_ang) {
		gimbal_ctrl_data.yaw = yaw_motor->angle_data.min_ang;
	}
	angle_pid(gimbal_ctrl_data.pitch, pitch_motor->angle_data.adj_ang,
			pitch_motor);
	angle_pid(gimbal_ctrl_data.yaw, yaw_motor->angle_data.adj_ang, yaw_motor);

	pitch_motor->output = pitch_motor->rpm_pid.output;
	yaw_motor->output = yaw_motor->rpm_pid.output;
}
