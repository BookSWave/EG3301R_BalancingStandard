/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"

extern EventGroupHandle_t chassis_event_group;
extern chassis_control_t chassis_ctrl_data;

extern gimbal_control_t gimbal_ctrl_data;
extern motor_data_t g_can_motors[24];
extern referee_limit_t g_referee_limiters;
extern ref_game_robot_data_t ref_robot_data;
extern speed_shift_t gear_speed;
float g_chassis_yaw = 0;
int32_t chassis_rpm = MAX_SPEED;
extern uint8_t g_gimbal_state;
extern uint8_t hall_state;

extern orientation_data_t imu_heading;
float motor_yaw_mult[4];

extern QueueHandle_t telem_motor_queue;

static float g_chassis_rot;

pid_data_t b_speed_pid;
pid_data_t b_angle_pid;
pid_data_t b_accel_pid;


void b_pid_init(){
	b_speed_pid.kp = B_SPEED_KP;
	b_speed_pid.ki = B_SPEED_KI;
	b_speed_pid.kd = B_SPEED_KD;
	b_speed_pid.int_max = B_SPEED_INT_MAX;
	b_speed_pid.max_out = B_MAX_ANG;

	b_angle_pid.kp = B_ANGLE_KP;
	b_angle_pid.ki = B_ANGLE_KI;
	b_angle_pid.kd = B_ANGLE_KD;
	b_angle_pid.int_max = B_ANGLE_INT_MAX;
	b_angle_pid.max_out = B_MAX_CURRENT;
}


void movement_control_task(void *argument) {
	TickType_t start_time;
	//initialise in an array so it's possible to for-loop it later
	motor_yaw_mult[0] = FR_YAW_MULT;
	motor_yaw_mult[1] = FL_YAW_MULT;
	motor_yaw_mult[2] = BL_YAW_MULT;
	motor_yaw_mult[3] = BR_YAW_MULT;
	b_pid_init();
	while (1) {

#ifndef CHASSIS_MCU

		EventBits_t motor_bits;
		//wait for all motors to have updated data before PID is allowed to run
		motor_bits = xEventGroupWaitBits(chassis_event_group, 0b1111, pdTRUE,
		pdTRUE,
		portMAX_DELAY);
		if (motor_bits == 0b1111) {
			status_led(3, on_led);
			start_time = xTaskGetTickCount();
			if (chassis_ctrl_data.enabled) {

#ifdef HALL_ZERO
			if (g_gimbal_state){
				hall_enable();
				while(hall_state == HALL_ON)
				yaw_zeroing(g_can_motors + FR_MOTOR_ID - 1,
						g_can_motors + FL_MOTOR_ID - 1,
						g_can_motors + BL_MOTOR_ID - 1,
						g_can_motors + BR_MOTOR_ID - 1);
			}
#endif
				chassis_motion_control(g_can_motors + FR_MOTOR_ID - 1,
						g_can_motors + FL_MOTOR_ID - 1,
						g_can_motors + BL_MOTOR_ID - 1,
						g_can_motors + BR_MOTOR_ID - 1);
			} else {
				g_can_motors[FR_MOTOR_ID - 1].output = 0;
				g_can_motors[FL_MOTOR_ID - 1].output = 0;
				g_can_motors[BL_MOTOR_ID - 1].output = 0;
				g_can_motors[BR_MOTOR_ID - 1].output = 0;

			}
#else
		chassis_MCU_send_CAN();
#endif
			status_led(3, off_led);
		} else {
			//motor timed out
			g_can_motors[FR_MOTOR_ID - 1].output = 0;
			g_can_motors[FL_MOTOR_ID - 1].output = 0;
			g_can_motors[BL_MOTOR_ID - 1].output = 0;
			g_can_motors[BR_MOTOR_ID - 1].output = 0;
		}
		//clear bits if it's not already cleared
		xEventGroupClearBits(chassis_event_group, 0b1111);
		//delays task for other tasks to run
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}
void chassis_MCU_send_CAN() {

}
static uint32_t chassis_rpm_max = LV1_MAX_SPEED;


float vforward;
float pit_turned_spd;
void chassis_motion_control(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr) {

}

