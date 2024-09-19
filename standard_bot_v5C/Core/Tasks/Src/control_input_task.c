/*
 * control_input_task.c
 *
 *  Created on: 4 Jul 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "control_input_task.h"
#include "motor_control.h"
#include "control_keyboard.h"
#include "control_remote.h"
#include "control_sbc.h"

//extern TaskHandle_t buzzing_task_handle;
//extern TaskHandle_t gimbal_control_task_handle;
//extern TaskHandle_t movement_control_task_handle;
//extern TaskHandle_t control_input_task_handle;

extern motor_data_t can_motors[24];
extern referee_limit_t referee_limiters;
extern orientation_data_t imu_heading;

extern QueueHandle_t buzzing_task_msg;
extern remote_cmd_t remote_cmd;
extern ref_robot_dmg_t ref_dmg_data;
extern uint32_t ref_dmg_data_txno;

chassis_control_t chassis_ctrl_data;
gun_control_t launcher_ctrl_data;
gimbal_control_t gimbal_ctrl_data;
pid_data_t yaw_pid_data;
speed_shift_t gear_speed;
int g_spinspin_mode = 0;

uint8_t control_mode = CONTROL_DEFAULT;
uint8_t safety_toggle = ARM_SWITCH;
uint8_t launcher_safety_toggle = ARM_SWITCH;



void control_input_task(void *argument) {
	TickType_t start_time;
	control_reset();
	chassis_yaw_pid_init();
	gimbal_ctrl_data.imu_mode = GIMBAL_MODE;
	aimbot_pid_init();
	dbus_remote_start();
	gear_speed.curr_gear = 3;
	gear_speed.spin_mult = GEAR3_YAW_MULT;
	gear_speed.trans_mult = GEAR3_SPEED_MULT;
	gear_speed.accel_mult = GEAR3_ACCEL_MULT;
	safety_toggle = 1;
	vTaskDelay(100);
	uint8_t rc_check;
	while (fabs(remote_cmd.left_x) > 50 || fabs(remote_cmd.right_x) > 50 || fabs(remote_cmd.left_x) > 50 || fabs(remote_cmd.right_x) > 50){
		uint8_t temp_msg;
		temp_msg = not_ok;
		xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
		rc_check = ulTaskNotifyTake(pdTRUE, 200);
		HAL_UART_DMAStop(&DBUS_UART);
		dbus_remote_start();
		if (rc_check){
			vTaskDelay(200);
		}
	}
	safety_toggle = ARM_SWITCH;


	while (1) {
		rc_check = ulTaskNotifyTake(pdTRUE, 200);
		if (rc_check) {
			status_led(1, on_led);
			start_time = xTaskGetTickCount();
			if (remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
				laser_on();
				control_mode_change(remote_cmd.side_dial);
				safety_toggle = 0;
				launcher_safety_toggle = 0;
				control_reset();
			} else {

				switch (control_mode) {
				case KEYBOARD_CTRL_MODE:

					keyboard_gear_shifter(&gear_speed);
					set_gear();
					keyboard_control_input();
					break;
				case REMOTE_CTRL_MODE:
					remote_gear_shifter(&gear_speed);
					set_gear();
					remote_control_input();
					break;
#ifdef HAS_SBC
				case SBC_CTRL_MODE:
					sbc_control_input();
					break;
#endif HAS_SBC

				default:
					break;

				}
				status_led(1, off_led);
			}
		} else {
			//restart remote uart
			if (HAL_GetTick() - remote_cmd.last_time > 100) {
				HAL_UART_DMAStop(&DBUS_UART);
				dbus_remote_start();
				remote_cmd.last_time = HAL_GetTick();
			}
			kill_can();
			control_reset();
			launcher_safety_toggle = LAUNCHER_SAFETY;
			safety_toggle = ARM_SWITCH;

		}
		vTaskDelayUntil(&start_time, CONTROL_DELAY);
	}
	osThreadTerminate(NULL);
}

float chassis_center_yaw() {
	speed_pid(0, can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang,
			&yaw_pid_data);
	return yaw_pid_data.output;
}

void chassis_set_ctrl(float forward, float horizontal, float yaw){
	chassis_ctrl_data.enabled = 1;
	chassis_ctrl_data.horizontal = horizontal;
	chassis_ctrl_data.forward = forward;
	chassis_ctrl_data.yaw = yaw;
}

void chassis_kill_ctrl(){
	chassis_ctrl_data.enabled = 0;
	chassis_ctrl_data.forward = 0;
	chassis_ctrl_data.horizontal = 0;
	chassis_ctrl_data.yaw = 0;
}
uint8_t gimbal_aim_at_damaged_plate(float* yaw_rad) {
	static uint32_t last_dmg_data;
	if (last_dmg_data != ref_dmg_data_txno) {
		last_dmg_data = ref_dmg_data_txno;
		if (ref_dmg_data.dmg_type == 0) {
			switch (ref_dmg_data.armor_type) {
			case 1:
				*yaw_rad = imu_heading.yaw
						- can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang
						+ (PI / 2);

				return 1;
			case 2:
				*yaw_rad = imu_heading.yaw
						- can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang
						+ (PI);

				return 1;
			case 3:
				*yaw_rad = imu_heading.yaw
						- can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang
						- (PI / 2);

				return 1;
			case 0:
				*yaw_rad = imu_heading.yaw
						- can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;

				return 1;
			default:
				break;

			}
		}
	}
	return 0;
}

void control_reset() {
	chassis_ctrl_data.forward = 0;
	chassis_ctrl_data.horizontal = 0;
	chassis_ctrl_data.yaw = 0;
	chassis_ctrl_data.enabled = 0;
	gimbal_ctrl_data.pitch = 0;
	gimbal_ctrl_data.yaw = imu_heading.yaw;
	gimbal_ctrl_data.enabled = 0;
	launcher_ctrl_data.gun_feeding_speed = 0;
	launcher_ctrl_data.projectile_speed = 0;
	launcher_ctrl_data.enabled = 0;
	g_spinspin_mode = 0;
	laser_off();
}

void control_mode_change(int16_t left_dial_input) {
//assume already in shutdown mode here
	static uint32_t last_trig_time;
	uint8_t temp_msg;
	if (remote_cmd.left_switch == ge_LSW_CONFIG) {
		if (left_dial_input > 330 || left_dial_input < -330) {
			if (HAL_GetTick() - last_trig_time > 1000) {
				switch (control_mode) {
				case KEYBOARD_CTRL_MODE:
				case REMOTE_CTRL_MODE:
					control_mode = SBC_CTRL_MODE;
					temp_msg = control_sbc;
					xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
					break;
				default:
					last_trig_time = HAL_GetTick();
					break;
				}
			}
		} else {
			last_trig_time = HAL_GetTick();
		}

	} else {
		switch (control_mode) {
		case KEYBOARD_CTRL_MODE:
			if (left_dial_input < -330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = REMOTE_CTRL_MODE;
					temp_msg = control_control;
					xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
					launcher_safety_toggle = LAUNCHER_SAFETY;
				}
			} else {
				last_trig_time = HAL_GetTick();
			}
			break;
		case REMOTE_CTRL_MODE:
			if (left_dial_input > 330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = KEYBOARD_CTRL_MODE;
					temp_msg = control_keyboard;
					launcher_safety_toggle = LAUNCHER_SAFETY;
					xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
				}
			} else {
				last_trig_time = HAL_GetTick();
			}
			break;
		case SBC_CTRL_MODE:
			if (left_dial_input < -330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = REMOTE_CTRL_MODE;
					temp_msg = control_control;
					launcher_safety_toggle = LAUNCHER_SAFETY;
					xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
				}
			} else if (left_dial_input > 330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = KEYBOARD_CTRL_MODE;
					temp_msg = control_keyboard;
					launcher_safety_toggle = LAUNCHER_SAFETY;
					xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
				}
			} else {
				last_trig_time = HAL_GetTick();
			}
			break;
		default:
			break;
		}
	}
}

//ADDs angle to gimbal ctrl
void gimbal_turn_ang(float pit_radians, float yaw_radians) {
	yaw_radians += gimbal_ctrl_data.yaw += yaw_radians;
	while (yaw_radians > PI) {
		yaw_radians -= 2 * PI;
	}
	while (yaw_radians < -PI) {
		yaw_radians += 2 * PI;
	}
	gimbal_ctrl_data.pitch += pit_radians;
	gimbal_ctrl_data.yaw = yaw_radians;
}
//SETs angle to gimbal ctrl
void gimbal_set_ang(float pit_radians, float yaw_radians) {
	while (yaw_radians > PI) {
		yaw_radians -= 2 * PI;
	}
	while (yaw_radians < -PI) {
		yaw_radians += 2 * PI;
	}
	gimbal_ctrl_data.pitch = pit_radians;
	gimbal_ctrl_data.yaw = yaw_radians;
}

void set_gear() {
	switch (gear_speed.curr_gear) {
	case 1:
		gear_speed.spin_mult = GEAR1_YAW_MULT;
		gear_speed.trans_mult = GEAR1_SPEED_MULT;
		gear_speed.accel_mult = GEAR1_ACCEL_MULT;
		break;
	case 2:
		gear_speed.spin_mult = GEAR2_YAW_MULT;
		gear_speed.trans_mult = GEAR2_SPEED_MULT;
		gear_speed.accel_mult = GEAR2_ACCEL_MULT;
		break;
	case 3:
		gear_speed.spin_mult = GEAR3_YAW_MULT;
		gear_speed.trans_mult = GEAR3_SPEED_MULT;
		gear_speed.accel_mult = GEAR3_ACCEL_MULT;
		break;
	case 4:
		gear_speed.spin_mult = GEAR4_YAW_MULT;
		gear_speed.trans_mult = GEAR4_SPEED_MULT;
		gear_speed.accel_mult = GEAR4_ACCEL_MULT;
		break;
	case 5:
		gear_speed.spin_mult = GEAR5_YAW_MULT;
		gear_speed.trans_mult = GEAR5_SPEED_MULT;
		gear_speed.accel_mult = GEAR5_ACCEL_MULT;
		break;
	case 6:
		gear_speed.spin_mult = GEAR6_YAW_MULT;
		gear_speed.trans_mult = GEAR6_SPEED_MULT;
		gear_speed.accel_mult = GEAR6_ACCEL_MULT;
		break;
	default:
		gear_speed.spin_mult = GEAR3_YAW_MULT;
		gear_speed.trans_mult = GEAR3_SPEED_MULT;
		gear_speed.accel_mult = GEAR3_ACCEL_MULT;
		break;
	}

}

void chassis_yaw_pid_init() {
	yaw_pid_data.kp = CHASSIS_YAW_KP;
	yaw_pid_data.ki = CHASSIS_YAW_KI;
	yaw_pid_data.kd = CHASSIS_YAW_KD;
	yaw_pid_data.max_out = CHASSIS_YAW_MAX_RPM;
}



void dbus_reset() {
	remote_cmd.right_switch = ge_RSW_SHUTDOWN;
	remote_cmd.right_x = 0;
	remote_cmd.right_y = 0;
	remote_cmd.left_x = 0;
	remote_cmd.left_y = 0;
	remote_cmd.left_switch = 0;
	remote_cmd.mouse_x = 0;
	remote_cmd.mouse_y = 0;
	remote_cmd.mouse_z = 0;
	remote_cmd.mouse_left = 0;
	remote_cmd.mouse_right = 0;
	if (control_mode == 0) {
		gimbal_ctrl_data.pitch = 0;
		gimbal_ctrl_data.yaw = 0;
	}
	if (control_mode == 1) {
		gimbal_ctrl_data.pitch = imu_heading.pit;
		gimbal_ctrl_data.yaw = imu_heading.yaw;
	}
}

