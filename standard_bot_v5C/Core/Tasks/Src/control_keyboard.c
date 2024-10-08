/*
 * control_keyboard.c
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "control_input_task.h"
#include "control_keyboard.h"
#include "motor_control.h"

extern remote_cmd_t remote_cmd;
extern QueueHandle_t buzzing_task_msg;
extern orientation_data_t imu_heading;
extern chassis_control_t chassis_ctrl_data;
extern gun_control_t launcher_ctrl_data;
extern gimbal_control_t gimbal_ctrl_data;
extern uint8_t safety_toggle;
extern uint8_t launcher_safety_toggle;

extern int g_spinspin_mode;



void keyboard_control_input() {
	mouse_gimbal_input();
	keyboard_chassis_input();
	mouse_launcher_control_input();
}

void keyboard_gear_shifter(speed_shift_t *gear_speed) {
	static uint32_t shift_press_time;
	static uint32_t ctrl_press_time;
	uint8_t temp_msg;
	if ((remote_cmd.keyboard_keys & KEY_OFFSET_SHIFT)
			&& (remote_cmd.keyboard_keys & KEY_OFFSET_CTRL)) {
		gear_speed->curr_gear = 3;
		shift_press_time = HAL_GetTick();
		ctrl_press_time = HAL_GetTick();
	}

	else if (remote_cmd.keyboard_keys & KEY_OFFSET_SHIFT) {
		if (HAL_GetTick() - shift_press_time > 100) {
			gear_speed->curr_gear =
					(gear_speed->curr_gear < 5) ? gear_speed->curr_gear + 1 : 5;
		}
		shift_press_time = HAL_GetTick();
	}

	else if (remote_cmd.keyboard_keys & KEY_OFFSET_CTRL) {
		if (HAL_GetTick() - ctrl_press_time > 100) {
			gear_speed->curr_gear =
					(gear_speed->curr_gear > 1) ? gear_speed->curr_gear - 1 : 1;
		}
		ctrl_press_time = HAL_GetTick();
	}

}

void keyboard_chassis_input() {
	if (safety_toggle || remote_cmd.right_switch != ge_RSW_ALL_ON) {
		chassis_ctrl_data.enabled = 0;
		chassis_ctrl_data.horizontal = 0;
		chassis_ctrl_data.forward = 0;
		chassis_ctrl_data.yaw = 0;
	} else {
		if (remote_cmd.right_switch == ge_RSW_ALL_ON) {
			chassis_ctrl_data.enabled = 1;
			float horizontal_input = 0.0;
			float forward_input = 0.0;
			float yaw_input = 0.0;


#ifdef CHASSIS_CAN_SPINSPIN
			if (remote_cmd.keyboard_keys & KEY_OFFSET_Q) {
				g_spinspin_mode = 1;
			} else if (remote_cmd.keyboard_keys & KEY_OFFSET_E) {
				g_spinspin_mode = 0;
			}
#endif

			if (remote_cmd.keyboard_keys & KEY_OFFSET_W) {
				forward_input += KEYBD_MAX_SPD;
			}
			if (remote_cmd.keyboard_keys & KEY_OFFSET_S) {
				forward_input -= KEYBD_MAX_SPD;
			}

			if (remote_cmd.keyboard_keys & KEY_OFFSET_A) {
				horizontal_input -= KEYBD_MAX_SPD;
			}
			if (remote_cmd.keyboard_keys & KEY_OFFSET_D) {
				horizontal_input += KEYBD_MAX_SPD;
			}

			if (g_spinspin_mode) {
				yaw_input = g_spinspin_mode * CHASSIS_SPINSPIN_MAX;
			} else {
				//center yaw motor such that yaw motor = 0
				yaw_input = chassis_center_yaw();
			}
			chassis_ctrl_data.horizontal = horizontal_input;
			chassis_ctrl_data.forward = forward_input;
			chassis_ctrl_data.yaw = yaw_input;
		}
	}
}

void mouse_gimbal_input() {
	if (safety_toggle || remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
		gimbal_ctrl_data.enabled = 0;
	} else {
		gimbal_ctrl_data.enabled = 1;
		float pitch_mouse = (float) remote_cmd.mouse_y * MOUSE_Y_INVERT
				* PITCH_INVERT * MOUSE_Y_SENSITIVITY / 32768;
		float yaw_mouse = (float) remote_cmd.mouse_x * MOUSE_X_INVERT
				* YAW_INVERT * MOUSE_X_SENSITIVITY / 32768;
		gimbal_turn_ang(pitch_mouse, yaw_mouse);
	}
}

void mouse_launcher_control_input() {
	if (safety_toggle || launcher_safety_toggle
			|| remote_cmd.right_switch == ge_RSW_SHUTDOWN
			|| remote_cmd.left_switch != ge_LSW_UNSAFE) {
		if (remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
			launcher_ctrl_data.enabled = 0;
		}
		launcher_ctrl_data.gun_feeding_speed = 0;
		launcher_ctrl_data.projectile_speed = 0;
		if (remote_cmd.left_switch != ge_LSW_UNSAFE) {
			launcher_safety_toggle = 0;
		}
	} else {
		launcher_ctrl_data.enabled = 1;
		launcher_ctrl_data.projectile_speed = 1;
		if (remote_cmd.left_switch == ge_LSW_UNSAFE) {
			if (remote_cmd.mouse_right) {
				launcher_ctrl_data.override = 1;
			} else {
				launcher_ctrl_data.override = 0;
			}
		}
		if (remote_cmd.mouse_left) {
			launcher_ctrl_data.gun_feeding_speed =1;

		} else {
			launcher_ctrl_data.gun_feeding_speed = 0;
		}
	}
}
