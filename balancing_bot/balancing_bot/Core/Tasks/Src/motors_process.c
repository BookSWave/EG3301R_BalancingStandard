/*
 * motors_process.c
 *
 *  Created on: Jul 12, 2024
 *      Author: YI MING
 */

#include "board_lib.h"
#include "bsp_imu.h"
#include "robot_config.h"
#include "imu_processing_task.h"
#include <stdio.h>
#include <math.h>
#include "motor_config.h"
#include "arm_math.h"
#include "motor_control.h"
#include "can_msg_processor.h"
#include "motors_process.h"
#include "leg_pos.h"
#include "typedefs.h"
#include "PID.h"

extern motor_data_t g_can_motors[24];
double angle_dif[24];
double rev[24] = {0};
extern float motorOutRatio;



void motors_process_task(void *argument) {
	double starttime = 0;
	double endtime = 0;
	double dt;
	vTaskDelay(5);
	double leftWheel_offset = (double)g_can_motors[1].raw_data.angle[0];
	double rightWheel_offset = (double)g_can_motors[0].raw_data.angle[0];
	g_can_motors[1].rpm_pid.kp = 5000;
	g_can_motors[0].rpm_pid.kp = 5000;
	g_can_motors[1].rpm_pid.ki = 0;
	g_can_motors[0].rpm_pid.ki = 0;
	g_can_motors[1].rpm_pid.kd = 0;
	g_can_motors[0].rpm_pid.kd = 0;
	g_can_motors[1].rpm_pid.max_out = 10000;
	g_can_motors[0].rpm_pid.max_out = 10000;
    TickType_t start_time;
    while (1) {
        endtime = get_microseconds();
        dt = endtime - starttime;
        start_time = xTaskGetTickCount();
        raw_angle_to_rad(g_can_motors); // Pass the array directly
        motors_raw_angle_to_desired_angle_dir(&g_can_motors[4], +1.57, 1);//3.62+PI+ // left joint[1]
        motors_raw_angle_to_desired_angle_dir(&g_can_motors[5], +0.48, 1); // leftjoint[0]
        motors_raw_angle_to_desired_angle_dir(&g_can_motors[6], -1.5, 1);
        motors_raw_angle_to_desired_angle_dir(&g_can_motors[7], 0.53-1.103+1.07, 1);
        motors_raw_angle_to_desired_angle_dir(&g_can_motors[1], 0, -1);
        motors_raw_angle_to_desired_angle_dir(&g_can_motors[0], 0, 1);
        motors_torque_to_current_6020(&g_can_motors[4],0,1.0);
        motors_torque_to_current_6020(&g_can_motors[5],0,1.0);
        motors_torque_to_current_6020(&g_can_motors[6],0,1.0);
        motors_torque_to_current_6020(&g_can_motors[7],0,1.0);
        motors_torque_to_current_3508_gearbox(&g_can_motors[0],15,1.0);
        motors_torque_to_current_3508_gearbox(&g_can_motors[1],15,-1.0);


        speed_pid((double)g_can_motors[1].torque ,(double)g_can_motors[1].raw_data.rpm/1000, &g_can_motors[1].rpm_pid);
        speed_pid((double)g_can_motors[0].torque ,(double)g_can_motors[0].raw_data.rpm/1000, &g_can_motors[0].rpm_pid);
//        g_can_motors[1].output = g_can_motors[1].rpm_pid.output;
//        g_can_motors[0].output = g_can_motors[0].rpm_pid.output;
        starttime = get_microseconds();




        vTaskDelayUntil(&start_time, 5);
    }
}

void raw_angle_to_rad(motor_data_t motor[]) { // Change the parameter to accept an array
    for (int i = 0; i < 24; i++) {
        // Update moving average history
        double continuous_angle = motor[i].raw_data.angle[0] + (-motor[i].angle_data.rev*8192);
        motor[i].raw_angle_rad = (continuous_angle / 8191.0) * 2 * PI;
        motor[i].raw_angle_deg = (continuous_angle / 8191.0) * 360;
    }
}

void motors_raw_angle_to_desired_angle_dir(motor_data_t *motor, float offset, int dir) {
    if (dir == -1) {
        motor->angle_rad = -motor->raw_angle_rad + offset;
        motor->speed = -motor->raw_data.rpm * 0.10472;
    } else {
        motor->angle_rad = motor->raw_angle_rad + offset;
        motor->speed = motor->raw_data.rpm * 0.10472;
    }
    motor->angle_deg = motor->angle_rad * (180.0 / PI); // Update the degree value as well

}

void motors_torque_to_current_6020(motor_data_t *motor, float deadzone,double dir) {
	double current = 0;
	if (motor->torque > 0){
		current = 1.8 * motor->torque + 0.2;
	} else if(motor->torque < 0){
		current = 1.8 * motor->torque - 0.2;
	} else{
		current = 0;
	}
	motor->output = ((current * dir)/3.0)*16385.0;
}
void motors_torque_to_current_3508_gearbox(motor_data_t *motor, float maxCurrent,double dir) {
	double current = 0;
	if (motor->torque > 0){
		current = (motor->torque / 4.5) * 20 + 0.08;
	} else if(motor->torque < 0){
		current = (motor->torque / 4.5) * 20 - 0.08;
	} else{
		current = 0;
	}
	if (current > maxCurrent){
		current = maxCurrent;
	} else if(current < -maxCurrent){
		current = -maxCurrent;
	}
	motor->output = ((current * dir)/20.0)*16385.0;
}

