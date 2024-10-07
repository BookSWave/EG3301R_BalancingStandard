/*
 * leg_kinematic_task.c
 *
 *  Created on: Jul 14, 2024
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

extern LegPos leftLegPos;
extern LegPos rightLegPos;
double left_F_control;
double left_Tp_control;
double right_F_control;
double right_Tp_control;
extern Target target;

float clamp_angle(float angle, float min_angle, float max_angle) {
    if (angle < min_angle) {
        return min_angle;
    } else if (angle > max_angle) {
        return max_angle;
    } else {
        return angle;
    }
}

void leg_kinematic_task(void *argument) {
	double leftTorque[2];
	double rightTorque[2];
	double starttime = 0;
	double endtime = 0;
	double dt;
	double left_leg_pos[2];
	double right_leg_pos[2];
	PID left_F;
	PID left_Tp;
	PID right_F;
	PID right_Tp;
    TickType_t start_time;
    while (1) {
        endtime = get_microseconds();
        dt = endtime - starttime;
        start_time = xTaskGetTickCount();
        starttime = get_microseconds();
        g_can_motors[6].angle_rad= clamp_angle(g_can_motors[6].angle_rad,g_can_motors[6].angle_data.min_ang,g_can_motors[6].angle_data.max_ang);
        g_can_motors[7].angle_rad= clamp_angle(g_can_motors[7].angle_rad,g_can_motors[7].angle_data.min_ang,g_can_motors[7].angle_data.max_ang);

        /////////
       leg_pos(g_can_motors[6].angle_rad,g_can_motors[7].angle_rad,left_leg_pos); //8->4 9->5
       leftLegPos.length = left_leg_pos[0];
       leftLegPos.angle = left_leg_pos[1];
       leftLegPos.angle = leftLegPos.angle - PI/2;
    //    leg_pos(g_can_motors[6].angle_rad,g_can_motors[7].angle_rad,right_leg_pos);//17->6 16->7
    //    rightLegPos.length = right_leg_pos[0];
    //    rightLegPos.angle = right_leg_pos[1];
       PID_Init(&left_F, 5, 0, 0.01, -1000, 1000); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
       PID_Init(&left_Tp, 0.01, 0.01, 0.01, -10, 10); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
    //    PID_Init(&right_F, 5000, 0, 0.01, -1000, 1000); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
    //    PID_Init(&right_Tp, 0.01, 0.01, 0.01, -10, 10);
       left_F_control = PID_Compute(&left_F, 0.18, leftLegPos.length, dt,0.001);
      //  left_Tp_control = PID_Compute(&left_Tp, PI/2, leftLegPos.angle, dt,0.001);
    //    right_F_control = PID_Compute(&right_F, 0.13, rightLegPos.length, dt,0.001);
    //    right_Tp_control = -PID_Compute(&right_Tp, 0, rightLegPos.angle, dt,0.001);
       leg_conv(left_F_control,left_Tp_control,g_can_motors[6].angle_rad,g_can_motors[7].angle_rad,leftTorque);
      //  leg_conv(right_F_control,right_Tp_control,g_can_motors[6].angle_rad,g_can_motors[7].angle_rad,rightTorque);
       g_can_motors[6].torque = leftTorque[0];
       g_can_motors[7].torque = leftTorque[1];
    //    g_can_motors[4].torque = rightTorque[0];
    //    g_can_motors[5].torque = rightTorque[1];
        ///////////////////
        vTaskDelayUntil(&start_time, 5);
    }
}


