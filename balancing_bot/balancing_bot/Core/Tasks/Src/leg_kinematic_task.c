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
extern orientation_data_t imu_heading;
double left_F_control;
double left_Tp_control;
double right_F_control;
double right_Tp_control;
extern Target target;
float motor_output;

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
	PID leftwheel_rpm;
    TickType_t start_time;
    while (1) {
        endtime = get_microseconds();
        dt = endtime - starttime;
        start_time = xTaskGetTickCount();
        starttime = get_microseconds();
        /////////
//       leg_pos(g_can_motors[FR_MOTOR_ID-1].angle_rad,g_can_motors[FL_MOTOR_ID-1].angle_rad,left_leg_pos);
//       leftLegPos.length = left_leg_pos[0];
//       leftLegPos.angle = left_leg_pos[1];
//       leg_pos(g_can_motors[BR_MOTOR_ID-1].angle_rad,g_can_motors[BL_MOTOR_ID-1].angle_rad,right_leg_pos);
//       rightLegPos.length = right_leg_pos[0];
//       rightLegPos.angle = right_leg_pos[1];
//       PID_Init(&left_F, 5000, 0, 0.01, -1000, 1000); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
//       PID_Init(&left_Tp, 0.01, 0.01, 0.01, -10, 10); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
//       PID_Init(&right_F, 5000, 0, 0.01, -1000, 1000); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
//       PID_Init(&right_Tp, 0.01, 0.01, 0.01, -10, 10);
//       left_F_control = -PID_Compute(&left_F, 0.17, leftLegPos.length, dt, 0.0001);
//       left_Tp_control = -PID_Compute(&left_Tp, 0, leftLegPos.angle, dt, 0.0001);
//       right_F_control = PID_Compute(&right_F, 0.17, rightLegPos.length, dt, 0.0001);
//       right_Tp_control = -PID_Compute(&right_Tp, 0, rightLegPos.angle, dt, 0.0001);
//       leg_conv(left_F_control,left_Tp_control,g_can_motors[FR_MOTOR_ID-1].angle_rad,g_can_motors[FL_MOTOR_ID-1].angle_rad,leftTorque);
//       leg_conv(right_F_control,right_Tp_control,g_can_motors[BR_MOTOR_ID-1].angle_rad,g_can_motors[BL_MOTOR_ID-1].angle_rad,rightTorque);
//       g_can_motors[FR_MOTOR_ID-1].torque = leftTorque[0];
//       g_can_motors[FL_MOTOR_ID-1].torque = leftTorque[1];
//       g_can_motors[BL_MOTOR_ID-1].torque = rightTorque[0];
//       g_can_motors[BR_MOTOR_ID-1].torque = rightTorque[1];
        ///////////////////

		        // leftWheel.angle = (double)g_can_motors[LEFT_MOTOR_ID-1].angle_rad/19.2;
        		// rightWheel.angle = (double)g_can_motors[RIGHT_MOTOR_ID-1].angle_rad/19.2;

                leg_pos(g_can_motors[FR_MOTOR_ID-1].angle_rad,g_can_motors[FL_MOTOR_ID-1].angle_rad,left_leg_pos);
                leftLegPos.length = left_leg_pos[0];
                leftLegPos.angle = left_leg_pos[1];
                leg_pos(g_can_motors[BL_MOTOR_ID-1].angle_rad,g_can_motors[BR_MOTOR_ID-1].angle_rad,right_leg_pos);
                rightLegPos.length = right_leg_pos[0];
                rightLegPos.angle = right_leg_pos[1];
        		// PID_Init(&left_F, 5000, 0, 0.01, -1000, 1000); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
        		// PID_Init(&left_Tp, 0.01, 0.01, 0.01, -10, 10); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
                // PID_Init(&right_F, 5000, 0, 0.01, -1000, 1000); // Example gains: kp = 1.0, ki = 0.1, kd = 0.01, min_output = -10, max_output = 10
                // PID_Init(&right_Tp, 0.01, 0.01, 0.01, -10, 10);
                PID_Init(&left_F, 1000, 0, 0.01, -1000, 1000);
        		PID_Init(&right_F, 1000, 0, 0.01, -1000, 1000);
                PID_Init(&right_Tp, 1.2, 0, 0, -10, 10);
        		PID_Init(&left_Tp, 1.2, 0, 0, -10, 10);
				PID_Init(&leftwheel_rpm,2,0.001,0.01,-1000,1000);


                left_F_control = -PID_Compute(&left_F, 0.12, leftLegPos.length, dt, 0.0001);
                left_Tp_control = -PID_Compute(&left_Tp, PI/2, leftLegPos.angle, dt, 0.00001);

                right_F_control = PID_Compute(&right_F, 0.12, rightLegPos.length, dt, 0.0001);
                right_Tp_control = PID_Compute(&right_Tp, PI/2, rightLegPos.angle, dt, 0.00001);

				motor_output = PID_Compute(&leftwheel_rpm,0,imu_heading.pit,dt,0.0001);
				g_can_motors[0].torque = -motor_output;
				g_can_motors[1].torque = -motor_output;


                leg_conv(left_F_control,left_Tp_control,g_can_motors[FR_MOTOR_ID-1].angle_rad,g_can_motors[FL_MOTOR_ID-1].angle_rad,leftTorque);
                leg_conv(right_F_control,right_Tp_control,g_can_motors[BL_MOTOR_ID-1].angle_rad,g_can_motors[BR_MOTOR_ID-1].angle_rad,rightTorque);
                 g_can_motors[FR_MOTOR_ID-1].torque = leftTorque[0];
                 g_can_motors[FL_MOTOR_ID-1].torque = leftTorque[1];
                g_can_motors[BL_MOTOR_ID-1].torque = rightTorque[0];
                g_can_motors[BR_MOTOR_ID-1].torque = rightTorque[1];
        vTaskDelayUntil(&start_time, 5);
    }
}



