/*
 * PID.c
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




void PID_Init(PID *pid, double kp, double ki, double kd, double min_output, double max_output) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->max_output = max_output;
    pid->min_output = min_output;
}

double PID_Compute(PID *pid, double setpoint, double measured_value, double dt, double deadzone) {
    double error = setpoint - measured_value;
    if (error < deadzone && error > -deadzone ){
    	error = 0.0;
    }
    pid->integral += error * dt;
    double derivative = (error - pid->prev_error) / dt;
    double output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // Clamp the output to the specified max and min limits
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }

    pid->prev_error = error;
    pid->output =  output;

    // Comment this out if you are testing the leg locking only
    return output;
}

void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb, double dt)
{
	PID_Compute(&pid->outer,angleRef,angleFdb,dt,0.001);//�����⻷(�ǶȻ�)
	PID_Compute(&pid->inner,pid->outer.output,speedFdb,dt,0.001);//�����ڻ�(�ٶȻ�)
	pid->output=pid->inner.output;
}

