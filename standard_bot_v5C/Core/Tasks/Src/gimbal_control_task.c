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
#include "../BSP/Inc/bsp_imu.h"


extern uint8_t aimbot_mode;

extern EventGroupHandle_t gimbal_event_group;
extern float g_chassis_yaw;
extern motor_data_t can_motors[24];
extern gimbal_control_t gimbal_ctrl_data;
extern orientation_data_t imu_heading;
extern QueueHandle_t telem_motor_queue;
extern imu_raw_t imu_data;

static void jointmotor_control_loop(leg_move_t *leg_move_control_loop);
double L1setpoint  = -0.05;
double R1setpoint  = -0.05;
float dt= 0.04;

//半边机体重力（6kg×9.8=58.8）
#define G 58.8

//两轮间距
#define D 0.35

//Y方向运动的KP和KD值
#define KP_Y 100.0f
#define KD_Y 40.0f

//ROLL方向运动的KP和KD值
#define KP_ROLL 40.0f
#define KD_ROLL 5.0f

//+++++++++++++++++++new para

leg_move_t leg_move;
joint_motor_t joint_motor[4];

#include <math.h>
float height_multiplier = 1.0;


/**
 *
 * FreeRTOS task for gimbal controls
 * Has HIGH2 priority
 *
 */

void joint_motor_init(motor_data_t *motor_data){
	motor_data[7].id = L2_MOTOR_ID;
	motor_data[6].id = L1_MOTOR_ID;
	motor_data[5].id = R1_MOTOR_ID;
	motor_data[4].id = R2_MOTOR_ID;
}

void gimbal_control_task(void *argument) {
	TickType_t start_time;
	//initialize
	joint_motor_init(&can_motors[24]);

	while (1) {
		xEventGroupWaitBits(gimbal_event_group, 0b11, pdTRUE, pdTRUE,
		portMAX_DELAY);
		start_time = xTaskGetTickCount();
//		if (gimbal_ctrl_data.enabled) {
//			if (gimbal_ctrl_data.imu_mode) {
		leg_control(&leg_move,&joint_motor[4]);
		jointmotor_control_loop(&leg_move);
		// motor_send_can(can_motors, R1_MOTOR_ID, R2_MOTOR_ID, L1_MOTOR_ID, L2_MOTOR_ID);

//			} else {
//				gimbal_angle_control(can_motors + PITCH_MOTOR_ID - 1,
//						can_motors + YAW_MOTOR_ID - 1);
//			}
//		} else {
//			can_motors[PITCH_MOTOR_ID - 1].rpm_pid.output = 0;
//			can_motors[YAW_MOTOR_ID - 1].rpm_pid.output = 0;
//			motor_send_can(can_motors, PITCH_MOTOR_ID, YAW_MOTOR_ID, 0, 0);
//		}
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

float limit_angle(double current_angle, motor_data_t *can_motors) {
    // Check if the current angle exceeds the limits and clamp it
    if (current_angle > can_motors->angle_data.max_ang) {
        return can_motors->angle_data.max_ang;
    } else if (current_angle < can_motors->angle_data.min_ang) {
        return can_motors->angle_data.min_ang;
    }

    // Return the angle if it's within the allowed range
    return current_angle;
}



void leg_control(leg_move_t *leg_move_update, joint_motor_t *joint_motor) {
	leg_move_update->joint_motor[0].angle = can_motors[4].angle_data.adj_ang;//R2
	leg_move_update->joint_motor[1].angle = can_motors[5].angle_data.adj_ang;//R1
	leg_move_update->joint_motor[2].angle = can_motors[6].angle_data.adj_ang;//L1
	leg_move_update->joint_motor[3].angle = can_motors[7].angle_data.adj_ang;//L2

//	limit_angle(leg_move_update->joint_motor[0].angle,&can_motors[4]);


	//left and right leg height// todo:find average
	leg_move_update->yR = (sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[0].angle),2))-l2*sin(leg_move_update->joint_motor[0].angle))/height_multiplier;//0 3
	leg_move_update->yL = (sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[2].angle),2))-l2*sin(leg_move_update->joint_motor[2].angle))/height_multiplier;// 0 3
	leg_move_update->y  = (leg_move_update->yR +leg_move_update->yL)/2;

	// acc of Z-axis
	leg_move_update->z_accel = imu_data.accel_data.az;

	//末端执行力系数
	leg_move_update->vyR = -(l1-l2*cos(leg_move_update->joint_motor[0].angle))*l2*sin(leg_move_update->joint_motor[0].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[0].angle),2))-l2*cos(leg_move_update->joint_motor[0].angle);// 0 3
	leg_move_update->vyL = -(l1-l2*cos(leg_move_update->joint_motor[3].angle))*l2*sin(leg_move_update->joint_motor[3].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[3].angle),2))-l2*cos(leg_move_update->joint_motor[3].angle);

	//	Z-axis speed
	leg_move_update->vy  =  leg_move_update->vy + 0;//leg_move_update->y_accel * JOINTMOTOR_CONTROL_TIME_S;

	//chassis euler angle
	leg_move_update->chassis_yaw = imu_heading.yaw;
	leg_move_update->chassis_pitch=imu_heading.pit;
	leg_move_update->chassis_roll = imu_heading.rol;




	// chassis angle speed
	leg_move_update->chassis_yaw_speed = (leg_move_update->chassis_yaw-leg_move_update->chassis_yaw_prev)/dt;
	leg_move_update->chassis_pitch_speed =(leg_move_update->chassis_pitch-leg_move_update->chassis_pitch_prev)/dt;
	leg_move_update->chassis_roll_speed = (leg_move_update->chassis_roll-leg_move_update->chassis_roll_prev)/dt;

	leg_move_update->chassis_yaw_prev = imu_heading.yaw;
	leg_move_update ->chassis_pitch_prev =imu_heading.pit;
	leg_move_update->chassis_roll_prev = imu_heading.rol;

//		yangle_pid(R1setpoint,imu_heading.pit, R1_motor,imu_heading.pit, &prev_R1);


	//	float rel_L1_angle = L1_motor->angle_data.adj_ang - imu_heading.pit; // +remote control data
	//	if (rel_L1_angle > L1_motor->angle_data.max_ang) {
	//		rel_L1_angle = L1_motor->angle_data.max_ang;
	//
	//	}
	//	if (rel_L1_angle < L1_motor->angle_data.min_ang) {
	//		rel_L1_angle = L1_motor->angle_data.min_ang;
	//
	//	}
	//
	////	yangle_pid(L1setpoint,imu_heading.pit, L1_motor,imu_heading.pit, &prev_L1);
	//	float rel_R1_angle = R1_motor->angle_data.adj_ang - imu_heading.pit; // +remote control data
	//	if (rel_R1_angle > R1_motor->angle_data.max_ang) {
	//		rel_R1_angle = R1_motor->angle_data.max_ang;
	//
	//	}
	//	if (rel_R1_angle < R1_motor->angle_data.min_ang) {
	//		rel_R1_angle = R1_motor->angle_data.min_ang;
	//
	//	}


}

float compute_y(float x) {
    // Coefficients for the quadratic equation
    const float a = 0.5469;
    const float b = 0.7563;
    const float c = 0.1025;
    float y = 0;

    // Calculate y using the quadratic formula
    if (x>=0){
		y = a * x * x + b * x + c;
	}
    else{
     	y = -a * x * x + b * x + c;
    }

    	y = y/3*16385;
    if (abs(y)<350){
    	y = 0;

    }else if (abs(y)>10000){
    	if (y >0){
    		y = 10000;
    	}else{
    		y= -10000;

    	}
    }
    return y;
}

static void jointmotor_control_loop(leg_move_t *leg_move_control_loop)
{
	//计算y方向和ROLL方向设定值与反馈值的偏差
	leg_move_control_loop->delta_y = 0.15 - leg_move_control_loop->y;//leg_move_control_loop->y_set
	leg_move_control_loop->delta_roll = leg_move_control_loop->roll_set - leg_move_control_loop->chassis_roll;

	//计算左腿和右腿的末端执行力
	leg_move_control_loop->FL = (KP_Y * leg_move_control_loop->delta_y - KD_Y * leg_move_control_loop->vyL + G)/2 + (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;
	leg_move_control_loop->FR = (KP_Y * leg_move_control_loop->delta_y - KD_Y * leg_move_control_loop->vyR + G)/2 - (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;

	//计算每个关节电机的控制力矩
	leg_move_control_loop->joint_motor[0].torque_set = 0.5*leg_move_control_loop->FR*leg_move_control_loop->vyR; //R2
	leg_move_control_loop->joint_motor[1].torque_set = -0.5*leg_move_control_loop->FR*leg_move_control_loop->vyR;//R1
	leg_move_control_loop->joint_motor[2].torque_set = 0.5*leg_move_control_loop->FL*leg_move_control_loop->vyL;//L1
	leg_move_control_loop->joint_motor[3].torque_set = -0.5*leg_move_control_loop->FL*leg_move_control_loop->vyL;//L2
	leg_move_control_loop->joint_motor[0].current = compute_y(leg_move_control_loop->joint_motor[0].torque_set)*1.2;
	leg_move_control_loop->joint_motor[1].current = compute_y(leg_move_control_loop->joint_motor[1].torque_set)*1.2;
	leg_move_control_loop->joint_motor[2].current = compute_y(leg_move_control_loop->joint_motor[2].torque_set)*1.2;
	leg_move_control_loop->joint_motor[3].current = compute_y(leg_move_control_loop->joint_motor[3].torque_set)*1.2;
	leg_move_control_loop->joint_motor[0].id = can_motors[4].id;
	leg_move_control_loop->joint_motor[1].id = can_motors[5].id;
	leg_move_control_loop->joint_motor[2].id = can_motors[6].id;
	leg_move_control_loop->joint_motor[3].id = can_motors[7].id;
	leg_move_control_loop->joint_motor[0].raw_angle = can_motors[4].raw_data.angle[0];
	leg_move_control_loop->joint_motor[1].raw_angle = can_motors[5].raw_data.angle[0];
	leg_move_control_loop->joint_motor[2].raw_angle = can_motors[6].raw_data.angle[0];
	leg_move_control_loop->joint_motor[3].raw_angle = can_motors[7].raw_data.angle[0];


	can_motors[4].rpm_pid.output = leg_move_control_loop->joint_motor[0].current;
	can_motors[5].rpm_pid.output = leg_move_control_loop->joint_motor[1].current;
	can_motors[6].rpm_pid.output = leg_move_control_loop->joint_motor[2].current;
	can_motors[7].rpm_pid.output = leg_move_control_loop->joint_motor[3].current;

}

//void send_can_msg(){
//	leg_move_control_loop->joint_motor[0].current = compute_y(leg_move_control_loop->joint_motor[0].torque_set);
//	leg_move_control_loop->joint_motor[1].current = compute_y(leg_move_control_loop->joint_motor[1].torque_set);
//	leg_move_control_loop->joint_motor[2].current = compute_y(leg_move_control_loop->joint_motor[2].torque_set);
//	leg_move_control_loop->joint_motor[3].current = compute_y(leg_move_control_loop->joint_motor[3].torque_set);
//}

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
	// motor_send_can(can_motors, PITCH_MOTOR_ID, YAW_MOTOR_ID, 0, 0);
}
