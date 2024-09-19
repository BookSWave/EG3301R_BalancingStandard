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
void joint_motor_send_can(leg_move_t *leg_move, uint8_t id_one,uint8_t id_two,uint8_t id_three,uint8_t id_four);

//void jointmotor_control_loop(leg_move_t *leg_move_control_loop);
double L1setpoint  = -0.05;
double R1setpoint  = -0.05;
float dt= 0.04;

//半边机体重力（6kg×9.8=58.8）
#define G 58.8

//两轮间距
#define D 0.35

//Y方向运动的KP和KD值
#define KP_Y 1000.0f
#define KD_Y 400.0f

//ROLL方向运动的KP和KD值
#define KP_ROLL 0//40.0f
#define KD_ROLL 0//5.0f

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

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
	motor_data[7].id = L1_MOTOR_ID;
	motor_data[6].id = L2_MOTOR_ID;
	motor_data[5].id = R2_MOTOR_ID;
	motor_data[4].id = R1_MOTOR_ID;
	jointmotor_control_loop(&leg_move);

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
		leg_control(&leg_move);
		jointmotor_control_loop(&leg_move);
		// joint_motor_send_can(&leg_move, R1_MOTOR_ID, R2_MOTOR_ID, L1_MOTOR_ID, L2_MOTOR_ID);
//
//		}
		status_led(2, off_led);
		xEventGroupClearBits(gimbal_event_group, 0b11);
		vTaskDelayUntil(&start_time, GIMBAL_DELAY);
	}
	//should not run here
}


float calculate_torque(float x) {
    // Calculate y using the absolute value of x
    float y = (1.2 * fabs(x) + 0.1)/3*16000;
    LIMIT_MIN_MAX(y,-16000,16000);

    // Preserve the original sign of x
    if (x < 0) {
        y = -y;
    }

    return y;
}



void leg_control(leg_move_t *leg_move_update) {

	leg_move_update->joint_motor[0].angle = can_motors[4].angle_data.adj_ang;// R1
	leg_move_update->joint_motor[1].angle = can_motors[5].angle_data.adj_ang;//R2
	leg_move_update->joint_motor[2].angle = can_motors[6].angle_data.adj_ang;//L2
	leg_move_update->joint_motor[3].angle = can_motors[7].angle_data.adj_ang+PI;//L1


	//left and right leg height// todo:find average
	leg_move_update->yR1 = (sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[0].angle),2))-l2*sin(leg_move_update->joint_motor[0].angle));//0 3
	leg_move_update->yR2 = (sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[1].angle),2))-l2*sin(-1*leg_move_update->joint_motor[1].angle));//0 3
//	leg_move_update->yR = (leg_move_update->yR2+leg_move_update->yR1)/2;
	leg_move_update->yL2 = (sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[2].angle),2))-l2*sin(leg_move_update->joint_motor[2].angle));// 0 3
	leg_move_update->yL1 = (sqrt(pow(l3,2)-pow(l1-l2*cos(leg_move_update->joint_motor[3].angle),2))-l2*sin(-1*leg_move_update->joint_motor[3].angle));// 0 3
//	leg_move_update->yL = (leg_move_update->yL2+leg_move_update->yL1)/2;
//	leg_move_update->y  = (leg_move_update->yR +leg_move_update->yL)/2;

	// acc of Z-axis
	leg_move_update->z_accel = imu_data.accel_data.az;

	//末端执行力系数
	leg_move_update->vyR1 =-(l1-l2*cos(-1*leg_move_update->joint_motor[0].angle))*l2*sin(-1*leg_move_update->joint_motor[0].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(-1*leg_move_update->joint_motor[0].angle),2))-l2*cos(-1*leg_move_update->joint_motor[0].angle);// 0 3
	leg_move_update->vyR2 = (l1-l2*cos(-1*leg_move_update->joint_motor[1].angle))*l2*sin(-1*leg_move_update->joint_motor[1].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(-1*leg_move_update->joint_motor[1].angle),2))-l2*cos(-1*leg_move_update->joint_motor[1].angle);// 0 3
//	leg_move_update ->vyR = (leg_move_update->vyR1-leg_move_update->vyR2)/2;
	leg_move_update->vyL2 = -(l1-l2*cos(-1*leg_move_update->joint_motor[2].angle))*l2*sin(-1*leg_move_update->joint_motor[2].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(-1*leg_move_update->joint_motor[2].angle),2))-l2*cos(-1*leg_move_update->joint_motor[2].angle);
	leg_move_update->vyL1 = (l1-l2*cos(-1*leg_move_update->joint_motor[3].angle))*l2*sin(-1*leg_move_update->joint_motor[3].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(-1*leg_move_update->joint_motor[3].angle),2))-l2*cos(-1*leg_move_update->joint_motor[3].angle);
//	leg_move_update->vyL = (-leg_move_update->vyL1+leg_move_update->vyL2)/2;
//	//	Z-axis speed
	leg_move_update->vyspeedR2  =  leg_move_update->yR2/0.04;//imu_data.accel_data.az*contol time + 0;//leg_move_update->y_accel * JOINTMOTOR_CONTROL_TIME_S;
	leg_move_update->vyspeedR1  =  leg_move_update->yR1/0.04;//imu_data.accel_data.az*contol time + 0;//leg_move_update->y_accel * JOINTMOTOR_CONTROL_TIME_S;
	leg_move_update->vyspeedL1  =  leg_move_update->yL1/0.04;//imu_data.accel_data.az*contol time + 0;//leg_move_update->y_accel * JOINTMOTOR_CONTROL_TIME_S;
	leg_move_update->vyspeedL2  =  leg_move_update->yL2/0.04;//imu_data.accel_data.az*contol time + 0;//leg_move_update->y_accel * JOINTMOTOR_CONTROL_TIME_S;

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



}
void joint_motor_send_can(leg_move_t *leg_move, uint8_t id_one,uint8_t id_two,uint8_t id_three,uint8_t id_four) {
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint8_t CAN_send_data[8];
	uint32_t send_mail_box;
	uint32_t temp_checker = 0;
	int16_t temp_converter;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;
	if (id_one < 25 && id_one > 0) {
		temp_checker = temp_checker | 1 << (id_one - 1);
	}
	if (id_two < 25 && id_two > 0) {
		temp_checker = temp_checker | 1 << (id_two - 1);
	}
	if (id_three < 25 && id_three > 0) {
		temp_checker = temp_checker | 1 << (id_three - 1);
	}
	if (id_four < 25 && id_four > 0) {
		temp_checker = temp_checker | 1 << (id_four - 1);
	}


	if (temp_checker & 0x0000F0) {
		CAN_tx_message.StdId = 0x1FF;
		temp_converter = leg_move->joint_motor[0].torque_set;
		CAN_send_data[0] = temp_converter >> 8;
		CAN_send_data[1] = temp_converter;
		temp_converter = leg_move->joint_motor[1].torque_set;
		CAN_send_data[2] = temp_converter >> 8;
		CAN_send_data[3] = temp_converter;
		temp_converter = leg_move->joint_motor[2].torque_set;
		CAN_send_data[4] = temp_converter >> 8;
		CAN_send_data[5] = temp_converter;
		temp_converter = leg_move->joint_motor[3].torque_set;
		CAN_send_data[6] = temp_converter >> 8;
		CAN_send_data[7] = temp_converter;
		HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
}




int16_t float_to_uint(float x, float x_min, float x_max, int8_t bits)
{
	x = calculate_torque(x);
    float span = x_max - x_min;
    float offset = x_min;

    return (int16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}


void jointmotor_control_loop(leg_move_t *leg_move_control_loop)

{
	leg_move_control_loop->y_set = 0.2;
	//计算y方向和ROLL方向设定值与反馈值的偏差
	leg_move_control_loop->delta_yL1 = leg_move_control_loop->y_set - leg_move_control_loop->yL1;//leg_move_control_loop->y_set
	leg_move_control_loop->delta_yL2 = leg_move_control_loop->y_set - leg_move_control_loop->yL2;
	leg_move_control_loop->delta_yR1 = leg_move_control_loop->y_set - leg_move_control_loop->yR1;//leg_move_control_loop->y_set
	leg_move_control_loop->delta_yR2 = leg_move_control_loop->y_set - leg_move_control_loop->yR2;

	leg_move_control_loop->delta_roll = leg_move_control_loop->roll_set - leg_move_control_loop->chassis_roll;

	//计算左腿和右腿的末端执行力
	leg_move_control_loop->FL1 = (KP_Y * leg_move_control_loop->delta_yL1 - KD_Y * leg_move_control_loop->vyspeedL1 + G)/2 + (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;
	leg_move_control_loop->FL2 = (KP_Y * leg_move_control_loop->delta_yL2 - KD_Y * leg_move_control_loop->vyspeedL2 + G)/2 + (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;

	leg_move_control_loop->FR1 = (KP_Y * leg_move_control_loop->delta_yR1 - KD_Y * leg_move_control_loop->vyspeedR1 + G)/2 - (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;
	leg_move_control_loop->FR2 = (KP_Y * leg_move_control_loop->delta_yR2 - KD_Y * leg_move_control_loop->vyspeedR2 + G)/2 - (KP_ROLL * leg_move_control_loop->delta_roll - KD_ROLL * leg_move_control_loop->chassis_roll_speed)/D;

	//计算每个关节电机的控制力矩
	leg_move_control_loop->joint_motor[1].torque_set = calculate_torque(0.5*leg_move_control_loop->FR2*leg_move_control_loop->vyR2);//R2
	leg_move_control_loop->joint_motor[0].torque_set = calculate_torque(-0.5*leg_move_control_loop->FR1*leg_move_control_loop->vyR1);//R1

	leg_move_control_loop->joint_motor[2].torque_set = calculate_torque(0.5*leg_move_control_loop->FL2*leg_move_control_loop->vyL2);//L2
	leg_move_control_loop->joint_motor[3].torque_set = calculate_torque(-0.5*leg_move_control_loop->FL1*leg_move_control_loop->vyL1);//L1


	leg_move_control_loop->joint_motor[0].raw_angle = can_motors[4].raw_data.angle[0];
	leg_move_control_loop->joint_motor[1].raw_angle = can_motors[5].raw_data.angle[0];
	leg_move_control_loop->joint_motor[2].raw_angle = can_motors[6].raw_data.angle[0];
	leg_move_control_loop->joint_motor[3].raw_angle = can_motors[7].raw_data.angle[0];

}

