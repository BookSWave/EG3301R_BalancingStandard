/*
 * wheel_control_task.c
 *
 *  Created on: 28 Aug, 2024
 *      Author: wbo19
 */
#include "board_lib.h"
#include "bsp_imu.h"
#include "bsp_dbus_input.h"
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
#include "wheel_control_task.h"
#include <stdbool.h>
#include "lqr_k.h"

extern motor_data_t can_motors[24];
extern remote_cmd_t g_remote_cmd;
extern orientation_data_t imu_heading;
Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;
LegPos leftLegPos, rightLegPos;
StateVar stateVar;
Target target = {0, 0, 0, 0, 0, 0, 0.15f};
GroundDetector groundDetector = {10, 10, true, false};
StandupState standupState = StandupState_Standup; //StandupState_None; change after implementing switch
PID legAnglePID, legLengthPID; //腿部角度和长度控制PID
PID yawPID, rollPID; //机身yaw和roll控制PID

float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效
float sourceVoltage = 12; //当前电源电压
double lqrOutTp_check;
double lqrOutT_check;
double leftF_check;
double leftTp_check;
double rightF_check;
double rightTp_check;
double check_dt;
//double l1;
//double l4;
//double r1;
//double r4;
double k_check[12] = {0};
float check_x = 0;

void balancing_task(void *argument) {
	double starttime = 0;
	double endtime = 0;
	double dt;
    TickType_t start_time;
    Ctrl_Init();
    while (1) {
        endtime = get_microseconds();
        dt = endtime - starttime;
        start_time = xTaskGetTickCount();
        starttime = get_microseconds();

        target.speedCmd = 0;//((float)g_remote_cmd.left_y/660)*1.0f;
        target.yawSpeedCmd = 0;//-((float)g_remote_cmd.left_x/660)*2.0f;
        target.legLength = 0;//((float)g_remote_cmd.right_y/660)*0.04 + 0.12f;
        leftJoint[1].angle = (double)can_motors[8].angle_rad;
        leftJoint[1].speed = (double)can_motors[8].speed;
        leftJoint[0].angle = (double)can_motors[9].angle_rad;
        leftJoint[0].speed = (double)can_motors[9].speed;
        rightJoint[1].angle = (double)can_motors[16].angle_rad;
        rightJoint[1].speed = (double)can_motors[16].speed;
        rightJoint[0].angle = (double)can_motors[17].angle_rad;
        rightJoint[0].speed = (double)can_motors[17].speed;
        leftWheel.angle = (double)can_motors[1].angle_rad/19.2;
        rightWheel.angle = (double)can_motors[0].angle_rad/19.2;
        leftWheel.speed = -(double)can_motors[1].raw_data.rpm * (2*PI/60) /19.2;
        rightWheel.speed = (double)can_motors[0].raw_data.rpm * (2*PI/60) /19.2;

        vTaskDelayUntil(&start_time, 4);
    }
}

void Ctrl_TargetUpdateTask()
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 0.005f;
	while(1){
		//根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
			float legLength = (leftLegPos.length + rightLegPos.length) / 2;
			speedSlopeStep = -(legLength - 0.12f) * 0.03f + 0.005f;

			//计算速度斜坡，斜坡值更新到target.speed
			// if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
			// 	target.speed = target.speedCmd;
			// else
			// {
			// 	if(target.speedCmd - target.speed > 0)
			// 		target.speed += speedSlopeStep;
			// 	else
			// 		target.speed -= speedSlopeStep;
			// }
            //target.speed = target.speedCmd;

			//计算位置目标，并限制在当前位置的±0.1m内
			target.position += target.speed * 0.004f;
			if(target.position - stateVar.x > 0.1f)
				target.position = stateVar.x + 0.1f;
			else if(target.position - stateVar.x < -0.1f)
				target.position = stateVar.x - 0.1f;

			//限制速度目标在当前速度的±0.3m/s内
			if(target.speed - stateVar.dx > 1.0f)
				target.speed = stateVar.dx + 1.0f;
			else if(target.speed - stateVar.dx < -1.0f)
				target.speed = stateVar.dx - 1.0f;

			//计算yaw方位角目标
			target.yawAngle += target.yawSpeedCmd * 0.004f;
			vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}

void LegPos_UpdateTask(void *arg)
{
	const float lpfRatio = 0.5f; //低通滤波系数(新值的权重)
	float lastLeftDLength = 0, lastRightDLength = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		double legPos[2], legSpd[2];

		//计算左腿位置
		leg_pos(leftJoint[1].angle, leftJoint[0].angle, legPos);
		leftLegPos.length = legPos[0];
		leftLegPos.angle = legPos[1];

		//计算左腿速度
		leg_spd(leftJoint[1].speed, leftJoint[0].speed, leftJoint[1].angle, leftJoint[0].angle, legSpd);
		leftLegPos.dLength = legSpd[0];
		leftLegPos.dAngle = legSpd[1];

		//计算左腿腿长加速度
		leftLegPos.ddLength = ((leftLegPos.dLength - lastLeftDLength) * 1000 / 4) * lpfRatio + leftLegPos.ddLength * (1 - lpfRatio);
		lastLeftDLength = leftLegPos.dLength;

		//计算右腿位置
		leg_pos(rightJoint[1].angle, rightJoint[0].angle, legPos);
		rightLegPos.length = legPos[0];
		rightLegPos.angle = legPos[1];

		//计算右腿速度
		leg_spd(rightJoint[1].speed, rightJoint[0].speed, rightJoint[1].angle, rightJoint[0].angle, legSpd);
		rightLegPos.dLength = legSpd[0];
		rightLegPos.dAngle = legSpd[1];

		//计算右腿腿长加速度
		rightLegPos.ddLength = ((rightLegPos.dLength - lastRightDLength) * 1000 / 4) * lpfRatio + rightLegPos.ddLength * (1 - lpfRatio);
		lastRightDLength = rightLegPos.dLength;

		vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
	}
}

void Ctrl_Task(void *arg)
{
	double starttime = 0;
	double endtime = 0;
	double dt;
	const float wheelRadius = 0.045f; //m，车轮半径
	const float legMass = 0.01f; //kg，腿部质量

	TickType_t xLastWakeTime = xTaskGetTickCount();

	//手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
	float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
						{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
	float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;

	//设定初始目标值
	target.rollAngle = 0.0f;
	target.legLength = 0.12f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;

	while (1)
	{
		endtime = get_microseconds();
		dt = endtime - starttime;
		check_dt = dt;
		starttime = get_microseconds();
		//计算状态变量
		stateVar.phi = imu_heading.pit;
		stateVar.dPhi = imu_heading.dpit;
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
		stateVar.theta = 0;//(leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - imu_heading.pit;
		stateVar.dTheta = 0;//(leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imu_heading.dpit;
		double legLength = 0.12;//(leftLegPos.length + rightLegPos.length) / 2;
		double dLegLength = 0;//(leftLegPos.dLength + rightLegPos.dLength) / 2;

		//如果正在站立准备状态，则不进行后续控制
		if(standupState == StandupState_Prepare)
		{
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}

		//计算LQR反馈矩阵
		double kRes[12] = {0}, k[2][6] = {0};
		lqr_k(legLength, kRes);
		lqr_k(0.12, k_check);
		if(groundDetector.isTouchingGround) //正常触地状态
		{
            // double k[2][6] = {
            //     {-0.0224, -2.2530, -11.1054, -1.6155, 2.2361, 0.5513},
            //     {-0.0224, -2.2530, -11.1054, -1.6155, -2.2361, -0.5513}
            // };
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 2; j++)
					k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
			}
		}
		else //腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
		{
			memset(k, 0, sizeof(k));
			k[1][0] = kRes[1] * -2;
			k[1][1] = kRes[3] * -10;
		}

		//准备状态变量
		float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
		//与给定量作差
		x[2] -= target.position;
		x[3] -= target.speed;
		check_x = x[2];
		//矩阵相乘，计算LQR输出
		float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
		float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];
		lqrOutT_check = -lqrOutT;
		lqrOutTp_check = -lqrOutTp;
		//计算yaw轴PID输出
		PID_Compute(&yawPID, target.yawAngle, imu_heading.yaw,0.004,0);

		//设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
		if(groundDetector.isTouchingGround) //正常接地状态
		{
//			Motor_SetTorque(&leftWheel, -lqrOutT * lqrTRatio - yawPID.output);
//			Motor_SetTorque(&rightWheel, -lqrOutT * lqrTRatio + yawPID.output);
			can_motors[1].torque = lqrOutT * lqrTRatio + yawPID.output;
			can_motors[0].torque = -lqrOutT * lqrTRatio - yawPID.output;
		}
		else //腿部离地状态，关闭车轮电机
		{
//			Motor_SetTorque(&leftWheel, 0);
//			Motor_SetTorque(&rightWheel, 0);
			can_motors[1].torque = 0;
			can_motors[0].torque = 0;
		}

		//根据离地状态修改目标腿长，并计算腿长PID输出
		PID_Compute(&legLengthPID, (groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? target.legLength : 0.2f, legLength,0.004,0.001);
		//计算roll轴PID输出
		PID_Compute(&rollPID, target.rollAngle, imu_heading.rol,0.004,0.001);
		//根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
		double leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? +rollPID.output : 0) + 13;
		double rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? -rollPID.output : 0) + 13;
//		double leftForce = legLengthPID.output + 5.0;
//		double rightForce = legLengthPID.output + 5.0;
		if(leftLegPos.length > 0.22f) //保护腿部不能伸太长
			leftForce -= (leftLegPos.length - 0.2f) * 1;
		if(rightLegPos.length > 0.22f)
			rightForce -= (rightLegPos.length - 0.2f) * 1;

		//计算左右腿的地面支持力
		groundDetector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (leftLegPos.ddLength - imu_heading.ddz - 9.8f);
		groundDetector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (rightLegPos.ddLength - imu_heading.ddz - 9.8f);
		//更新离地检测器数据
		static uint32_t lastTouchTime = 0;
		bool isTouchingGround = groundDetector.leftSupportForce > -100 && groundDetector.rightSupportForce > -100; //判断当前瞬间是否接地
		if(!isTouchingGround && (get_microseconds()/1000) - lastTouchTime < 1000) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
			isTouchingGround = true;
		if(!groundDetector.isTouchingGround && isTouchingGround) //判断转为接地状态，标记进入缓冲状态
		{
			target.position = stateVar.x;
			groundDetector.isCuchioning = true;
			lastTouchTime = (get_microseconds()/1000);
		}
		if(groundDetector.isCuchioning && legLength < target.legLength) //缓冲状态直到腿长压缩到目标腿长结束
			groundDetector.isCuchioning = false;
		groundDetector.isTouchingGround = isTouchingGround;

		//计算左右腿角度差PID输出
		// PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,0.004,0.01);
        legAnglePID.output = 0;

		//计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
//		double leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.15f);
//		double rightTp = -lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.15f);
		double leftTp = -lqrOutTp * lqrTpRatio + legAnglePID.output * (leftLegPos.length / 0.1f);
		double rightTp = -lqrOutTp * lqrTpRatio - legAnglePID.output * (rightLegPos.length / 0.1f);



		//使用VMC计算各关节电机输出扭矩
		double leftJointTorque[2]={0};
		leg_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
		double rightJointTorque[2]={0};
		leg_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);

		leftF_check = leftForce;
		leftTp_check = leftTp;
		rightF_check = rightForce;
		rightTp_check = rightTp;
		//保护腿部角度不超限 to be enabled after inplimenting switch
		// if (g_remote_cmd.right_switch == 3){
		// 	standupState = StandupState_Standup;
		// }else{
		// 	standupState = StandupState_None;
		// }
		float leftTheta = leftLegPos.angle - imu_heading.pit - M_PI_2;
		float rightTheta = rightLegPos.angle - imu_heading.pit - M_PI_2;
		#define PROTECT_CONDITION (leftTheta < -M_PI_4 || leftTheta > M_PI_4 || \
								   rightTheta < -M_PI_4 || rightTheta > M_PI_4 || \
								   imu_heading.pit > M_PI_4 || imu_heading.pit < -M_PI_4) //腿部角度超限保护条件
//		if(PROTECT_CONDITION || standupState == StandupState_None) //当前达到保护条件 PROTECT_CONDITION
//		{
//			if(standupState == StandupState_None) //未处于起立过程中
//			{
//				//关闭所有电机
////				Motor_SetTorque(&leftWheel, 0);
////				Motor_SetTorque(&rightWheel, 0);
////				Motor_SetTorque(&leftJoint[0], 0);
////				Motor_SetTorque(&leftJoint[1], 0);
////				Motor_SetTorque(&rightJoint[0], 0);
////				Motor_SetTorque(&rightJoint[1], 0);
//				can_motors[1].torque = 0;
//				can_motors[0].torque = 0;
//				can_motors[8].torque = 0;
//				can_motors[9].torque = 0;
//				can_motors[17].torque = 0;
//				can_motors[16].torque = 0;
//
//				//阻塞等待腿部角度回到安全范围，再等待4s后恢复控制(若中途触发了起立则在起立准备完成后直接跳出)
//				while(PROTECT_CONDITION && standupState == StandupState_None)
//				{
//					leftTheta = leftLegPos.angle - imu_heading.pit - M_PI_2;
//					rightTheta = rightLegPos.angle - imu_heading.pit - M_PI_2;
//					vTaskDelay(100);
//				}
//				if(standupState == StandupState_None)
//					vTaskDelay(1000);
//				//退出保护后设定目标位置和yaw角度为当前值
//				target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
//				target.yawAngle = imu_heading.yaw;
//				continue;
//			}
//			if(standupState == StandupState_Standup && (leftTheta < -M_PI_4 || rightTheta > M_PI_4))
//				standupState = StandupState_None;
//		}

		//设定关节电机输出扭矩
//		Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
//		Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
//		Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
//		Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);
//		can_motors[8].torque = leftJointTorque[0];
//		can_motors[9].torque = leftJointTorque[1];
//		can_motors[16].torque = rightJointTorque[0];
//		can_motors[17].torque = rightJointTorque[1];
//		l1 = leftJointTorque[0];
//		l4 = leftJointTorque[1];
//		r1 = rightJointTorque[0];
//		r4 = rightJointTorque[1];

		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
	}
}
void Ctrl_Init()
{
	//初始化各个PID参数
//	PID_SetErrLpfRatio(&rollPID.inner, 0.1f);
	PID_Init(&legLengthPID, 500, 0.0, 0.0, -300.0, 300.0);
//	PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);
	PID_Init(&legAnglePID, 13, 0.0, 0.0, -3.0, 3.0);
//	PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);
	PID_Init(&rollPID, 55, 0.0, 0.0, -50.0, 50.0);
	PID_Init(&yawPID, 0.5f, 0.0, 0.0, -1, 1);
}
