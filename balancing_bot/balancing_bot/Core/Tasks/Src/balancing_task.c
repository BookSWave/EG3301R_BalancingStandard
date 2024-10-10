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
#include "balancing_task.h"
#include <stdbool.h>
#include "lqr_k.h"

extern motor_data_t g_can_motors[24];
extern remote_cmd_t g_remote_cmd;
extern orientation_data_t imu_heading;;
Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;
LegPos leftLegPos, rightLegPos;
StateVar stateVar;
Target target = {0, 0, 0, 0, 0, 0, 0.15f};
GroundDetector groundDetector = {10, 10, true, false};
StandupState standupState = StandupState_None;
PID legAnglePID, legLengthPID; //PID controller for leg angle and length control
PID yawPID, rollPID; //PID controller for body yaw and roll control

float motorOutRatio = 1.0f; //Motor output voltage ratio, effective for all motors simultaneously
float sourceVoltage = 12; //Current power supply voltage
double lqrOutTp_check;
double lqrOutT_check;
double leftF_check;
double leftTp_check;
double rightF_check;
double rightTp_check;
double check_dt;
double l1;
double l4;
double r1;
double r4;
double k_check[12] = {0};
float check_x = 0;

void balancing_task(void *argument) {
	double starttime = 0;
	double endtime = 0;
	double dt;
    TickType_t start_time;
    Ctrl_Init();
//    Ctrl_StandupPrepareTask();
    while (1) {
        endtime = get_microseconds();
        dt = endtime - starttime;
        start_time = xTaskGetTickCount();
        starttime = get_microseconds();
        /////////leftjoint[1] = phi1 = 8
        // leftjoint[0] =phi4= 9
        //rightjoint[1] = phi1 = 17
        //rightjoint[0] = phi4 = 16
        target.speedCmd = ((float)g_remote_cmd.left_y/660)*1.0f;
        target.yawSpeedCmd = -((float)g_remote_cmd.left_x/660)*2.0f;
//        target.legLength = ((float)g_remote_cmd.right_y/660)*0.04 + 0.12f;
        target.legLength = ((float)g_remote_cmd.right_y/660)*0.04 + 0.10f;
        leftJoint[1].angle = (double)g_can_motors[FR_MOTOR_ID-1].angle_rad;
        leftJoint[1].speed = (double)g_can_motors[FR_MOTOR_ID-1].speed;
        leftJoint[0].angle = (double)g_can_motors[FL_MOTOR_ID-1].angle_rad;
        leftJoint[0].speed = (double)g_can_motors[FL_MOTOR_ID-1].speed;
        rightJoint[1].angle = (double)g_can_motors[BR_MOTOR_ID-1].angle_rad;
        rightJoint[1].speed = (double)g_can_motors[BR_MOTOR_ID-1].speed;
        rightJoint[0].angle = (double)g_can_motors[BL_MOTOR_ID-1].angle_rad;
        rightJoint[0].speed = (double)g_can_motors[BL_MOTOR_ID-1].speed;
        leftWheel.angle = (double)g_can_motors[LEFT_MOTOR_ID-1].angle_rad/19.2;
        rightWheel.angle = (double)g_can_motors[RIGHT_MOTOR_ID-1].angle_rad/19.2;
        leftWheel.speed = -(double)g_can_motors[LEFT_MOTOR_ID-1].raw_data.rpm * (2*PI/60) /19.2;
        rightWheel.speed = (double)g_can_motors[RIGHT_MOTOR_ID-1].raw_data.rpm * (2*PI/60) /19.2;

        ///////////////////
        vTaskDelayUntil(&start_time, 4);
    }
}
void Ctrl_TargetUpdateTask()
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float speedSlopeStep = 0.005f;
	while(1){
		//Calculate the speed slope step based on the current leg length (the shorter the leg, the more stable, and the steeper the acceleration/deceleration slope)
			float legLength = (leftLegPos.length + rightLegPos.length) / 2;
//			speedSlopeStep = -(legLength - 0.12f) * 0.03f + 0.005f;
			speedSlopeStep = -(legLength - 0.10f) * 0.03f + 0.005f;

			//Calculate the speed ramp, and update the ramp value to target.speed
			if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
				target.speed = target.speedCmd;
			else
			{
				if(target.speedCmd - target.speed > 0)
					target.speed += speedSlopeStep;
				else
					target.speed -= speedSlopeStep;
			}

			//Calculate the position target and limit it within ±0.1m of the current position
			target.position += target.speed * 0.004f;
			if(target.position - stateVar.x > 0.1f)
				target.position = stateVar.x + 0.1f;
			else if(target.position - stateVar.x < -0.1f)
				target.position = stateVar.x - 0.1f;

			//Limit the target speed to ±0.3m/s of the current speed
			if(target.speed - stateVar.dx > 1.0f)
				target.speed = stateVar.dx + 1.0f;
			else if(target.speed - stateVar.dx < -1.0f)
				target.speed = stateVar.dx - 1.0f;

			//Calculate the yaw angle target
			target.yawAngle += target.yawSpeedCmd * 0.004f;
			vTaskDelayUntil(&xLastWakeTime, 4); //Update every 4ms
	}
}

void LegPos_UpdateTask(void *arg)
{
	const float lpfRatio = 0.5f; //Low-pass filter coefficient (weight of new value)
	float lastLeftDLength = 0, lastRightDLength = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
		double legPos[2], legSpd[2];

		//Calculate the left leg position
		leg_pos(leftJoint[1].angle, leftJoint[0].angle, legPos);
		leftLegPos.length = legPos[0];
		leftLegPos.angle = legPos[1];

		//Calculate the left leg speed
		leg_spd(leftJoint[1].speed, leftJoint[0].speed, leftJoint[1].angle, leftJoint[0].angle, legSpd);
		leftLegPos.dLength = legSpd[0];
		leftLegPos.dAngle = legSpd[1];

		//Calculate the left leg length acceleration
		leftLegPos.ddLength = ((leftLegPos.dLength - lastLeftDLength) * 1000 / 4) * lpfRatio + leftLegPos.ddLength * (1 - lpfRatio);
		lastLeftDLength = leftLegPos.dLength;

		//Calculate the right leg position
		leg_pos(rightJoint[1].angle, rightJoint[0].angle, legPos);
		rightLegPos.length = legPos[0];
		rightLegPos.angle = legPos[1];

		//Calculate the right leg speed
		leg_spd(rightJoint[1].speed, rightJoint[0].speed, rightJoint[1].angle, rightJoint[0].angle, legSpd);
		rightLegPos.dLength = legSpd[0];
		rightLegPos.dAngle = legSpd[1];

		//Calculate the right leg length acceleration
		rightLegPos.ddLength = ((rightLegPos.dLength - lastRightDLength) * 1000 / 4) * lpfRatio + rightLegPos.ddLength * (1 - lpfRatio);
		lastRightDLength = rightLegPos.dLength;

		vTaskDelayUntil(&xLastWakeTime, 4); //Update every 4ms
	}
}

void Ctrl_StandupPrepareTask()
{

	vTaskDelete(NULL);
}

void Ctrl_Task(void *arg)
{
	double starttime = 0;
	double endtime = 0;
	double dt;
//	const float wheelRadius = 0.045f; //m, wheel radius
	const float wheelRadius = 0.075f; //m, wheel radius
	const float legMass = 0.01f; //kg, leg mass

	TickType_t xLastWakeTime = xTaskGetTickCount();

	//Manually apply a coefficient to the feedback matrix for manually optimizing control performance
	float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
						{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
//	float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;
	float lqrTpRatio = 0.1f, lqrTRatio = 0.1f;

	//Set initial target values
	target.rollAngle = 0.0f;
//	target.legLength = 0.12f;
	target.legLength = 0.10f;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;

	while (1)
	{
		endtime = get_microseconds();
		dt = endtime - starttime;
		check_dt = dt;
		starttime = get_microseconds();
		//Calculate state variables
		stateVar.phi = imu_heading.pit;
		stateVar.dPhi = imu_heading.dpit;
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
		stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - imu_heading.pit;
		stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imu_heading.dpit;
		double legLength = (leftLegPos.length + rightLegPos.length) / 2;
		double dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;

		//If in stand-up preparation mode, skip further control
		if(standupState == StandupState_Prepare)
		{
			vTaskDelayUntil(&xLastWakeTime, 4);
			continue;
		}

		//Compute LQR feedback matrix
		double kRes[12] = {0}, k[2][6] = {0};
		lqr_k(legLength, kRes);
//		lqr_k(0.12, k_check);
		lqr_k(0.10, k_check);
		if(groundDetector.isTouchingGround) //Normal ground contact state
		{
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 2; j++)
					k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
			}
		}
		else //Leg-off-ground state, manually modify the feedback matrix to only keep the leg vertical
		{
			memset(k, 0, sizeof(k));
			k[1][0] = kRes[1] * -2;
			k[1][1] = kRes[3] * -10;
		}

		//Prepare state variables
		float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
		//Subtract given values
		x[2] -= target.position;
		x[3] -= target.speed;
		check_x = x[2];
		//Matrix multiplication, calculate LQR output
		float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
		float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];
		lqrOutT_check = -lqrOutT;
		lqrOutTp_check = -lqrOutTp;
		//Calculate yaw axis PID output
		PID_Compute(&yawPID, target.yawAngle, imu_heading.yaw,0.004,0);

		//Set wheel motor output torque, combined with LQR and yaw axis PID output
		if(groundDetector.isTouchingGround) //正常接地状态
		{
//			Motor_SetTorque(&leftWheel, -lqrOutT * lqrTRatio - yawPID.output);
//			Motor_SetTorque(&rightWheel, -lqrOutT * lqrTRatio + yawPID.output);
//			g_can_motors[LEFT_MOTOR_ID-1].torque = -lqrOutT * lqrTRatio + yawPID.output;
//			g_can_motors[RIGHT_MOTOR_ID-1].torque = -lqrOutT * lqrTRatio - yawPID.output;
		}
		else //Leg-off-ground state, turn off wheel motors
		{
//			Motor_SetTorque(&leftWheel, 0);
//			Motor_SetTorque(&rightWheel, 0);
			g_can_motors[LEFT_MOTOR_ID-1].torque = 0;
			g_can_motors[RIGHT_MOTOR_ID-1].torque = 0;
		}

		//Adjust target leg length based on ground contact state, and compute leg length PID output
//		PID_Compute(&legLengthPID, (groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? target.legLength : 0.2f, legLength,0.004,0.001);
		PID_Compute(&legLengthPID, (groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? target.legLength : 0.13f, legLength,0.004,0.001);
		//Calculate roll axis PID output
		PID_Compute(&rollPID, target.rollAngle, imu_heading.rol,0.004,0.001);
		//Calculate the push force for each leg, ignore roll PID output and feedforward when legs are off the ground
//		double leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? +rollPID.output : 0) + 13;
//		double rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? -rollPID.output : 0) + 13;
		double leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? +rollPID.output : 0) + 5;
		double rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? -rollPID.output : 0) + 5;

//		double leftForce = legLengthPID.output + 5.0;
//		double rightForce = legLengthPID.output + 5.0;
		if(leftLegPos.length < -0.13f) //Protect the leg from extending too long
			leftForce += (leftLegPos.length - 0.1f) * 1;
		if(rightLegPos.length > 0.13f)
			rightForce -= (rightLegPos.length - 0.1f) * 1;

		//Calculate ground support force for each leg
		groundDetector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (leftLegPos.ddLength - imu_heading.ddz - 9.8f);
		groundDetector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (rightLegPos.ddLength - imu_heading.ddz - 9.8f);
		//Update the ground contact detector data
		static uint32_t lastTouchTime = 0;
		bool isTouchingGround = groundDetector.leftSupportForce > -100 && groundDetector.rightSupportForce > -100; //Determine if currently in ground contact
		if(!isTouchingGround && (get_microseconds()/1000) - lastTouchTime < 1000) //If the last ground contact was less than 1 second ago, assume ground contact to prevent misjudgment due to bouncing
			isTouchingGround = true;
		if(!groundDetector.isTouchingGround && isTouchingGround) //Detect transition to ground contact state, mark cushioning state
		{
			target.position = stateVar.x;
			groundDetector.isCuchioning = true;
			lastTouchTime = (get_microseconds()/1000);
		}
		if(groundDetector.isCuchioning && legLength < target.legLength) //Cushioning state continues until leg length compresses to target leg length
			groundDetector.isCuchioning = false;
		groundDetector.isTouchingGround = isTouchingGround;

		//Calculate PID output for the difference in leg angles between the left and right legs
		PID_Compute(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle,0.004,0.01);

		//Calculate the hip joint torque output, which is the sum of the LQR output and the PID output for the difference in leg angles
//		double leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.15f);
//		double rightTp = -lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.15f);
		double leftTp = -lqrOutTp * lqrTpRatio + legAnglePID.output * (leftLegPos.length / 0.1f);
		double rightTp = -lqrOutTp * lqrTpRatio - legAnglePID.output * (rightLegPos.length / 0.1f);



		//Use VMC (Virtual Model Control) to calculate the output torques for each joint motor
		double leftJointTorque[2]={0};
		leg_conv(-leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
		double rightJointTorque[2]={0};
		leg_conv(-rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque); //put negative sign for right force

		leftF_check = leftForce;
		leftTp_check = leftTp;
		rightF_check = -rightForce;
		rightTp_check = rightTp;
		//Protect the legs from exceeding safe angles
		if (g_remote_cmd.right_switch == 3){
			standupState = StandupState_Standup;
		}else{
			standupState = StandupState_None;
		}
		float leftTheta = leftLegPos.angle - imu_heading.pit - M_PI_2;
		float rightTheta = rightLegPos.angle - imu_heading.pit - M_PI_2;
		#define PROTECT_CONDITION (leftTheta < -M_PI_4 || leftTheta > M_PI_4 || \
								   rightTheta < -M_PI_4 || rightTheta > M_PI_4 || \
								   imu_heading.pit > M_PI_4 || imu_heading.pit < -M_PI_4) //Condition to protect against excessive leg angles
		if(PROTECT_CONDITION || standupState == StandupState_None) //If the condition to protect against excessive angles is met
		{
			if(standupState == StandupState_None) //Not in stand-up process
			{
				//Turn off all motors
//				Motor_SetTorque(&leftWheel, 0);
//				Motor_SetTorque(&rightWheel, 0);
//				Motor_SetTorque(&leftJoint[0], 0);
//				Motor_SetTorque(&leftJoint[1], 0);
//				Motor_SetTorque(&rightJoint[0], 0);
//				Motor_SetTorque(&rightJoint[1], 0);
				g_can_motors[LEFT_MOTOR_ID-1].torque = 0;
				g_can_motors[RIGHT_MOTOR_ID-1].torque = 0;
				g_can_motors[FR_MOTOR_ID-1].torque = 0;
				g_can_motors[FL_MOTOR_ID-1].torque = 0;
				g_can_motors[BR_MOTOR_ID-1].torque = 0;
				g_can_motors[BL_MOTOR_ID-1].torque = 0;

				//Block and wait for leg angles to return to a safe range, then resume control after 4 seconds (or jump out if the stand-up process is triggered in the meantime)
				while(PROTECT_CONDITION && standupState == StandupState_None)
				{
					leftTheta = leftLegPos.angle - imu_heading.pit - M_PI_2;
					rightTheta = rightLegPos.angle - imu_heading.pit - M_PI_2;
					vTaskDelay(100);
				}
				if(standupState == StandupState_None)
					vTaskDelay(1000);
				//After exiting protection, set the target position and yaw angle to the current values
				target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
				target.yawAngle = imu_heading.yaw;
				continue;
			}
			if(standupState == StandupState_Standup && (leftTheta < -M_PI_4 || rightTheta > M_PI_4))
				standupState = StandupState_None;
		}

		//Set joint motor output torque
//		Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
//		Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
//		Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
//		Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);
		g_can_motors[FR_MOTOR_ID-1].torque = leftJointTorque[0];
		g_can_motors[FL_MOTOR_ID-1].torque = leftJointTorque[1];
		g_can_motors[BR_MOTOR_ID-1].torque = rightJointTorque[0];
		g_can_motors[BL_MOTOR_ID-1].torque = rightJointTorque[1];
		l1 = leftJointTorque[0];
		l4 = leftJointTorque[1];
		r1 = rightJointTorque[0];
		r4 = rightJointTorque[1];

		vTaskDelayUntil(&xLastWakeTime, 4); //4ms control cycle
	}
}
void Ctrl_Init()
{
	//Initialize various PID parameters
//	PID_SetErrLpfRatio(&rollPID.inner, 0.1f);
//	PID_Init(&legLengthPID, 500, 0.0, 0.0, -300.0, 300.0);
//	PID_Init(&legAnglePID, 13, 0.0, 0.0, -3.0, 3.0);
//	PID_Init(&rollPID, 55, 0.0, 0.0, -50.0, 50.0);
//	PID_Init(&yawPID, 0.5f, 0.0, 0.0, -1, 1);

//	PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);
//	PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);

	PID_Init(&legLengthPID, 500, 0.0, 0.0, -300.0, 300.0);
	PID_Init(&legAnglePID, 0, 0.0, 0.0, -1.0, 1.0);
//	PID_Init(&rollPID, 55, 0.0, 0.0, -50.0, 50.0);
//	PID_Init(&yawPID, 0.5f, 0.0, 0.0, -1, 1);
	PID_Init(&rollPID, 0, 0.0, 0.0, -50.0, 50.0);
	PID_Init(&yawPID, 0.0f, 0.0, 0.0, -1, 1);
}
