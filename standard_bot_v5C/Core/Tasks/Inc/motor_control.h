/*
 * motor_control.h
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_MOTOR_CONTROL_H_
#define TASKS_INC_MOTOR_CONTROL_H_


void yangle_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_data, float *prev_imu_data);
void yaw_pid(double setpoint, double curr_pt, pid_data_t *pid);

void yangle_o_pid(double setpoint, double curr_pt, motor_data_t *motor, float imu_data, float *prev_imu_data, float offset);
void oangle_pid(double setpoint, double curr_pt, motor_data_t *motor, float offset);
void angle_pid(double setpoint, double curr_pt, motor_data_t *motor);
void speed_pid(double setpoint, double curr_pt, pid_data_t *pid);

void motor_send_RTOS(void *argument);
void motor_send_can(motor_data_t motor_all[], uint8_t id_one, uint8_t id_two, uint8_t id_three, uint8_t id_four);
void kill_can();
void float_minmax(float *motor_in, float motor_max, float motor_min);
void reset_pid(motor_data_t *motor_data);


#endif /* TASKS_INC_MOTOR_CONTROL_H_ */
