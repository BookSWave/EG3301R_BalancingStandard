/*
 * motors_process.h
 *
 *  Created on: Jul 12, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_MOTORS_PROCESS_H_
#define TASKS_INC_MOTORS_PROCESS_H_
void motors_process_task(void *argument);
void raw_angle_to_rad(motor_data_t motor[]);
void motors_raw_angle_to_desired_angle_dir(motor_data_t *motor, float offset, int dir);
void motors_torque_to_current_6020(motor_data_t *motor, float deadzone,double dir);
void motors_torque_to_current_3508_gearbox(motor_data_t *motor, float deadzone,double dir);
#endif /* TASKS_INC_MOTORS_PROCESS_H_ */
