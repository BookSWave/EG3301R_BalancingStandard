/*
 * gimbal_control_task.h
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#ifndef TASKS_INC_GIMBAL_CONTROL_TASK_H_
#define TASKS_INC_GIMBAL_CONTROL_TASK_H_

void gimbal_control_task(void *argument);
void leg_control(leg_move_t *leg_move_update);
void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor);
void gimbal_pid_init();
void jointmotor_control_loop(leg_move_t *leg_move_control_loop);


#endif /* TASKS_INC_GIMBAL_CONTROL_TASK_H_ */
