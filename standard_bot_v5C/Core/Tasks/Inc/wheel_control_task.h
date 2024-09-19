/*
 * wheel_control_task.h
 *
 *  Created on: Aug 29, 2024
 *      Author: wbo19
 */

#ifndef TASKS_INC_WHEEL_CONTROL_TASK_H_
#define TASKS_INC_WHEEL_CONTROL_TASK_H_
void balancing_task(void *argument);
void Ctrl_TargetUpdateTask();
void LegPos_UpdateTask();
void Ctrl_Task(void *arg);
void Ctrl_StandupPrepareTask();
void Ctrl_Init();


#endif /* TASKS_INC_WHEEL_CONTROL_TASK_H_ */
