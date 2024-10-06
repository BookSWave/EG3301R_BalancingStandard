/*
 * balancing_task.h
 *
 *  Created on: Jul 16, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_BALANCING_TASK_H_
#define TASKS_INC_BALANCING_TASK_H_
void balancing_task(void *argument);
void Ctrl_TargetUpdateTask();
void LegPos_UpdateTask();
void Ctrl_Task(void *arg);
void Ctrl_StandupPrepareTask();
void Ctrl_Init();



#endif /* TASKS_INC_BALANCING_TASK_H_ */
