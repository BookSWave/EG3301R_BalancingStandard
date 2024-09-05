/*
 * PID.h
 *
 *  Created on: Jul 14, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_PID_H_
#define TASKS_INC_PID_H_
void PID_Init(PID *pid, double kp, double ki, double kd, double min_output, double max_output);
double PID_Compute(PID *pid, double setpoint, double measured_value, double dt, double deadzone);
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb, double dt);

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#endif /* TASKS_INC_PID_H_ */
