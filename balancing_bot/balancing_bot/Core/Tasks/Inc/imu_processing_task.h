/*
 * imu_processing_task.h
 *
 *  Created on: 24 Jan 2022
 *      Author: wx
 */

#ifndef TASKS_INC_IMU_PROCESSING_TASK_H_
#define TASKS_INC_IMU_PROCESSING_TASK_H_

void imu_proc_task_notif();
void gyro_data_ready(gyro_data_t gyro_data);
void accel_data_ready(accel_data_t accel_data);
void mag_data_ready(mag_data_t mag_data);
void imu_processing_task(void *argument);
void init_quaternion(void);
void imu_ahrs_update(void);
void imu_attitude_update(void);
double low_pass_filter(double input, double *prev_output, double alpha);
void kalman_filter_init(KalmanFilter *kf, double Q_angle, double Q_bias, double R_measure);
double kalman_filter_update(KalmanFilter *kf, double new_angle, double new_rate, double dt);
void imu_sensor_fusion(double accel_data[3], double gyro_data[3], double dt,
                       IMUSensorFusion *imu);
void initIMUSensorFusion(IMUSensorFusion *imu);
#endif /* TASKS_INC_IMU_PROCESSING_TASK_H_ */
