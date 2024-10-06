/*
 * imu_processing_task.c
 *
 *  Created on: 9 Jan 2022
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_imu.h"
#include "robot_config.h"
#include "imu_processing_task.h"
#include <stdio.h>
#include <math.h>

void quat_startup();

orientation_data_t imu_heading;
static accel_data_t accel_proc_data;
static gyro_data_t gyro_proc_data;
static mag_data_t mag_proc_data;
extern QueueHandle_t gyro_data_queue;
extern QueueHandle_t accel_data_queue;
extern QueueHandle_t mag_data_queue;
extern TaskHandle_t imu_processing_task_handle;
static uint8_t update_flag = 0;

double time_dif;
#define ALPHA 0.98
#define DT 0.01
#define LPF_ALPHA 0.5  // Low-pass filter coefficient

int fusion() {
    double accel_data[3];
    double gyro_data[3];
    double dt = DT;
    IMUSensorFusion imu;

    initIMUSensorFusion(&imu);

    double T_end = 0.1;
    double time_dif;


    float T_start = get_microseconds();

    // Simulate data (replace with actual sensor reading in a real application)
    while (1) {
        // Replace these functions with actual sensor data reading
        accel_data[0] = accel_proc_data.ax;  // x-axis acceleration
        accel_data[1] = accel_proc_data.ay;  // y-axis acceleration
        accel_data[2] = accel_proc_data.az;  // z-axis acceleration

        gyro_data[0] = gyro_proc_data.gx;  // roll rate
        gyro_data[1] = gyro_proc_data.gy;  // pitch rate
        gyro_data[2] = gyro_proc_data.gz;  // yaw rate

        T_end = get_microseconds();
        time_dif = (T_end - T_start) * 0.000001;
        dt = time_dif;
        imu_sensor_fusion(accel_data, gyro_data, dt, &imu);
        T_start = get_microseconds();

        // Store the results in the imu_heading structure
        imu_heading.pit = imu.pitch;
        imu_heading.dpit = imu.pitch_speed;
        imu_heading.ddpit = imu.pitch_accel;
        imu_heading.rol = imu.roll;
        imu_heading.drol = imu.roll_speed;
        imu_heading.ddrol = imu.roll_accel;
        imu_heading.yaw = imu.yaw;
        imu_heading.dyaw = imu.yaw_speed;
        imu_heading.ddyaw = imu.yaw_accel;
        imu_heading.ddz = imu.vertical_accel;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    return 0;
}

// Low-pass filter function
double low_pass_filter(double input, double *prev_output, double alpha) {
    *prev_output = alpha * input + (1 - alpha) * (*prev_output);
    return *prev_output;
}


void initIMUSensorFusion(IMUSensorFusion *imu) {
    imu->pitch = 0.0;
    imu->pitch_speed = 0.0;
    imu->pitch_accel = 0.0;
    imu->roll = 0.0;
    imu->roll_speed = 0.0;
    imu->roll_accel = 0.0;
    imu->yaw = 0.0;
    imu->yaw_speed = 0.0;
    imu->yaw_accel = 0.0;
    imu->vertical_accel = 0.0;
    imu->acc_x_lpf = 0.0;
    imu->acc_y_lpf = 0.0;
    imu->acc_z_lpf = 0.0;
}
void imu_sensor_fusion(double accel_data[3], double gyro_data[3], double dt,
                       IMUSensorFusion *imu) {
    // Low-pass filter accelerometer data
    accel_data[0] = low_pass_filter(accel_data[0], &(imu->acc_x_lpf), LPF_ALPHA);
    accel_data[1] = low_pass_filter(accel_data[1], &(imu->acc_y_lpf), LPF_ALPHA);
    accel_data[2] = low_pass_filter(accel_data[2], &(imu->acc_z_lpf), LPF_ALPHA);

    // Calculate pitch and roll from accelerometer data
    double pitch_accel_meas = atan2(-accel_data[0], sqrt(accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]));
    double roll_accel_meas = atan2(accel_data[1], accel_data[2]);

    // Apply complementary filter
    imu->pitch = ALPHA * (imu->pitch + gyro_data[1] * dt) + (1 - ALPHA) * pitch_accel_meas;
    imu->roll = ALPHA * (imu->roll + gyro_data[0] * dt) + (1 - ALPHA) * roll_accel_meas;
    imu->yaw += gyro_data[2] * dt;

    // Calculate speed (rate of change of angles) directly from gyroscope data
    imu->pitch_speed = gyro_data[1];
    imu->roll_speed = gyro_data[0];
    imu->yaw_speed = gyro_data[2];

    // Calculate acceleration (rate of change of speeds)
    static double last_pitch_speed = 0, last_roll_speed = 0, last_yaw_speed = 0;
    imu->pitch_accel = (imu->pitch_speed - last_pitch_speed) / dt;
    imu->roll_accel = (imu->roll_speed - last_roll_speed) / dt;
    imu->yaw_accel = (imu->yaw_speed - last_yaw_speed) / dt;

    // Update last speeds
    last_pitch_speed = imu->pitch_speed;
    last_roll_speed = imu->roll_speed;
    last_yaw_speed = imu->yaw_speed;

    // Calculate vertical acceleration (z-axis acceleration)
    imu->vertical_accel = sqrt((accel_data[0]*accel_data[0]) + (accel_data[1]*accel_data[1]) + (accel_data[2]*accel_data[2]));
}

void imu_proc_task_notif() {
	//resets the flags
	update_flag = 0b000;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(imu_processing_task_handle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken)
}

void gyro_data_ready(gyro_data_t gyro_data) {
	if (IMU_ORIENTATION == 2) {		// swap gyro inputs for vertically mounted devboard
		gyro_proc_data.gx = gyro_data.gz;
		gyro_proc_data.gy = gyro_data.gy;
		gyro_proc_data.gz = -gyro_data.gx;
	} else {

		gyro_proc_data.gx = gyro_data.gx;
		gyro_proc_data.gy = gyro_data.gy;
		if (gyro_data.gz > 0.02 || gyro_data.gz < -0.02){
			gyro_proc_data.gz = gyro_data.gz;
		} else {
			gyro_proc_data.gz = 0;
		}

	}
	gyro_proc_data.last_gyro_update = gyro_data.last_gyro_update;

	update_flag |= 1; //sets bit 0 to true
	//only allows task to be run when all the data is new
	if (update_flag == 0b111|| update_flag == 0b011) {
		imu_proc_task_notif();
	}
}

void accel_data_ready(accel_data_t accel_data) {
	if (IMU_ORIENTATION == 2) {		// swap accel inputs for vertically mounted devboard
		accel_proc_data.ax = accel_data.az;
		accel_proc_data.ay = accel_data.ay;
		accel_proc_data.az = -accel_data.ax;
	} else {

		accel_proc_data.ax = accel_data.ax;
		accel_proc_data.ay = accel_data.ay;
		accel_proc_data.az = accel_data.az;
	}

	accel_proc_data.last_accel_update = accel_data.last_accel_update;

	update_flag |= 1 << 1; //sets bit 1 to true
	//only allows task to be run when accel and gyro data are new
	if (update_flag == 0b111 || update_flag == 0b011) {
		imu_proc_task_notif();
	}
}

void mag_data_ready(mag_data_t mag_data) {
	mag_proc_data.mx = mag_data.mx;
	mag_proc_data.my = mag_data.my;
	mag_proc_data.mz = mag_data.mz;
	mag_proc_data.last_mag_update = mag_data.last_mag_update;

	update_flag |= 1 << 2;
	if (update_flag == 0b111) {
		//disabled as magnetometer data is not used
		//imu_proc_task_notif();
	}
}

void imu_processing_task(void *argument) {
	imu_start_ints();
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	imu_attitude_update();
//	quat_startup();
	while (1) {
		fusion();
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		imu_attitude_update();
		portYIELD();
	}
}


void imu_attitude_update(void) {

}
