/*
 * encoder.c
 *
 *  Created on: Dec 8, 2020
 *      Author: bruce
 */

#include <math.h>
#include "main.h"
#include "cmsis_os.h"

#include "pid_task.h"
#include "odometry.h"


// correct these value later
const float ticks_meter = 500000;
const float base_width = 0.1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static float odometry_x;
static float odometry_y;
static float odometry_theta;

static float last_odometry_x;
static float last_odometry_y;
static float last_odometry_theta;

static float wheel_left;
static float wheel_right;

void OdometryReset()
{
	osKernelLock();

	odometry_x = 0;
	odometry_y = 0;
	odometry_theta = 0;

	last_odometry_x = 0;
	last_odometry_y = 0;
	last_odometry_theta = 0;

	wheel_left = 0;
	wheel_right = 0;

	osKernelUnlock();
}

void OdometryGetData(struct OdometryData *data)
{
	float temp_x, temp_y, temp_theta;

	if (!data)
		return;

	temp_x = odometry_x;
	temp_y = odometry_y;

	osKernelLock();
	data->dtheta = odometry_theta - last_odometry_theta;
	odometry_theta = fmod(odometry_theta, 2*M_PI);
	temp_theta = odometry_theta;
	data->dl = wheel_left;
	data->dr = wheel_right;
	wheel_left = wheel_right = 0;
	osKernelUnlock();
	data->dx = temp_x - last_odometry_x;
	data->dy = temp_y - last_odometry_y;

	data->x = temp_x;
	data->y = temp_y;
	data->theta = temp_theta;

	last_odometry_x = temp_x;
	last_odometry_y = temp_y;
	last_odometry_theta = temp_theta;
}

static void CalcOdometry(int16_t left_signed, int16_t right_signed)
{
	float ds, dt;
	float dx, dy;
	float left, right;

	left = left_signed / ticks_meter;
	right = right_signed / ticks_meter;

	// https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf
//	ds = (left_signed + right_signed) / ticks_meter / 2;
//	dt = (left_signed - right_signed) / ticks_meter/ base_width;
	ds = (left + right) / 2;
	dt = (left - right) / base_width;

	if (ds) {
		dx = ds * cos(odometry_theta + dt/2);
		dy = ds * sin(odometry_theta + dt/2);

		odometry_x += dx;
		odometry_y += dy;
	}

	// odometry_theta will write in other task concurrently
	osKernelLock();
	odometry_theta += dt;
	wheel_left += left;
	wheel_right += right;
	osKernelUnlock();
}

void OdometryTask(void *argument)
{
	uint16_t left, right;
	int16_t left_signed, right_signed;
	int left_dir, right_dir;

	__HAL_TIM_ENABLE(&htim2);
	__HAL_TIM_ENABLE(&htim3);

	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
	HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
	for (;;) {
		osDelay(1);
		left = __HAL_TIM_GET_COUNTER(&htim2);
		left_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
		HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
		right = __HAL_TIM_GET_COUNTER(&htim3);
		right_dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
		HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);

		// AAR must be set to 0xFFFF
		left_signed = (int)left;
		right_signed =  (int)right;

		// reverse start from 0xFFFF = -1
		if (left_signed < 0) {
			left_signed += 1;
		}
		if (right_signed < 0) {
			right_signed += 1;
		}

		if (left_signed == 0 && right_signed == 0) {
			continue;
		}

		CalcOdometry(left_signed, right_signed);
	}
}
