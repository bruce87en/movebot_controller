/*
 * encoder.c
 *
 *  Created on: Dec 8, 2020
 *      Author: bruce
 */

#include "odometry.h"

#include <math.h>
#include "main.h"
#include "cmsis_os.h"


// correct these value later
const float ticks_meter = 5000;
const float base_width = 0.2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static float odometry_x;
static float odometry_y;
static float odometry_theta;

static float last_odometry_x;
static float last_odometry_y;
static float last_odometry_theta;

void OdometryReset()
{
	osKernelLock();

	odometry_x = 0;
	odometry_y = 0;
	odometry_theta = 0;

	last_odometry_x = 0;
	last_odometry_y = 0;
	last_odometry_theta = 0;

	osKernelUnlock();
}

void OdometryGetData(struct OdometryData *data)
{
	if (!data)
		return;

	data->x = odometry_x;
	data->y = odometry_y;
	data->theta = odometry_theta;

	data->dx = odometry_x - last_odometry_x;
	data->dy = odometry_y - last_odometry_y;
	data->dtheta = odometry_theta - last_odometry_theta;

	osKernelLock();
	odometry_theta = fmod(odometry_theta, 2*M_PI);
	osKernelUnlock();
}

static void CalcOdometry(int16_t left_signed, int16_t right_signed)
{
	float ds, dt;
	float dx, dy;

	// https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf
	ds = (left_signed + right_signed) / ticks_meter / 2;
	dt = (left_signed - right_signed) / ticks_meter/ base_width;

	if (ds) {
		dx = ds * cos(odometry_theta + dt/2);
		dy = ds * sin(odometry_theta + dt/2);

		odometry_x += dx;
		odometry_y += dy;
	}

	// odometry_theta will write in other task concurrently
	osKernelLock();
	odometry_theta += dt;
	osKernelUnlock();
}

void OdometryTask(void *argument)
{
	uint16_t left, right;
	int16_t left_signed, right_signed;

	__HAL_TIM_ENABLE(&htim2);
	__HAL_TIM_ENABLE(&htim3);

	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
	HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
	for (;;) {
		osDelay(1);
		left = __HAL_TIM_GET_COUNTER(&htim2);
		HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
		right = __HAL_TIM_GET_COUNTER(&htim3);
		HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);

		// AAR must be set to 0xFFFF
		left_signed = (int)left;
		right_signed =  (int)right;

		if (left_signed == 0 && right_signed == 0) {
			continue;
		}

		CalcOdometry(left_signed, right_signed);
	}
}
