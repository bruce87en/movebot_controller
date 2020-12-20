/*
 * pid_task.c
 *
 *  Created on: Dec 19, 2020
 *      Author: bruce
 */
#include "main.h"
#include "pid.h"

#include "pid_task.h"

extern TIM_HandleTypeDef htim4;

static Pid pid_left;
static Pid pid_right;

void PidInit() {
	pid_left.setParam(1600, 400, 10);
	pid_left.setOutLimit(1000, -1000);
	pid_right.setParam(1600, 400, 10);
	pid_right.setOutLimit(1000, -1000);
}

void PidSetLeftMeasure(float measure) {
	pid_left.setMeasure(measure);
	if (measure < -0.01) {
		measure++;
	}
}

void PidSetRightMeasure(float measure) {
	pid_right.setMeasure(measure);
	if (measure < -0.01) {
		measure++;
	}
}

void PidSetLeftTarget(float target) {
	pid_left.setTarget(target);
}

void PidSetRightTarget(float target) {
	pid_right.setTarget(target);
}

void PidTask(void *argument)
{
	int left_pwm, right_pwm;

	pid_left.init();
	pid_right.init();

	for (;;) {
		osDelay(10);
		pid_left.update();
		left_pwm = pid_left.getOutput();
		pid_right.update();
		right_pwm = pid_right.getOutput();

		if (left_pwm < 0) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, -left_pwm);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		} else if (left_pwm > 0) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, left_pwm);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		} else {
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
		}

		if (right_pwm < 0) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -right_pwm);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		} else if (right_pwm > 0) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_pwm);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		} else {
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
		}
	}
}
