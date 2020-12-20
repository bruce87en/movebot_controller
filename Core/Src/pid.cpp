/*
 * pid.cpp
 *
 *  Created on: Dec 19, 2020
 *      Author: bruce
 */

#include "main.h"

#include "pid.h"

Pid::~Pid() {
	// TODO Auto-generated destructor stub
}

Pid::Pid() {
	// TODO Auto-generated constructor stub
	target = 0;
	measure = 0;
	output = 0;
}

void Pid::setParam(float p, float i, float d) {
	kp = p;
	ki = i;
	kd = d;
}

void Pid::setOutLimit(int max, int min) {
	out_max = max;
	out_min = min;
}

void Pid::setTarget(float t) {
	target = t;
}

void Pid::setMeasure(float m) {
	measure = m;
}

void Pid::update() {
	uint32_t tick;
	float dt;
	float error;
	float out;
	float derivative;

	tick = HAL_GetTick();

//	if (target != last_target) {
//		integral = 0;
//		last_target = target;
//	}

	if (target == 0 && measure == 0) {
		integral = 0;
		output = 0;
		return;
	}

	dt = (tick-last_tick)/1000.0;
	last_tick = tick;
	error = target - measure;
	integral += error * dt;
	derivative = (error - last_error) / dt;
	last_error = error;
	out = kp*error + ki*integral + kd*derivative;
//	out = kp*error + kd*derivative;

	if (out > out_max) {
		out = out_max;
		integral -= error * dt;
	}
	if (out < out_min) {
		out = out_min;
		integral -= error * dt;
	}

	output = out;
}

void Pid::init() {
	last_tick = HAL_GetTick();
	last_error = 0;
	last_target = 0;
	integral = 0;
}

int Pid::getOutput() {
	return output;
}
