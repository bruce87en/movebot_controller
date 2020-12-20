/*
 * pid.h
 *
 *  Created on: Dec 19, 2020
 *      Author: bruce
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include <stdint.h>


class Pid {
public:
	virtual ~Pid();
	Pid();

	void init();

	void setParam(float p, float i, float d);
	void setOutLimit(int max, int min);
	void setTarget(float t);
	void setMeasure(float m);
	void update();
	int getOutput();

private:
	float kp;
	float ki;
	float kd;

	int out_max;
	int out_min;

	float target;
	float measure;
	int output;

	uint32_t last_tick;
	float last_error;
	float last_target;
	float integral;
};

#endif /* SRC_PID_H_ */
