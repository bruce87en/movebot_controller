/*
 * pid_task.h
 *
 *  Created on: Dec 19, 2020
 *      Author: bruce
 */

#ifndef SRC_PID_TASK_H_
#define SRC_PID_TASK_H_

#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void PidInit();

void PidSetLeftMeasure(float measure);

void PidSetRightMeasure(float measure);

void PidSetLeftTarget(float target);

void PidSetRightTarget(float target);

void PidTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SRC_PID_TASK_H_ */
