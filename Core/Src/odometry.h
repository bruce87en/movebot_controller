/*
 * odometry.h
 *
 *  Created on: Dec 8, 2020
 *      Author: bruce
 */

#ifndef SRC_ODOMETRY_H_
#define SRC_ODOMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

extern const float base_width;

struct OdometryData {
	float x;
	float y;
	float theta;
	float dx;
	float dy;
	float dtheta;
	float dl; // wheel left
	float dr; // wheel right
};

void OdometryReset();

void OdometryGetData(struct OdometryData *data);

void OdometryTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ODOMETRY_H_ */
