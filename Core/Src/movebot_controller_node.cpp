/*
 * movebot_controller_node.cpp
 *
 *  Created on: Dec 7, 2020
 *      Author: bruce
 */

#include "ros.h"

#include "movebot_controller_node.h"


ros::NodeHandle nh;

void MoveBotControllerNodeTask(void *argument)
{
	nh.initNode();

	for (;;) {
		nh.spinOnce();
	}
}
