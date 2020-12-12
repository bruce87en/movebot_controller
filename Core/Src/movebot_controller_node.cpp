/*
 * movebot_controller_node.cpp
 *
 *  Created on: Dec 7, 2020
 *      Author: bruce
 */

#include <cmsis_os.h>
#include <ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include "odometry.h"
#include "movebot_controller_node.h"


static struct OdometryData data;

ros::NodeHandle  nh;

ros::Time last_time;
ros::Time now;

std_msgs::Float32MultiArray odometry;
ros::Publisher odometry_pub("odometry_data", &odometry);
float odometry_data[7];


void MoveBotControllerNodeTask(void *argument)
{
	nh.initNode();

	nh.advertise(odometry_pub);

	odometry.data = odometry_data;
	odometry.data_length = 7;

	OdometryReset();
	last_time = nh.now();

	for (;;) {
		osDelay(10);

		OdometryGetData(&data);
		now = nh.now();
		float dt = (float)(now - last_time).toSec();
		last_time = now;

		odometry_data[0] = data.x;
		odometry_data[1] = data.y;
		odometry_data[2] = data.theta;
		odometry_data[3] = data.dx;
		odometry_data[4] = data.dy;
		odometry_data[5] = data.dtheta;
		odometry_data[6] = dt;

		odometry_pub.publish(&odometry);

		nh.spinOnce();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}
