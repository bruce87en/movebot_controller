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
#include <geometry_msgs/Twist.h>

#include "pid_task.h"
#include "odometry.h"
#include "movebot_controller_node.h"


static struct OdometryData data;

ros::NodeHandle  nh;

ros::Time last_time;
ros::Time now;

std_msgs::Float32MultiArray odometry;
ros::Publisher odometry_pub("odometry_data", &odometry);
float odometry_data[9];

void cmdMessageCb(const geometry_msgs::Twist& cmd_msg){
	float vl, vr;
	float vx = cmd_msg.linear.x;
//	float vy = cmd_msg.linear.y;
	float rotate = cmd_msg.angular.z;

	vl = vx - base_width*rotate/2;
	vr = vx + base_width*rotate/2;

	PidSetLeftTarget(vl);
	PidSetRightTarget(vr);
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdMessageCb);

void MoveBotControllerNodeTask(void *argument)
{
	nh.initNode();

	nh.advertise(odometry_pub);

	nh.subscribe(cmd_sub);

	odometry.data = odometry_data;
	odometry.data_length = 9;

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
		odometry_data[6] = data.dl;
		odometry_data[7] = data.dr;
		odometry_data[8] = dt;

		odometry_pub.publish(&odometry);

		PidSetLeftMeasure(data.dl/dt);
		PidSetRightMeasure(data.dr/dt);

		nh.spinOnce();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// when DMA complete, STM32Hardware.rbuf may still have unread data
	nh.getHardware()->reset_rbuf();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
		nh.logerror("UART overrun error");
		nh.getHardware()->reset_rbuf();
	} else {
		nh.logerror("UART other error");
		nh.getHardware()->reset_rbuf();
	}
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
	nh.logerror("UART abort error");
	nh.getHardware()->reset_rbuf();
}
