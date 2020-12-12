/*
 * movebot_controller_node.cpp
 *
 *  Created on: Dec 7, 2020
 *      Author: bruce
 */

#include <cmsis_os.h>
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "odometry.h"
#include "movebot_controller_node.h"


static struct OdometryData data;

ros::NodeHandle  nh;

ros::Time last_time;
ros::Time now;

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster odom_broadcaster;

char base_footprint[] = "/base_footprint";
char odom_name[] = "/odom";

void MoveBotControllerNodeTask(void *argument)
{
	nh.initNode();

	nh.advertise(odom_pub);
	odom_broadcaster.init(nh);

	OdometryReset();
	last_time = nh.now();

	for (;;) {
		osDelay(10);

		OdometryGetData(&data);
		now = nh.now();

		geometry_msgs::Quaternion odom_quat;

		t.header.frame_id = odom_name;
		t.child_frame_id = base_footprint;
		t.transform.translation.x = data.x;
		t.transform.translation.y = data.y;
		t.transform.translation.z = 0;
		t.transform.rotation = odom_quat;
		t.header.stamp = now;
		odom_broadcaster.sendTransform(t);

		float dt = (float)(now - last_time).toSec();

		odom.header.stamp = now;
		odom.header.frame_id = odom_name;
		odom.pose.pose.position.x = data.x;
		odom.pose.pose.position.y = data.y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.child_frame_id = base_footprint;
		odom.twist.twist.linear.x = data.dx / dt;
		odom.twist.twist.linear.y = data.dy / dt;
		odom.twist.twist.angular.z = data.dtheta / dt;
		odom_pub.publish(&odom);

		last_time = now;

		nh.spinOnce();
	}
}
