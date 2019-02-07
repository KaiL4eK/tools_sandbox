#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <svserver.h>

SVServer *server = nullptr;

void topicCallback(const std_msgs::Float32& msg)
{
	ROS_INFO("Received potentiometer data: %d", msg.data);

	server->sendData( SVServer::State::WAIT, 20, msg.data, 50, 55);
}

#include "svserver.h"
#include "datapackage.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gui_server");

	ros::NodeHandle n;

	server = new SVServer();
	// server->start(QHostAddress("0.0.0.0"), 5555);
	server->start(QHostAddress("192.168.31.130"), 5555);

	ros::Subscriber sub = n.subscribe("cpp_chatter", 1000, topicCallback);


	ros::spin();

	return 0;
}
