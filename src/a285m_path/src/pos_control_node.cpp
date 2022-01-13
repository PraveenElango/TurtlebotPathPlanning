#include <PosControl/PosControl.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace poscontrol;

int main(int argc, char** argv){

	ros::init(argc, argv, "pos_control_node");
	ros::NodeHandle NodeHandle("~");

	PosControl PC(NodeHandle);

	PC.spin();
	
}