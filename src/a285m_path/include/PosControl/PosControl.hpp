#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> // include the LaserScan msgs definition //#
#include <nav_msgs/Odometry.h> // include the odometry msgs definition
#include <geometry_msgs/Twist.h> // include the robot control msgs definition
#include <std_msgs/Float32.h> // include the msgs needed to be published
#include <std_msgs/Bool.h>
#include <gazebo_msgs/LinkStates.h> //# for true posiiton
#include <geometry_msgs/Pose.h> 
#include <ros/console.h>
#include <math.h> //#

static const double PI = 3.1415;

namespace poscontrol{
class PosControl{

private:

	ros::NodeHandle nodehandle_;

	// topics to be subscribed
    // ros::Subscriber scan_sub_; // laser scan //#
    ros::Subscriber odom_sub_; // odometry
    ros::Subscriber true_sub_; // odometryPosControl::
    ros::Subscriber start_pid_sub; 
	// topics to be published
	ros::Publisher vel_pub_; // twist control
	ros::Publisher error_forward_pub_; // log
	ros::Publisher error_angle_pub_; //log
	ros::Publisher control_signal_forward_pub_; // log
	ros::Publisher control_signal_angle_pub_;
	ros::Publisher dist_from_pub;

	
	double pos_x_, pos_y_; //# moved q_z_ to callback
	double ang_z_; // eular angle from quaternion q_z
	bool checked;
	geometry_msgs::Twist vel_cmd_; // control the robot msgs
	double trans_forward_, trans_angle_; // pid output

	// PID related
	double error_forward_, error_angle_, error_forward_prev_, error_angle_prev_;
	double I_forward_, I_angle_; // integral part
	double D_forward_, D_angle_; // derivative part

	// void odomCallBack(const nav_msgs::OdometryConstPtr& odomMsg);
    //	void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
	void trueCallBack(const nav_msgs::OdometryConstPtr& odomMsg);
	void pidCallback(const std_msgs::Bool& check_msg);
	void pidAlgorithm();
	bool loadParam();

	bool end = false;


public:
	
	double dt;
	double target_distance, target_angle;
	double Kp_f, Ki_f, Kd_f;
	double Kp_a_pos, Ki_a_pos, Kd_a_pos;
    double pillar_x, pillar_y; //# pillar position

	int cylinder_msg_count;
	double cylinder_pose_x;

	PosControl(ros::NodeHandle& nh);
	virtual ~PosControl();

	void spin();

};

}
