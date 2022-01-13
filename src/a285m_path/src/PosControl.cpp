#include <PosControl/PosControl.hpp>
#include <ros/ros.h>

using namespace poscontrol;

PosControl::PosControl(ros::NodeHandle& nh) : nodehandle_(nh){

	//load the param
	if(!loadParam()){
		ROS_ERROR("Error in loading the parameters.");
		// ros::requestShutdown();
	}

	// declare all the subscriber and publisher
	// odom_sub_ = nodehandle_.subscribe("/husky_velocity_controller/odom", 1, &BotControl::odomCallBack, this);
	odom_sub_ = nodehandle_.subscribe("/odom", 1, &PosControl::trueCallBack, this);
	start_pid_sub = nodehandle_.subscribe("/start_p", 1, &PosControl::pidCallback, this);
	
	vel_pub_ = nodehandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 200);
	error_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_forward", 1); 
	error_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_angle", 1);
	control_signal_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_forward", 1);
	control_signal_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_angle", 1);
	dist_from_pub = nodehandle_.advertise<std_msgs::Float32>("/dist_from", 1);

	//initialize variables
	error_forward_ = 0;
	error_angle_ = 0;
	error_forward_prev_ = 0;
	error_angle_prev_ = 0;
	I_forward_ = 0;
	I_angle_ = 0;
	D_forward_ = 0;
	D_angle_ = 0;

	ROS_INFO("Node Initialized");
}

PosControl::~PosControl(){}

void PosControl::trueCallBack(const nav_msgs::OdometryConstPtr& odomMsg){
    // get index
	pos_y_ = odomMsg->pose.pose.position.y;
	pos_x_ = odomMsg->pose.pose.position.x;
	
	ROS_INFO("Received /odom position (%f, %f).", pos_x_, pos_y_);
	void pidCallback(const std_msgs::Bool& check_msg);
	float q_x = odomMsg->pose.pose.orientation.x;
	float q_y = odomMsg->pose.pose.orientation.y;
	float q_z = odomMsg->pose.pose.orientation.z;
	float q_w = odomMsg->pose.pose.orientation.w;
	ang_z_ = atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_z*q_z + q_y*q_y));

}

void PosControl::pidCallback(const std_msgs::Bool& check_msg){
	checked = check_msg.data;
}

void PosControl::pidAlgorithm(){
	std_msgs::Float32 linear_error;
	std_msgs::Float32 angle_error;
	std_msgs::Float32 linear_velocity; // command
	std_msgs::Float32 angle_velocity; // command

    double Dx = pillar_x - pos_x_; // pos_x_ from odom (true sim. position)
    double Dy = pillar_y - pos_y_; // pos_y_ from odom (true sim. position)

	double dist_from = sqrt(Dx*Dx + Dy*Dy);
	std_msgs::Float32 dist_from_val;
	dist_from_val.data = dist_from;
	dist_from_pub.publish(dist_from_val);
    
	// update the pid status
	error_forward_prev_ = error_forward_;
	error_angle_prev_ = error_angle_;
	
	error_forward_ = sqrt(Dx*Dx + Dy*Dy) - target_distance;
	error_angle_ = atan2(Dy, Dx) - ang_z_;
    
	// regularize the error_angle_ within [-PI, PI]
	if(error_angle_ < -PI) error_angle_ += 2*PI;
	if(error_angle_ > PI) error_angle_ -= 2*PI;

	// integral term
	I_forward_ += dt * error_forward_;
	I_angle_ += dt * error_angle_;

	// derivative term
	D_forward_ = (-error_forward_prev_ + error_forward_) / dt;
	D_angle_ = (-error_angle_prev_ + error_angle_) / dt;

	// ENTER YOUR CODE HERE
	//steering PID
	trans_angle_ = Kp_a_pos * error_angle_ + Ki_a_pos * I_angle_ + Kd_a_pos * D_angle_;
	//drive PID
	trans_forward_ = Kp_f * error_forward_ + Ki_f * I_forward_ + Kd_f * D_forward_;

	// set limit
	if(trans_forward_ > 2) trans_forward_ = 2;
	if(trans_forward_ < -2) trans_forward_ = -2;

	// ROS_INFO("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Distance: %f",  //#
	//	trans_forward_, trans_angle_, error_angle_, scan_range_);
 	ROS_INFO("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Linear_error: %f",  
		trans_forward_, trans_angle_, error_angle_, error_forward_);


	double threshold = 0.01;
	if(dist_from<=threshold){
		vel_cmd_.linear.x = 0;
		vel_cmd_.angular.z = 0;
		vel_pub_.publish(vel_cmd_);
		end = true;	
		return;
	}
	//publish all
	vel_cmd_.linear.x = trans_forward_;
	vel_cmd_.angular.z = trans_angle_; //euler angle
	vel_pub_.publish(vel_cmd_);

	linear_error.data = error_forward_;
	error_forward_pub_.publish(linear_error);

	linear_velocity.data = trans_forward_;
	control_signal_forward_pub_.publish(linear_velocity);

	angle_error.data = error_angle_;
	error_angle_pub_.publish(angle_error);

	angle_velocity.data = trans_angle_;
	control_signal_angle_pub_.publish(angle_velocity);

}

void PosControl::spin(){
	ros::Rate loop_rate(1/dt);

    // //# sleep at start to wait for windows to load
	// ros::Rate init_rate(1);
    // for (int i=3; i>0; --i) {
    //     ROS_INFO("%d", i);
    //     init_rate.sleep();
    // } // sleep for # seconds where i=# above
	
	while(ros::ok()){
		ros::spinOnce();
		if(checked==true){
			pidAlgorithm();
		}
		loop_rate.sleep();
		if(end==true){
			return;
		}
	}

}

bool PosControl::loadParam(){


	if(!nodehandle_.getParam("/Kp_f", Kp_f)){
		ROS_ERROR("Kp_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_f", Ki_f)){
		ROS_ERROR("Ki_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kd_f", Kd_f)){
		ROS_ERROR("Kd_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kp_a", Kp_a_pos)){
		ROS_ERROR("Kp_a_pos Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_a", Ki_a_pos)){
		ROS_ERROR("Ki_a_pos Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kd_a", Kd_a_pos)){
		ROS_ERROR("Kd_a_pos Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_angle", target_angle)){
		ROS_ERROR("target_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_distance", target_distance)){
		ROS_ERROR("target_distance Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_x", pillar_x)){ //#
		ROS_ERROR("pillar_x Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_y", pillar_y)){ //#
		ROS_ERROR("pillar_y Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}
