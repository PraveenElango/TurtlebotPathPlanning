#include <BotControl/BotControl.hpp>

int main(int argc, char** argv){
  
  ros::init(argc, argv, "bot_control_node");
  ros::NodeHandle nh;
  
  BotControl BC(nh);
  BC.spin();
  ros::Duration(10.0).sleep();
  return 0;
  
}