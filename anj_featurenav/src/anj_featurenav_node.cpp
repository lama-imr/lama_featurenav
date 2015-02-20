/**
 * Large Map 
 * 
 * Camera based learning and navigating jockey.
 *
 */

#include <string>

#include <ros/ros.h>

#include <anj_featurenav/jockey.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "anj_featurenav");
  ros::NodeHandle private_nh("~");
  
  std::string jockey_base_name;
  std::string default_jockey_base_name = ros::this_node::getName();
  private_nh.param<std::string>("jockey_base_name", jockey_base_name, default_jockey_base_name);

  anj_featurenav::Jockey jockey(jockey_base_name);

  ROS_INFO_STREAM(ros::this_node::getName() << " started (with servers " <<
      jockey.getLearningJockeyName() << " and " << jockey.getNavigatingJockeyName() << ")");
  ros::spin();
  return 0;
}

