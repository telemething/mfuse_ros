/*
 * livox_to_octree_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mfuse_ros.hpp>

int mfuse_startAsync();

int main(int argc, char** argv) {

  /*if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
  {
    ros::console::notifyLoggerLevelsChanged();
  }*/

  ros::init(argc, argv, "mfuse_ros");
  ros::NodeHandle nodeHandle("~");
  //l2o::livox_to_octree livoxToOctree(nodeHandle);

  mfuse_startAsync();

  ros::spin();
  return 0;
}