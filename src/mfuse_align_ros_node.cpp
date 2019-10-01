/*
 * mfuse_align_ros_node.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mfuse_align.hpp>

int main(int argc, char** argv) {

  /*if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
  {
    ros::console::notifyLoggerLevelsChanged();
  }*/

  ros::init(argc, argv, "mfuse_align");
  ros::NodeHandle nodeHandle("~");

  //mfuse::CameraFuse cameraFuse(nodeHandle);

  ros::spin();
  return 0;
}