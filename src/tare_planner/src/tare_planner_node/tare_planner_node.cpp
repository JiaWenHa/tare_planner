#include <ros/ros.h>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tare_planner_node");
  ros::NodeHandle node_handle;
  // 私有命名空间，其话题名称的上一级是话题名称
  ros::NodeHandle private_node_handle("~");

  sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D tare_planner(node_handle, private_node_handle);

  ros::spin();
  return 0;
}