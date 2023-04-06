/*
 * @Author: jia
 * @Date: 2023-03-17 20:25:30
 * @LastEditors: jia
 * @LastEditTime: 2023-04-06 20:28:08
 * @Description: 请填写简介
 */
/**
 * @file exploration_path.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the exploration path
 * @version 0.1
 * @date 2020-10-22
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <utils/misc_utils.h>
#include <utils/pointcloud_utils.h>

namespace exploration_path_ns
{
enum class NodeType
{
  ROBOT = 0,
  LOOKAHEAD_POINT = 2,
  LOCAL_VIEWPOINT = 4,
  LOCAL_PATH_START = 6,
  LOCAL_PATH_END = 8,
  LOCAL_VIA_POINT = 10,
  GLOBAL_VIEWPOINT = 1,
  GLOBAL_VIA_POINT = 3,
  HOME = 5
};
struct Node
{
  NodeType type_; //节点类型
  Eigen::Vector3d position_;//节点的位置
  int local_viewpoint_ind_;
  int keypose_graph_node_ind_;
  int global_subspace_index_;
  bool nonstop_;
  explicit Node();
  explicit Node(Eigen::Vector3d position);
  explicit Node(geometry_msgs::Point point, NodeType type);
  ~Node() = default;
  bool IsLocal();
  friend bool operator==(const Node& n1, const Node& n2);
  friend bool operator!=(const Node& n1, const Node& n2);
};
struct ExplorationPath
{
  std::vector<Node> nodes_;  //用于存储路径上的节点
  //函数前面使用const表示返回值为const，函数后面加const表示函数不可以修改class的成员
  double GetLength() const;
  int GetNodeNum() const
  {
    return nodes_.size();
  }
  void Append(const Node& node);
  void Append(const ExplorationPath& path);
  void Reverse();
  nav_msgs::Path GetPath() const;
  void FromPath(const nav_msgs::Path& path);
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const;
  void GetKeyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const;
  void GetNodePositions(std::vector<Eigen::Vector3d>& positions) const;
  void Reset();
};
}  // namespace exploration_path_ns
