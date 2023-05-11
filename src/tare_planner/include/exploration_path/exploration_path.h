/*
 * @Author: jia
 * @Date: 2023-03-17 20:25:30
 * @LastEditors: jia
 * @LastEditTime: 2023-05-10 21:46:01
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
  //相当于给枚举添加作用域
  //节点类型
enum class NodeType
{
  ROBOT = 0,
  LOOKAHEAD_POINT = 2, //前瞻点
  LOCAL_VIEWPOINT = 4, //局部坐标系中的视点
  LOCAL_PATH_START = 6, //局部路径的起始点
  LOCAL_PATH_END = 8, //局部路径的终点
  LOCAL_VIA_POINT = 10, 
  GLOBAL_VIEWPOINT = 1, //全局坐标系中的视点
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
  bool IsLocal(); //判断节点是不是局部的
  friend bool operator==(const Node& n1, const Node& n2); //判断两个节点是否是同一个节点
  friend bool operator!=(const Node& n1, const Node& n2); //判断两个节点是不是不同的节点
};
struct ExplorationPath
{
  std::vector<Node> nodes_;  //用于存储路径上的节点
  //函数前面使用const表示返回值为const，函数后面加const表示函数不可以修改class的成员
  double GetLength() const; //计算一系列节点的距离
  int GetNodeNum() const //获取节点的数量
  {
    return nodes_.size();
  }
  void Append(const Node& node); //向一系列节点中增加节点。（相邻的两个节点不是同一个节点时才添加）
  void Append(const ExplorationPath& path); //向一系列节点中增加路径。
  void Reverse(); //反转路径中节点的顺序
  nav_msgs::Path GetPath() const; //根据节点获取路径
  void FromPath(const nav_msgs::Path& path); //从路径中得到节点
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const; //将所有节点转为可视点云，可视点云的强度信息存储的是节点的类型
  void GetKeyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const; //将关键点转为可视点云，可视点云的强度信息存储的是节点的类型
  void GetNodePositions(std::vector<Eigen::Vector3d>& positions) const; // 获取节点的位置
  void Reset(); //清空所有节点
};
}  // namespace exploration_path_ns
