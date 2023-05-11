/*
 * @Author: jia
 * @Date: 2023-03-17 20:25:30
 * @LastEditors: jia
 * @LastEditTime: 2023-05-10 21:02:12
 * @Description: A star算法相关用来构建图和获取A star路径
 */
/**
 * @file graph.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a graph
 * @version 0.1
 * @date 2021-07-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Core>

#include <nav_msgs/Path.h>

namespace tare
{
class Graph
{
public:
  explicit Graph(int node_number);
  ~Graph() = default;

  void AddNode(const Eigen::Vector3d& position); //增加一个节点
  void SetNodePosition(int node_index, const Eigen::Vector3d& position); //设置节点的位置
  void AddOneWayEdge(int from_node_index, int to_node_index, double distance); //给两个节点添加单向边
  void AddTwoWayEdge(int from_node_index, int to_node_index, double distance); //给两个节点添加双向边
  //获取两个节点之间的最短距离的路径shortest_path，并返回路径的长度
  double GetShortestPath(int from_node_index, int to_node_index, bool get_path, nav_msgs::Path& shortest_path,
                         std::vector<int>& node_indices);

private:
  bool NodeIndexInRange(int node_index)
  {
    return node_index >= 0 && node_index < connection_.size();
  }
  // 返回两个节点之间的最短距离
  double AStarSearch(int from_node_index, int to_node_index, bool get_path, std::vector<int>& node_indices);
  // Node connectivity 用来存储节点之间的连接性
  std::vector<std::vector<int>> connection_;
  // Distances between two nodes 用来存储节点之间的距离
  std::vector<std::vector<double>> distance_;
  // Node positions 用来存储节点的位置
  std::vector<Eigen::Vector3d> positions_;
};
}  // namespace tare
