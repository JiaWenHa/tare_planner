/**
 * @file graph.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a graph
 * @version 0.1
 * @date 2021-07-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <queue>
#include <ros/ros.h>
#include "utils/misc_utils.h"
#include "graph/graph.h"

namespace tare
{
Graph::Graph(int node_number)
{
  connection_.resize(node_number);
  distance_.resize(node_number);
  positions_.resize(node_number);
}

void Graph::AddNode(const Eigen::Vector3d& position)
{
  std::vector<int> connection;
  connection_.push_back(connection);
  std::vector<double> neighbor_distance;
  distance_.push_back(neighbor_distance);
  positions_.push_back(position);
}

void Graph::SetNodePosition(int node_index, const Eigen::Vector3d& position)
{
  if (NodeIndexInRange(node_index))
  {
    positions_[node_index] = position;
  }
  else if (node_index == positions_.size())
  {
    AddNode(position);
  }
  else
  {
    ROS_ERROR_STREAM("Graph::SetNodePosition: node_index: " << node_index << " not in range [0, "
                                                            << positions_.size() - 1 << "]");
  }
}

void Graph::AddOneWayEdge(int from_node_index, int to_node_index, double distance)
{
  if (NodeIndexInRange(from_node_index) && NodeIndexInRange(to_node_index))
  {
    connection_[from_node_index].push_back(to_node_index);
    distance_[from_node_index].push_back(distance);
  }
  else
  {
    ROS_ERROR_STREAM("Graph::AddOneWayEdge: from_node_index: " << from_node_index << " to_node_index: " << to_node_index
                                                               << " not in range [0, " << connection_.size() - 1
                                                               << "]");
  }
}

void Graph::AddTwoWayEdge(int from_node_index, int to_node_index, double distance)
{
  AddOneWayEdge(from_node_index, to_node_index, distance);
  AddOneWayEdge(to_node_index, from_node_index, distance);
}

double Graph::GetShortestPath(int from_node_index, int to_node_index, bool get_path, nav_msgs::Path& shortest_path,
                              std::vector<int>& node_indices)
{
  node_indices.clear();
  double path_length = AStarSearch(from_node_index, to_node_index, get_path, node_indices);
  if (get_path)
  {
    shortest_path.poses.clear();
    for (const auto& node_index : node_indices)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = positions_[node_index].x();
      pose.pose.position.y = positions_[node_index].y();
      pose.pose.position.z = positions_[node_index].z();
      shortest_path.poses.push_back(pose);
    }
  }
  return path_length;
}

/**
 * 该函数返回从起始节点到目标节点的最短距离。
*/
double Graph::AStarSearch(int from_node_index, int to_node_index, bool get_path, std::vector<int>& node_indices)
{
  MY_ASSERT(NodeIndexInRange(from_node_index));
  MY_ASSERT(NodeIndexInRange(to_node_index));

  double INF = 9999.0;
  //为一对双精度值（表示距离）和一个整数值（表示节点索引）定义了类型别名 iPair。
  typedef std::pair<double, int> iPair;
  double shortest_dist = 0; //最短路径的长度
  //std::greater<iPair>的意思是将具有最小优先级值的元素将位于优先级队列的顶部
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
  std::vector<double> g(connection_.size(), INF); //存储从起始节点到目前遇到的每个节点的成本。
  std::vector<double> f(connection_.size(), INF); //用于存储预估成本
  std::vector<int> prev(connection_.size(), -1); //跟踪最短路径上的前一个节点。
  std::vector<bool> in_pg(connection_.size(), false); //布尔标志，指示节点当前是否在优先级队列中。

  g[from_node_index] = 0;
  f[from_node_index] = (positions_[from_node_index] - positions_[to_node_index]).norm();//计算两个节点之间的欧式距离，即预估成本

  pq.push(std::make_pair(f[from_node_index], from_node_index));//优先级队列中存储两个节点之间的距离和序号
  in_pg[from_node_index] = true;

  while (!pq.empty())
  {
    int u = pq.top().second; //获取当前节点的索引
    pq.pop();
    in_pg[u] = false;

    //如果到达终点则退出
    if (u == to_node_index)
    {
      shortest_dist = g[u];
      break;
    }

    for (int i = 0; i < connection_[u].size(); i++)
    {
      int v = connection_[u][i]; //获取当前节点的邻接结点
      MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(connection_, v));
      double d = distance_[u][i]; //获取相邻两个节点的距离
      //如果到下一个位置的新路径优于先前的最佳路径，我们将添加它
      if (g[v] > g[u] + d)
      {
        prev[v] = u;
        g[v] = g[u] + d;
        f[v] = g[v] + (positions_[v] - positions_[to_node_index]).norm();
        if (!in_pg[v])
        {
          pq.push(std::make_pair(f[v], v));
          in_pg[v] = true;
        }
      }
    }
  }

  if (get_path)
  {
    node_indices.clear();
    int u = to_node_index;
    if (prev[u] != -1 || u == from_node_index)
    {
      while (u != -1)
      {
        node_indices.push_back(u);
        u = prev[u];
      }
    }
  }
  std::reverse(node_indices.begin(), node_indices.end());

  return shortest_dist;
}

}  // namespace tare
