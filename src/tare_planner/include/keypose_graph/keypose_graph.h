//
// Created by caochao on 12/31/19.
//

#ifndef SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
#define SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <planning_env/planning_env.h>
#include <utils/misc_utils.h>

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace keypose_graph_ns
{
struct KeyposeNode;
class KeyposeGraph;
const double INF = 9999.0;
typedef std::pair<int, int> iPair;
}  // namespace keypose_graph_ns

struct keypose_graph_ns::KeyposeNode
{
  geometry_msgs::Point position_; //位置
  geometry_msgs::Point offset_to_keypose_; //到关键位置的偏差
  int keypose_id_; //关键位置的ID
  int node_ind_; //节点的ID
  int cell_ind_; //单元格的ID
  bool is_keypose_; //是否是关键位置的标志
  bool is_connected_; //当前节点是否被连接

public:
  explicit KeyposeNode(double x = 0, double y = 0, double z = 0, int node_ind = 0, int keypose_id = 0,
                       bool is_keypose = true);
  explicit KeyposeNode(const geometry_msgs::Point& point, int node_ind = 0, int keypose_id = 0, bool is_keypose = true);
  ~KeyposeNode() = default;
  //返回当前节点是否是关键位置
  bool IsKeypose() const
  {
    return is_keypose_;
  }
  //返回当前节点是否被连接
  bool IsConnected() const
  {
    return is_connected_;
  }
  //设置节点到关键位置的偏差
  void SetOffsetToKeypose(const geometry_msgs::Point& offset_to_keypose)
  {
    offset_to_keypose_ = offset_to_keypose;
  }
  //设置当前关键位置的位置
  void SetCurrentKeyposePosition(const geometry_msgs::Point& current_keypose_position)
  {
    offset_to_keypose_.x = position_.x - current_keypose_position.x;
    offset_to_keypose_.y = position_.y - current_keypose_position.y;
    offset_to_keypose_.z = position_.z - current_keypose_position.z;
  }
};

class keypose_graph_ns::KeyposeGraph
{
private:
  bool allow_vertical_edge_; //是否运行垂直边
  int current_keypose_id_; //关键位姿的id
  geometry_msgs::Point current_keypose_position_; //关键位姿的位置
  std::vector<std::vector<int>> graph_; //关键位置图
  std::vector<std::vector<double>> dist_; //边的权重，即两个节点之间的距离
  std::vector<bool> in_local_planning_horizon_; //是否在本地规划器视野
  std::vector<KeyposeNode> nodes_; //节点
  std::vector<geometry_msgs::Point> node_positions_; //节点的位置
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_connected_nodes_; //存储有连接的节点的k-d tree
  pcl::PointCloud<pcl::PointXYZI>::Ptr connected_nodes_cloud_; //有连接的节点
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_nodes_; //存储节点的k-d tree
  pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_cloud_; //节点

  std::vector<int> connected_node_indices_; //存在连接的节点组

  double kAddNodeMinDist; //增加节点的最小距离
  double kAddNonKeyposeNodeMinDist; //增加非关键位置节点的最小距离
  double kAddEdgeConnectDistThr; //增加边连接的距离阈值
  double kAddEdgeToLastKeyposeDistThr; //将边缘添加到最后一个键位距离阈值
  double kAddEdgeVerticalThreshold; //增加垂直边的阈值
  double kAddEdgeCollisionCheckResolution; //增加边碰撞检查分辨率
  double kAddEdgeCollisionCheckRadius; //增加边碰撞检查的半径
  int kAddEdgeCollisionCheckPointNumThr; //正价边碰撞检查点的数量阈值

  //比较两个对
  static bool ComparePair(const std::pair<int, int>& a, const std::pair<int, int>& b)
  {
    return (a.first == b.first && a.second == b.second) || (a.first == b.second && a.second == b.first);
  }

public:
  KeyposeGraph(ros::NodeHandle& nh);
  ~KeyposeGraph() = default;
  //从yaml文件中读取参数值
  void ReadParameters(ros::NodeHandle& nh);
  //在图中添加节点
  void AddNode(const geometry_msgs::Point& position, int node_ind, int keypose_id, bool is_keypose);
  //在图中添加节点和边
  void AddNodeAndEdge(const geometry_msgs::Point& position, int node_ind, int keypose_id, bool is_keypose,
                      int connected_node_ind, double connected_node_dist);
  //给两个节点添加边和权重（距离）
  void AddEdge(int from_node_ind, int to_node_ind, double dist);
  //判断某个位置是否有节点
  bool HasNode(const Eigen::Vector3d& position);
  //判断某个序列号是否在范围内
  bool InBound(int index)
  {
    return index >= 0 && index < graph_.size();
  }
  //获得图中节点的数量
  int GetNodeNum()
  {
    return nodes_.size();
  }
  //获得图中有连接的节点的数量
  int GetConnectedNodeNum();
  //将图变成Marker可视化
  void GetMarker(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker);
  //节点变成可视点云数据
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  //获得有连接的图中的节点组
  std::vector<int> GetConnectedGraphNodeIndices()
  {
    return connected_node_indices_;
  }
  void GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices, std::vector<bool> constraints);
  //检查局部碰撞
  void CheckLocalCollision(const geometry_msgs::Point& robot_position,
                           const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  //更新节点到k-d tree中
  void UpdateNodes();
  //检查机器人当前位置与图的连接性
  void CheckConnectivity(const geometry_msgs::Point& robot_position);
  //增加关键位置节点
  int AddKeyposeNode(const nav_msgs::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env);
  //判断两个节点之间是否有边
  bool HasEdgeBetween(int node_ind1, int node_ind2);
  //判断两个位置是否已经有连接
  bool IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position);
  //增加非关键位置的节点
  int AddNonKeyposeNode(const geometry_msgs::Point& new_node_position);
  //向图中添加路径
  void AddPath(const nav_msgs::Path& path);
  //设置允许垂直边
  void SetAllowVerticalEdge(bool allow_vertical_edge)
  {
    allow_vertical_edge_ = allow_vertical_edge;
  }
  //判断某个点是否可达
  bool IsPositionReachable(const geometry_msgs::Point& point, double dist_threshold);
  bool IsPositionReachable(const geometry_msgs::Point& point);
  //获取距离当前点最近的节点的ID
  int GetClosestNodeInd(const geometry_msgs::Point& point);
  //获取距离当前点最近的节点的ID和距离
  void GetClosestNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist);
  //获取距离当前点最近的已连接的节点的ID和距离
  void GetClosestConnectedNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind, double& dist);
  //获得最近的关键位置的ID
  int GetClosestKeyposeID(const geometry_msgs::Point& point);
  //获得最近的节点的位置
  geometry_msgs::Point GetClosestNodePosition(const geometry_msgs::Point& point);

  bool GetShortestPathWithMaxLength(const geometry_msgs::Point& start_point, const geometry_msgs::Point& target_point,
                                    double max_path_length, bool get_path, nav_msgs::Path& path);
  //得到最短路径
  double GetShortestPath(const geometry_msgs::Point& start_point, const geometry_msgs::Point& target_point,
                         bool get_path, nav_msgs::Path& path, bool use_connected_nodes = false);

  //设置增加节点的最小距离
  double& SetAddNodeMinDist()
  {
    return kAddNodeMinDist;
  }
  //设置增加非关键位置的节点的最小距离
  double& SetAddNonKeyposeNodeMinDist()
  {
    return kAddNonKeyposeNodeMinDist;
  }

  double& SetAddEdgeCollisionCheckResolution()
  {
    return kAddEdgeCollisionCheckResolution;
  }
  //设置增加边的碰撞检测半径
  double& SetAddEdgeCollisionCheckRadius()
  {
    return kAddEdgeCollisionCheckRadius;
  }
  int& SetAddEdgeCollisionCheckPointNumThr()
  {
    return kAddEdgeCollisionCheckPointNumThr;
  }

  double& SetAddEdgeConnectDistThr()
  {
    return kAddEdgeConnectDistThr;
  }
  
  double& SetAddEdgeToLastKeyposeDistThr()
  {
    return kAddEdgeToLastKeyposeDistThr;
  }
  double& SetAddEdgeVerticalThreshold()
  {
    return kAddEdgeVerticalThreshold;
  }
  geometry_msgs::Point GetFirstKeyposePosition();
  geometry_msgs::Point GetKeyposePosition(int keypose_id);
  void GetKeyposePositions(std::vector<Eigen::Vector3d>& positions);
  geometry_msgs::Point GetNodePosition(int node_ind);
};

#endif  // SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
