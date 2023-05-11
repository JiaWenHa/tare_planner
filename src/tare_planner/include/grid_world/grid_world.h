/**
 * @file grid_world.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <memory>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <grid/grid.h>
#include <tsp_solver/tsp_solver.h>
#include <keypose_graph/keypose_graph.h>
#include <exploration_path/exploration_path.h>

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace grid_world_ns
{
enum class CellStatus
{
  UNSEEN = 0, //未探索
  EXPLORING = 1, //正在探索中
  COVERED = 2, //已经探索
  COVERED_BY_OTHERS = 3, //已经被其他智能体探索
  NOGO = 4 
};

class Cell
{
public:
  explicit Cell(double x = 0.0, double y = 0.0, double z = 0.0);
  explicit Cell(const geometry_msgs::Point& center);
  ~Cell() = default;
  bool IsCellConnected(int cell_ind);
  //添加视点
  void AddViewPoint(int viewpoint_ind)
  {
    viewpoint_indices_.push_back(viewpoint_ind);
  }
  //添加图的节点
  void AddGraphNode(int node_ind)
  {
    keypose_graph_node_indices_.push_back(node_ind);
  }
  void AddConnectedCell(int cell_ind)
  {
    connected_cell_indices_.push_back(cell_ind);
    misc_utils_ns::UniquifyIntVector(connected_cell_indices_);
  }
  //清除所有视点
  void ClearViewPointIndices()
  {
    viewpoint_indices_.clear();
  }
  //清除所有的关键位置点
  void ClearGraphNodeIndices()
  {
    keypose_graph_node_indices_.clear();
  }
  void ClearConnectedCellIndices()
  {
    connected_cell_indices_.clear();
  }
  //获取单元格的状态，这个单元格是本地规划器视野中的单元格
  CellStatus GetStatus()
  {
    return status_;
  }
  //设置单元格的状态
  void SetStatus(CellStatus status)
  {
    status_ = status;
  }
  //获取视点的索引号
  std::vector<int> GetViewPointIndices()
  {
    return viewpoint_indices_;
  }
  std::vector<int> GetConnectedCellIndices()
  {
    return connected_cell_indices_;
  }
  //获取关键位置节点的索引
  std::vector<int> GetGraphNodeIndices()
  {
    return keypose_graph_node_indices_;
  }
  // 获取局部视野的中心点
  geometry_msgs::Point GetPosition()
  {
    return center_;
  }
  //设置局部视野的中心点
  void SetPosition(const geometry_msgs::Point& position)
  {
    center_ = position;
  }
  //设置机器人的位置
  void SetRobotPosition(const geometry_msgs::Point& robot_position)
  {
    robot_position_ = robot_position;
    robot_position_set_ = true;
  }
  //设置关键位置点的ID
  void SetKeyposeID(int keypose_id)
  {
    keypose_id_ = keypose_id;
    robot_position_set_ = true;
  }
  //获取机器人的位置是否设置标志
  bool IsRobotPositionSet()
  {
    return robot_position_set_;
  }
  //获取机器人的位置
  geometry_msgs::Point GetRobotPosition()
  {
    return robot_position_;
  }
  //增加单元格被机器人的次数
  void AddVisitCount()
  {
    visit_count_++;
  }
  // 获取单元格被机器人访问的次数
  int GetVisitCount()
  {
    return visit_count_;
  }
  // 初始化所有的单元格的状态
  void Reset();
  //获取可以观察到当前单元格的机器人的位置ID
  int GetKeyposeID()
  {
    return keypose_id_;
  }
  //设置视点的位置
  void SetViewPointPosition(const Eigen::Vector3d& position)
  {
    viewpoint_position_ = position;
  }
  //获取视点的位置
  Eigen::Vector3d GetViewPointPosition()
  {
    return viewpoint_position_;
  }
  //设置单元格连接到全局路线图的位置点
  void SetRoadmapConnectionPoint(const Eigen::Vector3d& roadmap_connection_point)
  {
    roadmap_connection_point_ = roadmap_connection_point;
  }
  //获取单元格连接到全局路线图的位置点
  Eigen::Vector3d GetRoadmapConnectionPoint()
  {
    return roadmap_connection_point_;
  }
  //获取关键位置图上到达最近的关键位置的路径
  nav_msgs::Path GetPathToKeyposeGraph()
  {
    return path_to_keypose_graph_;
  }
  //设置关键位置图上到达最近的关键位置的路径
  void SetPathToKeyposeGraph(const nav_msgs::Path& path)
  {
    path_to_keypose_graph_ = path;
  }
  // 路径是否已经被添加到关键位置图
  bool IsPathAddedToKeyposeGraph()
  {
    return path_added_to_keypose_graph_;
  }
  // 设置路径是否已经被添加到关键位置图的标志位
  void SetPathAddedToKeyposeGraph(bool add_path)
  {
    path_added_to_keypose_graph_ = add_path;
  }
  // 是否路线图连接点已添加到单元格
  bool IsRoadmapConnectionPointSet()
  {
    return roadmap_connection_point_set_;
  }
  // 设置是否路线图连接点已添加到单元格的标志
  void SetRoadmapConnectionPointSet(bool set)
  {
    roadmap_connection_point_set_ = set;
  }

private:
  CellStatus status_;
  // The center location of this cell.
  // 此单元格的中心位置。
  geometry_msgs::Point center_;
  // Position of the robot where this cell is first observed and turned EXPLORING
  geometry_msgs::Point robot_position_;
  // Whether the robot position has been set for this cell
  // 该单元格是否设置了机器人位置
  bool robot_position_set_;
  // Number of times the cell is visited by the robot
  //单元格被机器人访问的次数
  int visit_count_;
  // Indices of the viewpoints within this cell.
  // 单元格内视点的索引
  std::vector<int> viewpoint_indices_;
  // Indices of other cells that are connected by a path.
  // 被一条路径连接的其他单元的序列号
  std::vector<int> connected_cell_indices_;
  // Indices of connected keypose graph nodes
  // 关键位置图节点的索引号
  std::vector<int> keypose_graph_node_indices_;
  // Whether this cell is in the planning horizon, which consists of nine cells around the robot.
  //这个单元格是否在规划范围内，它由机器人周围的九个单元格组成。
  bool in_horizon_;
  // ID of the keypose where viewpoints in this cell can be observed
  // 可以观察到此单元格中的视点的关键姿势的 ID
  int keypose_id_;
  // Position of the highest score viewpoint
  // 最高分视点的位置
  Eigen::Vector3d viewpoint_position_;
  // Position for connecting the cell to the global roadmap
  // 将单元格连接到全局路线图的位置
  Eigen::Vector3d roadmap_connection_point_;
  // Path to the nearest keypose on the keypose graph
  // 关键位置图上到达最近的关键位置的路径
  nav_msgs::Path path_to_keypose_graph_;
  // If the path has been added to the keypose graph
  // 是否路径已添加到关键姿势图中
  bool path_added_to_keypose_graph_;
  // If the roadmap connection point has been added to the cell
  // 是否路线图连接点已添加到单元格
  bool roadmap_connection_point_set_;
};

class GridWorld
{
public:
  explicit GridWorld(ros::NodeHandle& nh);
  explicit GridWorld(int row_num = 1, int col_num = 1, int level_num = 1, double cell_size = 6.0,
                     double cell_height = 6.0, int nearby_grid_num = 5);
  ~GridWorld() = default;
  void ReadParameters(ros::NodeHandle& nh);
  void UpdateNeighborCells(const geometry_msgs::Point& robot_position);
  void UpdateRobotPosition(const geometry_msgs::Point& robot_position);
  void UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  int GetMinAddPointNum()
  {
    return kMinAddPointNumSmall;
  }
  //获取xx变成前沿点的最少点云数量
  int GetMinAddFrontierPointNum()
  {
    return kMinAddFrontierPointNum;
  }

  //获取全局环境中子空间的原点
  geometry_msgs::Point GetOrigin()
  {
    // return origin_;
    Eigen::Vector3d origin = subspaces_->GetOrigin();
    geometry_msgs::Point geo_origin;
    geo_origin.x = origin.x();
    geo_origin.y = origin.y();
    geo_origin.z = origin.z();
    return geo_origin;
  }
  //将体素网格的位置编号转换为体素网格的序号
  int sub2ind(const Eigen::Vector3i& sub)
  {
    return subspaces_->Sub2Ind(sub);
  }
  int sub2ind(int row_idx, int col_idx, int level_idx)
  {
    return subspaces_->Sub2Ind(row_idx, col_idx, level_idx);
  }
  //将体素网格的序号转换为体素网格中的位置编号
  Eigen::Vector3i ind2sub(int ind)
  {
    return subspaces_->Ind2Sub(ind);
  }
  void ind2sub(int ind, int& row_idx, int& col_idx, int& level_idx)
  {
    Eigen::Vector3i sub = subspaces_->Ind2Sub(ind);
    row_idx = sub.x();
    col_idx = sub.y();
    level_idx = sub.z();
  }
  //判断是否在范围内
  bool SubInBound(const Eigen::Vector3i& sub)
  {
    return subspaces_->InRange(sub);
  }
  bool SubInBound(int row_idx, int col_idx, int level_idx)
  {
    return subspaces_->InRange(Eigen::Vector3i(row_idx, col_idx, level_idx));
  }
  bool IndInBound(int ind)
  {
    return subspaces_->InRange(ind);
  }
  // Get the cell index where the robot is currently in.
  //判断两个单元格是否是邻居
  bool AreNeighbors(int cell_ind1, int cell_ind2);
  //根据坐标获取单元格的ID
  int GetCellInd(double qx, double qy, double qz);
  //根据坐标获取单元格的位置编号
  void GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz);
  Eigen::Vector3i GetCellSub(const Eigen::Vector3d& point);
  // Get the visualization markers for Rviz display.
  //获取 Rviz 显示的可视化标记。
  void GetMarker(visualization_msgs::Marker& marker);
  // Get the visualization pointcloud for debugging purpose
  //获取用于调试目的的可视化点云
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);

  //获取初始化标志
  bool Initialized()
  {
    return initialized_;
  }
  //设置是否使用关键位位置图
  void SetUseKeyposeGraph(bool use_keypose_graph)
  {
    use_keypose_graph_ = use_keypose_graph;
  }
  //获取是否使用了关键位置图标志
  bool UseKeyposeGraph()
  {
    return use_keypose_graph_;
  }

  //在单元格内添加视点
  void AddViewPointToCell(int cell_ind, int viewpoint_ind);
  //在单元格内添加关键位置图节点
  void AddGraphNodeToCell(int cell_ind, int node_ind);
  //清除单元格内的视点
  void ClearCellViewPointIndices(int cell_ind);
  //获取单元格内的视点序列
  std::vector<int> GetCellViewPointIndices(int cell_ind);
  //获取单元格的领域单元格
  std::vector<int> GetNeighborCellIndices()
  {
    return neighbor_cell_indices_;
  };

  void GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);
  void GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);
  //获取正在探索中的单元格的索引
  void GetExploringCellIndices(std::vector<int>& exploring_cell_indices);
  // 获取单元格的状态
  CellStatus GetCellStatus(int cell_ind);
  //设置单元格的状态
  void SetCellStatus(int cell_ind, CellStatus status);
  //根据单元格的ID设置获取单元格的位置
  geometry_msgs::Point GetCellPosition(int cell_ind);
  //设置单元格中机器人的位置
  void SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position);
  //获取单元格中机器人的位置
  geometry_msgs::Point GetCellRobotPosition(int cell_ind);

  //增加单元格中视点的数量
  void CellAddVisitCount(int cell_ind);
  //获取单元格中视点的数量
  int GetCellVisitCount(int cell_ind);
  //获取单元格中机器人位置是否设置标志
  bool IsRobotPositionSet(int cell_ind);
  //重置所有单元格
  void Reset();
  //获得某个状态的单元格的数量
  int GetCellStatusCount(grid_world_ns::CellStatus status);
  //根据视点的数量的状态更新单元格的状态
  void UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  //求解全局TSP问题，获得全局TSP路径
  exploration_path_ns::ExplorationPath
  SolveGlobalTSP(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                 std::vector<int>& ordered_cell_indices,
                 const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph = nullptr);

  //设置当前位置图节点的ID
  inline void SetCurKeyposeGraphNodeInd(int node_ind)
  {
    cur_keypose_graph_node_ind_ = node_ind;
  }
  //设置当前位置图节点的位置
  inline void SetCurKeyposeGraphNodePosition(geometry_msgs::Point node_position)
  {
    cur_keypose_graph_node_position_ = node_position;
  }
  //设置当前关键位置的ID
  inline void SetCurKeyposeID(int keypose_id)
  {
    cur_keypose_id_ = keypose_id;
  }
  //设置当前关键位置的位置
  inline void SetCurKeypose(const Eigen::Vector3d& cur_keypose)
  {
    cur_keypose_ = cur_keypose;
  }
  //获取某个单元格内关键位置的ID
  int GetCellKeyposeID(int cell_ind);
  //设置家的位置
  void SetHomePosition(const Eigen::Vector3d& home_position)
  {
    home_position_ = home_position;
    set_home_ = true;
  }
  //获取家的位置是否设置的标志
  bool HomeSet()
  {
    return set_home_;
  }
  //获取是否回家的标志
  bool IsReturningHome()
  {
    return return_home_;
  }

  //获取单元格内视点的位置
  void GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions);
  //在两个单元格之间增加路径
  void AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                              const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  //判断两个单元格之间的路径是否有效
  bool PathValid(const nav_msgs::Path& path, int from_cell_ind, int to_cell_ind);
  //判断两个位置之间是否有直接的关键位置图连接
  bool HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                       const Eigen::Vector3d& start_position, const Eigen::Vector3d& goal_position);

private:
  int kRowNum; //行数
  int kColNum; //列数
  int kLevelNum; 
  double kCellSize; //单元格的尺寸
  double kCellHeight; //单元格的高度
  int KNearbyGridNum;
  int kMinAddPointNumSmall;
  int kMinAddPointNumBig;
  int kMinAddFrontierPointNum; //xx 变为前沿点的最小点云数量
  int kCellExploringToCoveredThr; //单元格从探索中到探索完的阈值
  int kCellCoveredToExploringThr; //单元格从探索完到正在探索中的阈值
  int kCellExploringToAlmostCoveredThr; //单元格从探索中到几乎探索完的阈值
  int kCellAlmostCoveredToExploringThr; //单元格从几乎探索完到探索中的阈值
  int kCellUnknownToExploringThr; //单元格从未知到正在探索中的阈值

  std::vector<Cell> cells_; //栅格世界中的小栅格
  std::unique_ptr<grid_ns::Grid<Cell>> subspaces_; //全局环境中的子空间
  bool initialized_; //初始化标志
  bool use_keypose_graph_; //使用关键位置图的标志
  int cur_keypose_id_; //当前的关键位置号
  geometry_msgs::Point robot_position_; // 机器人的位置
  geometry_msgs::Point origin_;
  std::vector<int> neighbor_cell_indices_; //用于存储当前单元格的领单元格的号
  std::vector<int> almost_covered_cell_indices_; //几乎覆盖的单元格的序号
  std::vector<std::pair<int, int>> to_connect_cell_indices_;
  std::vector<nav_msgs::Path> to_connect_cell_paths_;
  Eigen::Vector3d home_position_; //家的位置
  Eigen::Vector3d cur_keypose_; //当前的关键位置
  bool set_home_; //是否设置家的位置
  bool return_home_; //是否回家的标志
  geometry_msgs::Point cur_keypose_graph_node_position_; //当前关键位置图中节点的位置
  int cur_keypose_graph_node_ind_; //当前关键位置图中节点的序号
  int cur_robot_cell_ind_; //当前机器人所在单元格的序号
  int prev_robot_cell_ind_; //前一个机器人所在单元格的序号
};
}  // namespace grid_world_ns
