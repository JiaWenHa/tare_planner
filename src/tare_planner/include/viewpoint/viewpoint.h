/**
 * @file viewpoint.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a viewpoint
 * @version 0.1
 * @date 2019-11-04
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <geometry_msgs/Point.h>
#include <lidar_model/lidar_model.h>

namespace viewpoint_ns
{
class ViewPoint
{
public:
  explicit ViewPoint(double x = 0.0, double y = 0.0, double z = 0.0);
  explicit ViewPoint(const geometry_msgs::Point& position);
  ~ViewPoint() = default;

  template <class PCLPointType>
  void UpdateCoverage(const PCLPointType& point)
  {
    lidar_model_.UpdateCoverage<PCLPointType>(point);
  }
  template <class PCLPointType>
  bool CheckVisibility(const PCLPointType& point, double occlusion_threshold) const
  {
    return lidar_model_.CheckVisibility<PCLPointType>(point, occlusion_threshold);
  }
  void SetPosition(const geometry_msgs::Point& position)
  {
    lidar_model_.setPosition(position);
  }
  double GetX() const
  {
    return lidar_model_.getPosition().x;
  }
  double GetY() const
  {
    return lidar_model_.getPosition().y;
  }
  double GetHeight() const
  {
    return lidar_model_.getPosition().z;
  }

  void SetHeight(double height)
  {
    lidar_model_.SetHeight(height);
  }

  geometry_msgs::Point GetPosition() const
  {
    return lidar_model_.getPosition();
  }
  void ResetCoverage();
  void Reset();

  void SetInCollision(bool in_collision)
  {
    in_collision_ = in_collision;
  }
  bool InCollision() const
  {
    return in_collision_;
  }

  void SetInLineOfSight(bool in_line_of_sight)
  {
    in_line_of_sight_ = in_line_of_sight;
  }
  bool InLineOfSight() const
  {
    return in_line_of_sight_;
  }

  void SetInCurrentFrameLineOfSight(bool in_current_frame_line_of_sight)
  {
    in_current_frame_line_of_sight_ = in_current_frame_line_of_sight;
  }
  bool InCurrentFrameLineOfSight() const
  {
    return in_current_frame_line_of_sight_;
  }

  void SetConnected(bool connected)
  {
    connected_ = connected;
  }
  bool Connected() const
  {
    return connected_;
  }

  void SetVisited(bool visited)
  {
    visited_ = visited;
  }
  bool Visited() const
  {
    return visited_;
  }

  void SetSelected(bool selected)
  {
    selected_ = selected;
  }
  bool Selected() const
  {
    return selected_;
  }

  void SetCandidate(bool candidate)
  {
    is_candidate_ = candidate;
  }
  bool IsCandidate() const
  {
    return is_candidate_;
  }

  void SetHasTerrainHeight(bool has_terrain_height)
  {
    has_terrain_height_ = has_terrain_height;
  }
  bool HasTerrainHeight() const
  {
    return has_terrain_height_;
  }

  void SetTerrainHeight(double terrain_height)
  {
    terrain_height_ = terrain_height;
  }
  double GetTerrainHeight() const
  {
    return terrain_height_;
  }

  void SetHasTerrainNeighbor(bool has_terrain_neighbor)
  {
    has_terrain_neighbor_ = has_terrain_neighbor;
  }
  bool HasTerrainNeighbor() const
  {
    return has_terrain_neighbor_;
  }

  void SetInExploringCell(bool in_exploring_cell)
  {
    in_exploring_cell_ = in_exploring_cell;
  }
  bool InExploringCell() const
  {
    return in_exploring_cell_;
  }

  void SetCellInd(int cell_ind)
  {
    cell_ind_ = cell_ind;
  }
  int GetCellInd() const
  {
    return cell_ind_;
  }

  int GetCellIndex() const
  {
    return cell_ind_;
  }

  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) const
  {
    lidar_model_.GetVisualizationCloud(vis_cloud);
  }
  void ResetCoveredPointList()
  {
    covered_point_list_.clear();
  }
  void ResetCoveredFrontierPointList()
  {
    covered_frontier_point_list_.clear();
  }
  const std::vector<int>& GetCoveredPointList() const
  {
    return covered_point_list_;
  }
  const std::vector<int>& GetCoveredFrontierPointList() const
  {
    return covered_frontier_point_list_;
  }
  void AddCoveredPoint(int point_idx)
  {
    covered_point_list_.push_back(point_idx);
  }
  void AddCoveredFrontierPoint(int point_idx)
  {
    covered_frontier_point_list_.push_back(point_idx);
  }
  int GetCoveredPointNum() const
  {
    return covered_point_list_.size();
  }
  int GetCoveredFrontierPointNum() const
  {
    return covered_frontier_point_list_.size();
  }
  int GetCollisionFrameCount() const
  {
    //collision_frame_count_指一个viewpoint在连续多少帧内存在碰撞
    return collision_frame_count_;
  }
  void AddCollisionFrame()
  {
    collision_frame_count_++;
  }
  void ResetCollisionFrameCount()
  {
    collision_frame_count_ = 0;
  }

private:
  lidar_model_ns::LiDARModel lidar_model_;

  // Whether this viewpoint is in collision with the environment
  bool in_collision_;
  // Whether this viewpoint has been in the line of sight of the robot
  // in_line_of_sight_指一个viewpoint是否曾经被看（ray trace）到过。
  // 大概的思想是如果一个viewpoint在几帧之前被认为有collision但却能在当前帧被ray trace到，这个viewpoint就不被认为是有collision。
  bool in_line_of_sight_;
  // Whether this viewpoint and the robot’s current location are within the same connected component. It must be true to
  // have a collision-free path planned from the current robot position to this viewpoint.
  // 这个视点和机器人的当前位置是否在同一个连通分量内。 从当前机器人位置到该视点必须规划一条无碰撞路径。
  bool connected_;
  // Whether this viewpoint has been visited by the robot. If true, its coverage area will not be updated and it will
  // not be selected in the sampling process.
  // 机器人是否访问过该视点。 如果为 true，则其覆盖区域将不会更新，并且不会在采样过程中被选中。
  bool visited_;
  // Whether this viewpoint is selected to form the path.
  // 是否选择此视点来形成路径。
  bool selected_;
  // Whether this viewpoint is a candidate to be selected to form the path.
  // 此视点是否是要选择以形成路径的候选者。
  bool is_candidate_;
  // Whether this viewpoint has a height set from terrain analysis
  // 此视点是否具有地形分析的高度集
  bool has_terrain_height_;
  // Whether this viewpoint is in an EXPLORING cell
  // 此视点是否在 EXPLORING 单元格中
  bool in_exploring_cell_;
  // The index of the cell this viewpoint is in.
  // 此视点所在的单元格的索引。
  int cell_ind_;
  // The index of this viewpoint among all the candidate viewpoints.
  // 该视点在所有候选视点中的索引。
  int collision_frame_count_;
  // For debug
  double terrain_height_;
  bool has_terrain_neighbor_;
  // Whether the viewpoint is in line of sight in the current frame
  // in_current_frame_line_of_sight_指一个viewpoint是否能在当前帧被看（ray trace）到。
  // 视点是否在当前帧的视线内
  bool in_current_frame_line_of_sight_;
  // Indices of the covered points
  // 覆盖点的索引
  std::vector<int> covered_point_list_;
  // Indices of the covered frontier points
  // 覆盖边界点的索引
  std::vector<int> covered_frontier_point_list_;
};
}  // namespace viewpoint_ns
