/**
 * @file lidar_model.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the sensor model of a LiDAR
 * @version 0.1
 * @date 2019-09-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <string>
#include <vector>
#include <cmath>
// ROS
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utils/misc_utils.h>

namespace lidar_model_ns
{
class LiDARModel
{
public:
  static double pointcloud_resolution_; //点云分辨率
  explicit LiDARModel(double px = 0.0, double py = 0.0, double pz = 0.0, double rw = 1.0, double rx = 0.0,
                      double ry = 0.0, double rz = 0.0);
  explicit LiDARModel(const geometry_msgs::Pose& pose);
  ~LiDARModel() = default;

  /**
   * @brief
   * 更新雷达覆盖
   * @tparam PointType
   * @param point
   */
  template <class PointType>
  void UpdateCoverage(const PointType& point)
  {
    double distance_to_point = misc_utils_ns::PointXYZDist<PointType, geometry_msgs::Point>(point, pose_.position);

    if (isZero(distance_to_point))
      return;

    double dx = point.x - pose_.position.x;
    double dy = point.y - pose_.position.y;
    double dz = point.z - pose_.position.z;

    int horizontal_angle = GetHorizontalAngle(dx, dy);
    int vertical_angle = GetVerticalAngle(dz, distance_to_point);

    int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
    int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

    for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; n++)
    {
      int column_index = horizontal_angle + n;
      if (!ColumnIndexInRange(column_index))
        continue;
      for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; m++)
      {
        int row_index = vertical_angle + m;
        if (!RowIndexInRange(row_index))
          continue;
        int ind = sub2ind(row_index, column_index);
        double previous_distance_to_point = covered_voxel_[ind];
        if (isZero(previous_distance_to_point) || distance_to_point < previous_distance_to_point || reset_[ind])
        {
          covered_voxel_[ind] = distance_to_point;
          reset_[ind] = false;
        }
      }
    }
  }

  /**
   * @brief
   * TODO
   * @tparam PointType
   * @param point
   * @param occlusion_threshold
   * @return true
   * @return false
   */
  template <class PointType>
  bool CheckVisibility(const PointType& point, double occlusion_threshold) const
  {
    double distance_to_point = misc_utils_ns::PointXYZDist<PointType, geometry_msgs::Point>(point, pose_.position);

    if (isZero(distance_to_point))
      return false;

    double dx = point.x - pose_.position.x;
    double dy = point.y - pose_.position.y;
    double dz = point.z - pose_.position.z;

    int horizontal_angle = GetHorizontalAngle(dx, dy);
    int vertical_angle = GetVerticalAngle(dz, distance_to_point);

    int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
    int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

    for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; n++)
    {
      int column_index = horizontal_angle + n;
      if (!ColumnIndexInRange(column_index))
        continue;
      for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; m++)
      {
        int row_index = vertical_angle + m;
        if (!RowIndexInRange(row_index))
          continue;
        int ind = sub2ind(row_index, column_index);
        float previous_distance_to_point = covered_voxel_[ind];
        if ((!isZero(previous_distance_to_point) &&
             distance_to_point < previous_distance_to_point + occlusion_threshold && !reset_[ind]) ||
            reset_[ind])
        {
          return true;
        }
      }
    }
    return false;
  }
  /**
   * @brief
   * TODO
   */
  void ResetCoverage();
  /**
   * @brief Get the Visualization Cloud object
   * TODO
   * @param visualization_cloud
   * @param resol
   * @param max_range
   */
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& visualization_cloud, double resol = 0.2,
                             double max_range = 25.0) const;

  static void setCloudDWZResol(double cloud_dwz_resol)
  {
    pointcloud_resolution_ = cloud_dwz_resol * kCloudInflateRatio;
  }

  //设置激光雷达的位姿
  void setPose(const geometry_msgs::Pose& pose)
  {
    pose_ = pose;
  }

  //获取激光雷达的位姿
  geometry_msgs::Pose getPose()
  {
    return pose_;
  }

  //设置激光雷达的位置
  void setPosition(const geometry_msgs::Point& position)
  {
    pose_.position = position;
  }
  //获取激光雷达的位置
  geometry_msgs::Point getPosition() const
  {
    return pose_.position;
  }
  //设置激光雷达的高度
  void SetHeight(double height)
  {
    pose_.position.z = height;
  }

private:
  /**
   * @brief convert subscripts to linear indices
   *        将下标转换为线性索引
   * @param row_index row index
   * @param column_index column index
   * @return int linear index
   */
  int sub2ind(int row_index, int column_index) const;
  /**
   * @brief convert linear indices to subscripts
   *        将线性索引转换为下标
   * @param ind linear index
   * @param row_index row index
   * @param column_index column index
   */
  void ind2sub(int ind, int& row_index, int& column_index) const;
  /**
   * @brief whether a number is close to zero
   *        一个数字是否接近于零
   * @param x input number
   * @return true
   * @return false
   */
  bool isZero(double x) const
  {
    return std::abs(x) < kEpsilon;
  }

  /**
   * @brief 得到机器人位置和给定点之间的水平角度
   * TODO
   * @param dx
   * @param dy
   * @return int
   */
  inline int GetHorizontalAngle(double dx, double dy) const
  {
    double horizontal_angle = (misc_utils_ns::ApproxAtan2(dy, dx) * kToDegreeConst + 180) / kHorizontalResolution;
    return static_cast<int>(round(horizontal_angle));
  }
  /**
   * @brief 得到机器人位置和给定点之间的垂直距离
   * TODO
   * @param dz
   * @param distance_to_point
   * @return int
   */
  inline int GetVerticalAngle(double dz, double distance_to_point) const
  {
    double vertical_angle =
        (acos(dz / distance_to_point) * kToDegreeConst + kVerticalAngleOffset) / kVerticalResolution;
    return static_cast<int>(round(vertical_angle));
  }
  /**
   * @brief Get the Horizontal Neighbor Num object
   *        获取水平邻居数对象
   * @param distance_to_point
   * @return int
   */
  inline int GetHorizontalNeighborNum(double distance_to_point) const
  {
    return static_cast<int>(ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kHorizontalResolution)) /
           2;
  }
  /**
   * @brief Get the Vertical Neighbor Num object
   *        获取垂直邻居数对象
   * @param distance_to_point
   * @return int
   */
  inline int GetVerticalNeighborNum(double distance_to_point) const
  {
    return static_cast<int>(ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kVerticalResolution)) /
           2;
  }
  //行索引是否在范围内
  inline bool RowIndexInRange(int row_index) const
  {
    return row_index >= 0 && row_index < kVerticalVoxelSize;
  }
  //列索引是否在范围内
  inline bool ColumnIndexInRange(int column_index) const
  {
    return column_index >= 0 && column_index < kHorizontalVoxelSize;
  }

  // Constant converting radian to degree
  static const double kToDegreeConst;
  // Constant converting degree to radian
  static const double kToRadianConst;
  // 检查数字是否接近于零的阈值
  static const double kEpsilon;
  // 膨胀点云的比率
  static const double kCloudInflateRatio;
  // 以度为单位的水平视野
  static const int kHorizontalFOV = 360;
  // 以度为单位的垂直视野
  static const int kVerticalFOV = 24;
  // 水平分辨率
  static const int kHorizontalResolution = 2;
  // 垂直分辨率
  static const int kVerticalResolution = 2;
  // 体素网格的水平维度
  static const int kHorizontalVoxelSize = kHorizontalFOV / kHorizontalResolution;
  // 体素网格的垂直维度
  static const int kVerticalVoxelSize = kVerticalFOV / kVerticalResolution;
  // 垂直角度偏移，例如，角度 [75, 105] -> 索引 [0, 30]
  static const int kVerticalAngleOffset = -(90 - kVerticalFOV / 2);
  // 射线从由水平角和垂直角确定的方向所能到达的距离
  std::array<float, kHorizontalVoxelSize * kVerticalVoxelSize> covered_voxel_;
  // 体素是否重置
  std::array<bool, kHorizontalVoxelSize * kVerticalVoxelSize> reset_;
  // 激光雷达的姿态
  geometry_msgs::Pose pose_;
};
}  // namespace lidar_model_ns