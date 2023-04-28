//
// Created by caochao on 06/03/20.
//
#pragma once

// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <utils/misc_utils.h>

namespace pointcloud_utils_ns
{
class VerticalSurfaceExtractor;
template <typename PCLPointType>
class PointCloudDownsizer;
template <typename PCLPointType>
struct PCLCloud;
}  // namespace pointcloud_utils_ns

class pointcloud_utils_ns::VerticalSurfaceExtractor
{
private:
  double kRadiusThreshold;
  double kZDiffMax;
  double kZDiffMin;
  int kNeighborThreshold;
  pcl::PointCloud<pcl::PointXYZI>::Ptr extractor_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr extractor_kdtree_;

public:
  explicit VerticalSurfaceExtractor();
  ~VerticalSurfaceExtractor() = default;
  void SetRadiusThreshold(double radius_threshold)
  {
    kRadiusThreshold = radius_threshold;
  }
  void SetZDiffMax(double z_diff_max)
  {
    kZDiffMax = z_diff_max;
  }
  void SetZDiffMin(double z_diff_min)
  {
    kZDiffMin = z_diff_min;
  }
  void SetNeighborThreshold(int neighbor_threshold)
  {
    kNeighborThreshold = neighbor_threshold;
  }

  /**
   * 函数功能：
   * ExtractVerticalSurface这个函数是希望把墙上的点都提取出来，把地上和天花板上的点扔掉。
   * 主要的想法是看每个点邻域内其他点的分布。如果一个点的邻域内其他点都和它有一定的高度差，那这个点很有可能就是墙上的点（vertical surface）。 
   * 具体实现上用了一个操作点云的小技巧。先把点云的高度（z）存在PointXYZI的intensity里面，然后把z设成0，就得到了一个在xy平面上的扁平化的点云。
   * 然后对每一个点用kdtree搜索邻域，如果邻域内点的intensity，也就是z，存在一定高度差，那么它就是墙上的点。
   * 值得注意的是，如果这个高度差太大，那它可能是天花板或者地上的点。因为把点云在xy平面上压扁后天花板上的点和地上的点混合在了一起。
  */
  template <class PCLPointType>
  void ExtractVerticalSurface(typename pcl::PointCloud<PCLPointType>::Ptr& cloud, double z_max = DBL_MAX,
                              double z_min = -DBL_MAX)
  {
    if (cloud->points.empty())
    {
      return;
    }
    pcl::copyPointCloud(*cloud, *extractor_cloud_);
    for (auto& point : extractor_cloud_->points)
    {
      point.intensity = point.z;
      point.z = 0.0;
    }
    extractor_kdtree_->setInputCloud(extractor_cloud_);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_sqdist;
    for (int i = 0; i < extractor_cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = extractor_cloud_->points[i];
      if (point.intensity > z_max || point.intensity < z_min)
        continue;
      extractor_kdtree_->radiusSearch(point, kRadiusThreshold, neighbor_indices, neighbor_sqdist);
      bool is_vertical = false;
      int neighbor_count = 0;
      for (const auto& idx : neighbor_indices)
      {
        double z_diff = std::abs(point.intensity - extractor_cloud_->points[idx].intensity);
        if (z_diff > kZDiffMin && z_diff < kZDiffMax)
        {
          neighbor_count++;
          if (neighbor_count >= kNeighborThreshold)
          {
            is_vertical = true;
            break;
          }
        }
      }
      if (is_vertical)
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<PCLPointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  template <class InputPCLPointType, class OutputPCLPointType>
  void ExtractVerticalSurface(typename pcl::PointCloud<InputPCLPointType>::Ptr& cloud_in,
                              typename pcl::PointCloud<OutputPCLPointType>::Ptr& cloud_out, double z_max = DBL_MAX,
                              double z_min = -DBL_MAX)
  {
    if (cloud_in->points.empty())
    {
      return;
    }
    pcl::copyPointCloud(*cloud_in, *extractor_cloud_);
    for (auto& point : extractor_cloud_->points)
    {
      point.intensity = point.z;
      point.z = 0.0;
    }
    extractor_kdtree_->setInputCloud(extractor_cloud_);
    cloud_out->clear();
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_sqdist;
    for (int i = 0; i < extractor_cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = extractor_cloud_->points[i];
      if (point.intensity > z_max || point.intensity < z_min)
        continue;
      extractor_kdtree_->radiusSearch(point, kRadiusThreshold, neighbor_indices, neighbor_sqdist);
      bool is_vertical = false;
      int neighbor_count = 0;
      for (const auto& idx : neighbor_indices)
      {
        double z_diff = std::abs(point.intensity - extractor_cloud_->points[idx].intensity);
        if (z_diff > kZDiffMin && z_diff < kZDiffMax)
        {
          neighbor_count++;
          if (neighbor_count >= kNeighborThreshold)
          {
            is_vertical = true;
            break;
          }
        }
      }
      if (is_vertical)
      {
        OutputPCLPointType point_out;
        point_out.x = cloud_in->points[i].x;
        point_out.y = cloud_in->points[i].y;
        point_out.z = cloud_in->points[i].z;
        cloud_out->points.push_back(point_out);
      }
    }
  }
};

template <typename PCLPointType>
class pointcloud_utils_ns::PointCloudDownsizer
{
private:
  pcl::VoxelGrid<PCLPointType> pointcloud_downsize_filter_;

public:
  explicit PointCloudDownsizer()
  {
  }
  ~PointCloudDownsizer() = default;
  /**
   * 函数功能：对点云降采样
   * 参数：
   *  cloud: 即是要滤波的点云，也是滤波后的点云
   *  leaf_size_x, leaf_size_y, leaf_size_z: 滤波的体素的大小
  */
  void Downsize(typename pcl::PointCloud<PCLPointType>::Ptr& cloud, double leaf_size_x, double leaf_size_y,
                double leaf_size_z)
  {
    if (cloud->points.empty())
    {
      return;
    }
    pointcloud_downsize_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    pointcloud_downsize_filter_.setInputCloud(cloud);
    pointcloud_downsize_filter_.filter(*cloud);
  }
};

template <typename PCLPointType>
struct pointcloud_utils_ns::PCLCloud
{
  std::string pub_cloud_topic_; //指定用于发布点云数据的ROS主题名称
  std::string frame_id_; // 指定点云数据的坐标系
  typename pcl::PointCloud<PCLPointType>::Ptr cloud_; //指向类型为PCLPointType的点云对象的共享指针
  ros::Publisher cloud_pub_; //ROS话题发布者对象

  // 两个构造函数都创建了一个新的PCL点云对象，并初始化了用于发布点云数据的ROS发布者对象
  PCLCloud(ros::NodeHandle* nh, std::string pub_cloud_topic, std::string frame_id)
    : pub_cloud_topic_(pub_cloud_topic), frame_id_(frame_id)
  {
    cloud_ = typename pcl::PointCloud<PCLPointType>::Ptr(new pcl::PointCloud<PCLPointType>);
    cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(pub_cloud_topic_, 2);
  }
  //下面这个构造函数比较常用
  PCLCloud(ros::NodeHandle& nh, std::string pub_cloud_topic, std::string frame_id)
    : pub_cloud_topic_(pub_cloud_topic), frame_id_(frame_id)
  {
    cloud_ = typename pcl::PointCloud<PCLPointType>::Ptr(new pcl::PointCloud<PCLPointType>);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_cloud_topic_, 2);
  }
  ~PCLCloud() = default;

  // 将点云数据发布到指定的ROS主题
  void Publish()
  {
    misc_utils_ns::PublishCloud<pcl::PointCloud<PCLPointType>>(cloud_pub_, *cloud_, frame_id_);
  }
  // 定义指向类型为PCLPointType的点云对象的共享指针
  typedef std::shared_ptr<PCLCloud<PCLPointType>> Ptr;
};
