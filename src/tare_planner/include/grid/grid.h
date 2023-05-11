/**
 * @file grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a 3D grid
 * @version 0.1
 * @date 2021-01-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <Eigen/Core>
#include <utils/misc_utils.h>

namespace grid_ns
{
template <typename _T>
class Grid
{
public:
  explicit Grid(const Eigen::Vector3i& size, _T init_value, const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                const Eigen::Vector3d& resolution = Eigen::Vector3d(1, 1, 1), int dimension = 3)
  {
    // MY_ASSERT(size.x() > 0);
    // MY_ASSERT(size.y() > 0);
    // MY_ASSERT(size.z() > 0);

    origin_ = origin;
    size_ = size; //
    resolution_ = resolution;
    dimension_ = dimension;

    for (int i = 0; i < dimension_; i++)
    {
      resolution_inv_(i) = 1.0 / resolution_(i);
    }
    cell_number_ = size_.x() * size_.y() * size_.z();
    for (int i = 0; i < cell_number_; i++)
    {
      cells_.push_back(init_value);
      subs_.push_back(ind2sub_(i));
    }
  }

  virtual ~Grid() = default;

  //返回当前体素网格的数量
  int GetCellNumber() const
  {
    return cell_number_;
  }

  //返回当前体素网格x，y，z的大小
  Eigen::Vector3i GetSize() const
  {
    return size_;
  }

  //返回当前体素网格的起点位置
  Eigen::Vector3d GetOrigin() const
  {
    return origin_;
  }

  //设置当前体素网格的起点位置
  void SetOrigin(const Eigen::Vector3d& origin)
  {
    origin_ = origin;
  }

  //设置当前体素网格的分辨率
  void SetResolution(const Eigen::Vector3d& resolution)
  {
    resolution_ = resolution;
    for (int i = 0; i < dimension_; i++)
    {
      resolution_inv_(i) = 1.0 / resolution(i);
    }
  }

  //返回当前体素网格的分辨率
  Eigen::Vector3d GetResolution() const
  {
    return resolution_;
  }

  //返回当前体素网格的分辨率的倒数
  Eigen::Vector3d GetResolutionInv() const
  {
    return resolution_inv_;
  }

  //返回输入的参数是否在当前的体素网格内
  bool InRange(int x, int y, int z) const
  {
    return InRange(Eigen::Vector3i(x, y, z));
  }

  //返回输入的参数是否在当前的体素网格内
  bool InRange(const Eigen::Vector3i& sub) const
  {
    bool in_range = true;
    for (int i = 0; i < dimension_; i++)
    {
      in_range &= sub(i) >= 0 && sub(i) < size_(i);
    }
    return in_range;
  }

  //返回输入的参数是否在当前的体素网格内
  bool InRange(int ind) const
  {
    return ind >= 0 && ind < cell_number_;
  }

  //将体素网格的序号转换为体素网格中的位置编号
  Eigen::Vector3i Ind2Sub(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    return subs_[ind];
  }

  //将体素网格的位置编号转换为体素网格的序号
  int Sub2Ind(int x, int y, int z) const
  {
    return x + (y * size_.x()) + (z * size_.x() * size_.y());
  }

  //将体素网格的位置编号转换为体素网格的序号
  int Sub2Ind(const Eigen::Vector3i& sub) const
  {
    // MY_ASSERT(InRange(sub));
    return Sub2Ind(sub.x(), sub.y(), sub.z());
  }

  // 根据体素网格的位置编号转换为体素网格的位置
  Eigen::Vector3d Sub2Pos(int x, int y, int z) const
  {
    return Sub2Pos(Eigen::Vector3i(x, y, z));
  }

  // 根据体素网格的位置编号转换为体素网格的位置
  Eigen::Vector3d Sub2Pos(const Eigen::Vector3i& sub) const
  {
    Eigen::Vector3d pos(0, 0, 0);
    for (int i = 0; i < dimension_; i++)
    {
      pos(i) = origin_(i) + sub(i) * resolution_(i) + resolution_(i) / 2;
    }
    return pos;
  }

  //根据体素网格的序号转换为体素网格的位置
  Eigen::Vector3d Ind2Pos(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    return Sub2Pos(Ind2Sub(ind));
  }

  //根据体素网格的位置返回体素网格的位置编号
  Eigen::Vector3i Pos2Sub(double x, double y, double z) const
  {
    return Pos2Sub(Eigen::Vector3d(x, y, z));
  }

  //根据体素网格的位置返回体素网格的位置编号
  Eigen::Vector3i Pos2Sub(const Eigen::Vector3d& pos) const
  {
    Eigen::Vector3i sub(0, 0, 0);
    for (int i = 0; i < dimension_; i++)
    {
      sub(i) = pos(i) - origin_(i) > 0 ? static_cast<int>((pos(i) - origin_(i)) * resolution_inv_(i)) : -1;
    }
    return sub;
  }

  //根据体素网格的位置返回体素网格的序号
  int Pos2Ind(const Eigen::Vector3d& pos) const
  {
    return Sub2Ind(Pos2Sub(pos));
  }

  //获取体素网格内的内容
  _T& GetCell(int x, int y, int z)
  {
    return GetCell(Eigen::Vector3i(x, y, z));
  }

  //获取体素网格内的内容
  _T& GetCell(const Eigen::Vector3i& sub)
  {
    // MY_ASSERT(InRange(sub));
    int index = Sub2Ind(sub);
    return cells_[index];
  }

  //获取体素网格内的内容
  _T& GetCell(int index)
  {
    // MY_ASSERT(InRange(index));
    return cells_[index];
  }

  //获取体素网格内的内容
  _T GetCellValue(int x, int y, int z) const
  {
    int index = Sub2Ind(x, y, z);
    return cells_[index];
  }

  //获取体素网格内的内容
  _T GetCellValue(const Eigen::Vector3i& sub) const
  {
    // MY_ASSERT(InRange(sub));
    return GetCellValue(sub.x(), sub.y(), sub.z());
  }

  //获取体素网格内的内容
  _T GetCellValue(int index) const
  {
    // MY_ASSERT(InRange(index));
    return cells_[index];
  }

  //设置体素网格内的某个单元的值
  void SetCellValue(int x, int y, int z, _T value)
  {
    int index = Sub2Ind(x, y, z);
    cells_[index] = value;
  }

  //设置体素网格内的某个单元的值
  void SetCellValue(const Eigen::Vector3i& sub, _T value)
  {
    // MY_ASSERT(InRange(sub));
    SetCellValue(sub.x(), sub.y(), sub.z(), value);
  }

  //设置体素网格内的某个单元的值
  void SetCellValue(int index, const _T& value)
  {
    // MY_ASSERT(InRange(index));
    cells_[index] = value;
  }

private:
  Eigen::Vector3d origin_; //表示当前体素网格的起点
  Eigen::Vector3i size_; //表示当前体素网格x，y，z的大小
  Eigen::Vector3d resolution_; //表示当前体素网格x，y，z的分辨率
  Eigen::Vector3d resolution_inv_; //表示当前体素网格x，y，z的分辨率的倒数，相当于一个1m体素网格的数
  std::vector<_T> cells_; //每个体素网格的内容
  std::vector<Eigen::Vector3i> subs_; //每个体素网格的序号
  int cell_number_; //表示当前体素网格的个数
  int dimension_; //表示体素网格的维度，默认为三维，dimension_ = 3;

  //将体素网格的序号转换为体素网格中的位置编号
  Eigen::Vector3i ind2sub_(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    Eigen::Vector3i sub;
    sub.z() = ind / (size_.x() * size_.y());
    ind -= (sub.z() * size_.x() * size_.y());
    sub.y() = ind / size_.x();
    sub.x() = ind % size_.x();
    return sub;
  }
};
}  // namespace grid_ns