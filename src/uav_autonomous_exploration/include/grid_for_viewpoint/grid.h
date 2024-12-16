/**
 * @file grid_for_viewpoint.h
 * @author 
 * @brief Class that implements a 3D grid
 * @version 0.1
 * @date 2024-01-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <grid/grid.h>
namespace grid_for_viewpoint_ns
{
  template <typename _T,typename _for_little>
  class grid_for_viewpoint : public grid_ns::Grid<_T>
  {
  public://构造函数这里要传大正方形的，也要传小正方形的
    explicit grid_for_viewpoint(const Eigen::Vector3i& size, _T init_value,const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                const Eigen::Vector3d& resolution = Eigen::Vector3d(1, 1, 1), int dimension = 3
                // const Eigen::Vector3i& size_for_l, _for_big init_value_for_l, const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                  // const Eigen::Vector3d& resolution_for_l = Eigen::Vector3d(1, 1, 1), int dimension_for_l = 3
                  )
      // : Grid<_T>(size, init_value, origin, resolution, dimension)
    {
      origin_ = origin;
      size_ = size;
      resolution_ = resolution;
      dimension_ = dimension;

      for (int i = 0; i < dimension_; i++)
      {
        resolution_inv_(i) = 1.0 / resolution_(i);
      }
      cell_number_ = size_.x() * size_.y() * size_.z();
      cells_.resize(cell_number_);
      //48*48*10.8  1.2*1.2*1.2（分辨率）   5*5*3（那么多个大方格）   8*8*3（每个小格子那么多个点）
      Eigen::Vector3d size_for_l(8,8,3);
      Eigen::Vector3d little_cell_resolution(1.2, 1.2, 1.2);
      for (int i = 0; i < cells_.size(); i++)
      {
        cells_[i].reset(new grid_ns::Grid<_for_little>(size_for_l,Sub2Pos_ld_point(ind2sub_(i)),little_cell_resolution,3));
        subs_.push_back(ind2sub_(i));
      }

    }
    
    // Eigen::Vector3d get_cells_origin_from_ind(int ind)//ind转sub，再转pos（这个函数只是在初始化赋值的时候用一次而已）
    // {
    //   Eigen::Vector3d cell_origin_;
    //   ind2sub_(ind);
    //   return  
    // }
    Eigen::Vector3d Sub2Pos_ld_point(const Eigen::Vector3i& sub) const
  {
    Eigen::Vector3d pos(0, 0, 0);
    for (int i = 0; i < dimension_; i++)
    {
      pos(i) = origin_(i) + sub(i) * resolution_(i) ;
    }
    return pos;
  }

  private:


    // 如果有需要，你还可以添加其他成员函数或者重载其他函数
  };
}  // namespace grid_ns