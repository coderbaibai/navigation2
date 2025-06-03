/*
 * Copyright (c) 2008, 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Conor McGann
 */

#ifndef NAV2_COSTMAP_2D__OBSERVATION_HPP_
#define NAV2_COSTMAP_2D__OBSERVATION_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace nav2_costmap_2d
{

/**
 * @brief Stores an observation in terms of a point cloud and the origin of the source
 * @note Tried to make members and constructor arguments const but the compiler would not accept the default
 * assignment operator for vector insertion!
 */

class Observation
{
public:
  /**
   * @brief  Creates an empty observation
   */
  Observation()
  : cloud_(new sensor_msgs::msg::PointCloud2()), obstacle_max_range_(0.0), obstacle_min_range_(0.0),
    raytrace_max_range_(0.0),
    raytrace_min_range_(0.0)
  {
  }
  /**
   * @brief A destructor
   */
  virtual ~Observation()
  {
    delete cloud_;
  }

  /**
   * @brief  Copy assignment operator
   * @param obs The observation to copy
   */
  Observation & operator=(const Observation & obs)
  {
    origin_ = obs.origin_;
    cloud_ = new sensor_msgs::msg::PointCloud2(*(obs.cloud_));
    obstacle_max_range_ = obs.obstacle_max_range_;
    obstacle_min_range_ = obs.obstacle_min_range_;
    raytrace_max_range_ = obs.raytrace_max_range_;
    raytrace_min_range_ = obs.raytrace_min_range_;
    enable_constraints = obs.enable_constraints;
    constraint_polygon_ = obs.constraint_polygon_;
    transform_stamped_ = obs.transform_stamped_;

    return *this;
  }

  /**
   * @brief  Creates an observation from an origin point and a point cloud
   * @param origin The origin point of the observation
   * @param cloud The point cloud of the observation
   * @param obstacle_max_range The range out to which an observation should be able to insert obstacles
   * @param obstacle_min_range The range from which an observation should be able to insert obstacles
   * @param raytrace_max_range The range out to which an observation should be able to clear via raytracing
   * @param raytrace_min_range The range from which an observation should be able to clear via raytracing
   */
  Observation(
    geometry_msgs::msg::Point & origin, const sensor_msgs::msg::PointCloud2 & cloud,
    double obstacle_max_range, double obstacle_min_range, double raytrace_max_range,
    double raytrace_min_range)
  : origin_(origin), cloud_(new sensor_msgs::msg::PointCloud2(cloud)),
    obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
    raytrace_max_range_(raytrace_max_range), raytrace_min_range_(
      raytrace_min_range)
  {
  }

  /**
   * @brief  Copy constructor
   * @param obs The observation to copy
   */
  Observation(const Observation & obs)
  : origin_(obs.origin_), cloud_(new sensor_msgs::msg::PointCloud2(*(obs.cloud_))),
    obstacle_max_range_(obs.obstacle_max_range_), obstacle_min_range_(obs.obstacle_min_range_),
    raytrace_max_range_(obs.raytrace_max_range_),
    raytrace_min_range_(obs.raytrace_min_range_),
    enable_constraints(obs.enable_constraints),
    constraint_polygon_(obs.constraint_polygon_),
    transform_stamped_(obs.transform_stamped_)
  {
  }

  /**
   * @brief  Creates an observation from a point cloud
   * @param cloud The point cloud of the observation
   * @param obstacle_max_range The range out to which an observation should be able to insert obstacles
   * @param obstacle_min_range The range from which an observation should be able to insert obstacles
   */
  Observation(
    const sensor_msgs::msg::PointCloud2 & cloud, double obstacle_max_range,
    double obstacle_min_range)
  : cloud_(new sensor_msgs::msg::PointCloud2(cloud)), obstacle_max_range_(obstacle_max_range),
    obstacle_min_range_(obstacle_min_range),
    raytrace_max_range_(0.0), raytrace_min_range_(0.0)
  {
  }

  // 计算向量 (p1 -> p2) 与 (p1 -> p) 的叉积
  static inline double cross(const geometry_msgs::msg::Point32& p1,
    const geometry_msgs::msg::Point32& p2,
    const geometry_msgs::msg::Point32& p) 
  {
    double dy1 = p1.y - p.y;
    double dz1 = p1.z - p.z;
    double dy2 = p2.y - p.y;
    double dz2 = p2.z - p.z;
    return dy1 * dz2 - dy2 * dz1;
  }

  static inline bool isPointInConvexHull(const std::vector<geometry_msgs::msg::Point32> & hull, const geometry_msgs::msg::Point32& p) {
    int n = hull.size();
    if (n < 3) return false; // 凸包至少需要3个点

    bool isFirst = true;
    bool curDirection = true; // 用于判断点在凸包的哪一侧

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        double crossProduct = cross(hull[i], hull[j], p);
        if(isFirst){
          curDirection = (crossProduct > 0); // 第一次判断方向
          isFirst = false;
          continue;
        }
        if(curDirection != (crossProduct > 0)){
          return false; // 如果方向不一致，点在凸包外
        }
    }
    return true; // 所有叉积 >= 0，点在凸包内
  }

  bool isPointInPolygon(double x,double y, double z, const std::string& global_frame) const{
    if(!enable_constraints) {
      return true;
    }
    if(constraint_polygon_.polygon.points.size() < 3) {
      return true; // no constraints
    }
    geometry_msgs::msg::PointStamped in_point;
    in_point.header.frame_id = global_frame;
    in_point.point.x = x;
    in_point.point.y = y;
    in_point.point.z = z;
    geometry_msgs::msg::PointStamped out_point;
    out_point.header.stamp = transform_stamped_.header.stamp;
    out_point.header.frame_id = transform_stamped_.header.frame_id;
    tf2::doTransform(in_point, out_point, transform_stamped_);

    Eigen::Vector3f p(out_point.point.x, out_point.point.y, out_point.point.z);

    Eigen::Vector3f origin(0, 0, 0);
    Eigen::Vector3f plane_normal(1.0, 0.0, 0.0);
    // 计算从原点到当前点的方向向量
    Eigen::Vector3f direction = p - origin;
    // 计算直线参数t (直线方程: origin + t * direction)
    float t = -(plane_normal.dot(origin) - constraint_polygon_.polygon.points[0].x) / plane_normal.dot(direction);
    // 计算交点
    Eigen::Vector3f intersection = origin + t * direction;

    geometry_msgs::msg::Point32 point;
    point.x = intersection.x();
    point.y = intersection.y();
    point.z = intersection.z();

    return isPointInConvexHull(constraint_polygon_.polygon.points, point);
  }

  geometry_msgs::msg::Point origin_;
  sensor_msgs::msg::PointCloud2 * cloud_;
  double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
  bool enable_constraints = false;
  geometry_msgs::msg::PolygonStamped constraint_polygon_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
};

}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__OBSERVATION_HPP_
