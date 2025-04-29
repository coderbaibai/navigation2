#pragma once
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_voxel_grid/voxel_grid.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "nav2_util/topic_thread_pool.hpp"
#include <nav2_msgs/msg/voxel_grid.hpp>

namespace nav2_costmap_2d
{
    class LayeredCostmap;

    struct VoxelDataInfo
    {
        rclcpp::SubscriptionBase::SharedPtr sub; // 数据订阅器
        rclcpp::Time last_process_time = rclcpp::Clock().now(); // 上一次处理的时间点
        geometry_msgs::msg::TransformStamped transform; // 从 "sensor_frame" 到 "base_link"
        geometry_msgs::msg::Point sensor_point; // base_link坐标系的坐标
        std::mutex mtx; // 锁

        std::string topic;
        std::string frame_id;
        std::string data_type; // PointCloud2 LaserScan
        double process_interval = 0.160; // 处理间隔
        double obstacle_range = 1.5; // 障碍物标记距离
        double raytrace_range = 1.55; // 障碍物消除距离
        bool marking = true; // 是否标记
        bool clearing = true; // 是否消除
        double max_obstacle_height = 0.65; // 最大障碍物高度
        double max_dist_min_obs_height = 0.08; // 最远处障碍物的最低高度

        bool cliff_detect_enable = false; // 是否开启悬崖检测
        int cliff_cells_per_frame = 50; // 每一帧检测到的悬崖（格子/pixel）数
        int cliff_points_per_frame = 1000; // 每一帧检测到的悬崖（格子/pixel）数
        // int nbrhd_size = 7; // 判断悬崖的窗口长度 pixel
        // int nbrhd_cliff_cnt = 30; // 窗口中的悬崖数
        // int nbrhd_points_cnt = 200; // 窗口中的点云数
        double cliff_max_range = 1.3; // 检测悬崖的距离 m
        double cliff_detect_depth_min = -0.1; // 检测悬崖的深度 m
        double cliff_detect_depth_max = -0.6; // 检测悬崖的深度 m
        int cliff_inflation_radius = 0; // 悬崖膨胀的半径 pixel
        bool save_data = false;
        std::atomic<int> proc_seq{0};
        std::atomic<int> recv_seq{0};
    };

    class XlVoxelLayer : public nav2_costmap_2d::Layer
    {
    public:
        XlVoxelLayer() : voxel_grid(0, 0, 0), thread_pool() {}
        ~XlVoxelLayer() {}

        void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y);
        virtual void updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual void matchSize();
        virtual void reset();
        virtual bool isClearable();

        virtual void activate() {}
        virtual void deactivate() {}
        virtual bool isCurrent() const { return current_; }

    private:
        void PointCloud2Cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::shared_ptr<VoxelDataInfo> &info);
        void LaserScanCb(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg, const std::shared_ptr<VoxelDataInfo> &info);
        void ProcessPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, std::shared_ptr<VoxelDataInfo> info);

        void RaytraceFreespace(geometry_msgs::msg::Point &origin,  sensor_msgs::msg::PointCloud2 &cloud, std::shared_ptr<VoxelDataInfo> &info);
        bool WorldToMap3DFloat(double wx, double wy, double wz, double &mx, double &my, double &mz);
        bool WorldToMap3D(double wx, double wy, double wz, unsigned int &mx, unsigned int &my, unsigned int &mz);

        void resetCostmapCB(const std_msgs::msg::String::ConstSharedPtr &msg);
        void ObsDetectionHeight(const std_msgs::msg::Int32::ConstSharedPtr &msg);

        void updateOrigin(double new_origin_x, double new_origin_y);

        double z_resolution = 0.05; // 竖直方向分辨率
        double origin_z = 0.03; // 竖直方向原点高度
        double random_noise = 0.05; // 随机噪声
        int unknown_threshold = 15; // 未知阈值
        int mark_threshold = 0; // 标记阈值
        int z_voxels = 16; // 竖直方向的格子数
        std::vector<std::shared_ptr<VoxelDataInfo>> data_set; // 数据集合
        nav2_voxel_grid::VoxelGrid voxel_grid; // 体素格子
        nav2_util::TopicThreadPool thread_pool; // 线程池

        std::string global_frame; // 全局frame
        std::string robot_base_frame; // 机器人frame
        nav2_costmap_2d::Costmap2D inner_costmap; // 内部costmap
        laser_geometry::LaserProjection projector; // 投影器，scan 转 点云

        double resolution = 0; // 分辨率
        uint width = 0; // 宽
        uint height = 0; // 高
        double origin_x = 0; // 原点x
        double origin_y = 0; // 原点y

        rclcpp::SubscriptionBase::SharedPtr sub_reset;
        rclcpp::SubscriptionBase::SharedPtr sub_detection_height;
        std::atomic_bool reset_costmap_{true};

        std::atomic_bool topic_cliff_detect{false}; // 悬崖检测，topic设置的
        void EnableCliffDetectCB(const std_msgs::msg::Bool::ConstSharedPtr &msg);
        rclcpp::SubscriptionBase::SharedPtr sub_enable_cliff_detect; // 悬崖可用订阅

        std::string log_name;
        bool is_voxel_grid_pub = false;
        rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_publisher;
    };

}