#include "nav2_costmap_2d/xl_voxel_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>

// static void SavePointCloud(const sensor_msgs::msg::PointCloud2::ConstPtr& cloud_msg, std::string &name) 
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*cloud_msg, *cloud);
//     pcl::io::savePLYFile(name+".ply", *cloud);
// }

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::XlVoxelLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d {
    using namespace std::placeholders;

    void XlVoxelLayer::onInitialize()
    {
        this->log_name = "xl_voxel_layer" + name_;
        // INIT_SDK_LOG(this->log_name, "/home/robot/clean/log/"+this->log_name+".log", false, 1024 * 1024 *20, 5);

        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        enabled_ = true;

        // err_msg.type = "camera_cliff_trigger";
        // err_msg.val = true;

        sub_enable_cliff_detect = node->create_subscription<std_msgs::msg::Bool>("/cliff_detect_controller", rclcpp::QoS(1).transient_local(), std::bind(&XlVoxelLayer::EnableCliffDetectCB, this, std::placeholders::_1));
        // pub_error_warning = node->create_publisher<xl_robot_msg::msg::ErrorsAndWarnings>("/error_hub", rclcpp::QoS(1).transient_local());

        inner_costmap.setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
        auto master = layered_costmap_->getCostmap();
        resolution = master->getResolution();
        width = master->getSizeInCellsX();
        height = master->getSizeInCellsY();
        origin_x = master->getOriginX();
        origin_y = master->getOriginY();
        inner_costmap.resizeMap(width, height, resolution, origin_x, origin_y);

        global_frame = layered_costmap_->getGlobalFrameID();

        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("origin_z", rclcpp::ParameterValue(0.03));
        declareParameter("z_voxels", rclcpp::ParameterValue(16));
        declareParameter("z_resolution", rclcpp::ParameterValue(0.05));
        declareParameter("random_noise", rclcpp::ParameterValue(0.05));
        declareParameter("unknown_threshold", rclcpp::ParameterValue(15));
        declareParameter("mark_threshold", rclcpp::ParameterValue(0));
        declareParameter("robot_base_frame", rclcpp::ParameterValue("base_link"));
        declareParameter("observation_sources", rclcpp::ParameterValue(""));
        declareParameter("mark_threshold", rclcpp::ParameterValue(0));
        declareParameter("publish_voxel_map", rclcpp::ParameterValue(false));

        node->get_parameter(name_ + "." + "enabled", enabled_);
        node->get_parameter(name_ + "." + "origin_z", origin_z);
        node->get_parameter(name_ + "." + "z_voxels", z_voxels);
        node->get_parameter(name_ + "." + "z_resolution", z_resolution);
        node->get_parameter(name_ + "." + "random_noise", random_noise);
        node->get_parameter(name_ + "." + "unknown_threshold", unknown_threshold);
        node->get_parameter(name_ + "." + "mark_threshold", mark_threshold);
        node->get_parameter(name_ + "." + "robot_base_frame", robot_base_frame);

        node->get_parameter(name_ + "." + "mark_threshold", mark_threshold);
        node->get_parameter(name_ + "." + "publish_voxel_map", is_voxel_grid_pub);
        if(is_voxel_grid_pub){
            voxel_publisher = node->create_publisher<nav2_msgs::msg::VoxelGrid>("/voxel_grid", rclcpp::QoS(1).transient_local());
        }

        voxel_grid.resize(width, height, z_voxels);

        std::string topic_sources;
        node->get_parameter(name_ + "." + "observation_sources", topic_sources);
        std::stringstream ss(topic_sources);
        std::string source;
        while (ss >> source)
        {
            declareParameter(source + "." + "topic", rclcpp::ParameterValue(""));
            declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(""));
            declareParameter(source + "." + "data_type", rclcpp::ParameterValue(""));
            declareParameter(source + "." + "obstacle_range", rclcpp::ParameterValue(2.0));
            declareParameter(source + "." + "raytrace_range", rclcpp::ParameterValue(2.05));
            declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
            declareParameter(source + "." + "clearing", rclcpp::ParameterValue(true));
            declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(0.65));
            declareParameter(source + "." + "max_dist_min_obs_height", rclcpp::ParameterValue(0.06));
            declareParameter(source + "." + "update_rate", rclcpp::ParameterValue(8));
            declareParameter(source + "." + "cliff_detect_enable", rclcpp::ParameterValue(false));
            declareParameter(source + "." + "cliff_cells_per_frame", rclcpp::ParameterValue(50));
            declareParameter(source + "." + "cliff_points_per_frame", rclcpp::ParameterValue(1000));
            declareParameter(source + "." + "cliff_max_range", rclcpp::ParameterValue(1.3));
            declareParameter(source + "." + "cliff_detect_depth_min", rclcpp::ParameterValue(-0.05));
            declareParameter(source + "." + "cliff_detect_depth_max", rclcpp::ParameterValue(-0.6));
            declareParameter(source + "." + "cliff_inflation_radius", rclcpp::ParameterValue(0));
            declareParameter(source + "." + "save_data", rclcpp::ParameterValue(false));

            auto info = std::make_shared<VoxelDataInfo>();
            node->get_parameter(name_ + "." + source + "." + "topic", info->topic);
            node->get_parameter(name_ + "." + source + "." + "sensor_frame", info->frame_id);
            node->get_parameter(name_ + "." + source + "." + "data_type", info->data_type);
            node->get_parameter(name_ + "." + source + "." + "obstacle_range", info->obstacle_range);
            node->get_parameter(name_ + "." + source + "." + "raytrace_range", info->raytrace_range);
            node->get_parameter(name_ + "." + source + "." + "marking", info->marking);
            node->get_parameter(name_ + "." + source + "." + "clearing", info->clearing);
            node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", info->max_obstacle_height);
            node->get_parameter(name_ + "." + source + "." + "max_dist_min_obs_height", info->max_dist_min_obs_height);
            if (info->max_dist_min_obs_height <= origin_z)
            {
                info->max_dist_min_obs_height = origin_z + 0.01;
            }

            int update_rate = 8;
            node->get_parameter(name_ + "." + source + "." + "update_rate", update_rate);
            if (update_rate >= 1)
            {
                info->process_interval = 1.0/update_rate;
            }

            // initialize thread_pool
            try{
                this->thread_pool.add(info->topic);
            }catch(std::runtime_error& error){
                // SDK_LOG_E(this->log_name,error.what());
            }

            if (info->data_type == "PointCloud2")
            {
                // SDK_LOG_D(this->log_name,"subscribe:%s",info->topic.c_str());
                info->sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    info->topic, 1,
                    [this, info](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                        PointCloud2Cb(msg, info);
                    });
            }
            else if (info->data_type == "LaserScan")
            {
                info->sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                    info->topic, 1,
                    [this, info](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                        LaserScanCb(msg, info);
                    });
            }
            else
            {
                // LOG_W("unsupport type:%s", info->data_type.c_str());
                continue;
            }

            node->get_parameter(name_ + "." + source + "." + "cliff_detect_enable", info->cliff_detect_enable);
            node->get_parameter(name_ + "." + source + "." + "cliff_cells_per_frame", info->cliff_cells_per_frame);
            node->get_parameter(name_ + "." + source + "." + "cliff_points_per_frame", info->cliff_points_per_frame);
            node->get_parameter(name_ + "." + source + "." + "cliff_max_range", info->cliff_max_range);
            node->get_parameter(name_ + "." + source + "." + "cliff_detect_depth_min", info->cliff_detect_depth_min);
            node->get_parameter(name_ + "." + source + "." + "cliff_detect_depth_max", info->cliff_detect_depth_max);
            node->get_parameter(name_ + "." + source + "." + "cliff_inflation_radius", info->cliff_inflation_radius);
            node->get_parameter(name_ + "." + source + "." + "save_data", info->save_data);
            data_set.push_back(info);
        }
        this->thread_pool.start();
        sub_reset = node->create_subscription<std_msgs::msg::String>("/reset_costmap", 1, std::bind(&XlVoxelLayer::resetCostmapCB, this, _1));
        sub_detection_height = node->create_subscription<std_msgs::msg::Int32>("/obs_detection_height", 1, std::bind(&XlVoxelLayer::ObsDetectionHeight, this, _1));
    }

    void XlVoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        (void)robot_yaw;
        std::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));
        if (layered_costmap_->isRolling())
        {
            updateOrigin(robot_x - inner_costmap.getSizeInMetersX() / 2, robot_y - inner_costmap.getSizeInMetersY() / 2);
        }

        auto wx0 = std::max(origin_x+resolution, robot_x-2.0);
        auto wy0 = std::max(origin_y+resolution, robot_y-2.0);
        auto wx1 = std::min(origin_x+inner_costmap.getSizeInMetersX()-resolution, robot_x+2.0);
        auto wy1 = std::min(origin_y+inner_costmap.getSizeInMetersY()-resolution, robot_y+2.0);

        if (reset_costmap_)
        {
            wx0 = origin_x + resolution;
            wy0 = origin_y + resolution;
            wx1 = origin_x + inner_costmap.getSizeInMetersX() - resolution;
            wy1 = origin_y + inner_costmap.getSizeInMetersY() - resolution;
            reset_costmap_ = false;
        }

        *min_x = std::min(*min_x, wx0);
        *min_y = std::min(*min_y, wy0);
        *max_x = std::max(*max_x, wx1);
        *max_y = std::max(*max_y, wy1);
    }

    void XlVoxelLayer::updateCosts(Costmap2D &master, int min_i, int min_j, int max_i, int max_j)
    {
        std::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));
        unsigned char *master_array = master.getCharMap();
        unsigned int span = master.getSizeInCellsX();
        auto inner_charmap = inner_costmap.getCharMap();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = j * span + min_i;
            for (int i = min_i; i < max_i; i++)
            {
                if (inner_charmap[it] == nav2_costmap_2d::NO_INFORMATION)
                {
                    it++;
                    continue;
                }

                unsigned char old_cost = master_array[it];
                if (old_cost == nav2_costmap_2d::NO_INFORMATION || old_cost < inner_charmap[it])
                {
                    master_array[it] = inner_charmap[it];
                }
                it++;
            }
        }
    }

    void XlVoxelLayer::matchSize()
    {
        auto master = layered_costmap_->getCostmap();
        std::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));
        inner_costmap.resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
        voxel_grid.resize(master->getSizeInCellsX(), master->getSizeInCellsY(), z_voxels);
        resolution = master->getResolution();
        width = master->getSizeInCellsX();
        height = master->getSizeInCellsY();
        origin_x = master->getOriginX();
        origin_y = master->getOriginY();
    }

    void XlVoxelLayer::reset()
    {
        std::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));
        inner_costmap.resetMap(0, 0, inner_costmap.getSizeInCellsX(), inner_costmap.getSizeInCellsY());
        voxel_grid.reset();
        reset_costmap_ = true;
    }

    bool XlVoxelLayer::isClearable()
    {
        return true;
    }

    void XlVoxelLayer::PointCloud2Cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::shared_ptr<VoxelDataInfo> &info)
    {
        // if (!info->mtx.try_lock())
        // {
        //     return;
        // }
        // info->mtx.unlock();

        // auto ts = (rclcpp::Clock().now()-info->last_process_time).seconds();
        // if (ts < info->process_interval)
        // {
        //     return;
        // }

        // thread_pool.enqueue(std::bind(&XlVoxelLayer::ProcessPointCloud, this, _1, _2), msg, info);

        auto ts = (rclcpp::Clock().now()-info->last_process_time).seconds();
        if (ts < info->process_interval)
        {
            return;
        }

        // if (!info->mtx.try_lock())
        // {
        //     return;
        // }
        // info->mtx.unlock();
        info->recv_seq++;
        try{
            thread_pool.enqueue(info->topic,boost::bind(&XlVoxelLayer::ProcessPointCloud, this, _1, _2), msg, info);
        }catch(std::runtime_error& error){
            // SDK_LOG_E(this->log_name,error.what());
        }
    }

    void XlVoxelLayer::LaserScanCb(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg, const std::shared_ptr<VoxelDataInfo> &info)
    {
        // if (!info->mtx.try_lock())
        // {
        //     return;
        // }
        // info->mtx.unlock();

        auto ts = (rclcpp::Clock().now()-info->last_process_time).seconds();
        if (ts < info->process_interval)
        {
            return;
        }

        // 限制数据角度为270，nan/inf数据设置为最大距离
        float epsilon = 0.0001;
        sensor_msgs::msg::LaserScan scan = *msg;
        size_t start_idx = 0;
        size_t end_idx = scan.ranges.size();
        if (scan.angle_min < -2.356196)
        {
            start_idx = (-2.356196-scan.angle_min)/scan.angle_increment+1;
        }
        if (scan.angle_max > 2.356196)
        {
            end_idx = scan.ranges.size()-(scan.angle_max-2.356196)/scan.angle_increment-1;
        }

        for (size_t i = 0; i < start_idx; i++)
        {
            scan.ranges[i] = std::numeric_limits<float>::infinity();
        }
        for (size_t i = end_idx; i < scan.ranges.size(); i++)
        {
            scan.ranges[i] = std::numeric_limits<float>::infinity();
        }
        for (size_t i = start_idx; i < end_idx; i++)
        {
            float range = scan.ranges[i];
            if ((!std::isfinite(range) && range > 0) || fabs(range) <= epsilon)
            {
                scan.ranges[i] = scan.range_max - epsilon;
            }
        }

        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header = msg->header;
        try
        {
            projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, *cloud, *tf_);
        }
        catch (tf2::TransformException &ex)
        {
            projector.projectLaser(scan, *cloud);
        }

        try{
            thread_pool.enqueue(info->topic,boost::bind(&XlVoxelLayer::ProcessPointCloud, this, _1, _2), cloud, info);
        }catch(std::runtime_error& error){
            // SDK_LOG_E(this->log_name,error.what());
        }
    }

    inline double Dist(double x0, double y0, double z0, double x1, double y1, double z1)
    {
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
    }

    void XlVoxelLayer::EnableCliffDetectCB(const std_msgs::msg::Bool::ConstSharedPtr &msg)
    {
        // LOG_I("EnableCliffDetectCB:%d", msg->data);
        topic_cliff_detect = msg->data;
    }

    void XlVoxelLayer::ProcessPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, std::shared_ptr<VoxelDataInfo> info)
    {
        // if (!info->mtx.try_lock())
        // {
        //     return;
        // }
        // StopWatch sw;
        // int tf_ts, clear_ts = 0, make_ts = 0, cliff_ts = 0;
        // int tf_look_ts=0,tf_trans_ts=0;
        info->proc_seq++;
        // ON_SCOPE_EXIT( [&] {
        //     info->mtx.unlock();
        //     SDK_LOG_D(this->log_name, "topic: %s buffer_count: %d size: %d total:%d ms tf: %d ms tf_look_ts: %d tf_trans_ts: %d clear: %d ms, make: %d ms, cliff: %d ms", info->topic.c_str(),info->recv_seq.load() - info->proc_seq.load(), cloud->width*cloud->height,tf_ts+clear_ts+make_ts+cliff_ts ,tf_ts,tf_look_ts,tf_trans_ts, clear_ts, make_ts, cliff_ts);
        //     // LOG_D_THROTTLE(60, "topic: %s size: %d tf: %d ms clear: %d ms, make: %d ms, cliff: %d ms", info->topic.c_str(), cloud->width*cloud->height, tf_ts, clear_ts, make_ts, cliff_ts);
        // });
        // RCLCPP_WARN(logger_,"1111 in %d",info->proc_seq.load());
        info->last_process_time = rclcpp::Clock().now();

        try
        {
            // sw.reset();
            if (info->transform.child_frame_id == "")
            {
                info->frame_id = cloud->header.frame_id;

                // 获取变换: 从 "sensor_frame" 到 "base_link"
                info->transform = tf_->lookupTransform(robot_base_frame, cloud->header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
                geometry_msgs::msg::Point sensor_local_origin;
                sensor_local_origin.x = 0;
                sensor_local_origin.y = 0;
                sensor_local_origin.z = 0;
                tf2::doTransform(sensor_local_origin, info->sensor_point, info->transform);
            }

            auto transform_base2map = tf_->lookupTransform(global_frame, robot_base_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(2.0)); // 获取 base 到 map 的转换
            transform_base2map.transform.rotation.x = 0; // 不考虑roll
            transform_base2map.transform.rotation.y = 0; // 不考虑pitch

            tf2::Transform start_tf;
            tf2::Transform end_tf;
            tf2::fromMsg(info->transform.transform, start_tf);
            tf2::fromMsg(transform_base2map.transform, end_tf);
            // tf_look_ts = sw.elapsed();
            // sw.reset();
            // 计算组合变换
            tf2::Transform combined_tf = end_tf * start_tf;
            geometry_msgs::msg::TransformStamped result_tf;
            result_tf.transform = tf2::toMsg(combined_tf);
            result_tf.header = transform_base2map.header;
            result_tf.child_frame_id = info->transform.child_frame_id;

            geometry_msgs::msg::Point sensor_origin; // 传感器的全局坐标
            tf2::doTransform(info->sensor_point, sensor_origin, transform_base2map);

            sensor_msgs::msg::PointCloud2 global_cloud; // 全局坐标的点云
            tf2::doTransform(*cloud, global_cloud, result_tf);
            // tf_trans_ts = sw.elapsed();
            // sw.reset();
            // tf_ts = tf_trans_ts+tf_look_ts;
            // RCLCPP_WARN(logger_,"2222 in %d",info->proc_seq.load());
            // 消除障碍物
            if (random_noise != 0.0)
            {
                int random_numbers = 100;
                double random_z = rand()%random_numbers*(random_noise/random_numbers)-(random_noise/2);
                sensor_origin.z += random_z;
                double random_y = rand()%random_numbers * (random_noise/random_numbers)-(random_noise/2);
                sensor_origin.y += random_y;
            }
            boost::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));
            RaytraceFreespace(sensor_origin, global_cloud, info);
            // clear_ts = sw.elapsed();
            // sw.reset();

            struct CliffCell
            {
                int pt_cnt = 0; // 当前cell有多少个点
            };
            std::vector<std::vector<CliffCell>> cliff_grid;
            cliff_grid.resize(width);
            for (auto &&row : cliff_grid)
            {
                row.resize(height);
            }

            // RCLCPP_WARN(logger_,"3333 in %d",info->proc_seq.load());
            // 标记障碍物
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(global_cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(global_cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(global_cloud, "z");
            unsigned int mx, my, mz;
            bool has_cliff = false;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                double t_x = *iter_x, t_y = *iter_y, t_z = *iter_z;

                // 悬崖检测
                if (info->cliff_detect_enable && t_z <= info->cliff_detect_depth_min && t_z >= info->cliff_detect_depth_max && topic_cliff_detect) 
                {
                    auto dx = sensor_origin.x - t_x;  // 计算相机到点的距离
                    auto dy = sensor_origin.y - t_y;
                    auto dz = sensor_origin.z - t_z;
                    auto z = 0;
                    auto ratio = (sensor_origin.z - z)/dz; // 计算 相机到点的线 与 地面 的交点
                    double ground_x = sensor_origin.x - dx*ratio; // 交点x
                    double ground_y = sensor_origin.y - dy*ratio; // 交点y
                    if (Dist(ground_x, ground_y, 0, sensor_origin.x, sensor_origin.y, 0) < info->cliff_max_range)
                    {
                        if (WorldToMap3D(t_x, t_y, 0.05, mx, my, mz))
                        {
                            cliff_grid[mx][my].pt_cnt++; // 此cell有悬崖点
                            has_cliff = true;
                            continue;
                        }
                    }
                }

                if (t_z > info->max_obstacle_height || t_z < origin_z)
                {
                    continue;
                }

                auto dist = Dist(t_x, t_y, 0, sensor_origin.x, sensor_origin.y, 0);
                if (dist > info->obstacle_range)
                {
                    continue;
                }

                double height_offset = ((info->max_dist_min_obs_height-origin_z)/info->obstacle_range)*dist;
                double cur_height = origin_z;
                if (height_offset > 0)
                {
                    cur_height += height_offset;
                }
                if (t_z >= origin_z && t_z >= cur_height)
                {
                    if (!WorldToMap3D(t_x, t_y, t_z, mx, my, mz))
                    {
                        continue;
                    }

                    // boost::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));
                    if (voxel_grid.markVoxelInMap(mx, my, mz, mark_threshold))
                    {
                        inner_costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE); // 更新内部costmap
                    }
                }
            }
            // make_ts = sw.elapsed();
            // sw.reset();

            // RCLCPP_WARN(logger_,"4444 in %d",info->proc_seq.load());
            // 判断悬崖
            unsigned int mcx, mcy, mcz; // 相机在map中的坐标
            if (has_cliff && WorldToMap3D(sensor_origin.x, sensor_origin.y, sensor_origin.z, mcx, mcy, mcz))
            {
                int cell_cnt = 0;
                int point_cnt = 0;
                std::vector<cv::Point> cliff_pt;

                int rang = info->cliff_max_range/resolution+1; // 计算的范围
                // 在范围内计算
                int rang_min_x = (int)mcx-rang<0?0:(int)mcx-rang;
                int rang_max_x = mcx+rang>width?width:mcx+rang;
                int rang_min_y = (int)mcy-rang<0?0:(int)mcy-rang;
                int rang_max_y = mcy+rang>height?height:mcy+rang;
                for (int x=rang_min_x; x<rang_max_x; x++)
                {
                    for (int y=rang_min_y; y<rang_max_y; y++)
                    {
                        auto &grid = cliff_grid[x][y];
                        if (grid.pt_cnt <= 0)
                        {
                            continue;
                        }
                        cell_cnt++;
                        point_cnt += grid.pt_cnt;
                        cliff_pt.push_back(cv::Point(x, y));
                    }
                }

                if (cell_cnt >= info->cliff_cells_per_frame && point_cnt > info->cliff_points_per_frame)
                {
                    // pub_error_warning->publish(err_msg);
                    // LOG_D_THROTTLE(60, "%s has cliff cell_cnt: %d point_cnt: %d", info->topic.c_str(), cell_cnt, point_cnt);
                    {
                        // boost::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));
                        for (auto &&pt : cliff_pt)
                        {
                            if (voxel_grid.markVoxelInMap(pt.x, pt.y, 2, mark_threshold))
                            {
                                inner_costmap.setCost(pt.x, pt.y, nav2_costmap_2d::LETHAL_OBSTACLE); // 更新内部costmap
                            }
                        }
                    }

                    // if (info->save_data)
                    // {
                    //     static auto last_save_tp = NowTimePoint();
                    //     if (GetIntervalMs(last_save_tp) >= 1000 * 60)
                    //     {
                    //         last_save_tp = NowTimePoint();
                    //         auto file_name = StringFormat("%s/%s", "/home/robot/clean/cliff_pointcloud", GetTimeString().c_str());
                    //         SavePointCloud(cloud, file_name);
                    //     }
                    // }

                    // // 膨胀悬崖区域
                    // int nbrhd_size = info->cliff_inflation_radius/2;
                    // for (int xx = -nbrhd_size; xx <= nbrhd_size; xx++)
                    // {
                    //     for (int yy = -nbrhd_size; yy <= nbrhd_size; yy++)
                    //     {
                    //         if (x+xx<0 || x+xx>=width || y+yy<0 || y+yy>=height)
                    //         {
                    //             continue;
                    //         }

                    //         if (voxel_grid.markVoxelInMap(x+xx, y+yy, 2, mark_threshold))
                    //         {
                    //             should_pub_error = true;
                    //             inner_costmap.setCost(x+xx, y+yy, costmap_2d::LETHAL_OBSTACLE); // 更新内部costmap
                    //         }
                    //     }
                    // }
                }
            }

            // cliff_ts = sw.elapsed();
            if (is_voxel_grid_pub)
            {
                nav2_msgs::msg::VoxelGrid grid_msg;
                unsigned int size = voxel_grid.sizeX() * voxel_grid.sizeY();
                grid_msg.size_x = voxel_grid.sizeX();
                grid_msg.size_y = voxel_grid.sizeY();
                grid_msg.size_z = voxel_grid.sizeZ();
                grid_msg.data.resize(size);
                memcpy(&grid_msg.data[0], voxel_grid.getData(), size * sizeof(unsigned int));

                grid_msg.origin.x = origin_x;
                grid_msg.origin.y = origin_y;
                grid_msg.origin.z = origin_z;

                grid_msg.resolutions.x = resolution;
                grid_msg.resolutions.y = resolution;
                grid_msg.resolutions.z = z_resolution;
                grid_msg.header.frame_id = global_frame;
                grid_msg.header.stamp = rclcpp::Clock().now();
                voxel_publisher->publish(grid_msg);
            }
            // RCLCPP_WARN(logger_,"5555 in %d",info->proc_seq.load());
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(logger_,"exception:%s in %d", e.what(),info->proc_seq.load());
            // LOG_E("exception:%s", e.what());
        }
    }

    void XlVoxelLayer::RaytraceFreespace(geometry_msgs::msg::Point &origin,  sensor_msgs::msg::PointCloud2 &cloud, std::shared_ptr<VoxelDataInfo> &info)
    {
        if (cloud.height*cloud.width == 0)
        {
            return;
        }

        double sensor_mx, sensor_my, sensor_mz;
        double &owx = origin.x;
        double &owy = origin.y;
        double &owz = origin.z;

        if (!WorldToMap3DFloat(owx, owy, owz, sensor_mx, sensor_my, sensor_mz))
        {
            return;
        }

        double map_end_x = origin_x + inner_costmap.getSizeInMetersX();
        double map_end_y = origin_y + inner_costmap.getSizeInMetersY();

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            double wpx = *iter_x;
            double wpy = *iter_y;
            double wpz = *iter_z;

            double distance = Dist(owx, owy, owz, wpx, wpy, wpz);
            double scaling_fact = 1.0;
            scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution) / distance), 0.0);
            wpx = scaling_fact * (wpx - owx) + owx;
            wpy = scaling_fact * (wpy - owy) + owy;
            wpz = scaling_fact * (wpz - owz) + owz;

            double a = wpx - owx;
            double b = wpy - owy;
            double c = wpz - owz;
            double t = 1.0;

            if (wpz > info->max_obstacle_height)
            {
                t = std::max(0.0, std::min(t, (info->max_obstacle_height - 0.01 - owz) / c));
            }
            else if (wpz < origin_z)
            {
                t = std::min(t, (origin_z - owz) / c);
            }

            if (wpx < origin_x)
            {
                t = std::min(t, (origin_x - owx) / a);
            }
            if (wpy < origin_y)
            {
                t = std::min(t, (origin_y - owy) / b);
            }

            if (wpx > map_end_x)
            {
                t = std::min(t, (map_end_x - owx) / a);
            }
            if (wpy > map_end_y)
            {
                t = std::min(t, (map_end_y - owy) / b);
            }

            wpx = owx + a * t;
            wpy = owy + b * t;
            wpz = owz + c * t;
            if (wpz < origin_z)
            {
                wpz = origin_z;
            }

            double point_mx, point_my, point_mz;
            if (WorldToMap3DFloat(wpx, wpy, wpz, point_mx, point_my, point_mz))
            {
                unsigned int cell_raytrace_range = inner_costmap.cellDistance(info->raytrace_range);
                voxel_grid.clearVoxelLineInMap(sensor_mx, sensor_my, sensor_mz, point_mx, point_my, point_mz, inner_costmap.getCharMap(), unknown_threshold, mark_threshold, nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::NO_INFORMATION, cell_raytrace_range);
            }
        }
    }

    bool XlVoxelLayer::WorldToMap3DFloat(double wx, double wy, double wz, double &mx, double &my, double &mz)
    {
        if (wx < origin_x || wy < origin_y) // || wz < origin_z)
        {
            return false;
        }

        mx = ((wx - origin_x) / resolution);
        my = ((wy - origin_y) / resolution);
        mz = ((wz - origin_z) / z_resolution);
        if (mx < width && my < height && mz < z_voxels)
        {
            return true;
        }
        return false;
    }

    bool XlVoxelLayer::WorldToMap3D(double wx, double wy, double wz, unsigned int &mx, unsigned int &my, unsigned int &mz)
    {
        if (wx < origin_x || wy < origin_y || wz < origin_z)
        {
            return false;
        }

        mx = (unsigned int)((wx - origin_x) / resolution);
        my = (unsigned int)((wy - origin_y) / resolution);
        mz = (unsigned int)((wz - origin_z) / z_resolution);

        if (mx < width && my < height && mz < (unsigned int)z_voxels)
        {
            return true;
        }
        return false;
    }

    void XlVoxelLayer::resetCostmapCB(const std_msgs::msg::String::ConstSharedPtr &msg)
    {
        (void)msg;
        reset();
    }

    void XlVoxelLayer::ObsDetectionHeight(const std_msgs::msg::Int32::ConstSharedPtr &msg)
    {
        // LOG_I("Obs Detection Height:%d", msg->data);
        for (auto &&data : data_set)
        {
            // SCOPE_LOCK(data->mtx);
            std::unique_lock<std::mutex> ul(data->mtx);

            origin_z = ((double)(msg->data))/100.0f;

            if (origin_z < 0.03)
            {
                origin_z = 0.03;
            }
            if (origin_z >= data->max_dist_min_obs_height)
            {
                data->max_dist_min_obs_height = origin_z + 0.01;
            }
        }
    }

    template <typename data_type>
    static void copyMapRegion(data_type *source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y, unsigned int sm_size_x, data_type *dest_map, unsigned int dm_lower_left_x, unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x, unsigned int region_size_y)
    {
        // we'll first need to compute the starting points for each map
        data_type *sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
        data_type *dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

        // now, we'll copy the source map into the destination map
        for (unsigned int i = 0; i < region_size_y; ++i)
        {
            memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
            sm_index += sm_size_x;
            dm_index += dm_size_x;
        }
    }

    void XlVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
    {
        std::unique_lock<Costmap2D::mutex_t> lock(*(inner_costmap.getMutex()));

        // project the new origin into the grid
        int cell_ox, cell_oy;
        cell_ox = int((new_origin_x - origin_x) / resolution);
        cell_oy = int((new_origin_y - origin_y) / resolution);

        // Nothing to update
        if (cell_ox == 0 && cell_oy == 0)
        {
            return;
        }

        // compute the associated world coordinates for the origin cell
        // beacuase we want to keep things grid-aligned
        origin_x = origin_x + cell_ox * resolution;
        origin_y = origin_y + cell_oy * resolution;

        // To save casting from unsigned int to int a bunch of times
        int size_x = inner_costmap.getSizeInCellsX();
        int size_y = inner_costmap.getSizeInCellsY();

        // we need to compute the overlap of the new and existing windows
        int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
        lower_left_x = std::min(std::max(cell_ox, 0), size_x);
        lower_left_y = std::min(std::max(cell_oy, 0), size_y);
        upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
        upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

        unsigned int cell_size_x = upper_right_x - lower_left_x;
        unsigned int cell_size_y = upper_right_y - lower_left_y;

        // we need a map to store the obstacles in the window temporarily
        unsigned char *local_map = new unsigned char[cell_size_x * cell_size_y];
        unsigned int *local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
        unsigned int *voxel_map = voxel_grid.getData();
        auto costmap = inner_costmap.getCharMap();

        // copy the local window in the costmap to the local map
        copyMapRegion(costmap, lower_left_x, lower_left_y, size_x, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
        copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x, local_voxel_map, 0, 0, cell_size_x, cell_size_x,
                      cell_size_y);

        // we'll reset our maps to unknown space if appropriate
        inner_costmap.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
        costmap = inner_costmap.getCharMap();

        // compute the starting cell location for copying data back in
        int start_x = lower_left_x - cell_ox;
        int start_y = lower_left_y - cell_oy;

        // now we want to copy the overlapping information back into the map, but in its new location
        copyMapRegion(local_map, 0, 0, cell_size_x, costmap, start_x, start_y, size_x, cell_size_x, cell_size_y);
        copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x, cell_size_x, cell_size_y);

        // make sure to clean up
        delete[] local_map;
        delete[] local_voxel_map;
    }

}