/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:58:44
 * @LastEditTime: 2022-08-26 15:08:02
 * @Description:数据预处理工作流程类的头文件
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"

#include "preprocess_data/preprocess_data_node.hpp"
#include "subscriber/lidar_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "undistorted_point/undistorted_point.hpp"
#include "sensor_data/imu_data.hpp"
#include "global_path.h"
namespace localization
{
    class PreprocessDataFlow
    {
    public:
        PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr);

        bool run();

    private:
        void initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        void initPublisher(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config);
        void initExtrinsic_parameter();
        bool read();
        bool hasData();
        bool vaildData();
        bool syncData();
        void initGnss();
        bool undistortedPointCloud();
        void publishData();

        rclcpp::Node::SharedPtr node_ptr_;
        std::shared_ptr<ImuSubscriber> subImu_;
        std::shared_ptr<LidarSubscriber> subLidar_;
        std::shared_ptr<GnssSubscriber> subGnss_;
        std::shared_ptr<undistortedPoint> uP;

        std::deque<GnssData> unsync_gnss_buff_;
        std::deque<ImuData> unsync_imu_buff_;
        std::deque<sensor_msgs::msg::PointCloud2> unsync_cloud_buff_;

        GnssData sync_gnss_;
        ImuData sync_imu_;
        CloudData sync_cloud_;

        Eigen::Affine3d imu2lidar_;
        Eigen::Affine3d gnss2lidar_;

        bool init_gnss_flag_;
        double current_cloud_time_;
    };
} // namespace localization
