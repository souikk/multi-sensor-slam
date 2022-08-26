/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:08
 * @LastEditTime: 2022-08-23 09:23:41
 * @Description: 数据预处理的工作流程
 */
#include "preprocess_data/preprocess_data_flow.hpp"

namespace localization
{

    PreprocessDataFlow::PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        std::string config_file_path = WORKSPACE_PATH + "/config/front_end/config";
        YAML::Node config = YAML::Load(config_file_path);
        InitSubscriber(node_ptr_, config);
    }

    void PreprocessDataFlow::initSubscriber(rclcpp::Node::SharedPtr node_ptr, YAML::Node &config)
    {
        subImu_ = std::make_shared<ImuSubscriber>(
            node_ptr,
            config["imu"]["topic_name"].as<std::string>(),
            config["imu"]["queue_size"].as<int>());
        subLidar_ = std::make_shared<LidarSubscriber>(
            node_ptr,
            config["lidar"]["topic_name"].as<std::string>(),
            config["lidar"]["queue_size"].as<int>());
        subGnss_ = std::make_shared<GnssSubscriber>(
            node_ptr,
            config["gnss"]["topic_name"].as<std::string>(),
            config["gnss"]["queue_size"].as<int>());
    }

}