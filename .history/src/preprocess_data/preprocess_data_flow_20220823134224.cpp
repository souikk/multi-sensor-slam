/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:08
 * @LastEditTime: 2022-08-23 13:42:24
 * @Description: 数据预处理的工作流程
 */
#include "preprocess_data/preprocess_data_flow.hpp"

namespace localization
{

    PreprocessDataFlow::PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
        RCLCPP_INFO(node_ptr_->get_logger(), "file path: ", config_file_path);
        YAML::Node config = YAML::Load(config_file_path);
        RCLCPP_INFO(node_ptr_->get_logger(), "*********");
        initSubscriber(node_ptr_, config["measurements"]);
    }

    void PreprocessDataFlow::initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config)
    {
        // RCLCPP_INFO(node_ptr_->get_logger(), "*********");
        subImu_ = std::make_shared<ImuSubscriber>(
            node_ptr,
            config["imu"]["topic_name"].as<std::string>(),
            config["imu"]["queue_size"].as<size_t>(),
            config["imu"]["policy_history"].as<int>());
        subLidar_ = std::make_shared<LidarSubscriber>(
            node_ptr,
            config["lidar"]["topic_name"].as<std::string>(),
            config["lidar"]["queue_size"].as<size_t>(),
            config["lidar"]["policy_history"].as<int>());
        subGnss_ = std::make_shared<GnssSubscriber>(
            node_ptr,
            config["gnss"]["topic_name"].as<std::string>(),
            config["gnss"]["queue_size"].as<size_t>(),
            config["gnss"]["policy_history"].as<int>());
    }

}