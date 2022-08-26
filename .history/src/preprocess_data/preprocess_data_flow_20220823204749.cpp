/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:08
 * @LastEditTime: 2022-08-23 20:47:19
 * @Description: 数据预处理的工作流程
 */
#include "preprocess_data/preprocess_data_flow.hpp"

namespace localization
{

    PreprocessDataFlow::PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        initSubscriber(node_ptr_, config["measurements"]);
    }

    void PreprocessDataFlow::initSubscriber(rclcpp::Node::SharedPtr node_ptr, const YAML::Node &config)
    {
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

    /**
     * @description:数据预处理的主要步骤
     * @return {bool}
     */
    bool PreprocessDataFlow::run()
    {
        if (!read())
        {
            return false;
        }
        while (hasData())
        {
        }
    }

    /**
     * @description:从订阅类中读取数据
     * @return {bool}
     */
    bool PreprocessDataFlow::read()
    {
        subLidar_->accessData(cloud_buff_);
        subImu_->accessData(imu_buff_);
        subGnss_->accessData(gnss_buff_);

        if (cloud_buff_.empty() || imu_buff_.empty() || gnss_buff_.empty())
        {
            return false;
        }

        return true;
    }

    bool PreprocessDataFlow::hasData()
    {
        if (cloud_buff_.empty() || imu_buff_.empty() || gnss_buff_.empty())
        {
            return false;
        }
        return true;
    }
}