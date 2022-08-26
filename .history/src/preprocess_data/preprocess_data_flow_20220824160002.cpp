/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:08
 * @LastEditTime: 2022-08-24 16:00:02
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

        initSubscriber(node_ptr_, config["subscriber"]);
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
            if (!vaildData)
            {
                continue;
            }
            if (!syncData())
            {
                continue;
            }
        }
        return false;
    }

    /**
     * @description:从订阅类中读取数据
     * @return {bool}
     */
    bool PreprocessDataFlow::read()
    {

        subLidar_->accessData(unsync_cloud_buff_);
        subImu_->accessData(unsync_imu_buff_);
        subGnss_->accessData(unsync_gnss_buff_);

        if (unsync_cloud_buff_.empty() || unsync_imu_buff_.empty() || unsync_gnss_buff_.empty())
        {
            return false;
        }

        return true;
    }

    /**
     * @description: 判断缓存里是否有数据
     * @return {bool}
     */
    bool PreprocessDataFlow::hasData()
    {
        if (unsync_cloud_buff_.empty() || unsync_imu_buff_.empty() || unsync_gnss_buff_.empty())
        {
            return false;
        }
        return true;
    }

    /**
     * @description: 判断数据是否有效
     * @return {bool}
     */
    bool PreprocessDataFlow::vaildData()
    {
        current_cloud_time_ = rclcpp::Time(unsync_cloud_buff_.front().header.stamp).seconds();
        if (current_cloud_time_ < unsync_imu_buff_.front().time || current_cloud_time_ < unsync_gnss_buff_.front().time)
        {
            unsync_cloud_buff_.pop_front();
            return false;
        }
        if (current_cloud_time_ > unsync_imu_buff_.back().time)
        {
            unsync_imu_buff_.clear();
            if (current_cloud_time_ > unsync_gnss_buff_.back().time)
            {
                unsync_gnss_buff_.clear();
                return false;
            }
            return false;
        }

        return true;
    }

    /**
     * @description: 判断数据是否可以同步到点云
     * @return {bool}
     */
    bool PreprocessDataFlow::syncData()
    {

        if (!ImuData::syncPointCloud(current_cloud_time_, unsync_imu_buff_, sync_imu_))
        {
            return false;
        }
        if (!GnssData::syncPointCloud(current_cloud_time_, unsync_gnss_buff_, sync_gnss_))
        {
            return false;
        }
        return true;
    }
}