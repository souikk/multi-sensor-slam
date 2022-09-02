/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:08
 * @LastEditTime: 2022-09-02 16:57:08
 * @Description: 数据预处理的工作流程
 */
#include "preprocess_data/preprocess_data_flow.hpp"

namespace localization
{

    PreprocessDataFlow::PreprocessDataFlow(std::shared_ptr<PreprocessDataNode> node_ptr)
    {
        node_ptr_ = node_ptr;

        std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);

        // uP = std::make_shared<UndistortedPoint>(config["undistortion"]["lidar"].as<int>());
        initSubscriber(node_ptr_, config["subscriber"]);
        initPublisher(node_ptr_, config["publisher"]);

        init_gnss_flag_ = false;
    }

    /**
     * @description: 初始化订阅类
     * @param {shared_ptr<PreprocessDataNode>} node_ptr
     * @param {Node} &config
     * @return {*}
     */
    void PreprocessDataFlow::initSubscriber(std::shared_ptr<PreprocessDataNode> node_ptr, const YAML::Node &config)
    {
        subImu_ = std::make_shared<ImuSubscriber>(
            node_ptr,
            config["imu"]["topic_name"].as<std::string>(),
            config["imu"]["queue_size"].as<size_t>(),
            config["policy_history"].as<int>());
        subCloud_ = std::make_shared<CloudSubscriber>(
            node_ptr,
            config["lidar"]["topic_name"].as<std::string>(),
            config["lidar"]["queue_size"].as<size_t>(),
            config["policy_history"].as<int>());
        subGnss_ = std::make_shared<GnssSubscriber>(
            node_ptr,
            config["gnss"]["topic_name"].as<std::string>(),
            config["gnss"]["queue_size"].as<size_t>(),
            config["policy_history"].as<int>());
    }

    /**
     * @description: 初始化发布类
     * @param {shared_ptr<PreprocessDataNode>} node_ptr
     * @param {Node} &config
     * @return {*}
     */
    void PreprocessDataFlow::initPublisher(std::shared_ptr<PreprocessDataNode> node_ptr, const YAML::Node &config)
    {
        pubCloud_ = std::make_shared<CloudPublisher>(
            node_ptr,
            config["lidar"]["topic_name"].as<std::string>(),
            config["lidar"]["frame_id"].as<std::string>(),
            config["lidar"]["queue_size"].as<size_t>(),
            config["policy_history"].as<int>());

        pubGnss_ = std::make_shared<GnssPublisher>(
            node_ptr,
            config["gnss"]["topic_name"].as<std::string>(),
            config["gnss"]["frame_id"].as<std::string>(),
            config["gnss"]["queue_size"].as<size_t>(),
            config["policy_history"].as<int>());
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
            if (!vaildData())
            {
                continue;
            }
            if (!syncData())
            {
                continue;
            }
            if (init_gnss_flag_ == false)
            {
                initGnss();
            }

            if (!undistortedPointCloud())
            {
                continue;
            }
            publishData();
        }
        return false;
    }

    /**
     * @description:从订阅类中读取数据
     * @return {bool}
     */
    bool PreprocessDataFlow::read()
    {

        subCloud_->accessData(unsync_cloud_buff_);
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
        //$ 判断gnss和imu队列的大小，如果小于2退出。
        if (unsync_gnss_buff_.size() < 2 || unsync_imu_buff_.size() < 2)
        {
            return false;
        }
        current_cloud_time_ = rclcpp::Time(unsync_cloud_buff_.front().header.stamp).seconds();

        //$ 如果当前点云时间小于gnss，imu队列最早时间，则删除该点云。
        if (current_cloud_time_ < unsync_imu_buff_.front().time || current_cloud_time_ < unsync_gnss_buff_.front().time)
        {
            unsync_cloud_buff_.pop_front();
            LOG(INFO) << "this point cloud data too old, pop it" << std::endl;
            return false;
        }

        //$ 如果当前点云时间加上0.15大于gnss最新时间，需要等待gnss数据。并去除比较早的gnss数据
        if (current_cloud_time_ + 0.15 > unsync_imu_buff_.back().time)
        {
            while (unsync_imu_buff_.size() > 2)
            {
                if (current_cloud_time_ >= unsync_imu_buff_.front().time && current_cloud_time_ < unsync_imu_buff_.at(1).time)
                {
                    break;
                }
                unsync_imu_buff_.pop_front();
            }

            LOG(INFO) << "waiting imu data" << std::endl;
            return false;
        }

        //$ 如果当前点云时间加上0.15大于imu最新时间，需要等待imu数据。并去除比较早的imu数据
        if (current_cloud_time_ + 0.15 > unsync_gnss_buff_.back().time)
        {
            while (unsync_gnss_buff_.size() > 2)
            {
                if (current_cloud_time_ >= unsync_gnss_buff_.front().time && current_cloud_time_ < unsync_gnss_buff_.at(1).time)
                {
                    break;
                }
                unsync_gnss_buff_.pop_front();
            }

            LOG(INFO) << "waiting gnss data" << std::endl;
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

    /**
     * @description: 初始化gnss
     * @return {*}
     */
    void PreprocessDataFlow::initGnss()
    {

        unsync_gnss_buff_.front().initOriginPosition();
        init_gnss_flag_ = true;
    }

    /**
     * @description: 点云去畸变
     * @return {bool}
     */
    bool PreprocessDataFlow::undistortedPointCloud()
    {
        node_ptr_->preprocessCloud(unsync_imu_buff_, unsync_gnss_buff_, unsync_cloud_buff_.front(), sync_cloud_);
        // uP->deskew(unsync_imu_buff_, unsync_gnss_buff_, imu2lidar_, gnss2lidar_, unsync_cloud_buff_.front(), sync_cloud_);
    }

    /**
     * @description: 发布同步数据
     * @return {*}
     */
    void PreprocessDataFlow::publishData()
    {
        pubCloud_->publishMsg(sync_cloud_);
        pubGnss_->publishMsg(sync_gnss_);
    }
}