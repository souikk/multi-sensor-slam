/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 15:41:51
 * @LastEditTime: 2022-08-29 15:37:09
 * @Description:
 */
#pragma once
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sensor_data/cloud_data.hpp"
#include "preprocess_data/preprocess_data_node.hpp"

namespace localization
{
    class CloudPublisher
    {
    public:
        CloudPublisher(std::shared_ptr<PreprocessDataNode> node_ptr, std::string topic_name, std::string frame_id, int queue_size, int policy_history);

        void publishMsg(CloudData msg);

    private:
        std::shared_ptr<PreprocessDataNode> node_ptr_;
        std::string frame_id_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud_;
    };
} // namespace localization
