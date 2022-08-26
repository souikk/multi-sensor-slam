/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 16:07:12
 * @LastEditTime: 2022-08-26 16:24:43
 * @Description:
 */

#include "publisher/cloud_publisher.hpp"

namespace localization
{
    CloudPublisher::CloudPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id, int policy_history)
    {
        node_ptr_ = node_ptr;
        pubCloud_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, )
    }
} // namespace localization
