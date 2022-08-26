/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 16:07:12
 * @LastEditTime: 2022-08-26 16:21:16
 * @Description:
 */

#include "publisher/cloud_publisher.hpp"

namespace localization
{
    CloudPublisher::CloudPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id)
    {
        node_ptr_ = node_ptr;
    }
} // namespace localization
