/*
 * @Author: Ke Zhang
 * @Date: 2022-08-26 16:59:47
 * @LastEditTime: 2022-08-26 17:03:39
 * @Description:
 */

#include "publisher/gnss_publisher.hpp"

namespace localization
{
    GnssPublisher::GnssPublisher(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, std::string frame_id, int queue_size, int policy_history)
        : frame_id_(frame_id)
    {
    }

} // namespace localization
