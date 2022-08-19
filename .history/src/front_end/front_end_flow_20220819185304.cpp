/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:35
 * @LastEditTime: 2022-08-19 18:53:03
 * @Description:
 */
#include "front_end/front_end_flow.hpp"
namespace localization
{
    FrontEndFlow ::FrontEndFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        subImu_ = std::make_shared<localization::ImuSubscriber>(node_ptr);
        subLidar_ = std::make_shared<localization::LidarSubscriber>(node_ptr);
    }
}
