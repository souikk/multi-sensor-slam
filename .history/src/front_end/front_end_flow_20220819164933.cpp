/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:35
 * @LastEditTime: 2022-08-19 16:49:33
 * @Description:
 */
#include "front_end/front_end_flow.hpp"
namespace localization
{
    FrontEndFlow ::FrontEndFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(node_ptr);
    }
}
