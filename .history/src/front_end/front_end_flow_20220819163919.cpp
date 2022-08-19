/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:22:35
 * @LastEditTime: 2022-08-19 16:39:19
 * @Description:
 */
#include "front_end/front_end_flow.hpp"
namespace localization
{
    FrontEndFlow ::FrontEndFlow(std::shared_ptr<rclcpp::Node> node_ptr)
    {
        node_ptr_ = std::make_shared<rclcpp::Node>(node_ptr);
    }
}
