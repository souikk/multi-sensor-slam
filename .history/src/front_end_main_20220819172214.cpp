/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 15:42:07
 * @LastEditTime: 2022-08-19 17:21:44
 * @Description:前端节点主程序
 */
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "front_end/front_end_node.hpp"
#include "front_end/front_end_flow.hpp"
using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr front_end_node = std::make_shared<localization::FrontEndNode>();
    std::shared_ptr<localization::FrontEndFlow> front_end_flow(front_end_node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 12, true);
    rclcpp::Rate rate(1e8);
    while (rclcpp::ok())
    {
        executor.spin_node_once(front_end_node, 0ms);

        rate.sleep();
    }
    return 1;
}