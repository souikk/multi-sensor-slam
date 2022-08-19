/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 15:42:07
 * @LastEditTime: 2022-08-19 17:18:41
 * @Description:前端节点主程序
 */
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "front_end/front_end_node.hpp"
#include ""
using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto front_end_node = std::make_shared<localization::FrontEndNode>();
    localization::FrontEndNode
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 12, true);
    rclcpp::Rate rate(1e8);
    while (rclcpp::ok())
    {
        executor.spin_node_once(front_end_node, 0ms);

        rate.sleep();
    }
    return 1;
}