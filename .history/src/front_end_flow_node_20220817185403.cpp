/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 15:42:07
 * @LastEditTime: 2022-08-17 18:53:28
 * @Description:前端节点
 */
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "front_end_flow.hpp"

using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto front_end_node = std::make_shared<localization::FrontEndFlow>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 12, true);
    executor.add_node(front_end_node)
    rclcpp::Rate rate(1e8);
    while (rclcpp::ok())
    {
        executor.spin_once(0ns);

        rate.sleep();
    }
    return 1;
}