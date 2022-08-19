/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 16:14:15
 * @LastEditTime: 2022-08-17 17:44:11
 * @Description:前端节点处理流程
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
namespace localli
class FrontEndFlow : public rclcpp::Node
{
public:
    FrontEndFlow();

private:

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
};
