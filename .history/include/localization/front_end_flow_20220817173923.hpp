/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 16:14:15
 * @LastEditTime: 2022-08-17 17:39:12
 * @Description:前端节点处理流程
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class FrontEndFlow : public rclcpp::Node
{
public:
    FrontEndFlow();

private:
    rclcpp::Subscription<sensor_msgs::msg::imu.hpp>::
};
