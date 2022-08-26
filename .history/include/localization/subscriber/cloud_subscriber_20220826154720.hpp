/*
 * @Author: Ke Zhang
 * @Date: 2022-08-19 16:54:38
 * @LastEditTime: 2022-08-26 15:47:20
 * @Description: 订阅lidar数据类
 */
#pragma once

#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sensor_data/cloud_data.hpp"
namespace localization
{
    class cloudSubscriber
    {
    public:
        cloudSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, size_t queue_size, int policy_history);

        void accessData(std::deque<sensor_msgs::msg::PointCloud2> &cloud_buff);

    private:
        void lidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLidar_;
        std::deque<sensor_msgs::msg::PointCloud2> cloud_data_buf_;
    };
}