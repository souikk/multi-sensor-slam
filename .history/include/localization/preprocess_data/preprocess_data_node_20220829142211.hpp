/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:55:26
 * @LastEditTime: 2022-08-29 14:21:48
 * @Description:数据预处理节点头文件
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

enum lidarType
{
    Pandar

};

namespace localization
{
    class PreprocessDataNode : public rclcpp::Node
    {
    public:
        PreprocessDataNode(const rclcpp::NodeOptions &options);

    private:
    };
} // namespace localization