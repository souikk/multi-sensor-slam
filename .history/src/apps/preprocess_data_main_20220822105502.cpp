/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:22:42
 * @LastEditTime: 2022-08-22 10:54:53
 * @Description: 数据预处理主程序
 */

#include <chrono>
#include < memorry>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr preprocess_data_node = std::make_shared<localization::PreprocessDataNode>
}