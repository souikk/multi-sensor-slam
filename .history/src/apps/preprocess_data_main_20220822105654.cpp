/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:22:42
 * @LastEditTime: 2022-08-22 10:56:17
 * @Description: 数据预处理主程序
 */

#include <chrono>

#include "preprocess_data/preprocess_data_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr preprocess_data_node = std::make_shared<localization::PreprocessDataNode>();
}