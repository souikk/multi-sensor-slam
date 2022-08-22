/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:55:26
 * @LastEditTime: 2022-08-22 10:46:37
 * @Description:数据预处理节点头文件
 */

#include "rclcpp/rclcpp.hpp"

namespace localization
{
    class PreprocessDataNode : public rclcpp::Node
    {
    public:
        PreprocessDataNode();
    };
} // namespace localization