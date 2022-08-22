/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:58:44
 * @LastEditTime: 2022-08-22 11:16:51
 * @Description:数据预处理工作流程类的头文件
 */

#include "rclcpp/rclcpp.hpp"

#include "preprocess_data/preprocess_data_node.hpp"

namespace localization
{
    class PreprocessDataFlow
    {
    public:
        PreprocessDataFlow();

    private:
        rclcpp::Node::SharedPtr node_ptr_;
    };
} // namespace localization
