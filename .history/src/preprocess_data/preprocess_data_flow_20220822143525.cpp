/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:08
 * @LastEditTime: 2022-08-22 14:35:25
 * @Description: 数据预处理的工作流程
 */
#include "preprocess_data/preprocess_data_flow.hpp"

namespace localization
{

    PreprocessDataFlow::PreprocessDataFlow(rclcpp::Node::SharedPtr node_ptr)
    {
        node_ptr_ = node_ptr;
        std::string config_file_path = WORKSPACE_PATH + "/config/front_end/config";
        YAML::Node config = YAML::Load(config_file_path);
        subImu_ = std::make_shared<ImuSubscriber>(node_ptr);
        subLidar_ = std::make_shared<LidarSubscriber>(node_ptr);
    }

}