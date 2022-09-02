/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:47
 * @LastEditTime: 2022-08-29 15:09:02
 * @Description:
 */
#include "preprocess_data/preprocess_data_node.hpp"

namespace localization
{
    PreprocessDataNode::PreprocessDataNode(const rclcpp::NodeOptions &options) : Node("PreprocessData", options)
    {
        std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        lidar_ = lidarType(config["undistortion"]["lidar"].as<int>());
        initLidar();
    }

    /**
     * @description: 初始化lidar型号
     * @return {*}
     */
    void PreprocessDataNode::initLidar()
    {
        if (lidar_ == lidarType::Pandar)
        {
            using PointXYZIRT = PandarPointXYZIRT;
        }
        else
        {
            LOG(ERROR) << "unkown lidar type.";
        }
    }
}