/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:22:42
 * @LastEditTime: 2022-09-02 13:10:16
 * @Description: 数据预处理主程序
 */

#include <chrono>

#include "glog/logging.h"

#include "preprocess_data/preprocess_data_node.hpp"
#include "preprocess_data/preprocess_data_flow.hpp"
#include "global_path.h"

using namespace std::chrono_literals;
using namespace localization;
int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORKSPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;
    std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
    YAML::Node config = YAML::LoadFile(config_file_path);
    int policy_history = config["lidar"]["policy_history"].as<int>();
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    if (policy_history == 1)
    {
        options.use_intra_process_comms(true);
    }
    else
    {
        options.use_intra_process_comms(false);
    }

    std::shared_ptr<PreprocessDataNode> preprocess_data_node = std::make_shared<PreprocessDataNode>(options);
    std::shared_ptr<PreprocessDataFlow> preprocess_data_flow = std::make_shared<PreprocessDataFlow>(preprocess_data_node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 0, true);
    rclcpp::Rate rate(1e8);
    while (rclcpp::ok())
    {
        executor.spin_node_once(preprocess_data_node, 0ms);
        preprocess_data_flow->run();
        rate.sleep();
    }
    return 0;
}