/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 09:22:42
 * @LastEditTime: 2022-08-22 20:02:27
 * @Description: 数据预处理主程序
 */

#include <chrono>

#include "glog/logging.h"

#include "preprocess_data/preprocess_data_node.hpp"
#include "preprocess_data/preprocess_data_flow.hpp"
#include "global_path.h"

using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORKSPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::Node::SharedPtr preprocess_data_node = std::make_shared<localization::PreprocessDataNode>(options);
    std::shared_ptr<localization::PreprocessDataFlow> preprocess_data_flow = std::make_shared<localization::PreprocessDataFlow>(preprocess_data_node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 0, true);
    rclcpp::Rate rate(1e8);
    while (rclcpp::ok())
    {
        executor.spin_node_once(preprocess_data_node, 0ms);

        rate.sleep();
    }
    return 0;
}