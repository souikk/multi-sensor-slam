/*
 * @Author: Ke Zhang
 * @Date: 2022-08-17 15:42:07
 * @LastEditTime: 2022-08-25 09:33:32
 * @Description:前端节点主程序
 */
#include <chrono>

#include "glog/logging.h"

#include "front_end/front_end_node.hpp"
#include "front_end/front_end_flow.hpp"
#include "global_path.h"

using namespace std::chrono_literals;
using namespace localization;

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORKSPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::Node::SharedPtr front_end_node = std::make_shared<localization::FrontEndNode>(options);
    std::shared_ptr<localization::FrontEndFlow> front_end_flow = std::make_shared<localization::FrontEndFlow>(front_end_node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 0, true);
    rclcpp::Rate rate(1e8);
    while (rclcpp::ok())
    {
        executor.spin_node_once(front_end_node, 0ms);

        rate.sleep();
    }
    return 0;
}