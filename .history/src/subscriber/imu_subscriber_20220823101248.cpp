/*
 * @Author: Ke Zhang
 * @Date: 2022-08-18 09:55:49
 * @LastEditTime: 2022-08-23 10:12:13
 * @Description:
 */
#include "subscriber/imu_subscriber.hpp"

namespace localization
{
    ImuSubscriber::ImuSubscriber(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, int queue_size)
    {
        node_ptr_ = node_ptr;
        rmw_qos_profile_t qos_profile{
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,    // history,保留近期记录(Keep last)： 缓存最多N条记录，可通过队列长度选项来配置。
                                                 //保留所有记录(Keep all)：缓存所有记录，但受限于底层中间件可配置的最大资源。
            queue_size,                          // Size of the message queue,队列深度(Size of the queue)：只能与Keep last配合使用。
            RMW_QOS_POLICY_RELIABILITY_RELIABLE, //可靠性(Reliability),尽力的(Best effort)：尝试传输数据但不保证成功传输(当网络不稳定时可能丢失数据)。
                                                 //可靠的(Reliable)：反复重传以保证数据成功传输。
            RMW_QOS_POLICY_DURABILITY_VOLATILE,  //持续性(Durability),局部瞬态(Transient local)：发布器为晚连接(late-joining)的订阅器保留数据。
                                                 //易变态(Volatile)：不保留任何数据。
                                                 //以上每个策略都有系统默认值。这个默认值就是底层中间件的默认值，由DDS供应商工具(如XML配置文件)定义。
                                                 // DDS本身提供了许多可配置的策略。这些策略与ROS1的特征相似，所以在ROS1中是可见的。之后可能会有更多的策略在ROS2中可见。
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false};

        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                qos_profile.history,
                qos_profile.depth),
            qos_profile);
        subImu_ = node_ptr_->create_subscription<sensor_msgs::msg::Imu>(
            topic_name, qos, std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1));
    }

    void ImuSubscriber::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // sensor_msgs::msg::Imu thisMsg = *msg;
        ImuData imudata;
        imudata.time = rclcpp::Time(msg->header.stamp).seconds();
        imudata.angular_velocity.x = msg->angular_velocity.x;
        imudata.angular_velocity.y = msg->angular_velocity.y;
        imudata.angular_velocity.z = msg->angular_velocity.z;

        imudata.linear_accleration.x = msg->linear_acceleration.x;
        imudata.linear_accleration.y = msg->linear_acceleration.y;
        imudata.linear_accleration.z = msg->linear_acceleration.z;

        imudata.orientation.w = msg->orientation.w;
        imudata.orientation.x = msg->orientation.x;
        imudata.orientation.y = msg->orientation.y;
        imudata.orientation.z = msg->orientation.z;
        imudata.orientation.Normlize();
    }
}