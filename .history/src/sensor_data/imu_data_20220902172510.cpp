/*
 * @Author: Ke Zhang
 * @Date: 2022-08-23 20:59:17
 * @LastEditTime: 2022-09-02 17:25:01
 * @Description:
 */
#include "sensor_data/imu_data.hpp"
namespace localization
{
    /**
     * @description: 同步imu数据
     * @param {double} timestamp
     * @param {deque<ImuData>} imu_buf
     * @param {ImuData} &sync_imu_data
     * @return {bool}
     */
    bool ImuData::syncPointCloud(double timestamp, std::deque<ImuData> imu_buf, ImuData &sync_imu_data)
    {
        //$ 判断点云时间戳是否在imu队列时间戳范围内
        if (timestamp < imu_buf.front().time || timestamp > imu_buf.back().time)
        {
            return false;
        }

        while (imu_buf.size() >= 2 && timestamp >= imu_buf.at(1).time)
        {
            if (timestamp >= imu_buf.front().time && timestamp < imu_buf.at(1).time)
            {
                break;
            }
            imu_buf.pop_front();
        }
        if (imu_buf.size() < 2)
        {
            return false;
        }

        if (timestamp == imu_buf.front().time)
        {
            sync_imu_data = imu_buf.front();
            return true;
        }
        ImuData data_front = imu_buf.at(0);
        ImuData data_back = imu_buf.at(1);
        double diff = data_back.time - data_front.time;
        double front_scale = (data_back.time - timestamp) / diff;
        double back_scale = (timestamp - data_front.time) / diff;
        if (timestamp - data_front.time > 0.1 || data_back.time - timestamp > 0.1)
        {
            return false;
        }

        sync_imu_data.angular_velocity.x = front_scale * data_front.angular_velocity.x + back_scale * data_back.angular_velocity.x;
        sync_imu_data.angular_velocity.y = front_scale * data_front.angular_velocity.y + back_scale * data_back.angular_velocity.y;
        sync_imu_data.angular_velocity.z = front_scale * data_front.angular_velocity.z + back_scale * data_back.angular_velocity.z;

        sync_imu_data.linear_accleration.x = front_scale * data_front.linear_accleration.x + back_scale * data_back.linear_accleration.x;
        sync_imu_data.linear_accleration.y = front_scale * data_front.linear_accleration.y + back_scale * data_back.linear_accleration.y;
        sync_imu_data.linear_accleration.z = front_scale * data_front.linear_accleration.z + back_scale * data_back.linear_accleration.z;

        Eigen::Quaterniond q0(data_front.orientation.w, data_front.orientation.x,
                              data_front.orientation.y, data_front.orientation.z);
        Eigen::Quaterniond q1(data_back.orientation.w, data_back.orientation.x,
                              data_back.orientation.y, data_back.orientation.z);
        Eigen::Quaterniond q = q0.slerp(back_scale, q1);

        sync_imu_data.orientation.w = q.w();
        sync_imu_data.orientation.x = q.x();
        sync_imu_data.orientation.y = q.y();
        sync_imu_data.orientation.z = q.z();
        sync_imu_data.orientation.Normlize();

        return true;
    }

} // namespace localization
