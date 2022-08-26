/*
 * @Author: Ke Zhang
 * @Date: 2022-08-23 20:59:17
 * @LastEditTime: 2022-08-24 13:08:29
 * @Description:
 */
#include "sensor_data/imu_data.hpp"
namespace localization
{
    bool ImuData::syncPointCloud(double timestamp, std::deque<ImuData> imu_buf, ImuData &sync_imu_data)
    {
        if (timestamp < imu_buf.front().time)
        {
            return false;
        }
        while (imu_buf.size() >= 2 && timestamp >= imu_buf.at(1).time)
        {
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

        sync_imu_data.angular_velocity.x = front_scale * data_front.angular_velocity.x + back_scale * data_back.angular_velocity.x;
        sync_imu_data.angular_velocity.y = front_scale * data_front.angular_velocity.y + back_scale * data_back.angular_velocity.y;
        sync_imu_data.angular_velocity.z = front_scale * data_front.angular_velocity.z + back_scale * data_back.angular_velocity.z;
    }

} // namespace localization
