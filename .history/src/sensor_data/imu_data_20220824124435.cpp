/*
 * @Author: Ke Zhang
 * @Date: 2022-08-23 20:59:17
 * @LastEditTime: 2022-08-24 12:44:34
 * @Description:
 */
#include "sensor_data/imu_data.hpp"
namespace localization
{
    bool ImuData::syncPointCloud(double timestamp, std::deque<ImuData> &imu_buf, ImuData &sync_imu_data)
    {
        if (timestamp < imu_buf.front().time)
        {
            return false;
        }
        while (timestamp >= imu_buf.at(1).time)
        {
            imu_buf.pop_front();
        }
    }
} // namespace localization
