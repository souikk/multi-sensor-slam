/*
 * @Author: Ke Zhang
 * @Date: 2022-08-23 20:59:17
 * @LastEditTime: 2022-08-24 12:50:39
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
    }

} // namespace localization
