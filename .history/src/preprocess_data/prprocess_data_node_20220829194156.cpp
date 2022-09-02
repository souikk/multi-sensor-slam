/*
 * @Author: Ke Zhang
 * @Date: 2022-08-22 10:31:47
 * @LastEditTime: 2022-08-29 19:41:56
 * @Description:
 */
#include "preprocess_data/preprocess_data_node.hpp"

namespace localization
{
    PreprocessDataNode::PreprocessDataNode(const rclcpp::NodeOptions &options) : Node("PreprocessData", options)
    {
        std::string config_file_path = WORKSPACE_PATH + "/config/preprocess_data/config.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        lidar_ = lidarType(config["preprocess"]["lidar"].as<int>());
        downsampleRate_ = config["preprocess"]["downsampleRate"].as<int>();
        lidar_min_range_ = config["preprocess"]["lidarMinRange"].as<double>();
        lidar_max_range_ = config["preprocess"]["lidarMaxRange"].as<double>();
        N_SCAN_ = config["preprocess"]["N_SCAN"].as<int>();
        Horizon_SCAN_ = config["preprocess"]["Horizon_SCAN"].as<int>();
        numberOfCores_ = config["preprocess"]["numberOfCores"].as<int>();
        deskew_flag_ = 0;
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
            rclcpp::shutdown();
        }
    }

    bool PreprocessDataNode::preprocessCloud(std::deque<ImuData> &imu_data,
                                             std::deque<GnssData> &gnss_data,
                                             Eigen::Affine3d imu2lidar,
                                             Eigen::Affine3d gnss2lidar,
                                             sensor_msgs::msg::PointCloud2 currentCloudMsg,
                                             CloudData deskewedCloud)
    {
        if (!currentCloudInfo(currentCloudMsg))
        {
            return false;
        }
    }

    /**
     * @description: 获取当前帧的信息
     * @param {PointCloud2} currentCloudMsg
     * @return {bool}
     */
    bool PreprocessDataNode::currentCloudInfo(sensor_msgs::msg::PointCloud2 currentCloudMsg)
    {

        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn_);
        timeScanCur_ = rclcpp::Time(currentCloudMsg.header.stamp).seconds();
        timeScanEnd_ = laserCloudIn_->points.back().timestamp;
        if (laserCloudIn_->is_dense == false)
        {
            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }

        // check ring channel
        static int ring_flag = 0;
        if (ring_flag == 0)
        {
            ring_flag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ring_flag = 1;
                    break;
                }
            }
            if (ring_flag == -1)
            {
                RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available, please configure your point cloud data!");
                rclcpp::shutdown();
            }
        }

        // check point time
        if (deskew_flag_ == 0)
        {
            deskew_flag_ = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t" || field.name == "timestamp")
                {
                    deskew_flag_ = 1;
                    break;
                }
            }
            if (deskew_flag_ == -1)
                RCLCPP_WARN(get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool PreprocessDataNode::deskew(std::deque<ImuData> &imu_data,
                                    std::deque<GnssData> &gnss_data,
                                    Eigen::Affine3d imu2lidar,
                                    Eigen::Affine3d gnss2lidar,
                                    sensor_msgs::msg::PointCloud2 raw_cloud_msg,
                                    CloudData deskewedCloud)
    {
        //# 获取点云的初始时刻和结束时刻
        double cloud_start_time = rclcpp::Time(raw_cloud_msg.header.stamp).seconds();
        pcl::PointCloud<PointXYZIRT> raw_cloud;
        pcl::moveFromROSMsg(raw_cloud_msg, raw_cloud);
        double cloud_end_time = raw_cloud.end()->timestamp;
        //# 删除老的imu和gnss数据
        while (imu_data.size() >= 2 && imu_data.front().time < cloud_start_time)
        {
            if (imu_data.at(1).time > cloud_start_time)
            {
                break;
            }
            imu_data.pop_front();
        }
        if (imu_data.size() < 2)
            return false;

        while (gnss_data.size() >= 2 && gnss_data.front().time < cloud_start_time)
        {
            if (gnss_data.at(1).time > cloud_start_time)
            {
                break;
            }
            gnss_data.pop_front();
        }
        if (gnss_data.size() < 2)
            return false;

        //# 缓存imu积分的旋转值
        double imu_start = imu_data.front().time;
        double imu_time[1000];
        Eigen::Quaterniond imu_rot[1000];
        double last_imu_time = imu_start;
        Eigen::Matrix3d last_rot = Eigen::Matrix3d::Identity();
        imu_rot[0] = last_rot;
        imu_time[0] = imu_start;
        int num = 0;
        for (int i = 0; i < (int)imu_data.size() - 1; i++)
        {
            Eigen::Vector3d gyr1(imu_data[i].angular_velocity.x, imu_data[i].angular_velocity.y, imu_data[i].angular_velocity.z);
            gyr1 = imu2lidar * gyr1;
            Eigen::Vector3d gyr2(imu_data[i + 1].angular_velocity.x, imu_data[i + 1].angular_velocity.y, imu_data[i + 1].angular_velocity.z);
            gyr2 = imu2lidar * gyr2;
            Eigen::Vector3d angle = (imu_data[i + 1].time - imu_data[i].time) * (gyr1 + gyr2) / 2;
            //! 小角度计算旋转矩阵与旋转顺序无关
            Eigen::AngleAxisd roll(angle[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch(angle[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw(angle[2], Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond rot = yaw * pitch * roll;
            imu_rot[i + 1] = last_rot * rot;
            imu_time[i + 1] = imu_data[i + 1].time;

            last_rot = imu_rot[i];
            num++;
            if (imu_data[i + 1].time > cloud_end_time + 0.05)
                break;
        }

        //# 更新gnss数据的东北天坐标
        for (int i = 0; i < (int)gnss_data.size(); i++)
        {
            gnss_data[i].updateENU();
            if (gnss_data[i].time > cloud_end_time + 0.1)
                break;
        }

        Eigen::Vector3d ori_point_trans_gnss;
        Eigen::Quaterniond ori_point_rot_gnss;
        Eigen::Quaterniond ori_point_rot_imu;
        //$ 遍历每个点云
        for (int i = 0; i < (int)raw_cloud.size(); i++)
        {
            CloudData::pointType thisPoint;
            thisPoint.x = raw_cloud.points[i].x;
            thisPoint.y = raw_cloud.points[i].y;
            thisPoint.z = raw_cloud.points[i].z;
            thisPoint.intensity = raw_cloud.points[i].intensity;

            double range = pointDistance(thisPoint);
            if (range < lidarMinRange_ || range > lidarMaxRange_)
                continue;

            int rowIdn = raw_cloud.points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN_)
                continue;

            if (rowIdn % downsampleRate_ != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            static float ang_res_x = 360.0 / float(Horizon_SCAN_);
            int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN_ / 2;
            if (columnIdn >= Horizon_SCAN_)
                columnIdn -= Horizon_SCAN_;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN_)
                continue;
            double point_time = raw_cloud[i].timestamp;
            //# 计算当前点到第一个时刻的imu的旋转
            int j = 0;
            for (; j < num; j++)
            {
                if (point_time < imu_time[j])
                {
                    break;
                }
            }
            double ratio = (point_time - imu_time[j - 1]) / (imu_time[j] - imu_time[j - 1]);
            Eigen::Quaterniond cur_point_rot_imu = imu_rot[j - 1].slerp(ratio, imu_rot[j]);

            //# 计算当前点到的第一个时刻的gnss的位移
            j = 0;
            for (; j < (int)gnss_data.size(); j++)
            {
                if (point_time < gnss_data[j].time)
                {
                    break;
                }
            }
            double front_scale = (gnss_data[j].time - point_time) / (gnss_data[j].time - gnss_data[j - 1].time);
            double back_scale = (point_time - gnss_data[j - 1].time) / (gnss_data[j].time - gnss_data[j - 1].time);
            double x = front_scale * gnss_data[j - 1].local_E + back_scale * gnss_data[j].local_E;
            double y = front_scale * gnss_data[j - 1].local_N + back_scale * gnss_data[j].local_N;
            double z = front_scale * gnss_data[j - 1].local_U + back_scale * gnss_data[j].local_U;
            Eigen::Vector3d cur_point_trans_gnss(x, y, z);

            //# 如果是第一个点，不需要去畸变，只记录它相对imu和gnss的位姿
            if (i == 0)
            {
                ori_point_rot_imu = cur_point_rot_imu;
                ori_point_trans_gnss = cur_point_trans_gnss;
                Eigen::Quaterniond q0(gnss_data[j - 1].orientation.w,
                                      gnss_data[j - 1].orientation.x,
                                      gnss_data[j - 1].orientation.y,
                                      gnss_data[j - 1].orientation.z);
                Eigen::Quaterniond q1(gnss_data[j].orientation.w,
                                      gnss_data[j].orientation.x,
                                      gnss_data[j].orientation.y,
                                      gnss_data[j].orientation.z);
                ori_point_rot_gnss = q0.slerp(back_scale, q1);

                deskewedCloud.cloud_ptr_->points[i].x = raw_cloud.points[i].x;
                deskewedCloud.cloud_ptr_->points[i].y = raw_cloud.points[i].y;
                deskewedCloud.cloud_ptr_->points[i].z = raw_cloud.points[i].z;
                deskewedCloud.cloud_ptr_->points[i].intensity = raw_cloud.points[i].intensity;
                continue;
            }

            Eigen::Vector3d cur2ori_trans = ori_point_rot_gnss.inverse() * (cur_point_trans_gnss - ori_point_trans_gnss);
            Eigen::Quaterniond cur2ori_rot = ori_point_rot_imu.inverse() * cur_point_rot_imu;
            Eigen::Vector3d raw_point(raw_cloud.points[i].x, raw_cloud.points[i].y, raw_cloud.points[i].z);

            Eigen::Vector3d deskewedPoint = cur2ori_rot * raw_point + cur2ori_trans;
            deskewedCloud.cloud_ptr_->points[i].x = deskewedPoint.x();
            deskewedCloud.cloud_ptr_->points[i].y = deskewedPoint.y();
            deskewedCloud.cloud_ptr_->points[i].z = deskewedPoint.z();
        }
        return true;
    }

    double PreprocessDataNode::pointDistance(CloudData::pointType point)
    {
        return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    }
}