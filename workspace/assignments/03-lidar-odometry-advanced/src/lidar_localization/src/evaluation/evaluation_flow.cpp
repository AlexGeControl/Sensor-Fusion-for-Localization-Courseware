/*
 * @Description: evo evaluation facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#include "lidar_localization/evaluation/evaluation_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

EvaluationFlow::EvaluationFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitSubscribers(nh, config_node["evaluation"]["subscriber"]);
    InitPublishers(nh, config_node["evaluation"]["publisher"]);
}

bool EvaluationFlow::Run() {
    if (!ReadData()) {
        return false;
    }
        
    if (!InitCalibration())  {
        return false;
    }


    if (!InitGNSS()) {      
        return false; 
    }

    while( HasData() ) {
        if (!ValidData()) {
            continue;
        }
        
        UpdateLaserOdometry();
        UpdateGNSSOdometry();

        SaveTrajectory();
    }

    return true;
}

bool EvaluationFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    odom_scan_to_scan_sub_ptr_ = std::make_unique<OdometrySubscriber>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(), 
        config_node["odometry"]["queue_size"].as<int>()
    );

    lidar_to_imu_ptr_ = std::make_unique<TFListener>(
        nh, 
        config_node["imu"]["frame_id"].as<std::string>(), 
        config_node["velodyne"]["frame_id"].as<std::string>()
    );
    imu_sub_ptr_ = std::make_unique<IMUSubscriber>(
        nh, 
        config_node["imu"]["topic_name"].as<std::string>(), 
        config_node["imu"]["queue_size"].as<int>()
    );
    gnss_sub_ptr_ = std::make_unique<GNSSSubscriber>(
        nh, 
        config_node["gnss"]["topic_name"].as<std::string>(), 
        config_node["gnss"]["queue_size"].as<int>()
    );

    return true;
}

bool EvaluationFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    odom_ground_truth_pub_ptr_ = std::make_unique<OdometryPublisher>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(),
        config_node["odometry"]["frame_id"].as<std::string>(),
        config_node["odometry"]["child_frame_id"].as<std::string>(),
        config_node["odometry"]["queue_size"].as<int>()
    );

    return true;
}

bool EvaluationFlow::ReadData() {
    static bool evaluator_inited = false;

    odom_scan_to_scan_sub_ptr_->ParseData(odom_scan_to_scan_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSData> unsynced_gnss_;

    imu_sub_ptr_->ParseData(unsynced_imu_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if ( odom_scan_to_scan_buff_.empty() ) {
        return false;
    }
        
    double laser_odom_time = odom_scan_to_scan_buff_.front().time;

    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, laser_odom_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, laser_odom_time);

    if ( !evaluator_inited ) {
        if (!valid_imu || !valid_gnss) {
            odom_scan_to_scan_buff_.pop_front();
            return false;
        }
        evaluator_inited = true;
    }

    return true;
}

bool EvaluationFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool EvaluationFlow::InitGNSS() {
    static bool gnss_inited = false;
    
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool EvaluationFlow::HasData() {
    if ( odom_scan_to_scan_buff_.empty() )
        return false;
    if ( imu_data_buff_.empty() )
        return false;
    if ( gnss_data_buff_.empty() )
        return false;
    
    return true;
}

bool EvaluationFlow::ValidData() {
    odom_scan_to_scan_ = odom_scan_to_scan_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double d_time = odom_scan_to_scan_.time - current_imu_data_.time;
    if (d_time < -0.05) {
        odom_scan_to_scan_buff_.pop_front();
        return false;
    }

    if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        return false;
    }

    odom_scan_to_scan_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool EvaluationFlow::UpdateLaserOdometry() {
    laser_odometry_ = odom_scan_to_scan_.pose;

    return true;
}

bool EvaluationFlow::UpdateGNSSOdometry() {
    static bool is_synced = false;
    static Eigen::Matrix4f gnss_to_odom = Eigen::Matrix4f::Identity();

    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= lidar_to_imu_;

    // init transform from GNSS frame to laser odometry frame:
    if (!is_synced) {
        gnss_to_odom = laser_odometry_ * gnss_odometry_.inverse();
        is_synced = true;
    }

    gnss_odometry_ = gnss_to_odom * gnss_odometry_;

    odom_ground_truth_pub_ptr_->Publish(gnss_odometry_);

    return true;
}

bool EvaluationFlow::SaveTrajectory() {
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;

    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
            return false;
        if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
            return false;
        if (!FileManager::CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
            return false;
        is_file_created = true;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
                ground_truth << " ";
                laser_odom << " ";
            }
        }
    }

    return true;
}

} // namespace lidar_localization