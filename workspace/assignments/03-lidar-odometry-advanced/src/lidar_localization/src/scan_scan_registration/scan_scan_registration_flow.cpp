/*
 * @Description: LOAM scan-scan registration facade
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#include "lidar_localization/scan_scan_registration/scan_scan_registration_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

ScanScanRegistrationFlow::ScanScanRegistrationFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // init params:
    InitParam(config_node["scan_scan_registration"]["param"]);

    // subscriber to registered scans:
    InitSubscribers(nh, config_node["data_pretreat"]["publisher"]);

    // LOAM front end workflow:
    scan_scan_registration_ptr_ = std::make_unique<ScanScanRegistration>();

    // publisher for point clous used by mapping:
    InitPublishers(nh, config_node["scan_scan_registration"]["publisher"]);
}

bool ScanScanRegistrationFlow::InitParam(const YAML::Node& config_node) {
    config_.num_frames_skip = config_node["num_frames_skip"].as<int>();

    return true;
}

bool ScanScanRegistrationFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    filtered_cloud_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["filtered"]["topic_name"].as<std::string>(), 
        config_node["filtered"]["queue_size"].as<int>()
    );

    corner_points_sharp_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["queue_size"].as<int>()
    );

    corner_points_less_sharp_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["less_sharp"]["topic_name"].as<std::string>(), 
        config_node["less_sharp"]["queue_size"].as<int>()
    );

    surf_points_flat_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["queue_size"].as<int>()
    );

    surf_points_less_flat_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["less_flat"]["topic_name"].as<std::string>(), 
        config_node["less_flat"]["queue_size"].as<int>()
    );

    return true;
}

bool ScanScanRegistrationFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    mapping_full_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["full"]["topic_name"].as<std::string>(), 
        config_node["full"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["full"]["queue_size"].as<int>())
    );
    mapping_sharp_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["sharp"]["queue_size"].as<int>())
    );
    mapping_flat_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["flat"]["queue_size"].as<int>())
    );

    odom_scan_to_scan_pub_ptr_ = std::make_unique<OdometryPublisher>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(),
        config_node["odometry"]["frame_id"].as<std::string>(),
        config_node["odometry"]["child_frame_id"].as<std::string>(),
        config_node["odometry"]["queue_size"].as<int>()
    );

    return true;
}

bool ScanScanRegistrationFlow::Run(void) {
    if (!ReadData()) {
        return false;
    }

    while( HasData() && ValidData() ) {
        // update scan-to-scan odometry:
        UpdateData();

        // publish data:
        PublishData();
    }

    return true;
}

bool ScanScanRegistrationFlow::ReadData(void) {
    filtered_cloud_sub_ptr_->ParseData(filtered_cloud_buff_);
    corner_points_sharp_sub_ptr_->ParseData(corner_points_sharp_buff_);
    corner_points_less_sharp_sub_ptr_->ParseData(corner_points_less_sharp_buff_);
    surf_points_flat_sub_ptr_->ParseData(surf_points_flat_buff_);
    surf_points_less_flat_sub_ptr_->ParseData(surf_points_less_flat_buff_);

    return true;
}

bool ScanScanRegistrationFlow::HasData(void) {
    if ( 
        filtered_cloud_buff_.empty() || 
        corner_points_sharp_buff_.empty() || 
        corner_points_less_sharp_buff_.empty() ||
        surf_points_flat_buff_.empty() ||
        surf_points_less_flat_buff_.empty() 
    ) {
        return false;
    }

    return true;
}

bool ScanScanRegistrationFlow::ValidData() {
    filtered_cloud_ = filtered_cloud_buff_.front();

    corner_points_sharp_ = corner_points_sharp_buff_.front();
    corner_points_less_sharp_ = corner_points_less_sharp_buff_.front();
    surf_points_flat_ = surf_points_flat_buff_.front();
    surf_points_less_flat_ = surf_points_less_flat_buff_.front();

    double d_time = filtered_cloud_.time - corner_points_sharp_.time;
    if (d_time < -0.05) {
        filtered_cloud_buff_.pop_front();
        return false;
    }

    if (d_time > 0.05) {
        corner_points_sharp_buff_.pop_front();
        corner_points_less_sharp_buff_.pop_front();
        surf_points_flat_buff_.pop_front();
        surf_points_less_flat_buff_.pop_front();
        return false;
    }

    filtered_cloud_buff_.pop_front();

    corner_points_sharp_buff_.pop_front();
    corner_points_less_sharp_buff_.pop_front();
    surf_points_flat_buff_.pop_front();
    surf_points_less_flat_buff_.pop_front();

    return true;
}

bool ScanScanRegistrationFlow::UpdateData(void) {
    scan_scan_registration_ptr_->Update(
        corner_points_sharp_.cloud_ptr,
        corner_points_less_sharp_.cloud_ptr,
        surf_points_flat_.cloud_ptr,
        surf_points_less_flat_.cloud_ptr,
        odometry_
    );

    return true;
}

bool ScanScanRegistrationFlow::PublishData(void) {
    static Eigen::Matrix4f gnss_to_odom = Eigen::Matrix4f::Identity();
    static int frame_count{0};

    odom_scan_to_scan_pub_ptr_->Publish(gnss_to_odom * odometry_, filtered_cloud_.time);

    //
    // publish point cloud for mapping:
    // 
    if ( 0 == (++frame_count % config_.num_frames_skip) ) {
        // reset frame count:
        frame_count = 0;
        
        mapping_full_points_pub_ptr_->Publish(filtered_cloud_.cloud_ptr, filtered_cloud_.time);
        mapping_sharp_points_pub_ptr_->Publish(corner_points_less_sharp_.cloud_ptr, filtered_cloud_.time);
        mapping_flat_points_pub_ptr_->Publish(surf_points_less_flat_.cloud_ptr, filtered_cloud_.time);
    }

    return true;
}

} // namespace lidar_localization