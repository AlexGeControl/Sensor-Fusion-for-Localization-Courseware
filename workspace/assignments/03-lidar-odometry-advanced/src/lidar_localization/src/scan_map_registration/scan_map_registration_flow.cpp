/*
 * @Description: LOAM scan-map registration facade
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#include "lidar_localization/scan_map_registration/scan_map_registration_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

ScanMapRegistrationFlow::ScanMapRegistrationFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/loam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // init params:
    InitParam(config_node["scan_map_registration"]["param"]);

    // subscriber to registered scans:
    InitSubscribers(nh, config_node["scan_scan_registration"]["publisher"]);

    // LOAM front end workflow:
    scan_map_registration_ptr_ = std::make_unique<ScanMapRegistration>();

    // publisher for point clous used by mapping:
    InitPublishers(nh, config_node["scan_map_registration"]["publisher"]);
}

bool ScanMapRegistrationFlow::InitParam(const YAML::Node& config_node) {
    config_.num_frames_skip = config_node["num_frames_skip"].as<int>();

    return true;
}

bool ScanMapRegistrationFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    mapping_sharp_points_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["queue_size"].as<int>()
    );

    mapping_flat_points_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["queue_size"].as<int>()
    );

    odom_scan_to_scan_sub_ptr_ = std::make_unique<OdometrySubscriber>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(), 
        config_node["odometry"]["queue_size"].as<int>()
    );

    return true;
}

bool ScanMapRegistrationFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    odom_scan_to_map_pub_ptr_ = std::make_unique<OdometryPublisher>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(),
        config_node["odometry"]["frame_id"].as<std::string>(),
        config_node["odometry"]["child_frame_id"].as<std::string>(),
        config_node["odometry"]["queue_size"].as<int>()
    );

    lidar_to_map_tf_pub_ptr_ = std::make_unique<TFBroadCaster>(
        "/map", "/velo_link"
    );

    return true;
}

bool ScanMapRegistrationFlow::Run(void) {
    if (!ReadData()) {
        return false;
    }

    // TODO: support high-freq prediction in ALOAM:
    while( HasData() && ValidData() ) {
        // update scan-to-map odometry:
        UpdateData();

        // publish data:
        PublishData();
    }

    return true;
}

bool ScanMapRegistrationFlow::ReadData(void) {
    // parse registered scans:
    mapping_sharp_points_sub_ptr_->ParseData(mapping_sharp_points_buff_);
    mapping_flat_points_sub_ptr_->ParseData(mapping_flat_points_buff_);

    // parse scan-scan odometry estimation:
    odom_scan_to_scan_sub_ptr_->ParseData(odom_scan_to_scan_buff_);

    return true;
}

bool ScanMapRegistrationFlow::HasOdomData(void) {
    return !odom_scan_to_scan_buff_.empty();
}

bool ScanMapRegistrationFlow::HasCloudData(void) {
    return (!mapping_sharp_points_buff_.empty()) && (!mapping_flat_points_buff_.empty());
}

bool ScanMapRegistrationFlow::HasData(void) {
    if ( 
        !(HasOdomData() && HasCloudData())
    ) {
        return false;
    }

    return true;
}

bool ScanMapRegistrationFlow::ValidData() {
    const auto& odom_scan_to_scan_time = odom_scan_to_scan_buff_.front().time;

    const auto& mapping_sharp_points_time = mapping_sharp_points_buff_.front().time;
    const auto& mapping_flat_points_time = mapping_flat_points_buff_.front().time;

    if (
        (odom_scan_to_scan_time < mapping_sharp_points_time) ||
        (odom_scan_to_scan_time < mapping_flat_points_time)
    ) {
        odom_scan_to_scan_buff_.pop_front();
    }

    if (
        (odom_scan_to_scan_time == mapping_sharp_points_time) &&
        (odom_scan_to_scan_time == mapping_flat_points_time)
    ) {
        odom_scan_to_scan_ = std::move(odom_scan_to_scan_buff_.front());
        mapping_sharp_points_ = std::move(mapping_sharp_points_buff_.front());
        mapping_flat_points_ = std::move(mapping_flat_points_buff_.front());

        odom_scan_to_scan_buff_.pop_front();
        mapping_sharp_points_buff_.pop_front();
        mapping_flat_points_buff_.pop_front();

        return true;
    }

    return false;
}

bool ScanMapRegistrationFlow::UpdateData(void) {
    return scan_map_registration_ptr_->Update(
        mapping_sharp_points_.cloud_ptr, mapping_flat_points_.cloud_ptr, 
        odom_scan_to_scan_.pose,
        odometry_
    );
}

bool ScanMapRegistrationFlow::PublishData(void) {
    //
    // publish scan-map odometry:
    //
    odom_scan_to_map_pub_ptr_->Publish(odometry_, odom_scan_to_scan_.time);
    lidar_to_map_tf_pub_ptr_->SendTransform(odometry_, odom_scan_to_scan_.time);

    return true;
}

} // namespace lidar_localization