/*
 * @Description: LOAM data pre-processing facade
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/loam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // subscriber to raw Velodyne measurements:
    InitSubscribers(nh, config_node["data_pretreat"]["subscriber"]);

    // scan registration workflow:
    InitMotionCompensator();
    data_pretreat_ptr_ = std::make_unique<DataPretreat>();

    filtered_cloud_data_.reset(new CloudDataXYZI::CLOUD());
    corner_sharp_.reset(new CloudDataXYZI::CLOUD());
    corner_less_sharp_.reset(new CloudDataXYZI::CLOUD());
    surf_flat_.reset(new CloudDataXYZI::CLOUD());
    surf_less_flat_.reset(new CloudDataXYZI::CLOUD());

    // publishers of registered scans:
    InitPublishers(nh, config_node["data_pretreat"]["publisher"]);
}

bool DataPretreatFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    velocity_sub_ptr_ = std::make_unique<VelocitySubscriber>(
        nh, 
        config_node["rtk"]["topic_name"].as<std::string>(), 
        config_node["rtk"]["queue_size"].as<int>()
    );

    lidar_to_imu_ptr_ = std::make_unique<TFListener>(
        nh, 
        config_node["tf"]["frame_id"].as<std::string>(), 
        config_node["tf"]["child_frame_id"].as<std::string>()
    );

    cloud_sub_ptr_ = std::make_unique<CloudSubscriber<CloudDataXYZ>>(
        nh, 
        config_node["velodyne"]["topic_name"].as<std::string>(), 
        config_node["velodyne"]["queue_size"].as<int>()
    );

    return true;
}

bool DataPretreatFlow::InitMotionCompensator(void) {
    motion_compensator_ptr_ = std::make_unique<DistortionAdjust>();

    return true;
}

bool DataPretreatFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    // corner points:
    filtered_cloud_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["filtered"]["topic_name"].as<std::string>(), 
        config_node["filtered"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["filtered"]["queue_size"].as<int>())
    );
    corner_points_sharp_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["sharp"]["queue_size"].as<int>())
    );
    corner_points_less_sharp_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["less_sharp"]["topic_name"].as<std::string>(), 
        config_node["less_sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["less_sharp"]["queue_size"].as<int>())
    );
    // surface points:
    surf_points_flat_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["flat"]["queue_size"].as<int>())
    );
    surf_points_less_flat_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["less_flat"]["topic_name"].as<std::string>(), 
        config_node["less_flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["less_flat"]["queue_size"].as<int>())
    );
    // removed points:
    removed_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["removed"]["topic_name"].as<std::string>(), 
        config_node["removed"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["removed"]["queue_size"].as<int>())
    );

    return true;
}

bool DataPretreatFlow::Run(void) {
    if (!InitCalibration()) {
        // LOG(WARNING) << "DataPretreatFlow: InitCalibration..." << std::endl;
        return false;
    }

    if (!ReadData()) {
        // LOG(WARNING) << "DataPretreatFlow: ReadData..." << std::endl;
        return false;
    }

    while( HasData() && ValidData()) {
        // LOG(WARNING) << "DataPretreatFlow: Do update..." << std::endl;

        // update velodyne measurement:
        UpdateData();

        // publish data:
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::InitCalibration(void) {
    // lookup imu pose in lidar frame:
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::ReadData(void) {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    velocity_sub_ptr_->ParseData(raw_velocity_data_buff_);

    return true;
}

bool DataPretreatFlow::HasData(void) {
    if ( cloud_data_buff_.empty() || raw_velocity_data_buff_.empty() ) {
        return false;
    }

    return true;
}

bool DataPretreatFlow::ValidData() {
    // use timestamp of lidar measurement as reference:
    const auto cloud_time = cloud_data_buff_.front().time;
    // const auto velocity_time = raw_velocity_data_buff_.front().time;

    // if (std::fabs(cloud_time - velocity_time) > 0.10) {
    //     return false;
    // }

    // current_cloud_data_ = std::move(cloud_data_buff_.front());
    // current_velocity_data_ = std::move(raw_velocity_data_buff_.front());

    // cloud_data_buff_.pop_front();
    // raw_velocity_data_buff_.pop_front();

    bool valid_velocity = VelocityData::SyncData(raw_velocity_data_buff_, synced_velocity_data_buff_, cloud_time);

    if (!valid_velocity) {
        if (!raw_velocity_data_buff_.empty() && raw_velocity_data_buff_.front().time > cloud_time) {
            cloud_data_buff_.pop_front();
        }
        return false;
    }

    current_cloud_data_ = std::move(cloud_data_buff_.front());
    current_velocity_data_ = std::move(synced_velocity_data_buff_.front());

    cloud_data_buff_.pop_front();
    synced_velocity_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::UpdateData(void) {
    //
    // TO-BE-EVALUATED-BY-MAINTAINER: first apply motion compensation:
    // 
    // current_velocity_data_.TransformCoordinate(lidar_to_imu_);

    // motion_compensator_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    // motion_compensator_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    // extract scan lines:
    data_pretreat_ptr_->Update(
        current_cloud_data_, 
        filtered_cloud_data_,
        corner_sharp_,
        corner_less_sharp_,
        surf_flat_,
        surf_less_flat_
    );

    return true;
}

bool DataPretreatFlow::PublishData(void) {
    filtered_cloud_pub_ptr_->Publish(filtered_cloud_data_, current_cloud_data_.time);

    corner_points_sharp_pub_ptr_->Publish(corner_sharp_, current_cloud_data_.time);
    corner_points_less_sharp_pub_ptr_->Publish(corner_less_sharp_, current_cloud_data_.time);
    surf_points_flat_pub_ptr_->Publish(surf_flat_, current_cloud_data_.time);
    surf_points_less_flat_pub_ptr_->Publish(surf_less_flat_, current_cloud_data_.time);

    return true;
}

} // namespace lidar_localization