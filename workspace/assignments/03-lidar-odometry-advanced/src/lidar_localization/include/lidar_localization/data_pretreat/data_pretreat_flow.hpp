/*
 * @Description: LOAM data pre-processing facade
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/subscriber/tf_listener.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"

#include "lidar_localization/data_pretreat/data_pretreat.hpp"

namespace lidar_localization {

class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
    bool InitMotionCompensator(void);
    bool InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool InitCalibration(void);
    bool ReadData(void);
    bool HasData(void);
    bool ValidData(void);
    bool UpdateData(void);
    bool PublishData(void);
    
  private:
    // input: velodyne measurements
    std::unique_ptr<VelocitySubscriber> velocity_sub_ptr_{nullptr};
    std::deque<VelocityData> raw_velocity_data_buff_, synced_velocity_data_buff_;
    VelocityData current_velocity_data_;

    std::unique_ptr<TFListener> lidar_to_imu_ptr_{nullptr};
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::unique_ptr<CloudSubscriber<CloudDataXYZ>> cloud_sub_ptr_{nullptr};
    std::deque<CloudDataXYZ> cloud_data_buff_;
    CloudDataXYZ current_cloud_data_;

    // motion compensator:
    std::unique_ptr<DistortionAdjust> motion_compensator_ptr_;

    // scan registration implementation:
    std::unique_ptr<DataPretreat> data_pretreat_ptr_{nullptr};
    std::unique_ptr<CloudPublisher> filtered_cloud_pub_ptr_{nullptr};
    CloudDataXYZI::CLOUD_PTR filtered_cloud_data_;

    // outputs: registered scans
    std::unique_ptr<CloudPublisher> corner_points_sharp_pub_ptr_{nullptr};
    CloudDataXYZI::CLOUD_PTR corner_sharp_;
    std::unique_ptr<CloudPublisher> corner_points_less_sharp_pub_ptr_{nullptr};
    CloudDataXYZI::CLOUD_PTR corner_less_sharp_;
    std::unique_ptr<CloudPublisher> surf_points_flat_pub_ptr_{nullptr};
    CloudDataXYZI::CLOUD_PTR surf_flat_;
    std::unique_ptr<CloudPublisher> surf_points_less_flat_pub_ptr_{nullptr};
    CloudDataXYZI::CLOUD_PTR surf_less_flat_;
    std::unique_ptr<CloudPublisher> removed_points_pub_ptr_{nullptr};
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_