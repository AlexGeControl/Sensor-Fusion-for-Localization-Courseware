/*
 * @Description: LOAM scan-map registration facade
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#ifndef LIDAR_LOCALIZATION_SCAN_MAP_REGISTRATION_FLOW_HPP_
#define LIDAR_LOCALIZATION_SCAN_MAP_REGISTRATION_FLOW_HPP_

#include <memory>

#include <deque>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

#include "lidar_localization/publisher/tf_broadcaster.hpp"

#include "lidar_localization/scan_map_registration/scan_map_registration.hpp"

namespace lidar_localization {

class ScanMapRegistrationFlow {
  public:
    ScanMapRegistrationFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
    bool InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool ReadData(void);

    bool HasOdomData(void);
    bool HasCloudData(void);
    bool HasData(void);
    
    bool ValidData(void);
    bool UpdateData(void);
    bool PublishData(void);
    
  private:
    struct {
      int num_frames_skip{1};
    } config_;

    // whether the front end is inited:
    bool inited_{false};

    // inputs: registered scans & scan-scan odometry:
    std::unique_ptr<CloudSubscriber> mapping_sharp_points_sub_ptr_{nullptr};
    std::deque<CloudData> mapping_sharp_points_buff_;
    CloudData mapping_sharp_points_;

    std::unique_ptr<CloudSubscriber> mapping_flat_points_sub_ptr_{nullptr};
    std::deque<CloudData> mapping_flat_points_buff_;
    CloudData mapping_flat_points_;

    std::shared_ptr<OdometrySubscriber> odom_scan_to_scan_sub_ptr_;
    std::deque<PoseData> odom_scan_to_scan_buff_;
    PoseData odom_scan_to_scan_;

    // LOAM scan-map registration implementation:
    std::unique_ptr<ScanMapRegistration> scan_map_registration_ptr_{nullptr};

    // outputs:
    std::unique_ptr<OdometryPublisher> odom_scan_to_map_pub_ptr_;
    Eigen::Matrix4f odometry_ = Eigen::Matrix4f::Identity();

    std::unique_ptr<TFBroadCaster> lidar_to_map_tf_pub_ptr_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SCAN_MAP_REGISTRATION_FLOW_HPP_