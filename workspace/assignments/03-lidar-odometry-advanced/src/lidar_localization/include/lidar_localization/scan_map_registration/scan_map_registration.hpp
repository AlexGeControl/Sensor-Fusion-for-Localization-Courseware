/*
 * @Description: LOAM scan-map registration interface
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#ifndef LIDAR_LOCALIZATION_SCAN_MAP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_SCAN_MAP_REGISTRATION_HPP_

#include <memory>

#include <array>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/loam/sub_map.hpp"

#include "lidar_localization/models/loam/aloam_registration.hpp"

namespace lidar_localization {

class ScanMapRegistration {
  public:
    //
    // submap configuration:
    // TODO: move this to config
    //
    static constexpr double kSubMapTileResolution = 50.0;

    static constexpr int kNumSubMapTilesX = 21;
    static constexpr int kNumSubMapTilesY = 21;
    static constexpr int kNumSubMapTilesZ = 11;
    static constexpr int kNumSubMapTiles = kNumSubMapTilesX * kNumSubMapTilesY * kNumSubMapTilesZ;

    static constexpr int kReanchorMargin = 3;
    static constexpr int kLocalMapRadius = 2;

    ScanMapRegistration(void);

    bool Update(
      const CloudData::CLOUD_PTR sharp_points,
      const CloudData::CLOUD_PTR flat_points,
      const Eigen::Matrix4f& odom_scan_to_scan,
      Eigen::Matrix4f& lidar_odometry
    );

  private:
    //
    // matching config:
    //
    struct {
      int min_num_sharp_points{10};
      int min_num_flat_points{50};
      int max_num_iteration{5};
      double distance_thresh{1.0};
      CeresALOAMRegistration::Config registration_config;
    } config_;

    //
    // odometry:
    //
    struct Pose {
      Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
      Eigen::Vector3f t = Eigen::Vector3f::Zero();
    };

    struct {
      Pose scan_map_odometry;
      Pose relative;
      Pose scan_scan_odometry;
    } pose_;

    //
    // submap:
    //
    struct {
      std::unique_ptr<CloudFilterInterface> sharp_filter_ptr_{nullptr};
      std::unique_ptr<CloudFilterInterface> flat_filter_ptr_{nullptr};
    } filter_;
    // target line & plane feature points:
    struct {
      pcl::KdTreeFLANN<CloudData::POINT>::Ptr sharp;
      pcl::KdTreeFLANN<CloudData::POINT>::Ptr flat;
    } kdtree_;

    std::unique_ptr<aloam::SubMap> submap_ptr_{nullptr};

    bool InitParams(const YAML::Node& config_node);
    bool InitFilters(const YAML::Node& config_node);
    bool InitKdTrees(void);
    bool InitSubMap(const YAML::Node& config_node);

    bool HasSufficientFeaturePoints(const aloam::SubMap::LocalMap &local_map);
    bool SetTargetPoints(aloam::SubMap::LocalMap& local_map);

    bool ProjectToMapFrame(
      const CloudData::POINT &input,
      CloudData::POINT &output
    );
    int AddEdgeFactors(
      const CloudData::CLOUD_PTR source,
      const CloudData::CLOUD_PTR target,
      CeresALOAMRegistration &aloam_registration 
    );
    int AddPlaneFactors(
      const CloudData::CLOUD_PTR source,
      const CloudData::CLOUD_PTR target,
      CeresALOAMRegistration &aloam_registration 
    );

    bool PredictScanMapOdometry(const Eigen::Matrix4f& odom_scan_to_scan);
    bool UpdateRelativePose(void);
    bool UpdateOdometry(Eigen::Matrix4f& lidar_odometry);
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SCAN_MAP_REGISTRATION_HPP_