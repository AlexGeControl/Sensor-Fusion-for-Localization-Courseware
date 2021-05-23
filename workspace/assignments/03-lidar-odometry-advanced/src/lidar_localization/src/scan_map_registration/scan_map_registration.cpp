/*
 * @Description: scan-map registration implementation
 * @Author: Ge Yao
 * @Date: 2021-05-09 14:38:03
 */
#include "lidar_localization/scan_map_registration/scan_map_registration.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

#include <chrono>

#include <limits>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

namespace lidar_localization {

ScanMapRegistration::ScanMapRegistration(void) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/loam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // init matching config:
    InitParams(config_node["scan_map_registration"]["param"]["matching"]);

    // init filters:
    InitFilters(config_node["scan_map_registration"]["param"]["filter"]);

    // init submap:
    InitKdTrees();
    InitSubMap(config_node["scan_map_registration"]["param"]["submap"]);
}

bool ScanMapRegistration::Update(
    const CloudData::CLOUD_PTR sharp_points,
    const CloudData::CLOUD_PTR flat_points,
    const Eigen::Matrix4f& odom_scan_to_scan,
    Eigen::Matrix4f& lidar_odometry
) {
    // down sample feature points:
    CloudData::CLOUD_PTR filtered_sharp_points(new CloudData::CLOUD());
    filter_.sharp_filter_ptr_->Filter(sharp_points, filtered_sharp_points);
    CloudData::CLOUD_PTR filtered_flat_points(new CloudData::CLOUD());
    filter_.flat_filter_ptr_->Filter(flat_points, filtered_flat_points);

    // predict scan-map odometry:
    PredictScanMapOdometry(odom_scan_to_scan);
    
    // get local map:
    auto local_map = submap_ptr_->GetLocalMap(pose_.scan_map_odometry.t);

    // if sufficient feature points for matching have been found:
    if ( HasSufficientFeaturePoints(local_map) ) {
        // set targets:
        SetTargetPoints(local_map);

        // iterative optimization:
        // LOG(WARNING) << "Scan-Map Registration: " << std::endl;
        for (int i = 0; i < config_.max_num_iteration; ++i) {
            // build problem:
            CeresALOAMRegistration aloam_registration(
                config_.registration_config,
                pose_.scan_map_odometry.q, pose_.scan_map_odometry.t
            );
            AddEdgeFactors(filtered_sharp_points, local_map.sharp, aloam_registration);
            AddPlaneFactors(filtered_flat_points, local_map.flat, aloam_registration);

            // get relative pose:
            aloam_registration.Optimize();
            aloam_registration.GetOptimizedRelativePose(pose_.scan_map_odometry.q, pose_.scan_map_odometry.t);
        }
    }

    // update relative pose estimation:
    UpdateRelativePose();

    // register feature points:
    submap_ptr_->RegisterLineFeaturePoints(filtered_sharp_points, pose_.scan_map_odometry.q, pose_.scan_map_odometry.t);
    submap_ptr_->RegisterPlaneFeaturePoints(filtered_flat_points, pose_.scan_map_odometry.q, pose_.scan_map_odometry.t);

    // downsample local map:
    static int dowmsample_count{0};
    if (0 == ++dowmsample_count % 1) {
        submap_ptr_->DownsampleSubMap(
            filter_.sharp_filter_ptr_, filter_.flat_filter_ptr_
        );

        dowmsample_count = 0;
    }

    // update odometry:
    UpdateOdometry(lidar_odometry);

    return true;
}

bool ScanMapRegistration::InitParams(const YAML::Node& config_node) {
    config_.min_num_sharp_points = config_node["min_num_sharp_points"].as<int>();
    config_.min_num_flat_points = config_node["min_num_flat_points"].as<int>();
    
    config_.distance_thresh = config_node["distance_thresh"].as<double>();

    config_.num_threads = config_node["num_threads"].as<int>();
    config_.max_num_iteration = config_node["max_num_iteration"].as<int>();
    config_.max_solver_time = config_node["max_solver_time"].as<double>();

    config_.registration_config.set_num_threads(config_.num_threads)
                               .set_max_num_iterations(config_.max_num_iteration)
                               .set_max_solver_time_in_seconds(config_.max_solver_time);

    return true;
}

bool ScanMapRegistration::InitFilters(const YAML::Node& config_node) {
    filter_.sharp_filter_ptr_ = std::make_unique<VoxelFilter>(config_node["sharp"]);
    filter_.flat_filter_ptr_ = std::make_unique<VoxelFilter>(config_node["flat"]);

    return true;
}

bool ScanMapRegistration::InitKdTrees(void) {
    kdtree_.sharp.reset(new pcl::KdTreeFLANN<CloudData::POINT>());
    kdtree_.flat.reset(new pcl::KdTreeFLANN<CloudData::POINT>());

    return true;
}

bool ScanMapRegistration::InitSubMap(const YAML::Node& config_node) {
    // init submap config:
    aloam::SubMap::Config config;

    config.set_resolution(config_node["resolution"].as<double>())
          .set_num_tiles_x(config_node["num_tiles_x"].as<int>())
          .set_num_tiles_y(config_node["num_tiles_y"].as<int>())
          .set_num_tiles_z(config_node["num_tiles_z"].as<int>())
          .set_num_tiles(config_node["num_tiles"].as<int>())
          .set_reanchor_margin(config_node["reanchor_margin"].as<int>())
          .set_local_map_radius(config_node["local_map_radius"].as<int>());
    
    // init submap:
    submap_ptr_ = std::make_unique<aloam::SubMap>(config);

    return true;
}

bool ScanMapRegistration::HasSufficientFeaturePoints(const aloam::SubMap::LocalMap &local_map) {
    const auto num_sharp_points = static_cast<int>(local_map.sharp->points.size());
    const auto num_flat_points = static_cast<int>(local_map.flat->points.size());

    return (
        (num_sharp_points > config_.min_num_sharp_points) && 
        (num_flat_points > config_.min_num_flat_points)
    );
}

bool ScanMapRegistration::SetTargetPoints(
    aloam::SubMap::LocalMap& local_map
) {
    filter_.sharp_filter_ptr_->Filter(local_map.sharp, local_map.sharp);
    filter_.flat_filter_ptr_->Filter(local_map.flat, local_map.flat);

    kdtree_.sharp->setInputCloud(local_map.sharp);
    kdtree_.flat->setInputCloud(local_map.flat);

    return true;
}

bool ScanMapRegistration::ProjectToMapFrame(
    const CloudData::CLOUD_PTR& source,
    CloudData::CLOUD_PTR& query
) {
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

    transform_matrix.block<3, 3>(0, 0) = pose_.scan_map_odometry.q.toRotationMatrix().cast<float>();
    transform_matrix.block<3, 1>(0, 3) = pose_.scan_map_odometry.t.cast<float>();
    
    pcl::transformPointCloud(*source, *query, transform_matrix);

    return true;
}

int ScanMapRegistration::AddEdgeFactors(
    const CloudData::CLOUD_PTR source,
    const CloudData::CLOUD_PTR target,
    CeresALOAMRegistration &aloam_registration 
) {
    const auto num_feature_points = source->points.size();

    CloudData::CLOUD_PTR query(new CloudData::CLOUD());
    ProjectToMapFrame(source, query);

    std::vector<int> target_candidate_indices;
    std::vector<float> target_candidate_distances;

    int num_factors{0};
    for (size_t i = 0; i < num_feature_points; ++i) {
        const auto& feature_point_in_lidar_frame = source->points[i];
        const auto& feature_point_in_map_frame = query->points[i];

        // search in target:
        const int num_neighbors = 5;
        kdtree_.sharp->nearestKSearch(feature_point_in_map_frame, num_neighbors, target_candidate_indices, target_candidate_distances);
        if (target_candidate_distances.back() < config_.distance_thresh) {
            //
            // estimate line direction using Eigen decomposition:
            //
            Eigen::Vector3d mu = Eigen::Vector3d::Zero();
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

            for (size_t i = 0; i < num_neighbors; ++i) {
                const auto& target_point = target->points[target_candidate_indices[i]];

                Eigen::Vector3d x{
                    target_point.x,
                    target_point.y,
                    target_point.z
                };

                mu += x;
                cov += x*x.transpose();
            }

            mu = (1.0/num_neighbors)*mu;
            cov = (1.0/num_neighbors)*cov - mu*mu.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
            //
            // check goodness of fitness:
            //
            if ( saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1] )
            {   
                const auto &unit_direction = saes.eigenvectors().col(2);

                // source in lidar frame:
                const Eigen::Vector3d source{
                    feature_point_in_lidar_frame.x,
                    feature_point_in_lidar_frame.y,
                    feature_point_in_lidar_frame.z
                };

                // target in map frame:
                const auto target_x =  0.1 * unit_direction + mu;
                const auto target_y = -0.1 * unit_direction + mu;

                // use predicted odometry from scan-scan odometry as initial value:
                aloam_registration.AddEdgeFactor(
                    source,
                    target_x, target_y,
                    1.0
                );

                ++num_factors;
            }	
        }
    }

    return num_factors;
}

int ScanMapRegistration::AddPlaneFactors(
    const CloudData::CLOUD_PTR source,
    const CloudData::CLOUD_PTR target,
    CeresALOAMRegistration &aloam_registration 
) {
    const auto num_feature_points = source->points.size();

    CloudData::CLOUD_PTR query(new CloudData::CLOUD());
    ProjectToMapFrame(source, query);

    std::vector<int> target_candidate_indices;
    std::vector<float> target_candidate_distances;

    int num_factors{0};
    for (size_t i = 0; i < num_feature_points; ++i) { 
        const auto& feature_point_in_lidar_frame = source->points[i];
        const auto& feature_point_in_map_frame = query->points[i];

        // search in target:
        const int num_neighbors = 5;
        kdtree_.sharp->nearestKSearch(feature_point_in_map_frame, num_neighbors, target_candidate_indices, target_candidate_distances);
        if (target_candidate_distances.back() < config_.distance_thresh) {
            //
            // estimate plane normal direction using least square:
            //
            Eigen::MatrixXd X = Eigen::MatrixXd::Zero(num_neighbors, 3);
            Eigen::MatrixXd b = Eigen::MatrixXd::Constant(num_neighbors, 1, -1.0);

            for (size_t i = 0; i < num_neighbors; ++i) {
                const auto &target_point = target->points[target_candidate_indices[i]];

                X(i, 0) = target_point.x;
                X(i, 1) = target_point.y;
                X(i, 2) = target_point.z;
            }

            Eigen::Vector3d norm = X.colPivHouseholderQr().solve(b);
            const auto negative_oa_dot_norm = norm.norm();
            norm.normalize();

            //
            // check goodness of fitness:
            //
            const auto residual = ((X*norm).array() + negative_oa_dot_norm).abs();
            if ( residual.maxCoeff() <= 0.2 ) {
                // source in lidar frame:
                const Eigen::Vector3d source{
                    feature_point_in_lidar_frame.x,
                    feature_point_in_lidar_frame.y,
                    feature_point_in_lidar_frame.z
                };

                // use predicted odometry from scan-scan odometry as initial value:
                aloam_registration.AddPlaneNormFactor(
                    source,
                    norm, negative_oa_dot_norm
                );

                ++num_factors;
            }
        }
    }

    return num_factors;
}

bool ScanMapRegistration::PredictScanMapOdometry(const Eigen::Matrix4f& odom_scan_to_scan) {
    // set scan-scan estimation, ALOAM original implementation:
    // pose_.scan_scan_odometry.q = odom_scan_to_scan.block<3, 3>(0, 0).cast<double>();
    // pose_.scan_scan_odometry.t = odom_scan_to_scan.block<3, 1>(0, 3).cast<double>();

    // set scan-scan estimation, my own modification:
    pose_.scan_scan_odometry.q = pose_.scan_map_odometry.q; 
    pose_.scan_scan_odometry.t = pose_.scan_map_odometry.t; 

    // predict scan-map estimation:
    const auto& dq = pose_.relative.q;
    const auto& dt = pose_.relative.t;

    pose_.scan_map_odometry.q = dq*pose_.scan_scan_odometry.q;
    pose_.scan_map_odometry.t = dq*pose_.scan_scan_odometry.t + dt;

    return true;
}

bool ScanMapRegistration::UpdateRelativePose(void) {
    // update relative pose for next round estimation:
    pose_.relative.q = pose_.scan_map_odometry.q * pose_.scan_scan_odometry.q.inverse();
    pose_.relative.t = pose_.scan_map_odometry.t - pose_.relative.q * pose_.scan_scan_odometry.t;

    return true;
}

bool ScanMapRegistration::UpdateOdometry(Eigen::Matrix4f& lidar_odometry) {
    lidar_odometry.block<3, 3>(0, 0) = pose_.scan_map_odometry.q.toRotationMatrix().cast<float>();
    lidar_odometry.block<3, 1>(0, 3) = pose_.scan_map_odometry.t.cast<float>();

    return true;
}

} // namespace lidar_localization