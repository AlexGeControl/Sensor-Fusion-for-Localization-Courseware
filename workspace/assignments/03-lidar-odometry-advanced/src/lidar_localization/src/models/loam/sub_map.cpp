/*
 * @Description: LOAM sub map for scan-map registration, implementation
 * @Author: Ge Yao
 * @Date: 2021-05-15 13:25:21
 */

#include <chrono>

#include <pcl/common/transforms.h>

#include "glog/logging.h"

#include "lidar_localization/models/loam/sub_map.hpp"

namespace lidar_localization {

namespace aloam {

SubMap::SubMap(const SubMap::Config& config) {
    // init config:
    config_ = config;

    // init tiles:
    tiles_.sharp.clear();
    tiles_.flat.clear();

    tiles_.sharp.resize(config_.num_tiles);
    tiles_.flat.resize(config_.num_tiles);

    for (int i = 0; i < config_.num_tiles; ++i) {
        tiles_.sharp.at(i).reset(new CloudDataXYZI::CLOUD());
        tiles_.flat.at(i).reset(new CloudDataXYZI::CLOUD());
    }

    recently_accessed_.sharp.clear();
    recently_accessed_.flat.clear();

    // init center index:
    center_.x = config_.num_tiles_x >> 1;
    center_.y = config_.num_tiles_y >> 1;
    center_.z = config_.num_tiles_z >> 1;
}

SubMap::LocalMap SubMap::GetLocalMap(
    const Eigen::Vector3d &query_position
) {
    // first, convert query position to tile index:
    auto query_index = GetTileIndex(query_position);

    // anchor sub map to query position:
    Reanchor(query_index);

    // get local map:
    const auto local_map = GetLocalMap(query_index);

    // get result:
    return local_map;
};

bool SubMap::RegisterLineFeaturePoints(
    const CloudDataXYZI::CLOUD_PTR points, 
    const Eigen::Quaterniond& q, const Eigen::Vector3d& t
) {
    return RegisterFeaturePoints(points, q, t, tiles_.sharp, recently_accessed_.sharp);
}

bool SubMap::RegisterPlaneFeaturePoints(
    const CloudDataXYZI::CLOUD_PTR points, 
    const Eigen::Quaterniond& q, const Eigen::Vector3d& t
) {
    return RegisterFeaturePoints(points, q, t, tiles_.flat, recently_accessed_.flat);
}

bool SubMap::DownsampleSubMap(
    std::unique_ptr<CloudFilterInterface>& sharp_filter_ptr,
    std::unique_ptr<CloudFilterInterface>& flat_filter_ptr
) {
    for (const auto &sharp_tile_id: recently_accessed_.sharp) {
        auto& curr_sharp_tile = tiles_.sharp.at(sharp_tile_id);
        sharp_filter_ptr->Filter(curr_sharp_tile, curr_sharp_tile);
    }
    recently_accessed_.sharp.clear();

    for (const auto &flat_tile_id: recently_accessed_.flat) {
        auto& curr_flat_tile = tiles_.sharp.at(flat_tile_id);
        flat_filter_ptr->Filter(curr_flat_tile, curr_flat_tile);
    }
    recently_accessed_.flat.clear();

    return true;
}

SubMap::Index SubMap::GetTileIndex(const Eigen::Vector3d &t) {
    Index index;
    
    const double& resolution = config_.resolution;
    const double& half_resolution = resolution / 2;

    index.x = static_cast<int>((t.x() + half_resolution) / resolution) + center_.x;
    if (t.x() < -half_resolution) { --index.x; }

    index.y = static_cast<int>((t.y() + half_resolution) / resolution) + center_.y;
    if (t.y() < -half_resolution) { --index.y; }

    index.z = static_cast<int>((t.z() + half_resolution) / resolution) + center_.z;
    if (t.z() < -half_resolution) { --index.z; }

    return index;
}

SubMap::Index SubMap::GetTileIndex(const CloudDataXYZI::POINT &point) {
    Eigen::Vector3d t{
        point.x, point.y, point.z
    };

    return GetTileIndex(t);
}

bool SubMap::IsValidIndex(const SubMap::Index& index) {
    if (
        (0 <= index.x) && (index.x < config_.num_tiles_x) && 
        (0 <= index.y) && (index.y < config_.num_tiles_y) && 
        (0 <= index.z) && (index.z < config_.num_tiles_z) 
    ) {
        return true;
    }

    return false;
}

int SubMap::GetTileId(const SubMap::Index& index) {
    return index.x + index.y*(config_.num_tiles_x) + index.z*(config_.num_tiles_x*config_.num_tiles_y);
}

int SubMap::GetTileId(const int x, const int y, const int z) {
    return x + y*(config_.num_tiles_x) + z*(config_.num_tiles_x*config_.num_tiles_y);
}

void SubMap::ShiftForwardX(void) {
    for (int j = 0; j < config_.num_tiles_y; ++j) {
        for (int k = 0; k < config_.num_tiles_z; ++k) {
            const auto last_tile_id = GetTileId(config_.num_tiles_x - 1, j, k);

            const auto last_sharp_tile = tiles_.sharp.at(last_tile_id);
            const auto last_flat_tile = tiles_.flat.at(last_tile_id);

            for (int i = config_.num_tiles_x - 1; i > 0; --i) {
                const auto curr_tile_id = GetTileId(    i, j, k);
                const auto prev_tile_id = GetTileId(i - 1, j, k);

                tiles_.sharp.at(curr_tile_id) = tiles_.sharp.at(prev_tile_id);
                tiles_.flat.at(curr_tile_id) = tiles_.flat.at(prev_tile_id);
            }

            const auto first_tile_id = GetTileId(0, j, k);

            tiles_.sharp.at(first_tile_id) = last_sharp_tile;
            tiles_.flat.at(first_tile_id) = last_flat_tile;

            last_sharp_tile->clear();
            last_flat_tile->clear();
        }
    }

    ++center_.x;
}

void SubMap::ShiftBackwardX(void) {
    for (int j = 0; j < config_.num_tiles_y; ++j) {
        for (int k = 0; k < config_.num_tiles_z; ++k) {
            const auto first_tile_id = GetTileId(0, j, k);

            const auto first_sharp_tile = tiles_.sharp.at(first_tile_id);
            const auto first_flat_tile = tiles_.flat.at(first_tile_id);

            for (int i = 0; i < config_.num_tiles_x - 1; ++i) {
                const auto curr_tile_id = GetTileId(    i, j, k);
                const auto next_tile_id = GetTileId(i + 1, j, k);

                tiles_.sharp.at(curr_tile_id) = tiles_.sharp.at(next_tile_id);
                tiles_.flat.at(curr_tile_id) = tiles_.flat.at(next_tile_id);
            }

            const auto last_tile_id = GetTileId(config_.num_tiles_x - 1, j, k);

            tiles_.sharp.at(last_tile_id) = first_sharp_tile;
            tiles_.flat.at(last_tile_id) = first_flat_tile;

            first_sharp_tile->clear();
            first_flat_tile->clear();
        }
    }

    --center_.x;
}

void SubMap::ShiftForwardY(void) {
    for (int i = 0; i < config_.num_tiles_x; ++i) {
        for (int k = 0; k < config_.num_tiles_z; ++k) {
            const auto last_tile_id = GetTileId(i, config_.num_tiles_y - 1, k);

            const auto last_sharp_tile = tiles_.sharp.at(last_tile_id);
            const auto last_flat_tile = tiles_.flat.at(last_tile_id);

            for (int j = config_.num_tiles_y - 1; j > 0; --j) {
                const auto curr_tile_id = GetTileId(i,     j, k);
                const auto prev_tile_id = GetTileId(i, j - 1, k);

                tiles_.sharp.at(curr_tile_id) = tiles_.sharp.at(prev_tile_id);
                tiles_.flat.at(curr_tile_id) = tiles_.flat.at(prev_tile_id);
            }
            
            const auto first_tile_id = GetTileId(i, 0, k);

            tiles_.sharp.at(first_tile_id) = last_sharp_tile;
            tiles_.flat.at(first_tile_id) = last_flat_tile;

            last_sharp_tile->clear();
            last_flat_tile->clear();
        }
    }

    ++center_.y;
}

void SubMap::ShiftBackwardY(void) {
    for (int i = 0; i < config_.num_tiles_x; ++i) {
        for (int k = 0; k < config_.num_tiles_z; ++k) {
            const auto first_tile_id = GetTileId(i, 0, k);

            const auto first_sharp_tile = tiles_.sharp.at(first_tile_id);
            const auto first_flat_tile = tiles_.flat.at(first_tile_id);

            for (int j = 0; j < config_.num_tiles_y - 1; ++j) {
                const auto curr_tile_id = GetTileId(i,     j, k);
                const auto next_tile_id = GetTileId(i, j + 1, k);

                tiles_.sharp.at(curr_tile_id) = tiles_.sharp.at(next_tile_id);
                tiles_.flat.at(curr_tile_id) = tiles_.flat.at(next_tile_id);
            }

            const auto last_tile_id = GetTileId(i, config_.num_tiles_y - 1, k);

            tiles_.sharp.at(last_tile_id) = first_sharp_tile;
            tiles_.flat.at(last_tile_id) = first_flat_tile;

            first_sharp_tile->clear();
            first_flat_tile->clear();
        }
    }

    --center_.y;
}

void SubMap::ShiftForwardZ(void) {
    for (int i = 0; i < config_.num_tiles_x; ++i) {
        for (int j = 0; j < config_.num_tiles_y; ++j) {
            const auto last_tile_id = GetTileId(i, j, config_.num_tiles_z - 1);

            const auto last_sharp_tile = tiles_.sharp.at(last_tile_id);
            const auto last_flat_tile = tiles_.flat.at(last_tile_id);

            for (int k = config_.num_tiles_z - 1; k > 0; --k) {
                const auto curr_tile_id = GetTileId(i, j,     k);
                const auto prev_tile_id = GetTileId(i, j, k - 1);

                tiles_.sharp.at(curr_tile_id) = tiles_.sharp.at(prev_tile_id);
                tiles_.flat.at(curr_tile_id) = tiles_.flat.at(prev_tile_id);
            }
            
            const auto first_tile_id = GetTileId(i, j, 0);

            tiles_.sharp.at(first_tile_id) = last_sharp_tile;
            tiles_.flat.at(first_tile_id) = last_flat_tile;

            last_sharp_tile->clear();
            last_flat_tile->clear();
        }
    }

    ++center_.z;
}

void SubMap::ShiftBackwardZ(void) {
    for (int i = 0; i < config_.num_tiles_x; ++i) {
        for (int j = 0; j < config_.num_tiles_y; ++j) {
            const auto first_tile_id = GetTileId(i, j, 0);

            const auto first_sharp_tile = tiles_.sharp.at(first_tile_id);
            const auto first_flat_tile = tiles_.flat.at(first_tile_id);

            for (int k = 0; k < config_.num_tiles_z - 1; ++k) {
                const auto curr_tile_id = GetTileId(i, j,     k);
                const auto next_tile_id = GetTileId(i, j, k + 1);

                tiles_.sharp.at(curr_tile_id) = tiles_.sharp.at(next_tile_id);
                tiles_.flat.at(curr_tile_id) = tiles_.flat.at(next_tile_id);
            }

            const auto last_tile_id = GetTileId(i, j, config_.num_tiles_z - 1);

            tiles_.sharp.at(last_tile_id) = first_sharp_tile;
            tiles_.flat.at(last_tile_id) = first_flat_tile;

            first_sharp_tile->clear();
            first_flat_tile->clear();
        }
    }

    --center_.z;
}

void SubMap::Reanchor(SubMap::Index &query_index) {
    // re-anchor sub map:
    while (query_index.x < config_.reanchor_margin) {
        ShiftForwardX();
        ++query_index.x;
    }

    while (query_index.x >= config_.num_tiles_x - config_.reanchor_margin) {
        ShiftBackwardX();
        --query_index.x;
    }

    while (query_index.y < config_.reanchor_margin) {
        ShiftForwardY();
        ++query_index.y;
    }

    while (query_index.y >= config_.num_tiles_y - config_.reanchor_margin) {
        ShiftBackwardY();
        --query_index.y;
    }

    while (query_index.z < config_.reanchor_margin) {
        ShiftForwardZ();
        ++query_index.z;
    }

    while (query_index.z >= config_.num_tiles_z - config_.reanchor_margin) {
        ShiftBackwardZ();
        --query_index.z;
    }
}

SubMap::LocalMap SubMap::GetLocalMap(
    const SubMap::Index& query_index
) {
    SubMap::LocalMap local_map;

    local_map.query_index = query_index;
    local_map.sharp.reset(new CloudDataXYZI::CLOUD());
    local_map.flat.reset(new CloudDataXYZI::CLOUD());

    const auto local_map_radius = config_.local_map_radius;
    for (int dx = -local_map_radius; dx <= local_map_radius; ++dx) {
        for (int dy = -local_map_radius; dy <= local_map_radius; ++dy) {
            for (int dz = -(local_map_radius >> 1); dz <= (local_map_radius >> 1); ++dz) {
                const SubMap::Index curr_tile_index{
                    query_index.x + dx, 
                    query_index.y + dy, 
                    query_index.z + dz
                };

                if (IsValidIndex(curr_tile_index)) {
                    const auto curr_tile_id = GetTileId(curr_tile_index);

                    *local_map.sharp += *(tiles_.sharp.at(curr_tile_id));
                    *local_map.flat += *(tiles_.flat.at(curr_tile_id));
                }
            }
        }
    }

    return local_map;
}

bool SubMap::ProjectToMapFrame(
    const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
    const CloudDataXYZI::CLOUD_PTR& source,
    CloudDataXYZI::CLOUD_PTR& target
) {
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

    transform_matrix.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
    transform_matrix.block<3, 1>(0, 3) = t.cast<float>();
    
    pcl::transformPointCloud(*source, *target, transform_matrix);

    return true;
}

bool SubMap::RegisterFeaturePoints(
    const CloudDataXYZI::CLOUD_PTR points, 
    const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
    std::vector<CloudDataXYZI::CLOUD_PTR> &tiles,
    std::set<size_t>& recently_accessed
) {
    const auto num_feature_points = points->points.size();

    CloudDataXYZI::CLOUD_PTR target(new CloudDataXYZI::CLOUD());
    ProjectToMapFrame(q, t, points, target);

    for (size_t i = 0; i < num_feature_points; ++i)
    {   
        const auto& feature_point_in_map_frame = target->points[i];

        // register to associated tile:
        const auto associated_tile_index = GetTileIndex(feature_point_in_map_frame);
        if (IsValidIndex(associated_tile_index))
        {
            const auto associated_tile_id = GetTileId(associated_tile_index);
            tiles.at(associated_tile_id)->push_back(feature_point_in_map_frame);
            recently_accessed.insert(associated_tile_id);
        }
    }

    return true;
}

} // namespace aloam

} // namespace lidar_localization