#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
namespace LI2Sup {
// Polar occupancy + height descriptor. This is an in-project descriptor, not
// an implementation of the published Scan Context algorithm.
struct PlaceEntry {
  Eigen::Vector3d position;
  Eigen::Matrix<float, 20, 60> occupancy, height;
};
PlaceEntry describePlace(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                         const Eigen::Matrix3d &rotation,
                         const Eigen::Vector3d &position);
uint64_t mapFingerprint(const pcl::PointCloud<pcl::PointXYZI> &cloud);
void savePlaceIndex(const std::string &file, uint64_t fingerprint,
                    const std::vector<PlaceEntry> &entries);
std::vector<PlaceEntry> loadPlaceIndex(const std::string &file,
                                       uint64_t fingerprint);
std::vector<std::pair<double, Eigen::Isometry3d>>
retrievePlaces(const PlaceEntry &query, const Eigen::Matrix3d &rotation,
               const std::vector<PlaceEntry> &entries, int limit);
} // namespace LI2Sup
