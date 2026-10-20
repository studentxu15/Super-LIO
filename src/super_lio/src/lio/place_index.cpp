#include "lio/place_index.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
namespace LI2Sup {
PlaceEntry describePlace(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                         const Eigen::Matrix3d &rotation,
                         const Eigen::Vector3d &position) {
  PlaceEntry d;
  d.position = position;
  d.occupancy.setZero();
  d.height.setZero();
  for (const auto &p : cloud) {
    Eigen::Vector3d q = rotation * p.getVector3fMap().cast<double>();
    double r = std::hypot(q.x(), q.y());
    if (r < 1 || r >= 40 || q.z() < -3 || q.z() > 5)
      continue;
    int row = std::min(19, int(r / 2));
    int col = std::clamp(
        int((std::atan2(q.y(), q.x()) + M_PI) / (2 * M_PI) * 60), 0, 59);
    d.occupancy(row, col) += 1;
    d.height(row, col) = std::max(d.height(row, col), float(q.z() + 3));
  }
  d.occupancy = d.occupancy.unaryExpr([](float v) { return std::log1p(v); });
  return d;
}
uint64_t mapFingerprint(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  // Order-independent fingerprint, including cardinality. No absolute map path
  // is embedded so a map and index can be moved together.
  uint64_t sum = 0, xorhash = 0;
  for (const auto &p : cloud) {
    uint64_t h = 1469598103934665603ULL;
    for (float v : {p.x, p.y, p.z}) {
      uint32_t bits;
      std::memcpy(&bits, &v, 4);
      h ^= bits;
      h *= 1099511628211ULL;
    }
    sum += h;
    xorhash ^= h;
  }
  return sum ^ (xorhash << 1) ^ uint64_t(cloud.size());
}
void savePlaceIndex(const std::string &file, uint64_t fingerprint,
                    const std::vector<PlaceEntry> &entries) {
  std::filesystem::path path(file);
  if (path.has_parent_path())
    std::filesystem::create_directories(path.parent_path());
  std::ofstream out(file + ".tmp", std::ios::binary);
  uint64_t magic = 0x534c494f49445831ULL, count = entries.size();
  out.write((char *)&magic, 8);
  out.write((char *)&fingerprint, 8);
  out.write((char *)&count, 8);
  for (const auto &e : entries) {
    out.write((char *)e.position.data(), 24);
    out.write((char *)e.occupancy.data(), 4800);
    out.write((char *)e.height.data(), 4800);
  }
  out.close();
  if (!out)
    throw std::runtime_error("Failed to save place index: " + file);
  std::filesystem::rename(file + ".tmp", file);
}
std::vector<PlaceEntry> loadPlaceIndex(const std::string &file,
                                       uint64_t fingerprint) {
  std::ifstream in(file, std::ios::binary);
  uint64_t magic = 0, hash = 0, count = 0;
  in.read((char *)&magic, 8);
  in.read((char *)&hash, 8);
  in.read((char *)&count, 8);
  if (!in || magic != 0x534c494f49445831ULL || hash != fingerprint ||
      count == 0 || count > 100000 ||
      std::filesystem::file_size(file) != 24 + count * 9624)
    throw std::runtime_error(
        "Missing, invalid or mismatched place index: " + file +
        "; build a map with loop closure and map saving enabled");
  std::vector<PlaceEntry> entries(count);
  for (auto &e : entries) {
    in.read((char *)e.position.data(), 24);
    in.read((char *)e.occupancy.data(), 4800);
    in.read((char *)e.height.data(), 4800);
    if (!in || !e.position.allFinite() || !e.occupancy.allFinite() ||
        !e.height.allFinite())
      throw std::runtime_error("Corrupt place index: " + file);
  }
  return entries;
}
std::vector<std::pair<double, Eigen::Isometry3d>>
retrievePlaces(const PlaceEntry &query, const Eigen::Matrix3d &rotation,
               const std::vector<PlaceEntry> &entries, int limit) {
  std::vector<std::pair<double, size_t>> short_list;
  auto key = query.occupancy.rowwise().mean().eval();
  for (size_t k = 0; k < entries.size(); ++k)
    short_list.emplace_back(
        (key - entries[k].occupancy.rowwise().mean()).squaredNorm(), k);
  std::sort(short_list.begin(), short_list.end());
  if (short_list.size() > 80)
    short_list.resize(80);
  std::vector<std::pair<double, Eigen::Isometry3d>> out;
  for (auto [unused, k] : short_list) {
    const auto &entry = entries[k];
    double best = 1;
    int shift = 0;
    for (int s = 0; s < 60; ++s) {
      double score = 0;
      int count = 0;
      for (int c = 0; c < 60; ++c) {
        auto a = query.occupancy.col(c);
        auto b = entry.occupancy.col((c + s) % 60);
        double n = a.norm() * b.norm();
        if (n < 1e-6)
          continue;
        auto ah = query.height.col(c);
        auto bh = entry.height.col((c + s) % 60);
        double hn = ah.norm() * bh.norm();
        score += .7 * a.dot(b) / n + (hn > 1e-6 ? .3 * ah.dot(bh) / hn : 0);
        ++count;
      }
      double distance = count >= 15 ? 1 - score / count : 1;
      if (distance < best) {
        best = distance;
        shift = s;
      }
    }
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.linear() =
        Eigen::AngleAxisd(shift * 2 * M_PI / 60., Eigen::Vector3d::UnitZ())
            .toRotationMatrix() *
        rotation;
    p.translation() = entry.position;
    out.emplace_back(best, p);
  }
  std::sort(out.begin(), out.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });
  if (out.size() > size_t(limit))
    out.resize(limit);
  return out;
}
} // namespace LI2Sup
