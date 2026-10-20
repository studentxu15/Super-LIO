#include "lio/map_localizer.h"
#include "lio/params.h"
#include "lio/place_index.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

namespace LI2Sup {
namespace {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

using Pose = Eigen::Isometry3d;
using Clock = std::chrono::steady_clock;
Pose poseOf(const NavState &s) {
  Pose t = Pose::Identity();
  t.linear() =
      Eigen::Quaterniond(s.R.R_.cast<double>()).normalized().toRotationMatrix();
  t.translation() = s.p.cast<double>();
  return t;
}
geometry_msgs::msg::Pose message(const Pose &t) {
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation().x();
  p.position.y = t.translation().y();
  p.position.z = t.translation().z();
  Eigen::Quaterniond q(t.linear());
  q.normalize();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}
builtin_interfaces::msg::Time stamp(double t) {
  return rclcpp::Time(int64_t(t * 1e9));
}
Cloud::Ptr filtered(const Cloud::ConstPtr &c, double leaf) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<Point> v;
  v.setInputCloud(c);
  v.setLeafSize(leaf, leaf, leaf);
  v.filter(*out);
  return out;
}

} // namespace
struct MapLocalizer::Impl {
  rclcpp::Node &node;
  bool enabled = false, stopped = false, ready = false, valid = false,
       manual = false;
  std::mutex mutex;
  std::condition_variable cv;
  std::thread worker;
  struct Job {
    Pose raw;
    Cloud::Ptr cloud;
    double time;
    uint64_t generation;
  };
  std::optional<Job> pending;
  uint64_t generation = 0;
  Pose correction = Pose::Identity(), prior = Pose::Identity(),
       candidate = Pose::Identity();
  bool candidate_valid = false;
  int confirmations = 0, failures = 0;
  double last_submit = -1, last_accept = -1, last_global = -1, last_mse = 1.;
  Clock::time_point accepted_wall;
  Cloud::Ptr map;
  std::vector<PlaceEntry> place_entries;
  pcl::KdTreeFLANN<Point> map_tree;
  double leaf, period, crop_radius, max_mse, min_overlap, max_translation,
      max_angle, max_age, global_period, min_eigen, ambiguity_distance,
      ambiguity_margin;
  int iterations, min_points, confirm_count, max_failures, path_limit,
      global_candidates;
  std::string map_frame, odom_frame, child_frame, state = "INITIALIZING";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_sub;
  rclcpp::TimerBase::SharedPtr watchdog;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  nav_msgs::msg::Path path;
  template <class T> T param(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    return node.declare_parameter<T>("lio.localization." + name, value, d);
  }
  void status(const std::string &name, const std::string &detail = "") {
    state = name;
    std_msgs::msg::String m;
    m.data = name + (detail.empty() ? "" : " " + detail);
    status_pub->publish(m);
    std_msgs::msg::Bool b;
    b.data = valid;
    valid_pub->publish(b);
  }
  explicit Impl(rclcpp::Node &n) : node(n) {
    enabled = param("enabled", false);
    if (!enabled)
      return;
    if (g_save_map)
      throw std::invalid_argument("Disable lio.map.save_map in localization "
                                  "mode to protect the reference map");
    if (node.has_parameter("lio.loop.enabled") &&
        node.get_parameter("lio.loop.enabled").as_bool())
      throw std::invalid_argument(
          "Localization and loop mapping cannot be enabled together");
    std::string file =
        param<std::string>("map_path", "~/road_map/loop_map.pcd");
    leaf = param("voxel_size", .4);
    period = param("period", .5);
    crop_radius = param("crop_radius", 30.);
    max_mse = param("max_mse", .12);
    min_overlap = param("min_overlap", .7);
    max_translation = param("max_correction", 2.);
    max_angle = param("max_rotation_deg", 20.) * M_PI / 180.;
    max_age = param("max_age", 3.);
    global_period = param("global_retry_period", 5.);
    min_eigen = param("min_eigen_ratio", .0001);
    ambiguity_distance = param("ambiguity_distance", 1.5);
    ambiguity_margin = param("ambiguity_margin", .015);
    global_candidates = param("global_candidates", 12);
    iterations = param("gicp_iterations", 40);
    min_points = param("min_points", 150);
    confirm_count = param("confirmations", 2);
    max_failures = param("max_failures", 3);
    path_limit = param("path_limit", 10000);
    map_frame = param<std::string>("map_frame", "map");
    odom_frame = g_odom_frame_id;
    child_frame = g_odom_child_frame_id;
    manual = param("use_initial_pose", false);
    auto initial =
        param<std::vector<double>>("initial_pose", {0., 0., 0., 0., 0., 0.});
    const std::array<double, 13> values = {leaf,
                                           period,
                                           crop_radius,
                                           max_mse,
                                           min_overlap,
                                           max_translation,
                                           max_angle,
                                           max_age,
                                           global_period,
                                           min_eigen,
                                           double(iterations),
                                           ambiguity_distance,
                                           ambiguity_margin};
    if (!std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v) && v > 0; }) ||
        min_overlap > 1 || min_eigen >= 1 || min_points < 25 ||
        confirm_count < 1 || max_failures < 1 || path_limit < 1 ||
        global_candidates < 2 || global_candidates > 80 ||
        initial.size() != 6 ||
        !std::all_of(initial.begin(), initial.end(),
                     [](double v) { return std::isfinite(v); }) ||
        map_frame.empty() || map_frame == odom_frame)
      throw std::invalid_argument("Invalid lio.localization parameters");
    prior.translation() = Eigen::Vector3d(initial[0], initial[1], initial[2]);
    prior.linear() =
        (Eigen::AngleAxisd(initial[5] * M_PI / 180., Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(initial[4] * M_PI / 180., Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(initial[3] * M_PI / 180., Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    if (file == "~" || file.rfind("~/", 0) == 0) {
      const char *home = std::getenv("HOME");
      if (!home)
        throw std::invalid_argument("HOME is unset");
      file = std::string(home) + file.substr(1);
    }
    Cloud::Ptr input(new Cloud);
    if (pcl::io::loadPCDFile<Point>(file, *input) != 0)
      throw std::runtime_error("Cannot load localization map: " + file);
    Cloud::Ptr finite(new Cloud);
    for (const auto &p : *input)
      if (pcl::isFinite(p))
        finite->push_back(p);
    map = filtered(finite, leaf);
    if (map->size() < size_t(min_points))
      throw std::runtime_error(
          "Localization map has insufficient finite points");
    std::string index_file = param<std::string>("index_path", file + ".index");
    if (index_file.empty())
      index_file = file + ".index";
    if (index_file.rfind("~/", 0) == 0) {
      const char *home = std::getenv("HOME");
      if (!home)
        throw std::invalid_argument("HOME is unset");
      index_file = std::string(home) + index_file.substr(1);
    }
    if (!manual || std::filesystem::exists(index_file))
      place_entries = loadPlaceIndex(index_file, mapFingerprint(*finite));
    map_tree.setInputCloud(map);
    auto q = rclcpp::QoS(1).reliable().transient_local();
    map_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/localization/map", q);
    path_pub =
        node.create_publisher<nav_msgs::msg::Path>("/lio/localization/path", q);
    odom_pub = node.create_publisher<nav_msgs::msg::Odometry>(
        "/lio/localization/odom", 10);
    status_pub = node.create_publisher<std_msgs::msg::String>(
        "/lio/localization/status", q);
    valid_pub = node.create_publisher<std_msgs::msg::Bool>(
        "/lio/localization/valid", q);
    tf = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = map_frame;
    msg.header.stamp = node.now();
    map_pub->publish(msg);
    path.header.frame_id = map_frame;
    initial_sub =
        node.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
                       m) {
              const auto &p = m->pose.pose;
              Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                                   p.orientation.y, p.orientation.z);
              Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
              if (m->header.frame_id != map_frame || !t.allFinite() ||
                  !q.coeffs().allFinite() || q.norm() < 1e-6) {
                RCLCPP_WARN(node.get_logger(),
                            "Rejected /initialpose: expected finite pose in %s",
                            map_frame.c_str());
                return;
              }
              std::lock_guard<std::mutex> l(mutex);
              prior = Pose::Identity();
              prior.linear() = q.normalized().toRotationMatrix();
              prior.translation() = t;
              manual = true;
              ready = valid = false;
              candidate_valid = false;
              confirmations = failures = 0;
              last_global = -1;
              pending.reset();
              ++generation;
              path.poses.clear();
              status("SEARCHING", "manual prior");
            });
    watchdog = node.create_wall_timer(std::chrono::milliseconds(500), [this] {
      std::lock_guard<std::mutex> l(mutex);
      if (valid &&
          std::chrono::duration<double>(Clock::now() - accepted_wall).count() >
              max_age) {
        valid = false;
        status("LOST", "map match timeout");
      }
    });
    status("SEARCHING", manual ? "initial pose" : "global descriptor search");
    worker = std::thread([this] { run(); });
    RCLCPP_INFO(node.get_logger(),
                "Localization map loaded: %zu points from %s", map->size(),
                file.c_str());
  }
  void submit(const NavState &s, const BASIC::CloudPtr &cloud) {
    if (!enabled)
      return;
    Pose raw = poseOf(s);
    std::lock_guard<std::mutex> l(mutex);
    if (stopped)
      return;
    if (valid && s.timestamp - last_accept > max_age) {
      valid = false;
      status("LOST", "stale map constraint");
    }
    if (valid) {
      nav_msgs::msg::Odometry o;
      o.header.frame_id = map_frame;
      o.header.stamp = stamp(s.timestamp);
      o.child_frame_id = child_frame;
      o.pose.pose = message(correction * raw);
      auto v = raw.linear().transpose() * s.v.cast<double>();
      o.twist.twist.linear.x = v.x();
      o.twist.twist.linear.y = v.y();
      o.twist.twist.linear.z = v.z();
      for (int axis = 0; axis < 6; ++axis) {
        o.pose.covariance[axis * 7] =
            (axis < 3 ? std::max(.01, last_mse)
                      : std::max(.001, last_mse / 100.)) +
            std::max(0., s.timestamp - last_accept) * (axis < 3 ? .05 : .001);
        o.twist.covariance[axis * 7] = 1e6;
      }
      odom_pub->publish(o);
      geometry_msgs::msg::TransformStamped tr;
      tr.header = o.header;
      tr.child_frame_id = odom_frame;
      auto p = message(correction);
      tr.transform.translation.x = p.position.x;
      tr.transform.translation.y = p.position.y;
      tr.transform.translation.z = p.position.z;
      tr.transform.rotation = p.orientation;
      tf->sendTransform(tr);
    }
    if (last_submit >= 0 && s.timestamp - last_submit < period)
      return;
    last_submit = s.timestamp;
    pending = Job{raw, filtered(cloud, leaf), s.timestamp, generation};
    cv.notify_one();
  }
  bool verify(const Cloud::Ptr &source, const Pose &result, double &mse,
              double &overlap) {
    int count = 0;
    double squared = 0;
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
    std::vector<int> ids(8);
    std::vector<float> ds(8);
    for (const auto &p : *source) {
      const Eigen::Vector3d world = result * p.getVector3fMap().cast<double>();
      Point q;
      q.x = world.x();
      q.y = world.y();
      q.z = world.z();
      if (map_tree.nearestKSearch(q, 8, ids, ds) < 8 || ds[0] > .25)
        continue;
      ++count;
      squared += ds[0];
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (int k : ids)
        mean += map->points[k].getVector3fMap().cast<double>();
      mean /= 8;
      Eigen::Matrix3d c = Eigen::Matrix3d::Zero();
      for (int k : ids) {
        Eigen::Vector3d d =
            map->points[k].getVector3fMap().cast<double>() - mean;
        c += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(c);
      if (e.eigenvalues()[1] < 1e-6 ||
          e.eigenvalues()[0] > .2 * e.eigenvalues()[1])
        continue;
      auto normal = e.eigenvectors().col(0).eval();
      Eigen::Matrix<double, 6, 1> j;
      j.head<3>() = normal;
      j.tail<3>() = (world - result.translation()).cross(normal) / 10.;
      info += j * j.transpose();
    }
    overlap = double(count) / source->size();
    mse = count ? squared / count : 1e9;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> e(info);
    return count >= min_points && overlap >= min_overlap && mse <= max_mse &&
           e.eigenvalues()[0] > min_eigen * e.eigenvalues()[5];
  }
  std::optional<Pose> refine(const Job &j, const Pose &guess) {
    std::vector<int> indices;
    std::vector<float> distances;
    Point center;
    center.x = guess.translation().x();
    center.y = guess.translation().y();
    center.z = guess.translation().z();
    map_tree.radiusSearch(center, crop_radius, indices, distances);
    Cloud::Ptr target(new Cloud);
    for (int i : indices)
      target->push_back(map->points[i]);
    if (target->size() < size_t(min_points))
      return {};
    pcl::GeneralizedIterativeClosestPoint<Point, Point> g;
    g.setInputSource(j.cloud);
    g.setInputTarget(target);
    g.setMaximumIterations(iterations);
    g.setMaximumOptimizerIterations(20);
    g.setCorrespondenceRandomness(20);
    g.setMaxCorrespondenceDistance(3.);
    g.setTransformationEpsilon(1e-5);
    Cloud aligned;
    g.align(aligned, guess.matrix().cast<float>());
    if (!g.hasConverged() || !g.getFinalTransformation().allFinite())
      return {};
    Pose p = Pose::Identity();
    p.matrix() = g.getFinalTransformation().cast<double>();
    p.linear() = Eigen::Quaterniond(p.linear()).normalized().toRotationMatrix();
    return p;
  }
  std::optional<Pose> global(const Job &j) {
    const auto query =
        describePlace(*j.cloud, j.raw.linear(), Eigen::Vector3d::Zero());
    auto candidates =
        retrievePlaces(query, j.raw.linear(), place_entries, global_candidates);
    std::optional<Pose> best;
    double best_score = 1e9, runner_up = 1e9;
    for (const auto &[distance, seed] : candidates) {
      if (distance > .35)
        continue;
      auto found = refine(j, seed);
      if (!found)
        continue;
      Pose corr = *found * j.raw.inverse();
      if (std::acos(std::clamp(corr.linear()(2, 2), -1., 1.)) >
          20. * M_PI / 180.)
        continue;
      double mse, overlap;
      if (!verify(j.cloud, *found, mse, overlap) || overlap < .8 || mse > .08)
        continue;
      const double score = mse + .15 * (1 - overlap);
      RCLCPP_DEBUG(node.get_logger(),
                   "Global candidate desc=%.3f score=%.4f xyz=%.2f %.2f %.2f",
                   distance, score, found->translation().x(),
                   found->translation().y(), found->translation().z());
      if (score < best_score) {
        if (best && (found->translation() - best->translation()).norm() >
                        ambiguity_distance)
          runner_up = best_score;
        best = *found;
        best_score = score;
      } else if (best && (found->translation() - best->translation()).norm() >
                             ambiguity_distance)
        runner_up = std::min(runner_up, score);
    }
    if (runner_up < best_score + ambiguity_margin)
      return {};
    return best;
  }
  void process(const Job &j) {
    Pose guess;
    bool searching;
    uint64_t gen;
    bool need_global = false;
    {
      std::lock_guard<std::mutex> l(mutex);
      if (j.generation != generation)
        return;
      gen = generation;
      searching = !ready || !valid;
      if (candidate_valid)
        guess = candidate * j.raw;
      else if (ready)
        guess = correction * j.raw;
      else if (manual)
        guess = prior;
      else {
        if (last_global >= 0 && j.time - last_global < global_period)
          return;
        last_global = j.time;
        need_global = true;
      }
    }
    if (j.cloud->size() < size_t(min_points))
      return;
    const auto start = Clock::now();
    if (need_global) {
      auto found = global(j);
      if (!found) {
        reject(gen, "global candidate not found");
        return;
      }
      guess = *found;
    }
    auto refined = refine(j, guess);
    if (!refined) {
      reject(gen, "GICP did not converge");
      return;
    }
    Pose result = *refined;
    Pose innovation = guess.inverse() * result;
    Pose corr = result * j.raw.inverse();
    double tilt = std::acos(std::clamp(corr.linear()(2, 2), -1., 1.));
    double mse, overlap;
    if (tilt > 20. * M_PI / 180. ||
        (!need_global && (innovation.translation().norm() >
                              (searching ? 5. : max_translation) ||
                          Eigen::AngleAxisd(innovation.linear()).angle() >
                              (searching ? M_PI / 3 : max_angle))) ||
        !verify(j.cloud, result, mse, overlap)) {
      reject(gen, "geometry or innovation gate rejected");
      return;
    }
    std::lock_guard<std::mutex> l(mutex);
    if (gen != generation || stopped)
      return;
    if (candidate_valid) {
      Pose d = candidate.inverse() * corr;
      if (d.translation().norm() > .75 ||
          Eigen::AngleAxisd(d.linear()).angle() > 10. * M_PI / 180.)
        confirmations = 0;
    }
    candidate = corr;
    candidate_valid = true;
    ++confirmations;
    failures = 0;
    if (searching && confirmations < confirm_count) {
      status("CONFIRMING");
      return;
    }
    correction = corr;
    ready = valid = true;
    last_accept = j.time;
    last_mse = mse;
    accepted_wall = Clock::now();
    candidate_valid = false;
    confirmations = 0;
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = map_frame;
    p.header.stamp = stamp(j.time);
    p.pose = message(result);
    path.poses.push_back(p);
    if (path.poses.size() > size_t(path_limit))
      path.poses.erase(path.poses.begin());
    path.header = p.header;
    path_pub->publish(path);
    status("TRACKING", "mse=" + std::to_string(mse) +
                           " overlap=" + std::to_string(overlap));
    RCLCPP_INFO(
        node.get_logger(),
        "Map match accepted: t=%.3f mse=%.4f overlap=%.3f elapsed=%.3fs",
        j.time, mse, overlap,
        std::chrono::duration<double>(Clock::now() - start).count());
  }
  void reject(uint64_t gen, const std::string &reason) {
    std::lock_guard<std::mutex> l(mutex);
    if (gen != generation)
      return;
    candidate_valid = false;
    confirmations = 0;
    if (++failures >= max_failures) {
      valid = false;
      ready = false;
      manual = false;
      // Keep global retry throttling even after rejected candidates.
    }
    status(valid ? "DEGRADED" : (ready ? "LOST" : "SEARCHING"), reason);
  }
  void run() {
    try {
      while (true) {
        Job j;
        {
          std::unique_lock<std::mutex> l(mutex);
          cv.wait(l, [this] { return stopped || pending.has_value(); });
          if (stopped)
            break;
          j = std::move(*pending);
          pending.reset();
        }
        process(j);
      }
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> l(mutex);
      valid = false;
      stopped = true;
      status("ERROR", e.what());
      RCLCPP_ERROR(node.get_logger(), "Localization stopped: %s", e.what());
    }
  }
  void finish() {
    if (!enabled)
      return;
    {
      std::lock_guard<std::mutex> l(mutex);
      stopped = true;
      pending.reset();
    }
    cv.notify_one();
    if (worker.joinable())
      worker.join();
  }
  ~Impl() { finish(); }
};
MapLocalizer::MapLocalizer(rclcpp::Node &n)
    : impl_(std::make_unique<Impl>(n)) {}
MapLocalizer::~MapLocalizer() = default;
void MapLocalizer::submit(const NavState &s, const BASIC::CloudPtr &c) {
  impl_->submit(s, c);
}
void MapLocalizer::finish() { impl_->finish(); }
} // namespace LI2Sup
