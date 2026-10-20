#include "lio/loop_closure.h"
#include "lio/loop_pose_graph.h"
#include "lio/params.h"
#include "lio/place_index.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

namespace LI2Sup {
namespace {
using Pose = Eigen::Isometry3d;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Descriptor = Eigen::Matrix<double, 20, 60>;
builtin_interfaces::msg::Time stamp(double t) {
  return rclcpp::Time(static_cast<int64_t>(t * 1e9));
}
Pose poseOf(const NavState &s) {
  Pose t = Pose::Identity();
  // Frontend rotations are stored in float. Project each input onto SO(3)
  // before composing Isometries; transpose is an inverse only for rigid R.
  t.linear() =
      Eigen::Quaterniond(s.R.R_.cast<double>()).normalized().toRotationMatrix();
  t.translation() = s.p.cast<double>();
  return t;
}
geometry_msgs::msg::Pose poseMsg(const Pose &t) {
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
Cloud::Ptr filtered(const Cloud::ConstPtr &c, double leaf) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZI> f;
  f.setLeafSize(leaf, leaf, leaf);
  f.setInputCloud(c);
  f.filter(*out);
  return out;
}
Descriptor describe(const Cloud &cloud, double radius) {
  Descriptor d = Descriptor::Zero();
  for (const auto &p : cloud) {
    double r = std::hypot(p.x, p.y);
    if (r < 1.0 || r >= radius)
      continue;
    int ring = std::min(19, int(r / radius * 20));
    int sector =
        std::min(59, int((std::atan2(p.y, p.x) + M_PI) / (2 * M_PI) * 60));
    // Occupancy avoids dependence on mounting height / world gravity offset.
    d(ring, sector) += 1.0;
  }
  return d.unaryExpr([](double x) { return std::log1p(x); });
}
std::pair<double, int> descriptorDistance(const Descriptor &a,
                                          const Descriptor &b) {
  double best = 1.;
  int shift = 0;
  for (int s = 0; s < 60; ++s) {
    double sum = 0;
    int count = 0;
    for (int c = 0; c < 60; ++c) {
      auto u = a.col(c);
      auto v = b.col((c + s) % 60);
      double n = u.norm() * v.norm();
      if (n > 1e-9) {
        sum += u.dot(v) / n;
        ++count;
      }
    }
    double dist = count >= 15 ? 1. - sum / count : 1.;
    if (dist < best) {
      best = dist;
      shift = s;
    }
  }
  return {best, shift};
}
using Edge = LoopPoseGraph::Edge;
} // namespace
struct LoopClosure::Impl {
  rclcpp::Node &node;
  bool enabled = false, stopped = false;
  double key_distance, key_angle, key_time, min_time, radius, leaf, fitness,
      overlap, max_correction, descriptor_threshold, map_period,
      min_eigen_ratio;
  int submap_frames, max_candidates, queue_limit, iterations, min_points,
      max_keyframes, gicp_neighbors, gicp_optimizer_iterations;
  std::string map_frame, odom_frame, child_frame, place_index_path,
      place_index_reference;
  bool save_place_index = true;
  struct Frame {
    double time;
    Pose raw, optimized;
    Cloud::Ptr cloud;
    Descriptor descriptor;
  };
  std::vector<Frame> frames;
  std::vector<Edge> edges;
  std::unique_ptr<LoopPoseGraph> graph;
  std::deque<Frame> queue;
  std::mutex mutex;
  std::condition_variable cv;
  std::thread worker;
  Pose correction = Pose::Identity(), last_submitted = Pose::Identity();
  double last_submit_time = -1, last_map_time = -1;
  size_t loops = 0, dropped = 0;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  template <class T> T param(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description =
        "Loop backend startup parameter; restart the node to change it";
    return node.declare_parameter<T>("lio.loop." + name, value, descriptor);
  }
  explicit Impl(rclcpp::Node &n) : node(n) {
    enabled = param("enabled", false);
    if (!enabled)
      return;
    place_index_path = param<std::string>("place_index_path", "");
    place_index_reference = param<std::string>("place_index_reference_map", "");
    save_place_index = param("save_place_index", true);
    for (auto *path : {&place_index_path, &place_index_reference}) {
      if (path->rfind("~/", 0) == 0) {
        const char *home = std::getenv("HOME");
        if (!home)
          throw std::invalid_argument("HOME is unset");
        *path = std::string(home) + path->substr(1);
      }
    }
    key_distance = param("keyframe_distance", 1.0);
    key_angle = param("keyframe_angle_deg", 15.0) * M_PI / 180.;
    key_time = param("keyframe_max_time", 5.0);
    min_time = param("min_time_separation", 30.0);
    radius = param("search_radius", 8.0);
    leaf = param("voxel_size", 0.4);
    fitness = param("max_fitness", 0.15);
    overlap = param("min_overlap", 0.65);
    max_correction = param("max_correction", 10.0);
    descriptor_threshold = param("descriptor_threshold", 0.22);
    submap_frames = param("submap_frames", 5);
    max_candidates = param("max_candidates", 3);
    queue_limit = param("queue_limit", 16);
    // Keep the old iteration key as a deprecated fallback for existing
    // overlays.
    int legacy_iterations = param("icp_iterations", 60);
    iterations = param("gicp_iterations", legacy_iterations);
    gicp_neighbors = param("gicp_neighbors", 20);
    gicp_optimizer_iterations = param("gicp_optimizer_iterations", 20);
    min_points = param("min_points", 100);
    max_keyframes = param("max_keyframes", 2000);
    min_eigen_ratio = param("min_eigen_ratio", 0.0001);
    map_period = param("map_publish_period", 3.0);
    map_frame = param<std::string>("map_frame", "loop_map");
    odom_frame = g_odom_frame_id;
    child_frame = g_odom_child_frame_id;
    const std::array<double, 13> numeric = {key_distance,
                                            key_angle,
                                            key_time,
                                            min_time,
                                            radius,
                                            leaf,
                                            fitness,
                                            overlap,
                                            max_correction,
                                            descriptor_threshold,
                                            map_period,
                                            min_eigen_ratio,
                                            double(submap_frames)};
    if (!std::all_of(numeric.begin(), numeric.end(),
                     [](double v) { return std::isfinite(v); }) ||
        key_distance <= 0 || key_angle <= 0 || key_time <= 0 ||
        min_time <= key_time || radius <= 0 || leaf <= 0 || fitness <= 0 ||
        overlap <= 0 || overlap > 1 || max_correction <= 0 ||
        descriptor_threshold <= 0 || descriptor_threshold >= 1 ||
        max_keyframes < 2 || min_eigen_ratio <= 0 || min_eigen_ratio >= 1 ||
        submap_frames < 0 || max_candidates < 1 || queue_limit < 1 ||
        iterations < 1 || gicp_neighbors < 3 || gicp_optimizer_iterations < 1 ||
        min_points < gicp_neighbors || map_period <= 0 || map_frame.empty() ||
        map_frame == odom_frame)
      throw std::invalid_argument("Invalid lio.loop parameters (map_frame must "
                                  "differ from odom frame)");
    graph = std::make_unique<LoopPoseGraph>();
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub =
        node.create_publisher<nav_msgs::msg::Path>("/lio/loop/path", qos);
    map_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/loop/map", qos);
    edge_pub = node.create_publisher<visualization_msgs::msg::Marker>(
        "/lio/loop/constraints", qos);
    odom_pub =
        node.create_publisher<nav_msgs::msg::Odometry>("/lio/loop/odom", 10);
    tf = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    worker = std::thread([this] { run(); });
    RCLCPP_INFO(node.get_logger(),
                "Loop closure enabled: PCL GICP + GTSAM iSAM2, frame %s",
                map_frame.c_str());
  }
  void submit(const NavState &state, const BASIC::CloudPtr &cloud) {
    if (!enabled)
      return;
    Pose raw = poseOf(state), corr;
    {
      std::lock_guard<std::mutex> lock(mutex);
      corr = correction;
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp(state.timestamp);
    odom.header.frame_id = map_frame;
    odom.child_frame_id = child_frame;
    odom.pose.pose = poseMsg(corr * raw);
    Eigen::Vector3d v = corr.linear() * state.v.cast<double>();
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();
    odom_pub->publish(odom);
    geometry_msgs::msg::TransformStamped tr;
    tr.header = odom.header;
    tr.child_frame_id = odom_frame;
    auto p = poseMsg(corr);
    tr.transform.translation.x = p.position.x;
    tr.transform.translation.y = p.position.y;
    tr.transform.translation.z = p.position.z;
    tr.transform.rotation = p.orientation;
    tf->sendTransform(tr);
    Pose delta = last_submitted.inverse() * raw;
    if (last_submit_time >= 0 &&
        (state.timestamp <= last_submit_time ||
         (delta.translation().norm() < key_distance &&
          Eigen::AngleAxisd(delta.linear()).angle() < key_angle &&
          state.timestamp - last_submit_time < key_time)))
      return;
    Cloud::Ptr copy(new Cloud);
    copy->reserve(cloud->size());
    for (const auto &pt : *cloud)
      if (pcl::isFinite(pt)) {
        pcl::PointXYZI p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        copy->push_back(p);
      }
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopped)
        return;
      if (int(queue.size()) >= queue_limit) {
        ++dropped;
        RCLCPP_WARN_THROTTLE(
            node.get_logger(), *node.get_clock(), 5000,
            "Loop queue full; skipping keyframe (local LIO unaffected)");
        return;
      }
      queue.push_back(
          {state.timestamp, raw, corr * raw, copy, Descriptor::Zero()});
    }
    last_submitted = raw;
    last_submit_time = state.timestamp;
    cv.notify_one();
  }
  Cloud::Ptr submap(int id) {
    Cloud::Ptr out(new Cloud);
    // Express the historical submap in the candidate keyframe's body
    // coordinates.
    for (int j = std::max(0, id - submap_frames);
         j <= std::min(int(frames.size()) - 2, id + submap_frames); ++j) {
      if (frames.back().time - frames[j].time < min_time)
        continue;
      Cloud part;
      Pose relative = frames[id].optimized.inverse() * frames[j].optimized;
      pcl::transformPointCloud(*frames[j].cloud, part,
                               relative.matrix().cast<float>());
      *out += part;
    }
    return filtered(out, leaf);
  }
  bool verify(int candidate, Pose &measurement) {
    const auto &current = frames.back();
    const auto &old = frames[candidate];
    auto target = submap(candidate);
    if (int(target->size()) < min_points ||
        int(current.cloud->size()) < min_points)
      return false;
    Pose initial = old.optimized.inverse() * current.optimized;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setCorrespondenceRandomness(gicp_neighbors);
    gicp.setMaximumOptimizerIterations(gicp_optimizer_iterations);
    gicp.setInputSource(current.cloud);
    gicp.setInputTarget(target);
    gicp.setMaximumIterations(iterations);
    gicp.setMaxCorrespondenceDistance(std::max(1.0, leaf * 3));
    gicp.setTransformationEpsilon(1e-7);
    gicp.setEuclideanFitnessEpsilon(1e-7);
    Cloud aligned;
    gicp.align(aligned, initial.matrix().cast<float>());
    if (!gicp.hasConverged() || !gicp.getFinalTransformation().allFinite())
      return false;
    Pose result = Pose::Identity();
    result.matrix() = gicp.getFinalTransformation().cast<double>();
    result.linear() =
        Eigen::Quaterniond(result.linear()).normalized().toRotationMatrix();
    Pose change = initial.inverse() * result;
    if (change.translation().norm() > max_correction ||
        Eigen::AngleAxisd(change.linear()).angle() > M_PI / 4)
      return false;
    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(target);
    size_t count = 0;
    double squared_error = 0;
    std::vector<int> idx(1);
    std::vector<float> dist(1);
    double threshold = std::max(0.5, leaf * 1.5);
    threshold *= threshold;
    for (const auto &p : aligned)
      if (tree.nearestKSearch(p, 1, idx, dist) > 0 && dist[0] < threshold) {
        ++count;
        squared_error += dist[0];
      }
    double ratio = double(count) / aligned.size();
    double mse = count ? squared_error / count : 1e9;
    if (ratio < overlap || mse > fitness)
      return false;
    // Reverse overlap against a single historical scan rejects one-way fits to
    // large submaps.
    Cloud old_aligned;
    Pose reverse = result.inverse();
    pcl::transformPointCloud(*old.cloud, old_aligned,
                             reverse.matrix().cast<float>());
    tree.setInputCloud(current.cloud);
    size_t reverse_count = 0;
    for (const auto &p : old_aligned)
      if (tree.nearestKSearch(p, 1, idx, dist) > 0 && dist[0] < threshold)
        ++reverse_count;
    if (double(reverse_count) / old_aligned.size() < overlap * 0.7)
      return false;
    // A low GICP score alone cannot reject sliding along a wall/corridor.
    // Estimate normalized point-to-plane information from local target
    // neighborhoods.
    Eigen::Matrix<double, 6, 6> information =
        Eigen::Matrix<double, 6, 6>::Zero();
    std::vector<int> neighbors(8);
    std::vector<float> distances(8);
    tree.setInputCloud(target);
    int planes = 0;
    const size_t stride = std::max<size_t>(1, aligned.size() / 500);
    for (size_t i = 0; i < aligned.size(); i += stride) {
      const auto &p = aligned[i];
      if (tree.nearestKSearch(p, 8, neighbors, distances) < 8 ||
          distances[0] > threshold)
        continue;
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (int k : neighbors)
        mean += (*target)[k].getVector3fMap().cast<double>();
      mean /= 8.;
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (int k : neighbors) {
        Eigen::Vector3d d = (*target)[k].getVector3fMap().cast<double>() - mean;
        cov += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
      if (eig.eigenvalues()[1] < 1e-8 ||
          eig.eigenvalues()[0] > .15 * eig.eigenvalues()[1])
        continue;
      Eigen::Vector3d normal = eig.eigenvectors().col(0);
      Eigen::Matrix<double, 6, 1> jac;
      Eigen::Vector3d point =
          p.getVector3fMap().cast<double>() - result.translation();
      jac.head<3>() = normal;
      jac.tail<3>() = point.cross(normal) / 10.;
      information += jac * jac.transpose();
      ++planes;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(information);
    if (planes < 30 ||
        eig.eigenvalues()[0] < min_eigen_ratio * eig.eigenvalues()[5])
      return false;
    measurement = result;
    RCLCPP_INFO(
        node.get_logger(),
        "Loop verified %d -> %zu: overlap=%.3f mse=%.5f correction=%.3fm",
        candidate, frames.size() - 1, ratio, mse, change.translation().norm());
    return true;
  }
  bool optimize() {
    std::vector<Pose> optimized;
    optimized.reserve(frames.size());
    for (const auto &f : frames)
      optimized.push_back(f.optimized);
    double initial_error = 0., final_error = 0.;
    if (!graph->optimize(optimized, edges, initial_error, final_error))
      return false;
    for (size_t i = 0; i < frames.size(); ++i)
      frames[i].optimized = optimized[i];
    {
      std::lock_guard<std::mutex> lock(mutex);
      correction = frames.back().optimized * frames.back().raw.inverse();
    }
    RCLCPP_INFO(
        node.get_logger(),
        "Pose graph optimized: %zu keyframes, %zu loops, cost %.3f -> %.3f",
        frames.size(), loops + 1, initial_error, final_error);
    return true;
  }
  Cloud::Ptr globalMap() {
    Cloud::Ptr all(new Cloud);
    for (const auto &f : frames) {
      Cloud part;
      pcl::transformPointCloud(*f.cloud, part,
                               f.optimized.matrix().cast<float>());
      *all += part;
    }
    return filtered(all, leaf);
  }
  void publish(bool force) {
    if (frames.empty() ||
        !rclcpp::ok(node.get_node_base_interface()->get_context()))
      return;
    const double time = frames.back().time;
    if (!force && last_map_time >= 0 && time - last_map_time < map_period)
      return;
    last_map_time = time;
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame;
    path.header.stamp = stamp(time);
    for (const auto &f : frames) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.header.stamp = stamp(f.time);
      p.pose = poseMsg(f.optimized);
      path.poses.push_back(p);
    }
    path_pub->publish(path);
    auto all = globalMap();
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*all, msg);
    msg.header = path.header;
    map_pub->publish(msg);
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "loop_edges";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.;
    marker.scale.x = .12;
    marker.color.r = 1.;
    marker.color.g = .2;
    marker.color.a = 1.;
    for (const auto &e : edges)
      if (e.loop)
        for (int id : {e.a, e.b}) {
          geometry_msgs::msg::Point p;
          auto t = frames[id].optimized.translation();
          p.x = t.x();
          p.y = t.y();
          p.z = t.z();
          marker.points.push_back(p);
        }
    edge_pub->publish(marker);
  }
  void process(Frame frame) {
    if (int(frames.size()) >= max_keyframes) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 30000,
                           "Loop keyframe memory limit reached; increase "
                           "lio.loop.max_keyframes for longer runs");
      return;
    }
    frame.cloud = filtered(frame.cloud, leaf);
    if (int(frame.cloud->size()) < min_points)
      return;
    frame.descriptor = describe(*frame.cloud, 60.);
    if (!frames.empty())
      frame.optimized =
          frames.back().optimized * frames.back().raw.inverse() * frame.raw;
    int cur = frames.size();
    frames.push_back(std::move(frame));
    if (cur > 0)
      edges.push_back({cur - 1, cur,
                       frames[cur - 1].raw.inverse() * frames[cur].raw, false});
    std::vector<std::pair<double, int>> candidates;
    for (int j = 0; j < cur; ++j) {
      if (frames[cur].time - frames[j].time < min_time)
        continue;
      double d = (frames[j].optimized.translation() -
                  frames[cur].optimized.translation())
                     .norm();
      if (d > radius)
        continue;
      double score =
          descriptorDistance(frames[cur].descriptor, frames[j].descriptor)
              .first;
      if (score < descriptor_threshold)
        candidates.emplace_back(score, j);
    }
    std::sort(candidates.begin(), candidates.end());
    bool closed = false;
    int tested = 0;
    for (auto [score, j] : candidates) {
      if (tested++ >= max_candidates)
        break;
      // Do not add repeated constraints between neighboring observations of the
      // same revisit.
      bool duplicate = false;
      for (const auto &e : edges)
        if (e.loop && std::abs(e.a - j) <= submap_frames &&
            cur - e.b <= submap_frames) {
          duplicate = true;
          break;
        }
      if (duplicate)
        continue;
      Pose z;
      if (!verify(j, z))
        continue;
      edges.push_back({j, cur, z, true});
      if (optimize()) {
        ++loops;
        closed = true;
      } else {
        edges.pop_back();
        RCLCPP_WARN(node.get_logger(), "Rejected graph update: %s",
                    graph->lastError().c_str());
      }
      break;
    }
    publish(closed);
  }
  void run() {
    try {
      while (true) {
        Frame frame;
        {
          std::unique_lock<std::mutex> lock(mutex);
          cv.wait(lock, [this] { return stopped || !queue.empty(); });
          if (queue.empty() && stopped)
            break;
          frame = std::move(queue.front());
          queue.pop_front();
        }
        process(std::move(frame));
      }
      publish(true);
      if (!frames.empty() && (g_save_map || !place_index_path.empty())) {
        auto final_map = globalMap();
        if (g_save_map) {
          std::filesystem::create_directories(g_save_map_dir);
          if (pcl::io::savePCDFileBinary(g_save_map_dir + "/loop_map.pcd",
                                         *final_map) != 0)
            throw std::runtime_error("Failed to save loop map");
        }
        if (save_place_index) {
          std::vector<PlaceEntry> entries;
          Cloud::Ptr reference;
          pcl::KdTreeFLANN<Point> reference_tree;
          if (!place_index_reference.empty()) {
            reference.reset(new Cloud);
            if (pcl::io::loadPCDFile<Point>(place_index_reference,
                                            *reference) != 0)
              throw std::runtime_error("Cannot load index reference map");
            reference_tree.setInputCloud(reference);
          }
          for (size_t i = 0; i < frames.size(); ++i) {
            const auto &frame = frames[i];
            Pose pose = frame.optimized;
            if (reference) {
              // Offline index migration for an existing PCD. Do not modify the
              // PCD.
              if (i % 3)
                continue;
              std::vector<int> indices;
              std::vector<float> distances;
              Point center;
              center.x = pose.translation().x();
              center.y = pose.translation().y();
              center.z = pose.translation().z();
              reference_tree.radiusSearch(center, 35., indices, distances);
              Cloud::Ptr crop(new Cloud);
              for (int k : indices)
                crop->push_back(reference->points[k]);
              if (crop->size() < 100)
                continue;
              pcl::GeneralizedIterativeClosestPoint<Point, Point> g;
              g.setInputSource(frame.cloud);
              g.setInputTarget(crop);
              g.setMaximumIterations(40);
              g.setMaxCorrespondenceDistance(3.);
              Cloud unused;
              g.align(unused, pose.matrix().cast<float>());
              if (!g.hasConverged() || !g.getFinalTransformation().allFinite())
                continue;
              Pose refined = Pose::Identity();
              refined.matrix() = g.getFinalTransformation().cast<double>();
              refined.linear() = Eigen::Quaterniond(refined.linear())
                                     .normalized()
                                     .toRotationMatrix();
              int count = 0;
              double error = 0;
              std::vector<int> id(1);
              std::vector<float> dist(1);
              for (const auto &p : *frame.cloud) {
                Point q;
                Eigen::Vector3d v = refined * p.getVector3fMap().cast<double>();
                q.x = v.x();
                q.y = v.y();
                q.z = v.z();
                if (reference_tree.nearestKSearch(q, 1, id, dist) > 0 &&
                    dist[0] < .25) {
                  ++count;
                  error += dist[0];
                }
              }
              if (count < 100 || double(count) / frame.cloud->size() < .7 ||
                  error / count > .1 ||
                  (refined.translation() - pose.translation()).norm() > 6)
                continue;
              pose = refined;
            }
            entries.push_back(
                describePlace(*frame.cloud, pose.linear(), pose.translation()));
          }
          if (entries.empty())
            throw std::runtime_error("No verified place entries to save");
          const std::string file = place_index_path.empty()
                                       ? g_save_map_dir + "/loop_map.pcd.index"
                                       : place_index_path;
          savePlaceIndex(file,
                         mapFingerprint(reference ? *reference : *final_map),
                         entries);
          RCLCPP_INFO(node.get_logger(), "Place index saved: %zu entries to %s",
                      entries.size(), file.c_str());
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node.get_logger(),
                   "Loop backend stopped: %s (local LIO continues)", e.what());
      std::lock_guard<std::mutex> lock(mutex);
      stopped = true;
      queue.clear();
    }
    RCLCPP_INFO(node.get_logger(),
                "Loop backend finished: %zu keyframes, %zu loops, %zu dropped",
                frames.size(), loops, dropped);
  }
  void finish() {
    if (!enabled)
      return;
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopped = true;
    }
    cv.notify_one();
    if (worker.joinable())
      worker.join();
  }
  ~Impl() { finish(); }
};
LoopClosure::LoopClosure(rclcpp::Node &n) : impl_(std::make_unique<Impl>(n)) {}
LoopClosure::~LoopClosure() = default;
void LoopClosure::submit(const NavState &s, const BASIC::CloudPtr &c) {
  impl_->submit(s, c);
}
void LoopClosure::finish() { impl_->finish(); }
} // namespace LI2Sup
