// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
//
// Measures the perception pipeline on a synthetic warehouse aisle scan with the
// same scan pattern as the simulated LiDAR (16 x 900). Runs in isolation from
// ROS and Gazebo, so the numbers reflect algorithm cost only. Prints JSON.
//   pipeline_benchmark [iterations] [channels] [samples]
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "support/synthetic_scan.hpp"
#include "wsl_perception/cloud_pipeline.hpp"

using wsl_perception::CloudPipeline;
using wsl_perception::PipelineConfig;
using wsl_perception::PipelineResult;
using wsl_perception::test::Box;
using wsl_perception::test::Scene;

namespace
{

// Robot in a 3 m aisle between two rack rows; scene coordinates are in the
// sensor frame with the sensor 0.66 m above the floor.
Scene warehouseAisle()
{
  const float floor_z = -0.66f;
  Scene scene;
  scene.ground_z = floor_z;
  for (float x = -13.5f; x < 13.5f; x += 2.7f) {
    for (float side : {1.0f, -1.0f}) {
      const float y_near = side * 1.5f;
      const float y_far = side * 2.6f;
      const Eigen::Vector3f lo(x, std::min(y_near, y_far), floor_z);
      const Eigen::Vector3f hi(x + 0.08f, std::max(y_near, y_far), floor_z + 2.4f);
      scene.boxes.push_back({lo, hi});  // upright
      scene.boxes.push_back(
        {Eigen::Vector3f(x + 0.2f, lo.y(), floor_z + 0.13f),
         Eigen::Vector3f(x + 1.3f, hi.y(), floor_z + 0.88f)});
    }
  }
  scene.boxes.push_back(
    {{4.0f, -0.6f, floor_z}, {5.2f, 0.4f, floor_z + 1.25f}});  // pallet in the aisle
  scene.boxes.push_back({{-20.0f, -20.0f, floor_z}, {-19.7f, 20.0f, floor_z + 5.0f}});  // end walls
  scene.boxes.push_back({{19.7f, -20.0f, floor_z}, {20.0f, 20.0f, floor_z + 5.0f}});
  return scene;
}

struct Summary
{
  double mean, p50, p95, max;
};

Summary summarise(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  double sum = 0.0;
  for (double x : v) {
    sum += x;
  }
  auto pct = [&](double q) { return v[static_cast<std::size_t>(q * (v.size() - 1))]; };
  return {sum / v.size(), pct(0.5), pct(0.95), v.back()};
}

}  // namespace

int main(int argc, char ** argv)
{
  const int iterations = argc > 1 ? std::max(1, std::atoi(argv[1])) : 200;
  wsl_perception::test::ScanPattern pattern;
  if (argc > 2) {
    pattern.channels = std::max(1, std::atoi(argv[2]));
  }
  if (argc > 3) {
    pattern.samples = std::max(1, std::atoi(argv[3]));
  }

  const auto msg = wsl_perception::test::simulateScan(warehouseAisle(), pattern);
  Eigen::Isometry3f sensor_to_base = Eigen::Isometry3f::Identity();
  sensor_to_base.translation().z() = 0.5f;
  const CloudPipeline pipeline{PipelineConfig{}};

  std::vector<double> extract, roi, voxel, ground, cluster, total;
  PipelineResult last;
  pipeline.process(msg, sensor_to_base);  // warm-up: page faults, allocator
  for (int i = 0; i < iterations; ++i) {
    last = pipeline.process(msg, sensor_to_base);
    extract.push_back(last.timings.extract_ms);
    roi.push_back(last.timings.transform_roi_ms);
    voxel.push_back(last.timings.voxel_ms);
    ground.push_back(last.timings.ground_ms);
    cluster.push_back(last.timings.cluster_ms);
    total.push_back(last.timings.total_ms);
  }

  std::printf("{\n  \"iterations\": %d,\n", iterations);
  std::printf(
    "  \"scan\": {\"channels\": %d, \"samples\": %d},\n", pattern.channels, pattern.samples);
  std::printf(
    "  \"points\": {\"raw\": %zu, \"in_range\": %zu, \"filtered\": %zu, \"obstacles\": %zu, "
    "\"ground\": %zu, \"clusters\": %zu},\n",
    last.extraction.input_points,
    last.extraction.input_points - last.extraction.non_finite - last.extraction.out_of_range,
    last.filtered->size(), last.obstacles->size(), last.ground_points, last.clusters.size());
  std::printf("  \"stage_ms\": {\n");
  const std::pair<const char *, std::vector<double> *> stages[] = {
    {"extract", &extract}, {"transform_roi", &roi}, {"voxel", &voxel},
    {"ground", &ground},   {"cluster", &cluster},   {"total", &total}};
  for (std::size_t i = 0; i < std::size(stages); ++i) {
    const Summary s = summarise(*stages[i].second);
    std::printf(
      "    \"%s\": {\"mean\": %.3f, \"p50\": %.3f, \"p95\": %.3f, \"max\": %.3f}%s\n",
      stages[i].first, s.mean, s.p50, s.p95, s.max, i + 1 < std::size(stages) ? "," : "");
  }
  std::printf("  }\n}\n");
  return 0;
}
