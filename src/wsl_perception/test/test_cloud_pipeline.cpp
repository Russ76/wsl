// Copyright 2026 Azer Ben Ali
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

#include "support/synthetic_scan.hpp"
#include "wsl_perception/cloud_pipeline.hpp"

namespace wsl_perception
{
namespace
{

using test::Box;
using test::makeCloud;
using test::Scene;
using test::simulateScan;

// LiDAR mounted 0.5 m above the target frame origin.
const Eigen::Isometry3f kSensorToBase = [] {
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.translation() = Eigen::Vector3f(0.0f, 0.0f, 0.5f);
  return t;
}();

// Box given in the base frame (z = 0 on the ground), converted to the sensor frame.
Box baseBox(Eigen::Vector3f min, Eigen::Vector3f max)
{
  return Box{min - kSensorToBase.translation(), max - kSensorToBase.translation()};
}

void expectInvalid(const std::function<void(PipelineConfig &)> & mutate)
{
  PipelineConfig config;
  mutate(config);
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(PipelineConfig, DefaultsAreValid) { EXPECT_NO_THROW(PipelineConfig{}.validate()); }

TEST(PipelineConfig, RejectsInconsistentValues)
{
  expectInvalid([](auto & c) { c.voxel_leaf_size_m = -0.1; });
  expectInvalid([](auto & c) { c.voxel_leaf_size_m = 0.0; });
  expectInvalid([](auto & c) { c.min_range_m = -1.0; });
  expectInvalid([](auto & c) {
    c.min_range_m = 30.0;
    c.max_range_m = 25.0;
  });
  expectInvalid([](auto & c) { c.roi_min_z_m = 3.0; });
  expectInvalid([](auto & c) { c.roi_min_x_m = c.roi_max_x_m; });
  expectInvalid([](auto & c) { c.cluster_tolerance_m = 0.0; });
  expectInvalid([](auto & c) { c.cluster_min_points = 0; });
  expectInvalid([](auto & c) {
    c.cluster_max_points = 5;
    c.cluster_min_points = 6;
  });
  expectInvalid([](auto & c) { c.ground_max_tilt_rad = 2.0; });
  expectInvalid([](auto & c) { c.ground_ransac_iterations = 0; });
  expectInvalid([](auto & c) { c.ground_max_height_m = -1.0; });
  // 40 m x 40 m x 2.8 m at 1 mm needs 4.5e12 voxels: overflows PCL's int32 index.
  expectInvalid([](auto & c) { c.voxel_leaf_size_m = 0.001; });
}

TEST(CloudPipeline, ConstructorValidates)
{
  PipelineConfig config;
  config.cluster_tolerance_m = -1.0;
  EXPECT_THROW(CloudPipeline{config}, std::invalid_argument);
}

TEST(CloudPipeline, PropagatesConversionErrors)
{
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto result = pipeline.process(makeCloud({{1, 0, 0}}, ""), kSensorToBase);
  EXPECT_EQ(result.error, CloudError::kMissingFrameId);
  EXPECT_TRUE(result.filtered->empty());
}

TEST(CloudPipeline, SinglePointProducesNoGroundAndNoCluster)
{
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto result = pipeline.process(makeCloud({{2, 0, 0}}), kSensorToBase);
  ASSERT_EQ(result.error, CloudError::kNone);
  ASSERT_EQ(result.filtered->size(), 1u);
  EXPECT_NEAR((*result.filtered)[0].z, 0.5f, 1e-6f);  // transformed into the base frame
  EXPECT_FALSE(result.ground_found);
  EXPECT_EQ(result.obstacles->size(), 1u);
  EXPECT_TRUE(result.clusters.empty());
}

TEST(CloudPipeline, IdenticalPointsCollapseToOneVoxel)
{
  const CloudPipeline pipeline{PipelineConfig{}};
  const std::vector<Eigen::Vector3f> points(1000, Eigen::Vector3f(2.0f, 1.0f, 0.0f));
  const auto result = pipeline.process(makeCloud(points), kSensorToBase);
  ASSERT_EQ(result.filtered->size(), 1u);
}

TEST(CloudPipeline, RoiIsAppliedInTargetFrame)
{
  PipelineConfig config;
  config.roi_max_z_m = 1.0;
  const CloudPipeline pipeline{config};
  // Sensor-frame z = 0.4 is base-frame z = 0.9 (kept); z = 0.6 is 1.1 (cropped).
  const auto result = pipeline.process(makeCloud({{2, 0, 0.4f}, {2, 1, 0.6f}}), kSensorToBase);
  ASSERT_EQ(result.filtered->size(), 1u);
  EXPECT_NEAR((*result.filtered)[0].z, 0.9f, 1e-5f);
}

TEST(CloudPipeline, RemovesGroundAndClustersObstacle)
{
  Scene scene;
  scene.boxes.push_back(baseBox({3.0f, -0.5f, 0.0f}, {3.6f, 0.5f, 1.2f}));
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto result = pipeline.process(simulateScan(scene), kSensorToBase);

  ASSERT_TRUE(result.ground_found);
  EXPECT_NEAR(result.ground_plane.z(), 1.0f, 1e-3f);
  EXPECT_NEAR(result.ground_plane.w(), 0.0f, 0.01f);
  EXPECT_GT(result.ground_points, result.obstacles->size());
  for (const auto & p : *result.obstacles) {
    EXPECT_GT(p.z, PipelineConfig{}.ground_distance_threshold_m);
  }
  ASSERT_EQ(result.clusters.size(), 1u);
  const Cluster & c = result.clusters.front();
  EXPECT_NEAR(c.min.x(), 3.0f, 0.1f);  // near face of the box
  EXPECT_NEAR(c.centroid.y(), 0.0f, 0.1f);
  EXPECT_LE(c.max.z(), 1.2f + 1e-3f);
}

TEST(CloudPipeline, RejectsTiltedPlaneAsGround)
{
  // A 20 deg ramp: within the candidate height band but too steep to be floor.
  std::vector<Eigen::Vector3f> points;
  const float slope = std::tan(20.0f * static_cast<float>(M_PI) / 180.0f);
  for (float x = 1.0f; x < 1.6f; x += 0.02f) {
    for (float y = -2.0f; y < 2.0f; y += 0.1f) {
      points.emplace_back(x, y, (x - 1.3f) * slope - 0.5f);  // base-frame z within +/-0.11 m
    }
  }
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto result = pipeline.process(makeCloud(points), kSensorToBase);
  EXPECT_FALSE(result.ground_found);
  EXPECT_EQ(result.obstacles->size(), result.filtered->size());
}

TEST(CloudPipeline, ShelfDeckIsNotMistakenForGround)
{
  // A wide deck 1.0 m up yields more points than the visible floor, but lies
  // outside the ground candidate band.
  Scene scene;
  scene.boxes.push_back(baseBox({1.5f, -6.0f, 0.95f}, {6.0f, 6.0f, 1.0f}));
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto result = pipeline.process(simulateScan(scene), kSensorToBase);
  ASSERT_TRUE(result.ground_found);
  EXPECT_NEAR(result.ground_plane.w(), 0.0f, 0.01f);
}

TEST(CloudPipeline, NoGroundVisibleMeansEverythingIsObstacle)
{
  Scene scene;
  scene.has_ground = false;
  scene.boxes.push_back(baseBox({2.0f, -3.0f, 0.5f}, {2.5f, 3.0f, 2.0f}));
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto result = pipeline.process(simulateScan(scene), kSensorToBase);
  EXPECT_FALSE(result.ground_found);
  EXPECT_EQ(result.ground_points, 0u);
  EXPECT_EQ(result.obstacles->size(), result.filtered->size());
}

PipelineResult clusterLine(float spacing, std::size_t count, const PipelineConfig & config)
{
  PipelineResult result;
  for (std::size_t i = 0; i < count; ++i) {
    result.filtered->push_back(pcl::PointXYZ(2.0f + spacing * i, 0.0f, 1.0f));
  }
  CloudPipeline{config}.segment(result);
  return result;
}

TEST(CloudPipeline, ClusterToleranceBoundary)
{
  PipelineConfig config;
  config.cluster_min_points = 2;
  // Two groups of 5 points with 0.05 m spacing, separated by `gap`.
  auto two_groups = [&](float gap) {
    PipelineResult result;
    for (int g = 0; g < 2; ++g) {
      for (int i = 0; i < 5; ++i) {
        result.filtered->push_back(pcl::PointXYZ(2.0f + g * (0.2f + gap) + 0.05f * i, 0, 1));
      }
    }
    CloudPipeline{config}.segment(result);
    return result.clusters.size();
  };
  EXPECT_EQ(two_groups(0.9f * config.cluster_tolerance_m), 1u);
  EXPECT_EQ(two_groups(1.1f * config.cluster_tolerance_m), 2u);
}

TEST(CloudPipeline, ClusterSizeLimitsAreInclusive)
{
  PipelineConfig config;
  config.cluster_min_points = 5;
  config.cluster_max_points = 8;
  EXPECT_EQ(clusterLine(0.05f, 4, config).clusters.size(), 0u);
  EXPECT_EQ(clusterLine(0.05f, 5, config).clusters.size(), 1u);
  EXPECT_EQ(clusterLine(0.05f, 8, config).clusters.size(), 1u);
  EXPECT_EQ(clusterLine(0.05f, 9, config).clusters.size(), 0u);
}

TEST(CloudPipeline, SeparatesMultipleObstacles)
{
  Scene scene;
  scene.boxes.push_back(baseBox({3.0f, -0.4f, 0.0f}, {3.5f, 0.4f, 1.0f}));
  scene.boxes.push_back(baseBox({-4.0f, 2.0f, 0.0f}, {-3.0f, 3.0f, 0.8f}));
  scene.boxes.push_back(baseBox({0.5f, -6.0f, 0.0f}, {1.5f, -5.0f, 1.5f}));
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto result = pipeline.process(simulateScan(scene), kSensorToBase);
  EXPECT_EQ(result.clusters.size(), 3u);
}

TEST(CloudPipeline, IsDeterministic)
{
  Scene scene;
  scene.boxes.push_back(baseBox({3.0f, -0.4f, 0.0f}, {3.5f, 0.4f, 1.0f}));
  const auto msg = simulateScan(scene);
  const CloudPipeline pipeline{PipelineConfig{}};
  const auto a = pipeline.process(msg, kSensorToBase);
  const auto b = pipeline.process(msg, kSensorToBase);
  EXPECT_EQ(a.ground_plane, b.ground_plane);
  EXPECT_EQ(a.ground_points, b.ground_points);
  ASSERT_EQ(a.clusters.size(), b.clusters.size());
  EXPECT_EQ(a.clusters[0].centroid, b.clusters[0].centroid);
}

}  // namespace
}  // namespace wsl_perception
