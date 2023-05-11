#include "dense-reconstruction/balm/bavoxel.h"
#include "dense-reconstruction/dense-reconstruction-plugin.h"
#include "dense-reconstruction/voxblox-params.h"

#include <chrono>
#include <cstring>
#include <malloc.h>
#include <string>
#include <vector>

#include <aslam/common/timer.h>
#include <console-common/console.h>
#include <dense-reconstruction/conversion-tools.h>
#include <dense-reconstruction/pmvs-file-utils.h>
#include <dense-reconstruction/pmvs-interface.h>
#include <dense-reconstruction/stereo-dense-reconstruction.h>
#include <depth-integration/depth-integration.h>
#include <gflags/gflags.h>
#include <map-manager/map-manager.h>
#include <maplab-common/conversions.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <visualization/common-rviz-visualization.h>
#include <visualization/rviz-visualization-sink.h>
#include <visualization/viwls-graph-plotter.h>
#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/mesh_vis.h>

DECLARE_string(map_mission_list);
DECLARE_bool(overwrite);

DEFINE_bool(
    dense_depth_map_reprojection_use_undistorted_camera, false,
    "If enabled, the depth map reprojection assumes that the map has "
    "been created using the undistorted camera model. Therefore, the no "
    "distortion is used during reprojection.");

DEFINE_string(
    dense_result_mesh_output_file, "",
    "Path to the PLY mesh file that is generated from the "
    "reconstruction command.");

DEFINE_string(
    dense_image_export_path, "",
    "Export folder for image export function. console command: "
    "export_timestamped_images");

DEFINE_int32(
    dense_depth_resource_output_type, 17,
    "Output resource type of the dense reconstruction algorithms."
    "Supported commands: "
    "stereo_dense_reconstruction "
    "Supported types: "
    "PointCloudXYZRGBN = 17, RawDepthMap = 8");

DEFINE_int32(
    dense_depth_resource_input_type, 21,
    "Input resource type of the dense reconstruction algorithms."
    "Supported commands: "
    "create_tsdf_from_depth_resource "
    "Supported types: "
    "RawDepthMap = 8, OptimizedDepthMap = 9, PointCloudXYZ = 16, "
    "PointCloudXYZRGBN = 17, kPointCloudXYZI = 21");

DEFINE_double(
    balm_kf_distance_threshold_m, 0.5,
    "BALM distance threshold to add a new keyframe [m].");
DEFINE_double(
    balm_kf_rotation_threshold_deg, 10.0,
    "BALM rotation threshold to add a new keyframe [deg].");
DEFINE_double(
    balm_kf_time_threshold_s, 1.0,
    "BALM force a keyframe at fixed time intervals [s].");

DEFINE_double(
    balm_vis_voxel_size, 0.2,
    "Voxel size to use for visualization when publishing pointclouds from "
    "BALM. This does not affect the accuracy of the optimization itself.");

namespace dense_reconstruction {

void publishMapFromBALM(
    aslam::TransformationVector poses_G,
    const std::vector<resources::PointCloud>& pointclouds_S,
    const std::string& path_topic, const visualization::Color& path_color,
    const std::string& cloud_topic) {
  CHECK_EQ(poses_G.size(), pointclouds_S.size());

  resources::PointCloud points_G;
  Eigen::Matrix3Xd positions_G(3, poses_G.size());
  for (size_t i = 0; i < poses_G.size(); ++i) {
    resources::PointCloud voxelized_S;
    pointclouds_S[i].downsampleVoxelized(
        FLAGS_balm_vis_voxel_size, &voxelized_S);
    points_G.appendTransformed(voxelized_S, poses_G[i]);
    positions_G.col(i) = poses_G[i].getPosition();
  }

  sensor_msgs::PointCloud2 ros_points_G;
  backend::convertPointCloudType(points_G, &ros_points_G);
  ros_points_G.header.frame_id = FLAGS_tf_map_frame;
  visualization::RVizVisualizationSink::publish(cloud_topic, ros_points_G);

  visualization::publish3DPointsAsPointCloud(
      positions_G, path_color, 1.0, FLAGS_tf_map_frame, path_topic);
}

bool parseMultipleMissionIds(
    const vi_map::VIMap& vi_map, vi_map::MissionIdList* mission_ids) {
  CHECK_NOTNULL(mission_ids);
  if (!vi_map::csvIdStringToIdList(FLAGS_map_mission_list, mission_ids)) {
    LOG(ERROR) << "The provided CSV mission id list is not valid!";
    return false;
  }

  if (mission_ids->empty()) {
    LOG(INFO) << "No mission id list was provided, operating on all missions!";
    return true;
  }

  LOG(INFO) << "Compute depth maps from multi view stereo for the "
               "following missions:";
  bool success = true;
  for (const vi_map::MissionId& mission_id : *mission_ids) {
    if (mission_id.isValid() && vi_map.hasMission(mission_id)) {
      LOG(INFO) << "-> " << mission_id;
    } else {
      LOG(ERROR) << "-> " << mission_id
                 << " does not exist in the selected map!";
      success = false;
    }
  }
  return success;
}

common::CommandStatus exportTsdfMeshToFile(
    const std::string& mesh_file_path, const voxblox::TsdfMap& tsdf_map) {
  if (mesh_file_path.empty()) {
    LOG(ERROR) << "No mesh output path specified, please set "
                  "--dense_result_mesh_output_file .";
    return common::kStupidUserError;
  }

  if (!common::createPathToFile(mesh_file_path)) {
    LOG(ERROR) << "Unable to create a path for the mesh output file: "
               << mesh_file_path;
    return common::kStupidUserError;
  }

  if (common::fileExists(mesh_file_path)) {
    LOG(ERROR) << "Output mesh file already exists: " << mesh_file_path;
    return common::kStupidUserError;
  }

  voxblox::MeshLayer mesh_layer(tsdf_map.block_size());

  voxblox::MeshIntegratorConfig mesh_config;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
      mesh_config, tsdf_map.getTsdfLayer(), &mesh_layer);
  // We mesh the whole grid at once anyways, so all of them should be
  // updated.
  constexpr bool kMeshOnlyUpdatedBlocks = false;

  // No need to reset, we are not gonna mesh again.
  constexpr bool kResetUpdatedFlag = false;
  mesh_integrator.generateMesh(kMeshOnlyUpdatedBlocks, kResetUpdatedFlag);

  voxblox::outputMeshLayerAsPly(mesh_file_path, mesh_layer);
  return common::kSuccess;
}

DenseReconstructionPlugin::DenseReconstructionPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"export_timestamped_images"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (FLAGS_dense_image_export_path.empty()) {
          LOG(ERROR) << "Please define the export path with "
                        "--dense_image_export_path!";
          return common::kStupidUserError;
        }

        if (!dense_reconstruction::exportAllImagesForCalibration(
                FLAGS_dense_image_export_path, map.get())) {
          return common::kUnknownError;
        }
        return common::kSuccess;
      },
      "Export timestamped image resources, such that they can be used for "
      "calibration. Use --dense_image_export_path to set the export path.",
      common::Processing::Sync);

  addCommand(
      {"export_for_pmvs_reconstruction", "export_for_pmvs"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapReadAccess map =
            map_manager.getMapReadAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        const dense_reconstruction::PmvsConfig config =
            dense_reconstruction::PmvsConfig::getFromGflags();
        if (mission_ids.empty()) {
          if (!dense_reconstruction::exportVIMapToPmvsSfmInputData(
                  config, *map)) {
            return common::kUnknownError;
          }
        } else {
          if (!dense_reconstruction::exportVIMapToPmvsSfmInputData(
                  config, mission_ids, *map)) {
            return common::kUnknownError;
          }
        }
        return common::kSuccess;
      },
      "Export the map and the associated image resources to the PMVS/CMVS "
      "input format, such that we can reconstruct the whole map.",
      common::Processing::Sync);

  addCommand(
      {"stereo_dense_reconstruction", "stereo_dense", "sdr"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        const backend::ResourceType output_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_output_type);

        if (mission_ids.empty()) {
          dense_reconstruction::computeDepthForAllStereoCameras(
              output_resource_type, map.get());
        } else {
          dense_reconstruction::computeDepthForAllStereoCameras(
              output_resource_type, mission_ids, map.get());
        }
        return common::kSuccess;
      },
      "Uses OpenCvs stereo matcher to compute depth resources for all stereo "
      "cameras in the map (or all selected missions). Use the "
      "--dense_stereo_* "
      "flags for configuration of the stereo matcher. Currently only the "
      "pinhole camera model is supported. The depth output type can be set "
      "using --dense_depth_resource_output_type",
      common::Processing::Sync);

  addCommand(
      {"convert_all_depth_maps_to_point_clouds"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (!dense_reconstruction::convertAllDepthMapsToPointClouds(
                map.get())) {
          return common::kUnknownError;
        }
        return common::kSuccess;
      },
      "Convert all depth maps into point clouds (which are stored as PLYs).",
      common::Processing::Sync);

  addCommand(
      {"create_tsdf_from_depth_resource", "tsdf", "depth_fusion"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        // Get configurations.
        voxblox::TsdfIntegratorBase::Config tsdf_integrator_config =
            getTsdfIntegratorConfigFromGflags();
        voxblox::TsdfMap::Config tsdf_map_config = getTsdfMapConfigFromGflags();
        voxblox::MeshIntegratorConfig mesh_config =
            getTsdfMeshIntegratorConfigFromGflags();
        voxblox::ICP::Config icp_config = getTsdfIcpConfigFromGflags();

        // Initialize map, integrators, mesh and ICP.
        voxblox::TsdfMap tsdf_map(tsdf_map_config);

        voxblox::TsdfIntegratorBase::Ptr integrator =
            voxblox::TsdfIntegratorFactory::create(
                FLAGS_dense_tsdf_integrator_type, tsdf_integrator_config,
                tsdf_map.getTsdfLayerPtr());

        voxblox::MeshLayer mesh_layer(tsdf_map.block_size());
        voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesh_integrator(
            mesh_config, tsdf_map.getTsdfLayerPtr(), &mesh_layer);

        voxblox::ICP icp(icp_config);

        voxblox::ColorMode color_mode =
            voxblox::getColorModeFromString(FLAGS_dense_tsdf_mesh_color_mode);

        voxblox::Transformation T_G_C_icp_correction;
        size_t num_pointclouds_integrated = 0u;
        size_t num_points_integrated = 0u;

        depth_integration::IntegrationFunctionPointCloudVoxblox
            integration_function = [&integrator, &icp, &T_G_C_icp_correction,
                                    &tsdf_map, &num_pointclouds_integrated,
                                    &color_mode, &mesh_layer, &mesh_integrator,
                                    &num_points_integrated](
                                       const voxblox::Transformation& T_G_C,
                                       const voxblox::Pointcloud& points,
                                       const voxblox::Colors& colors) {
              if (FLAGS_dense_tsdf_integrate_every_nth > 1 &&
                  (static_cast<int>(num_pointclouds_integrated) %
                       FLAGS_dense_tsdf_integrate_every_nth ==
                   0)) {
                ++num_pointclouds_integrated;
                return;
              }

              voxblox::Transformation T_G_C_refined = T_G_C;

              if (FLAGS_dense_tsdf_icp_enabled) {
                if (!FLAGS_dense_tsdf_icp_accumulate_transformations) {
                  T_G_C_icp_correction.setIdentity();
                }
                icp.runICP(
                    tsdf_map.getTsdfLayer(), points,
                    T_G_C_icp_correction * T_G_C, &T_G_C_refined);

                T_G_C_icp_correction = T_G_C_refined * T_G_C.inverse();

                if (!icp.refiningRollPitch()) {
                  // its already removed internally but small floating point
                  // errors can build up if accumulating transforms
                  voxblox::Transformation::Vector6 T_vec =
                      T_G_C_icp_correction.log();
                  T_vec[3] = 0.0;
                  T_vec[4] = 0.0;
                  T_G_C_icp_correction = voxblox::Transformation::exp(T_vec);
                }
              }

              integrator->integratePointCloud(T_G_C_refined, points, colors);

              ++num_pointclouds_integrated;
              num_points_integrated += points.size();

              if (num_pointclouds_integrated %
                      FLAGS_dense_tsdf_mesh_update_every_nth_cloud ==
                  0u) {
                constexpr bool only_mesh_updated_blocks = true;
                constexpr bool clear_updated_flag = true;
                mesh_integrator.generateMesh(
                    only_mesh_updated_blocks, clear_updated_flag);

                // Publish mesh.
                if (FLAGS_dense_tsdf_publish_mesh_ros) {
                  voxblox_msgs::Mesh mesh_msg;
                  voxblox::generateVoxbloxMeshMsg(
                      &mesh_layer, color_mode, &mesh_msg);
                  mesh_msg.header.frame_id = FLAGS_tf_map_frame;
                  visualization::RVizVisualizationSink::publish(
                      "surface", mesh_msg);
                }
              }
            };

        const backend::ResourceType input_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_input_type);

        // Print configs.
        LOG(INFO) << "\n" << tsdf_map_config.print();
        LOG(INFO) << "\n" << tsdf_integrator_config.print();
        LOG(INFO) << "\n" << mesh_config.print();

        // Do the integration here!
        timing::TimerImpl tsdf_timer("dense_reconstruction: TSDF");

        depth_integration::integrateAllDepthResourcesOfType(
            mission_ids, input_resource_type,
            FLAGS_dense_depth_map_reprojection_use_undistorted_camera, *map,
            integration_function);
        const double tsdf_integration_time_s = tsdf_timer.Stop();

        constexpr double kBytesToMegaBytes = 1e-6;
        LOG(INFO) << "TSDF map:";
        LOG(INFO) << "  allocated blocks: "
                  << tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks();
        LOG(INFO) << "  size: "
                  << tsdf_map.getTsdfLayer().getMemorySize() * kBytesToMegaBytes
                  << "MB";
        LOG(INFO) << "TSDF mesh layer: ";
        LOG(INFO) << "  size: "
                  << mesh_layer.getMemorySize() * kBytesToMegaBytes << "MB";

        const size_t k_points_per_s =
            (static_cast<double>(num_points_integrated) /
             tsdf_integration_time_s) *
            1e-3;
        LOG(INFO) << "Integrated " << (num_points_integrated / 1000u)
                  << "k points in " << tsdf_integration_time_s << "s ("
                  << k_points_per_s << "k points/s)";

        // Update the mesh one last time.
        constexpr bool only_mesh_updated_blocks = true;
        constexpr bool clear_updated_flag = true;
        mesh_integrator.generateMesh(
            only_mesh_updated_blocks, clear_updated_flag);
        voxblox_msgs::Mesh mesh_msg;
        voxblox::generateVoxbloxMeshMsg(&mesh_layer, color_mode, &mesh_msg);
        mesh_msg.header.frame_id = FLAGS_tf_map_frame;
        visualization::RVizVisualizationSink::publish("surface", mesh_msg);

        // Output the mesh to PLY if enabled.
        if (!FLAGS_dense_result_mesh_output_file.empty()) {
          LOG(INFO) << "Exporting mesh to '"
                    << FLAGS_dense_result_mesh_output_file << "' ...";

          timing::TimerImpl tsdf_mesh_timer(
              "dense_reconstruction: TSDF mesh export");
          bool export_success = voxblox::outputMeshLayerAsPly(
              FLAGS_dense_result_mesh_output_file, mesh_layer);
          const double mesh_export_time_s = tsdf_mesh_timer.Stop();

          if (!export_success) {
            LOG(ERROR) << "Failed to save mesh to disk!";
          } else {
            LOG(INFO) << "Exporting and connecting the mesh took "
                      << mesh_export_time_s << "s.";
          }
        }

        // Save the TSDF map as resource.
        const bool has_resource = map->hasMissionResource(
            backend::ResourceType::kVoxbloxTsdfMap, mission_ids);
        if (has_resource && FLAGS_overwrite) {
          map->replaceMissionResource(
              backend::ResourceType::kVoxbloxTsdfMap, tsdf_map, mission_ids);
        } else if (has_resource && !FLAGS_overwrite) {
          LOG(ERROR)
              << "Could not store the Voxblox TSDF map, because there is "
              << "already a map stored. Use --overwrite!";
          return common::kStupidUserError;
        } else {
          map->storeMissionResource(
              backend::ResourceType::kVoxbloxTsdfMap, tsdf_map, mission_ids);
        }

        LOG(INFO) << "\nTimings:\n" << timing::Timing::Print();
        return common::kSuccess;
      },
      "Use all depth resources the selected missions "
      "and integrate them into a Voxblox TSDF map. The map is then stored as "
      "resource associated with the selected set of missions. This command "
      "will use the resource type specified by "
      "--dense_depth_resource_input_type if available.",
      common::Processing::Sync);

  addCommand(
      {"create_esdf_from_depth_resource", "esdf"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        ros::NodeHandle& nh_private =
            visualization::RVizVisualizationSink::getNodeHandle();
        ros::NodeHandle nh;

        // Get configurations.
        voxblox::TsdfIntegratorBase::Config tsdf_integrator_config =
            getTsdfIntegratorConfigFromGflags();
        voxblox::TsdfMap::Config tsdf_map_config = getTsdfMapConfigFromGflags();
        voxblox::ICP::Config icp_config = getTsdfIcpConfigFromGflags();
        voxblox::EsdfIntegrator::Config esdf_integrator_config =
            getEsdfIntegratorConfigFromGflags();
        voxblox::EsdfMap::Config esdf_config = getEsdfMapConfigFromGflags();
        voxblox::MeshIntegratorConfig mesh_config =
            getTsdfMeshIntegratorConfigFromGflags();

        voxblox::EsdfServer esdf_server(
            nh, nh_private, esdf_config, esdf_integrator_config,
            tsdf_map_config, tsdf_integrator_config, mesh_config);
        esdf_server.setWorldFrame("map");
        esdf_server.setClearSphere(FLAGS_dense_esdf_use_clear_sphere);

        voxblox::ICP icp(icp_config);

        voxblox::Transformation T_G_C_icp_correction;

        voxblox::TsdfMap& tsdf_map = *(esdf_server.getTsdfMapPtr());
        voxblox::EsdfMap& esdf_map = *(esdf_server.getEsdfMapPtr());

        size_t num_pointcloud_integrated = 0u;

        depth_integration::IntegrationFunctionPointCloudVoxblox
            integration_function = [&esdf_server, &icp, &T_G_C_icp_correction,
                                    &tsdf_map, &num_pointcloud_integrated](
                                       const voxblox::Transformation& T_G_C,
                                       const voxblox::Pointcloud& points,
                                       const voxblox::Colors& colors) {
              CHECK_EQ(points.size(), colors.size());

              voxblox::Transformation T_G_C_refined = T_G_C;

              if (FLAGS_dense_tsdf_icp_enabled) {
                if (!FLAGS_dense_tsdf_icp_accumulate_transformations) {
                  T_G_C_icp_correction.setIdentity();
                }
                icp.runICP(
                    tsdf_map.getTsdfLayer(), points,
                    T_G_C_icp_correction * T_G_C, &T_G_C_refined);

                T_G_C_icp_correction = T_G_C_refined * T_G_C.inverse();

                if (!icp.refiningRollPitch()) {
                  // its already removed internally but small floating point
                  // errors can build up if accumulating transforms
                  voxblox::Transformation::Vector6 T_vec =
                      T_G_C_icp_correction.log();
                  T_vec[3] = 0.0;
                  T_vec[4] = 0.0;
                  T_G_C_icp_correction = voxblox::Transformation::exp(T_vec);
                }
              }
              esdf_server.integratePointcloud(T_G_C_refined, points, colors);
              esdf_server.newPoseCallback(T_G_C_refined);

              ++num_pointcloud_integrated;

              // Update ESDF layer only every 10th pointcloud.
              if (num_pointcloud_integrated % 10 == 0u) {
                esdf_server.updateEsdf();
              }

              // Visualize only every 100th pointcloud.
              if (num_pointcloud_integrated % 100u == 0u) {
                esdf_server.publishPointclouds();
                esdf_server.publishSlices();
              }
              ros::spinOnce();
            };

        const backend::ResourceType input_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_input_type);

        // Print configs.
        LOG(INFO) << "\n" << tsdf_map_config.print();
        LOG(INFO) << "\n" << tsdf_integrator_config.print();

        depth_integration::integrateAllDepthResourcesOfType(
            mission_ids, input_resource_type,
            FLAGS_dense_depth_map_reprojection_use_undistorted_camera, *map,
            integration_function);

        // Update esdf and visualizations
        esdf_server.updateEsdf();
        esdf_server.publishPointclouds();
        esdf_server.publishSlices();

        if (!FLAGS_dense_esdf_export_map_for_panning_path.empty()) {
          CHECK(common::createPathToFile(
              FLAGS_dense_esdf_export_map_for_panning_path));
          CHECK(esdf_server.saveMap(
              FLAGS_dense_esdf_export_map_for_panning_path));
        }

        const bool has_esdf_resource = map->hasMissionResource(
            backend::ResourceType::kVoxbloxEsdfMap, mission_ids);
        if (has_esdf_resource && FLAGS_overwrite) {
          map->replaceMissionResource(
              backend::ResourceType::kVoxbloxEsdfMap, esdf_map, mission_ids);
        } else if (has_esdf_resource && !FLAGS_overwrite) {
          LOG(ERROR)
              << "Could not store the Voxblox ESDF map, because there is "
              << "already a map stored. Use --overwrite!";
          return common::kStupidUserError;
        } else {
          map->storeMissionResource(
              backend::ResourceType::kVoxbloxEsdfMap, esdf_map, mission_ids);
        }

        const bool has_tsdf_resource = map->hasMissionResource(
            backend::ResourceType::kVoxbloxTsdfMap, mission_ids);
        if (has_tsdf_resource && FLAGS_overwrite) {
          map->replaceMissionResource(
              backend::ResourceType::kVoxbloxTsdfMap, tsdf_map, mission_ids);
        } else if (has_tsdf_resource && !FLAGS_overwrite) {
          LOG(ERROR)
              << "Could not store the Voxblox TSDF map, because there is "
              << "already a map stored. Use --overwrite!";
          return common::kStupidUserError;
        } else {
          map->storeMissionResource(
              backend::ResourceType::kVoxbloxTsdfMap, tsdf_map, mission_ids);
        }

        constexpr double kBytesToMegaBytes = 1e-6;
        LOG(INFO) << "=============  ESDF Map =============";
        LOG(INFO) << "TSDF layer:";
        LOG(INFO) << "  allocated blocks: "
                  << tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks();
        LOG(INFO) << "  size: "
                  << tsdf_map.getTsdfLayer().getMemorySize() * kBytesToMegaBytes
                  << "MB";
        LOG(INFO) << "ESDF layer:";
        LOG(INFO) << "  allocated blocks: "
                  << esdf_map.getEsdfLayer().getNumberOfAllocatedBlocks();
        LOG(INFO) << "  size: "
                  << esdf_map.getEsdfLayer().getMemorySize() * kBytesToMegaBytes
                  << "MB";

        if (!FLAGS_dense_result_mesh_output_file.empty()) {
          return exportTsdfMeshToFile(
              FLAGS_dense_result_mesh_output_file, tsdf_map);
        }
        return common::kSuccess;
      },
      "Use all depth resources the selected missions "
      "and integrate them into a Voxblox TSDF/ESDF map. The map is then "
      "stored as resource associated with the selected set of missions. This "
      "command will use the resource type specified by "
      "--dense_depth_resource_input_type if available.",
      common::Processing::Sync);

  addCommand(
      {"create_mesh_from_tsdf_grid", "export_tsdf"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        if (!parseMultipleMissionIds(*(map.get()), &mission_ids)) {
          return common::kStupidUserError;
        }

        // If no mission were selected, use all missions.
        if (mission_ids.empty()) {
          map.get()->getAllMissionIdsSortedByTimestamp(&mission_ids);
        }

        voxblox::TsdfMap::Config tsdf_map_config;
        voxblox::TsdfMap tsdf_map(tsdf_map_config);
        if (!map.get()->getMissionResource(
                backend::ResourceType::kVoxbloxTsdfMap, mission_ids,
                &tsdf_map)) {
          LOG(ERROR)
              << "No Voxblox TSDF grid stored for the selected missions!";
          return common::kStupidUserError;
        }

        return exportTsdfMeshToFile(
            FLAGS_dense_result_mesh_output_file, tsdf_map);
      },
      "Compute mesh of the Voxblox TSDF grid resource associated with "
      "the selected missions.",
      common::Processing::Sync);

  addCommand(
      {"bundle_adjust_lidar_map", "balm"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        vi_map::MissionIdList mission_ids;
        map->getAllMissionIdsSortedByTimestamp(&mission_ids);

        // Setting up BALM variables
        aslam::TransformationVector poses_G_S;
        std::vector<resources::PointCloud> pointclouds;

        // Keyframe the point clouds, otherwise the memory blows up.
        // Base logic on motion and at fixed time intervals otherwise.
        int64_t time_last_kf;
        vi_map::MissionId last_mission_id;
        last_mission_id.setInvalid();

        // Accumulate point cloud into BALM format. Play with vi_map cache size
        // to facilitate multiple iterations over the same resources as we do
        // that for the undistortion.
        const size_t original_cache_size = map->getMaxCacheSize();
        map->setMaxCacheSize(1);
        depth_integration::IntegrationFunctionPointCloudMaplabWithExtras
            integration_function =
                [&map, &poses_G_S, &time_last_kf, &last_mission_id, &pointclouds](
                    const aslam::Transformation& T_G_S,
                    const int64_t timestamp_ns,
                    const vi_map::MissionId& mission_id,
                    const size_t /*counter*/,
                    const resources::PointCloud& points_S) {
                  poses_G_S.emplace_back(T_G_S);
                  time_last_kf = timestamp_ns;
                  last_mission_id = mission_id;
                  pointclouds.emplace_back(points_S);

                  // Increase cache size by one
                  map->setMaxCacheSize(map->getMaxCacheSize() + 1u);
                };

        const int64_t time_threshold_ns =
            FLAGS_balm_kf_time_threshold_s * kSecondsToNanoSeconds;
        const double distance_threshold = FLAGS_balm_kf_distance_threshold_m;
        const double rotation_threshold =
            FLAGS_balm_kf_rotation_threshold_deg * kDegToRad;

        depth_integration::ResourceSelectionFunction selection_function =
            [&poses_G_S, &time_last_kf, &last_mission_id, &time_threshold_ns,
             &distance_threshold, &rotation_threshold](
                const aslam::Transformation& T_G_S, const int64_t timestamp_ns,
                const vi_map::MissionId& mission_id, const size_t /*counter*/) {
              // We started another mission, so insert a keyframe and
              // re-initialize the other keyframe metrics we keep track of.
              if (!last_mission_id.isValid() || mission_id != last_mission_id) {
                return true;
              }

              // Keyframe if enough time has elapsed since the last keyframe.
              if (timestamp_ns - time_last_kf >= time_threshold_ns) {
                return true;
              }

              // Finally keyframe based on travelled distance or rotation.
              const aslam::Transformation& T_G_Skf = poses_G_S.back();
              const aslam::Transformation T_Skf_S = T_G_Skf.inverse() * T_G_S;
              const double distance_to_last_kf_m = T_Skf_S.getPosition().norm();
              const double rotation_to_last_kf_rad =
                  aslam::AngleAxis(T_Skf_S.getRotation()).angle();
              if (distance_to_last_kf_m > distance_threshold ||
                  rotation_to_last_kf_rad > rotation_threshold) {
                return true;
              }

              return false;
            };

        const backend::ResourceType input_resource_type =
            static_cast<backend::ResourceType>(
                FLAGS_dense_depth_resource_input_type);

        depth_integration::integrateAllDepthResourcesOfType(
            mission_ids, input_resource_type,
            FLAGS_dense_depth_map_reprojection_use_undistorted_camera, *map,
            integration_function, selection_function);

        win_size = poses_G_S.size();
        LOG(INFO) << "Selected a total of " << poses_G_S.size()
                  << " LiDAR scans as keyframes.";

        publishMapFromBALM(
            poses_G_S, pointclouds, "balm_path_before",
            visualization::kCommonRed, "balm_map_before");

        SurfaceMap surface_map;

        for (size_t i = 0; i < win_size; ++i) {
          cut_voxel(surface_map, pointclouds[i], poses_G_S[i], i, win_size);
        }

        VoxHess voxhess;
        resources::PointCloud planes_G;
        for (auto iter = surface_map.begin(); iter != surface_map.end();) {
          if (iter->second->recut()) {
            iter->second->tras_opt(voxhess);
            iter->second->tras_display(&planes_G);
            ++iter;
          } else {
            delete iter->second;
            iter = surface_map.erase(iter);
          }
        }

        // Display planes computed by BALM.
        sensor_msgs::PointCloud2 ros_planes_G;
        backend::convertPointCloudType(planes_G, &ros_planes_G);
        ros_planes_G.header.frame_id = FLAGS_tf_map_frame;
        visualization::RVizVisualizationSink::publish(
            "balm_planes", ros_planes_G);

        const size_t num_constraints = voxhess.plvec_voxels.size();
        if (voxhess.plvec_voxels.size() < 3 * win_size) {
          LOG(WARNING) << "Initial error too large. Loose plane determination "
                       << "criteria or optimize futher with maplab. Got only "
                       << num_constraints << " planes versus a minimum of "
                       << 3 * win_size << ".";
        }

        BALM2 opt_lsv;
        opt_lsv.damping_iter(poses_G_S, voxhess);

        for (auto iter = surface_map.begin(); iter != surface_map.end();) {
          delete iter->second;
          surface_map.erase(iter++);
        }
        surface_map.clear();
        malloc_trim(0);

        publishMapFromBALM(
            poses_G_S, pointclouds, "balm_path_after",
            visualization::kCommonGreen, "balm_map_after");

        return common::kSuccess;
      },
      "Use BALM for point cloud alignment.", common::Processing::Sync);
}

}  // namespace dense_reconstruction

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    dense_reconstruction::DenseReconstructionPlugin);
