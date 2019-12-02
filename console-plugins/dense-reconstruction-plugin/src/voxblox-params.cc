#include "dense-reconstruction/voxblox-params.h"

DEFINE_bool(
    dense_esdf_use_clear_sphere, false,
    "If enabled, for every integrated point cloud, the unknown voxels in a "
    "double sphere around the robot pose are modified. In the inner sphere, "
    "the unknown voxels are turned into free space, in the outter sphere, they "
    "are turned into occupied space. The sphere can be configured with "
    "--dense_esdf_clear_sphere_inner_radius and "
    "--dense_esdf_clear_sphere_outer_radius");
DEFINE_double(
    dense_esdf_clear_sphere_inner_radius, 0.3,
    "Radius for the inner sphere of the ESDF clear sphere. Set "
    "--dense_esdf_use_clear_sphere=true to enable.");
DEFINE_double(
    dense_esdf_clear_sphere_outer_radius, 1.0,
    "Radius for the outer sphere of the ESDF clear sphere. Set "
    "--dense_esdf_use_clear_sphere=true to enable.");

DEFINE_string(
    dense_esdf_export_map_for_panning_path, "",
    "Path to where the ESDF/TSDF map is exported to. If empty, no "
    "map will be exported.");

DEFINE_bool(
    dense_tsdf_icp_refine_roll_and_pitch, false,
    "If enabled, ICP will refine roll and pitch as well when integrating point "
    "clouds into the TSDF or ESDF maps.");

DEFINE_string(
    dense_tsdf_integrator_type, "fast",
    "Voxblox TSDF integrator type [simple, merged, fast]");

DEFINE_double(
    dense_tsdf_max_weight, 10000.0,
    "Maximum weight a voxel can achieve. Influences how big of an impact a new "
    "measurment has on an existing voxel. The influence continously drops "
    "until the voxel reaches max weight, then it remains constant.");

DEFINE_double(
    dense_tsdf_clearing_ray_weight_factor, 0.1,
    "Fraction of the weight the clearing ray receives compared to the weight "
    "at the highest point along the ray, the zero crossing. Set to 1.0 if a "
    "clearing ray should be equally powerful.");

DEFINE_bool(
    dense_tsdf_weight_ray_by_range, false,
    "If enabled, the base-weight for each ray is set to 1/z^2, otherwise it is "
    "set to 1.0. Use this for depth sensors with degrading performance at "
    "longer range.");

DEFINE_bool(
    dense_tsdf_use_weight_dropoff, false,
    "If enabled, the weight of the ray to be integrated drops of behind the "
    "surface. This improves the results for large voxel sizes.");

DEFINE_bool(
    dense_tsdf_use_symmetric_weight_dropoff, true,
    "If enabled, the weight of the ray to be integrated drops of behind and in "
    "front of the surface. This improves the results for large voxel sizes.");

DEFINE_bool(
    dense_tsdf_use_const_weight, false,
    "If enabled, the ray inside the truncation distance receives a constant "
    "weight equal to the ray base weight.");

DEFINE_bool(
    dense_tsdf_icp_enabled, true,
    "If enabled, ICP is used to align the point clouds to the TSDF grid, "
    "before integrating them.");

DEFINE_bool(
    dense_tsdf_icp_accumulate_transformations, true,
    "If enabled, the ICP corrections are accumulated, such that subsequent "
    "alignments start from the pose obtained from the VIMap which is then "
    "corrected based on the previous ICP alignment.");

DEFINE_double(
    dense_tsdf_voxel_size_m, 0.10, "Voxel size of the TSDF grid [m].");

DEFINE_string(
    dense_tsdf_integration_order_mode, "sorted",
    "Order in which the rays are integrated, the options are 'mixed', which "
    "tries to distribute the rays and 'sorted', which will integrate the "
    "closer rays first.");

DEFINE_uint64(
    dense_tsdf_voxels_per_side, 16u,
    "Voxels per side of a Block of the TSDF grid.");

DEFINE_bool(
    dense_tsdf_voxel_carving_enabled, true,
    "Voxels per side of a Block of the TSDF grid.");

DEFINE_bool(
    dense_tsdf_voxel_use_clearing_rays, true,
    "If enabled it will ray-trace points that are beyond the maxium ray length "
    "defined by --dense_tsdf_max_ray_length_m up to the threshold to clear "
    "free space. This option is intended to create maps for path planning.");

DEFINE_double(
    dense_tsdf_truncation_distance_m, 0.4,
    "Truncation distance of the TSDF grid [m].");

DEFINE_double(
    dense_tsdf_min_ray_length_m, 0.05,
    "Minimum ray length integrated into the TSDF grid.");

DEFINE_double(
    dense_tsdf_mesh_min_weight, 1e-4,
    "Minimum weight of a voxel to be considered for surface "
    "reconstruction/meshing.");
DEFINE_int32(
    dense_tsdf_mesh_update_every_nth_cloud, 40,
    "Update the mesh every N point cloud that is integrated into the TSDF "
    "map.");

DEFINE_double(
    dense_tsdf_max_ray_length_m, 20.,
    "Maximum ray length integrated into the TSDF grid.");

DEFINE_int32(
    dense_tsdf_fast_integrator_max_consecutive_ray_collision, 0,
    "Only for FastTsdfIntegrator: Maximum number of consecutive times a ray "
    "can update an already updated voxel (in current scan) before the "
    "integrator stops the current ray.");

DEFINE_bool(
    dense_tsdf_publish_mesh_ros, true,
    "If enabled, the tsdf reconstruction progress will be published to ROS as "
    "voxblox surface mesh. Make sure to source the maplab workspace before "
    "launching rviz to support the custom voxblox mesh message type.");

DEFINE_string(
    dense_tsdf_mesh_color_mode, "color",
    "Color mode for the TSDF mesh generation. Options: [color, height, "
    "normals, lambert, lambert_color, gray]");

DEFINE_int32(
    dense_tsdf_integrate_every_nth, 1,
    "The TSDF integration considers only every N pointcloud.");

namespace dense_reconstruction {
voxblox::TsdfIntegratorBase::Config getTsdfIntegratorConfigFromGflags() {
  voxblox::TsdfIntegratorBase::Config config;

  config.max_weight = FLAGS_dense_tsdf_max_weight;
  config.clearing_ray_weight_factor =
      FLAGS_dense_tsdf_clearing_ray_weight_factor;
  config.weight_ray_by_range = FLAGS_dense_tsdf_weight_ray_by_range;

  config.use_const_weight = FLAGS_dense_tsdf_use_const_weight;
  config.use_weight_dropoff = FLAGS_dense_tsdf_use_weight_dropoff;
  config.use_symmetric_weight_dropoff =
      FLAGS_dense_tsdf_use_symmetric_weight_dropoff;

  config.voxel_carving_enabled = FLAGS_dense_tsdf_voxel_carving_enabled;
  config.integration_order_mode = FLAGS_dense_tsdf_integration_order_mode;

  config.max_consecutive_ray_collisions =
      FLAGS_dense_tsdf_fast_integrator_max_consecutive_ray_collision;

  config.allow_clear = FLAGS_dense_tsdf_voxel_use_clearing_rays;
  config.default_truncation_distance =
      static_cast<float>(FLAGS_dense_tsdf_truncation_distance_m);
  config.min_ray_length_m =
      static_cast<voxblox::FloatingPoint>(FLAGS_dense_tsdf_min_ray_length_m);
  config.max_ray_length_m =
      static_cast<voxblox::FloatingPoint>(FLAGS_dense_tsdf_max_ray_length_m);
  return config;
}

voxblox::TsdfMap::Config getTsdfMapConfigFromGflags() {
  voxblox::TsdfMap::Config config;
  config.tsdf_voxel_size =
      static_cast<voxblox::FloatingPoint>(FLAGS_dense_tsdf_voxel_size_m);
  config.tsdf_voxels_per_side = FLAGS_dense_tsdf_voxels_per_side;
  return config;
}

voxblox::MeshIntegratorConfig getTsdfMeshIntegratorConfigFromGflags() {
  voxblox::MeshIntegratorConfig config;
  config.min_weight = FLAGS_dense_tsdf_mesh_min_weight;
  return config;
}

voxblox::ICP::Config getTsdfIcpConfigFromGflags() {
  voxblox::ICP::Config config;
  config.refine_roll_pitch = FLAGS_dense_tsdf_icp_refine_roll_and_pitch;
  return config;
}

voxblox::EsdfIntegrator::Config getEsdfIntegratorConfigFromGflags() {
  voxblox::EsdfIntegrator::Config config;
  config.min_distance_m =
      static_cast<float>(FLAGS_dense_tsdf_truncation_distance_m) * 0.75f;
  config.clear_sphere_radius = FLAGS_dense_esdf_clear_sphere_inner_radius;
  config.occupied_sphere_radius = FLAGS_dense_esdf_clear_sphere_outer_radius;
  return config;
}

voxblox::EsdfMap::Config getEsdfMapConfigFromGflags() {
  voxblox::EsdfMap::Config config;
  config.esdf_voxel_size =
      static_cast<voxblox::FloatingPoint>(FLAGS_dense_tsdf_voxel_size_m);
  config.esdf_voxels_per_side = FLAGS_dense_tsdf_voxels_per_side;
  return config;
}

}  // namespace dense_reconstruction
