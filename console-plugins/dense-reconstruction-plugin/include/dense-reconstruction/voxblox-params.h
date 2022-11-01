#ifndef DENSE_RECONSTRUCTION_VOXBLOX_PARAMS_H_
#define DENSE_RECONSTRUCTION_VOXBLOX_PARAMS_H_

#include <string>

#include <gflags/gflags.h>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/mesh_vis.h>

DECLARE_bool(dense_esdf_use_clear_sphere);
DECLARE_bool(dense_tsdf_icp_accumulate_transformations);
DECLARE_bool(dense_tsdf_icp_enabled);
DECLARE_bool(dense_tsdf_icp_refine_roll_and_pitch);
DECLARE_bool(dense_tsdf_publish_mesh_ros);
DECLARE_bool(dense_tsdf_use_const_weight);
DECLARE_bool(dense_tsdf_use_weight_dropoff);
DECLARE_bool(dense_tsdf_use_symmetric_weight_dropoff);
DECLARE_bool(dense_tsdf_voxel_carving_enabled);
DECLARE_bool(dense_tsdf_voxel_use_clearing_rays);
DECLARE_bool(dense_tsdf_weight_ray_by_range);
DECLARE_double(dense_tsdf_max_ray_length_m);
DECLARE_double(dense_tsdf_mesh_min_weight);
DECLARE_double(dense_tsdf_min_ray_length_m);
DECLARE_double(dense_tsdf_truncation_distance_m);
DECLARE_double(dense_tsdf_voxel_size_m);
DECLARE_double(dense_tsdf_max_weight);
DECLARE_double(dense_tsdf_clearing_ray_weight_factor);
DECLARE_int32(dense_tsdf_integrate_every_nth);
DECLARE_int32(dense_tsdf_mesh_update_every_nth_cloud);
DECLARE_string(dense_tsdf_integrator_type);
DECLARE_string(dense_tsdf_mesh_color_mode);
DECLARE_uint64(dense_tsdf_voxels_per_side);

DECLARE_double(dense_esdf_clear_sphere_inner_radius);
DECLARE_double(dense_esdf_clear_sphere_outer_radius);
DECLARE_string(dense_esdf_export_map_for_panning_path);

namespace dense_reconstruction {

voxblox::TsdfIntegratorBase::Config getTsdfIntegratorConfigFromGflags();

voxblox::TsdfMap::Config getTsdfMapConfigFromGflags();

voxblox::MeshIntegratorConfig getTsdfMeshIntegratorConfigFromGflags();

voxblox::ICP::Config getTsdfIcpConfigFromGflags();

voxblox::EsdfIntegrator::Config getEsdfIntegratorConfigFromGflags();

voxblox::EsdfMap::Config getEsdfMapConfigFromGflags();

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_VOXBLOX_PARAMS_H_
