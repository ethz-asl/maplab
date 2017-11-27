#ifndef VI_MAP_MACROS_H_
#define VI_MAP_MACROS_H_

namespace vi_map {

#define VISUAL_FRAME_RESOURCE_CONVENIENCE_FUNCTIONS(                           \
    name, resource_type, data_type)                                            \
                                                                               \
  inline void store##name(                                                     \
      const data_type& resource, const unsigned int frame_idx,                 \
      Vertex* vertex_ptr) {                                                    \
    CHECK_NOTNULL(vertex_ptr);                                                 \
    storeFrameResource(resource, frame_idx, resource_type, vertex_ptr);        \
  }                                                                            \
                                                                               \
  inline bool get##name(                                                       \
      const Vertex& vertex, const unsigned int frame_idx, data_type* resource) \
      const {                                                                  \
    CHECK_NOTNULL(resource);                                                   \
    return getFrameResource(vertex, frame_idx, resource_type, resource);       \
  }                                                                            \
                                                                               \
  inline bool has##name(const Vertex& vertex, const unsigned int frame_idx)    \
      const {                                                                  \
    return hasFrameResource<data_type>(vertex, frame_idx, resource_type);      \
  }                                                                            \
                                                                               \
  inline void delete##name(const unsigned int frame_idx, Vertex* vertex_ptr) { \
    CHECK_NOTNULL(vertex_ptr);                                                 \
    deleteFrameResourcesOfType<data_type>(                                     \
        frame_idx, resource_type, vertex_ptr);                                 \
  }                                                                            \
                                                                               \
  inline void replace##name(                                                   \
      const data_type& resource, const unsigned int frame_idx,                 \
      Vertex* vertex_ptr) {                                                    \
    CHECK_NOTNULL(vertex_ptr);                                                 \
    return replaceFrameResource(                                               \
        resource, frame_idx, resource_type, vertex_ptr);                       \
  }

#define MISSION_RESOURCE_CONVENIENCE_FUNCTIONS(name, resource_type, data_type) \
                                                                               \
  inline bool has##name(const MissionIdList& involved_mission_ids) const {     \
    return hasMissionResource(resource_type, involved_mission_ids);            \
  }                                                                            \
                                                                               \
  inline bool get##name(                                                       \
      const MissionIdList& involved_mission_ids, data_type* resource) const {  \
    CHECK_NOTNULL(resource);                                                   \
    return getMissionResource(resource_type, involved_mission_ids, resource);  \
  }                                                                            \
                                                                               \
  inline void store##name(                                                     \
      const data_type& resource, const MissionIdList& involved_mission_ids) {  \
    CHECK(!involved_mission_ids.empty());                                      \
    storeMissionResource(resource_type, resource, involved_mission_ids);       \
  }                                                                            \
                                                                               \
  inline void delete##name(const MissionIdList& involved_mission_ids) {        \
    CHECK(!involved_mission_ids.empty());                                      \
    deleteMissionResource<data_type>(resource_type, involved_mission_ids);     \
  }                                                                            \
                                                                               \
  inline void replace##name(                                                   \
      const MissionIdList& involved_mission_ids, const data_type& resource) {  \
    CHECK(!involved_mission_ids.empty());                                      \
    replaceMissionResource(resource_type, resource, involved_mission_ids);     \
  }

#define OPTIONAL_CAMERA_RESOURCE_CONVENIENCE_FUNCTIONS(                \
    name, resource_type, data_type)                                    \
                                                                       \
  inline bool hasOptionalCamera##name(                                 \
      const VIMission& mission, const aslam::CameraId& camera_id,      \
      const int64_t timestamp_ns) const {                              \
    return hasOptionalCameraResource(                                  \
        mission, resource_type, camera_id, timestamp_ns);              \
  }                                                                    \
                                                                       \
  inline bool getOptionalCamera##name(                                 \
      const VIMission& mission, const aslam::CameraId& camera_id,      \
      const int64_t timestamp_ns, data_type* resource) const {         \
    CHECK_NOTNULL(resource);                                           \
    return getOptionalCameraResource(                                  \
        mission, resource_type, camera_id, timestamp_ns, resource);    \
  }                                                                    \
                                                                       \
  inline bool getClosestOptionalCamera##name(                          \
      const VIMission& mission, const aslam::CameraId& camera_id,      \
      const int64_t timestamp_ns, const int64_t tolerance_ns,          \
      data_type* resource, int64_t* closest_timestamp_ns) const {      \
    CHECK_NOTNULL(resource);                                           \
    CHECK_NOTNULL(closest_timestamp_ns);                               \
    return getClosestOptionalCameraResource(                           \
        mission, resource_type, camera_id, timestamp_ns, tolerance_ns, \
        resource, closest_timestamp_ns);                               \
  }                                                                    \
                                                                       \
  inline void storeOptionalCamera##name(                               \
      const aslam::CameraId& camera_id, const int64_t timestamp_ns,    \
      const data_type& resource, VIMission* mission) {                 \
    CHECK_NOTNULL(mission);                                            \
    addOptionalCameraResource(                                         \
        resource_type, camera_id, timestamp_ns, resource, mission);    \
  }                                                                    \
                                                                       \
  inline void deleteOptionalCamera##name(                              \
      const aslam::CameraId& camera_id, const int64_t timestamp_ns,    \
      VIMission* mission) {                                            \
    CHECK_NOTNULL(mission);                                            \
    deleteOptionalCameraResource<data_type>(                           \
        resource_type, camera_id, timestamp_ns, mission);              \
  }
}  // namespace vi_map

#endif  // VI_MAP_MACROS_H_
