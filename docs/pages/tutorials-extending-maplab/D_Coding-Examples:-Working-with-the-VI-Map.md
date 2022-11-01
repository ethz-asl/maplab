## Coding Examples: Woriking with the VI Map

One of the main goals of the maplab framework is to provide an extensible experimental platform that can be used for research focusing on visual-inertial estimation.
Such use is facilitated by a rich set of unit-tested helper functions that offer high-level functionality.
Multiple query interfaces permit to retrieve objects from the map based on geometric or topological (e.g. using the covisibility graph) associations.
The manipulation functions, on the other hand, enable the user to modify the map by adding, merging or deleting certain objects.
Over all those operations, the map consistency is guaranteed.
Below, an example algorithm that highlights the use of maplab helper functions is presented.
We would like to draw the reader's attention to high-level calls used to retrieve various information from the map.
This speeds up the development process and makes the actual C++ code resemble the algorithm pseudo-code.

### Example: Projection of map landmarks into an image

Let's assume we aim to improve the localization quality by reprojecting some of the map landmarks into a keyframe, based on a pose guess and the camera calibration.
First, we want to retrieve a global frame pose of the current location.
Let's assume `query_vertex_id` identifies the vertex.
The VI-map can provide a global frame pose in a single call:

```c++
const Eigen::Vector3d query_vertex_p_G =
    map->getVertex_G_p_I(query_vertex_id);
```

Now we would like to find landmarks that were observed by other keyframes close to the current location.
Let's try to look for observers within a certain distance.
We can use the `SpatialDatabase`, which efficiently retrieves a list of neighboring observers:
```c++
constexpr double kRadiusMeters = 0.5;
pose_graph::VertexIdSet neigboring_observers;
vi_map_helpers::SpatialDatabase<pose_graph::VertexId> 
  spatial_database(*map, grid_size_meters);
spatial_database.getObjectIdsInRadius(
  query_vertex_p_G, kRadiusMeters, &neigboring_observers);
```

Next, having a list of neighboring observers available, we want to know which landmarks were actually observed by them.
A class `VIMapQueries` contains various useful queries, among others one that retrieves all the observed landmarks per vertex:
```c++
vi_map_helpers::VIMapQueries queries(*map);
vi_map::LandmarkIdSet all_landmarks;
for (const pose_graph::VertexId& vertex_id :
     neighboring_observers) {
  vi_map::LandmarkIdSet landmarks;
  queries.getIdsOfLandmarksObservedByVertex(
      vertex_id, &landmarks);
  all_landmarks.insert(
      landmarks.begin(), landmarks.end());
}
```

Finally, after getting a list of all candidate landmarks, we will try to project each of them into the query frame.
The VI-map offers a convenience function that expresses the landmark position in the camera frame `C`.
Then, it suffices to retrieve the camera object itself and use the camera's projection function to obtain the projection in image coordinates.
The returned value informs us about the success of a projection.
```c++
constexpr size_t kFrameIndex = 0u;
vi_map::Vertex& query_vertex =
    map->getVertex(query_vertex_id);
for (const vi_map::LandmarkId& landmark : all_landmarks) {
  Eigen::Vector3d landmark_p_C = map->getLandmark_p_C_fi(
      landmark, query_vertex, kFrameIndex);

  Eigen::Vector2d keypoint;
  aslam::ProjectionResult result =
      query_vertex.getCamera(kFrameIndex)
          ->project3(landmark_p_C, &keypoint);
  if (result == aslam::ProjectionResult::KEYPOINT_VISIBLE)
    LOG(INFO) << "Great success!";
}
```
