## Resource Importer

### Description

This executable loads a maplab map, and attaches a specific resources from a rosbag as a sensor resource.
This means the resource will be loosely associated with the pose graph based on:
 - Type (Depth map, point cloud, color image, etc.)
 - Timestamp
 - Sensor calibration

In general the following things are required to attach resources:
 - A maplab map
 - A rosbag containing the resources (same time frame as the map, of course)
 - A calibration of the sensor that recorded the resources, this can be provided as:
   - CameraInfo and tf topic for image sensors
   - Maplab sensor.yaml file which contains:
     - The sensor that recorded the resource (e.g. NCamera, Lidar, etc.)
     - The base sensor of the map (e.g. IMU)
     - The extrinsics in between them.

### Usage Examples

Using data from a ZR300 sensor, the following resources can be attached:

 - [x] ZR300, point clouds with camera calibration from CameraInfo and tf

    ```bash
    rosrun resource_importer import_resources_w_camera_info.sh \
       <maplab map in> \
       <bag>  \
       <point cloud topic> \
       <camera info topic> \
       <maplab map out>
    ```

  - [x] ZR300, depth maps with camera calibration from CameraInfo and tf

    ```bash
    rosrun resource_importer import_resources_w_camera_info.sh \
       <maplab map in> \
       <bag>  \
       <depth map topic> \
       <camera info topic> \
       <maplab map out>
    ```

 - [x] ZR300, point clouds with camera calibration from sensors.yaml

    ```bash
    rosrun resource_importer import_resources_w_sensor_yaml.sh \
       <maplab map in> \
       <bag>  \
       <point cloud topic> \
       sensors.yaml \
       <maplab map out>
    ```

  - [x] ZR300, depth maps with camera calibration from sensors.yaml

    ```bash
    rosrun resource_importer import_resources_w_sensor_yaml.sh \
       <maplab map in> \
       <bag>  \
       <depth map topic> \
       sensors.yaml \
       <maplab map out>
    ```

 - [x] ZR300, color images, same as depth maps
