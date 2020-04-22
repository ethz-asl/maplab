## Resource Importer

**This feature is only available for the release candidate March 2018 or newer!**

The resource importer provides a tool to attach resources (images, depth maps, point clouds) from a rosbag to a map, based on timestamps. The main application we had in mind was to attach depth maps or color images of synchronized sensor which is not triggered at the same time as the primary tracking camera. This is for example the case if an RGB-D camera is used to build a map. The resources are attached to the VIMap and associated with a specific Mission and timestamp. The camera calibration can be either provided using the existing maplab yaml format, or parsed directly from the CameraInfo rostopic.

### Instructions

Requirements:
 * Calibrated camera setup. This includes both the primary visual inertial camera + the optional camera (depth, color, etc.).
 * Build the `resource_importer` package, it is not a dependency of the maplab console.

Procedure:

1. Record a rosbag (all topics needed for ROVIOLI + the camera (info) topic of the resource you want to attach)
2. Compute a maplab map using ROVIOLI
3. Run the resource importer:

   **Option A** - Get camera calibration from CameraInfo topic:
   ```bash
   rosrun resource_importer import_resources_w_camera_info.sh \
   $MAPLAB_MAP $ROSBAG  \
   $RESOURCE_TOPIC \
   $CAMERA_INFO_TOPIC \
   $MAPLAB_MAP_OUTPUT
   ```
   **Option B** - Get camera calibration from yaml file:
   ```bash
   rosrun resource_importer import_resources_w_ncamera_yaml.sh \
   $MAPLAB_MAP $ROSBAG  \
   $RESOURCE_TOPIC \
   $CAMERA_CALIBRATION_YAML \
   $MAPLAB_MAP_OUTPUT
   ```

### Examples

If you record a rosbag using the ZR300 sensor, there are 4 options to use the resource importer.
 * Attach depth maps to the VIMap and use the calibration provided by the CameraInfo topic of the `maplab_realsense` node
   ```bash
   rosrun resource_importer import_resources_w_camera_info.sh \
      zr300_map zr300_mapping_office_2_2018-03-15-16-05-46.bag  \
      /zr300_node/depth/image_raw \
      /zr300_node/depth/camera_info \
      zr300_map_dm_from_camera_info
   ```
 * Attach point clouds to the VIMap and use the calibration provided by the CameraInfo topic of the `maplab_realsense` node
   ```bash
   rosrun resource_importer import_resources_w_camera_info.sh \
      zr300_map zr300_mapping_office_2_2018-03-15-16-05-46.bag  \
      /zr300_node/pointcloud \
      /zr300_node/depth/camera_info \
      zr300_map_pc_from_camera_info
   ```
 * Attach depth maps to the VIMap and use the calibration provided by a ncamera.yaml (e.g. obtained using Kalibr)
   ```bash
   rosrun resource_importer import_resources_w_ncamera_yaml.sh \
      zr300_map zr300_mapping_office_2_2018-03-15-16-05-46.bag  \
      /zr300_node/depth/image_raw \
      depth-camera.yaml \
      zr300_map_dm_from_ncamera_yaml
   ```
 * Attach point clouds to the VIMap and use the calibration by a ncamera.yaml (e.g. obtained using Kalibr)
   ```bash
   rosrun resource_importer import_resources_w_ncamera_yaml.sh \
      zr300_map zr300_mapping_office_2_2018-03-15-16-05-46.bag  \
      /zr300_node/pointcloud \
      depth-camera.yaml \
      zr300_map_pc_from_ncamera_yaml
   ```

### Reconstruction Results

Once depth maps or point clouds are attached to the VIMap, they can be used to create 3D reconstructions, as explained in the following wiki page: [[Dense-Reconstruction]]

![image](https://user-images.githubusercontent.com/5071588/37495231-9afec4ba-28ac-11e8-9a62-29bc9e6e1d6b.png)

*Result of attaching ZR300 point clouds into a maplab map and using TSDF integration. Note that the IMU to depth map calibration is based on the camera calibration and extrinsics published by the maplab_realsense driver and would need to be calibrated with kalibr for more exact results.*
