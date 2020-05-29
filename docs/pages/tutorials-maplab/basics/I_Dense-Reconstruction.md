## Dense Reconstruction: attaching resources to map, available reconstruction tools

### Introduction

This page lists all the dense reconstruction tools available in maplab. If you would like try these tools yourself you will first need a map that contains images or depth maps. There are several ways to get such a map.


 * The simplest way is to run ROVIOLI with the following flags. If you are not sure if your map has resources attached to it, use the commands `res_stats` and `ms` to verify the presence of resources and the correct number of cameras.

    ```bash
      # By providing ROVIOLI with the calibration for both cameras of the stereo sensor,
      # it will build a map that also contains both of them. Please note that ROVIO will 
      # always run in monocular mode, regardless of the second camera. Even though rovio 
      # supports stereo cameras, there are some issues with this mode and the performance 
      # doesn't really improve.
      --ncamera_calibration=$ROVIO_CONFIG_DIR/ncamera-euroc-stereo.yaml

      # This will attach all grayscale images of the cameras used for pose estimation to the map.
      --map_builder_save_image_as_resources=true
    
    ```

 * Use the [[Resource-Importer]] to attach images, depth maps or point clouds to a VIMap.
 * If you develop your own image import function, your own interface to convert your mapping data to maplab maps or interface your own estimator, please have a look at [this](https://github.com/ethz-asl/maplab/wiki/Dense-Reconstruction#how-do-i-attach-my-own-rgb-imagesimagesdepth-mapspoint-clouds-to-the-vi-map) section on how to attach images and depth maps to the map in C++. Please don't hesitate to open an issue if you need help attaching your own resources.
 * **Wishlist**: We are thinking about a way to provide ROVIOLI with an additional calibration file with more cameras and depth sensors (in addition to the ones used for pose estimation), which it will then subscribe to the camera topics and attach the images/depth maps to the map for you. Please let us know if you are interested in this feature.


### How do I compute a reconstruction based on the images attached to the VI-map?

![stereo dense](https://github.com/ethz-asl/maplab/wiki/images/dense/stereo_dense_2.png)

 * **Stereo dense reconstruction:**
   * Description:

     A (semi-global) block matcher based stereo reconstruction pipeline that computes depth maps or point clouds based on gray-scale images that are attached to your VI-map. Have a look at this tutorial ([[Stereo Dense Reconstruction]]) on how to run this command on the EuRoC dataset. 
   * Prerequisites:
     * A VI-map with one or more missions that use a stereo camera suitable for [planar rectification](http://www.cs.unc.edu/~marc/tutorial/node98.html).
     * Gray-scale images attached to the VI-map (`backend::ResourceType::kRawImage`)
     * The stereo camera should be well calibrated to allow for accurate stereo matching.
   * Command:
     ```bash
     stereo_dense_reconstruction [--map_mission_list=ID_A,ID_B,ID_C]
     ```
   * Parameters:

     An example flagfile can be found here:
     ```
     maplab/algorithms/dense-reconstruction/stereo-dense-reconstruction/parameter/stereo_params.gflags
     ```
     Use the flag `--flagfile` to pass the parameters to the console command.

   * Output:
      * Depth resources (`backend::ResourceType::kRawDepthMap` or `backend::ResourceType::kPointCloudXYZRGBN`) attached to the vertices. 

![img by voxblox](https://github.com/ethz-asl/maplab/wiki/images/dense/voxblox_img.jpeg)
(Image taken from [voxblox](https://github.com/ethz-asl/voxblox/blob/master/README.md))

* **Depth fusion and surface reconstruction using [voxblox](https://github.com/ethz-asl/voxblox)**
   * Description:

     A depth fusion algorithm based on a TSDF-based voxel grid, with voxel-hashing. Enables depth fusion and surface reconstruction.
   * Prerequisites:
      * A VI-map with one or more aligned, loop closed and optimized missions (in order to achieve a consistent reconstruction).
      * Depth resources attached to the VI-map. Currently implemented are:
        * Depth maps (`backend::ResourceType::kRawDepthMap`) + optional gray-scale images (`backend::ResourceType::kRawImage` or `backend::ResourceType::kImageForDepthMap`)
        * Point clouds (`backend::ResourceType::kPointCloudXYZRGBN`)
   * Commands:
     ```bash
     # Inserts all depth maps into a voxblox TSDF grid and attaches it to the selected missions.
     create_tsdf_from_depth_resource [--map_mission_list=ID_A,ID_B,ID_C]
     # Loads the voxblox grid associated with the selected missions, extracts the ISO-surface and stores it as a PLY mesh file.
     create_mesh_from_tsdf_grid [--map_mission_list=ID_A,ID_B,ID_C] -dense_result_mesh_output_file=output_file.ply
     ```   
   * Parameters:

     An example flagfile can be found here:
     ```
     maplab/algorithms/dense-reconstruction/stereo-dense-reconstruction/parameters/stereo_params.gflags
     ```
     Use the flag `--flagfile` to pass the parameters to the console command.

   * Output:
     * Voxblox map attached to the selected missions (`backend::ResourceType::kVoxbloxTsdfMap`).
     * Surface reconstruction mesh as PLY file.

![pmvs export](https://github.com/ethz-asl/maplab/wiki/images/dense/pmvs_large.png)

 * **Map export und subsequent reconstruction using [CMVS](https://www.di.ens.fr/cmvs/)/[PMVS2](https://www.di.ens.fr/pmvs/)**
    * Description:
       
      Export function to convert the VI-map and its image resources to a format that is compatible with CMVS/PMVS, a patch-based multi-view stereo reconstruction pipeline (offline). 
    * Prerequisites:
       * Check out and build [cmvs_pmvs_catkin](https://github.com/ethz-asl/cmvs_pmvs_catkin) in your workspace. (Catkinized version of [CMVS](https://www.di.ens.fr/cmvs/)/[PMVS2](https://www.di.ens.fr/pmvs/))
       * A VI-map with one or more missions that have RGB images (`backend::ResourceType::kRawColorImage`) or gray-scale images (`backend::ResourceType::kRawImage`) attached to it. Be aware that using gray-scale images significantly decreases the performance of PMVS and requires some parameter tuning.
    * Command:
      ```bash 
      export_for_pmvs [--map_mission_list=ID_A,ID_B,ID_C] -pmvs_reconstruction_folder=export_folder
      ...
      # Get and build cmvs_pmvs_catkin
      ```
      Follow the instructions in the cmvs_pmvs_catkin README and execute:

      ```bash
      rosrun cmvs_catkin cmvs $PMVS_FOLDER 20
      rosrun pmvs_catkin genOption $PMVS_FOLDER
      sh $PMVS_FOLDER/pmvs.sh
      ```
    * Parameters:
      ```
      -cmvs_use_only_good_landmarks (If enabled, only good landmarks are used to
        cluster the images.) type: bool default: true
      -pmvs_reconstruction_folder (Output folder of the PMVS export.) type: string default: ""
      -pmvs_undistortion_alpha (Determines how many invalid pixels are part of
        the undistorted image. [0 = none, 1 = all]) type: double default: 0
      -pmvs_undistortion_scale (Scale of undistorted image.) type: double
        default: 1
      -pmvs_use_color_images (If true, PMVS will only use VisualFrames with color
        images, otherwise only the grayscale images.) type: bool default: true
      ```

### How do I attach my own rgb images/images/depth maps/point clouds to the VI-map?

* If your VI-sensor setup already includes a depth sensor or provides a dense stereo pipeline you can attach the depth maps/point clouds  to the VI-map in your front-end or during a post-processing step using the resource system.
   ```cpp
   // Example: depth maps that originated from the camera at index 0 of the primary visual odometry camera setup, e.g. by means of a dense stereo matcher.
   cv::Mat depth_map;
   vi_map.storeRawDepthMap(depth_map, 0, &vertex);
   ```
   ```cpp
   // Example: depth maps that were recorded by an additional, synchronized sensor, e.g. RGB-D sensor.
   // Add the external camera model to the VIMission (only once).
   aslam::Camera camera = ...;
   aslam::Transformation T_C_B = ...;
   ...
   vi_mission.addOptionalCameraWithExtrinsics(camera, T_C_B);
   ...
   // For every depth map:
   cv::Mat depth_map = ...;
   int64_t timestamp_ns = ...;
   vi_map.storeOptionalCameraRawDepthMap(camera.id(), timestamp_ns, depth_map, &vi_mission)
   ```
