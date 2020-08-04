## ROVIOLI Introduction

<img src="https://github.com/ethz-asl/maplab/wiki/images/rovioli-overview.png" width="650">

### Introduction

**ROVIOLI - RObust Visual Inertial Odometry  with Localization Integration,** is a visual-inertial mapping front-end for maplab that consists of an estimator, which is an adaptation of an existing estimation pipeline, [ROVIO](https://github.com/ethz-asl/rovio), and additional maplab modules for binary feature extraction and tracking, map building and localization.

ROVIOLI has two main purposes: 
 * to serve as a visual-inertial mapping front-end for maplab that can be used as is,
 * to serve as an example implementation of how to integrate other estimators into maplab.


### Features
 * Robust, real-time, visual-inertial 6DOF pose estimation
 * Builds maps for maplab
 * Localization from previous map of the environment.

### Workflow

<img src="https://github.com/ethz-asl/maplab/wiki/images/maplab-rovioli-workflow.png" width="500">

 * Create one or more maps of the environment. (a)
 * Load the maps in maplab and improve them by aligning multiple maps and applying loop closure and optimization. (b)
 * Run ROVIOLI in LOC mode and localize based on the previously build map. (c)

### Design

We use ROVIO as the core of our estimator mainly because it has been reliably deployed on many flying platforms by Autonomous Systems Lab MAV (micro aerial vehicle) team over the past couple of years. ROVIO is lightweight, fast, reliable. The features it tracks, which are based on image patches, have some complementary qualities compared to maplabs binary features (Brisk, FREAK), such as a better resistance to motion-blur. The photometric description of ROVIOs features, however, makes then less ideal to build VIMaps that are used for efficient loop closure and localization. That's why in parallel to ROVIO we run our own feature tracking module to get features that later can be used for map building. So it's important to note that the ROVIO features (visualized as green rectangles) are **NOT** used for map building at all. 

The map builder is composing a VIMap based on the keyframes and features tracked by the Feature Tracking module and the pose estimates of ROVIO, which serve as initial guess of the pose graph vertex poses. Even though a map is built at runtime, it only consists of vertices, edges and feature tracks. The landmarks are only initialized and triangulated right before saving the map, i.e. at shutdown time of ROVIOLI. Therefore ROVIOLI currently does not support any online mapping capabilities such as localizing from the map that is currently built.

For more details, please have a look at the [maplab](https://www.researchgate.net/profile/Marcin_Dymczyk/publication/321332545_Maplab_An_Open_Framework_for_Research_in_Visual-inertial_Mapping_and_Localization/links/5a1d46caa6fdcc0af326c67d/Maplab-An-Open-Framework-for-Research-in-Visual-inertial-Mapping-and-Localization.pdf) and [ROVIO](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/155340/eth-48374-01.pdf?sequence=1) paper.

### Known Issues

 * **Planar motion/ground vehicles:** Visual inertial mapping from single cameras can only get scale from the IMU. If ROVIOLI is deployed on a ground vehicle, the IMU is not excited in Z direction and if the vehicle moves at constant velocity there are barely any useful measurements to be obtained from the IMU. This has a significant impact on the estimation and can result in divergence, large drift error and scale drift.

 * **Changing the number of patches or other core ROVIO parameters:** If you want to change the number of patches tracked by ROVIO or any of the other core ROVIO parameters, you will need to adapt the code directly. Unlike the normal ROVIO version where you can set it at compile time with variables like these (`ROVIO_PATCHSIZE=4 ROVIO_NMAXFEATURE=250`).
