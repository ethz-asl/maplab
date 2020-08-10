## Sensor Calibration Format: ncamera, imu-sigmas

[ROVIOLI](Running-ROVIOLI-in-VIO-mode) uses three different calibration files:
- `ncamera_calibration`
- `imu_parameters_maplab`
- `imu_parameters_rovio`


Additionally, the standard config file [`rovio_default_config.info`](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/rovio_default_config.info) is required to sit in the same directory as the calibration files. Please consult the [rovio Wiki pages](https://github.com/ethz-asl/rovio/wiki) for the meaning of the specific parameters.

### `ncamera_calibration`
The ncamera file determines the camera calibration. It can be obtained from tools like [Kalibr](Initial-sensor-calibration-with-Kalibr). E.g., the ncamera file may look like [the following](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/ncamera-euroc.yaml):

```yaml
label: "Euroc - original_calibration"
id: 412eab8e4058621f7036b5e765dfe812
cameras:
- camera:
    label: cam0
    id: 54812562fa109c40fe90b29a59dd7798
    line-delay-nanoseconds: 0
    image_height: 480
    image_width: 752
    type: pinhole
    intrinsics:
      cols: 1
      rows: 4
      data: [458.6548807207614, 457.2966964634893, 367.2158039615726, 248.37534060980727]
    distortion:
      type: radial-tangential  
      parameters:
        cols: 1
        rows: 4
        data: [-0.28340811217029355, 0.07395907389290132, 0.00019359502856909603,
    1.7618711454538528e-05]
  T_B_C:
    cols: 4
    rows: 4
    data: [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
           0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
          0.0, 0.0, 0.0, 1.0]
```

### `imu_parameters_maplab`
This file contains IMU parameters to be used in maplab. These are used within the bundle adjustment and other console commands. The `label` field in this file determines the rostopic for data to listen on. The file may look like [this](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/imu-adis16488.yaml):
```yaml
label: imu0
saturation_accel_max: 150.0
saturation_gyro_max: 7.5
gravity_magnitude: 9.81
imu_sigmas:
    acc_noise_density: 4e-3
    acc_bias_random_walk_noise_density: 4e-3
    gyro_noise_density: 1e-4
gyro_bias_random_walk_noise_density: 1e-4
```

### `imu_parameters_rovio`
This file contains the IMU noise parameters to be used within ROVIO. This is only used within ROVIO itself, and not in any console commands. Note, that these values may differ to the ones provided in the maplab file. The ROVIO IMU parameters file may look like [this](https://github.com/ethz-asl/maplab/blob/master/applications/rovioli/share/imu-sigmas-rovio.yaml):
```yaml
acc_noise_density: 1e-4
acc_bias_random_walk_noise_density: 1e-8
gyro_noise_density: 7.6e-7
gyro_bias_random_walk_noise_density: 3.8e-7
```
