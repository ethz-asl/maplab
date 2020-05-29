## Initial sensor calibration with Kalibr

[Kalibr](https://github.com/ethz-asl/kalibr) can be used to get [ncamera calibration file](Sensor-Calibration-Format#ncamera_calibration) necessary for maplab and ROVIOLI.

1. Create a new catkin workspace, clone and build Kalibr using the [installation instructions from the Kalibr wiki](https://github.com/ethz-asl/kalibr/wiki/installation#b-building-from-source).
2. Calibrate your sensor and obtain a Kalibr camchain yaml file. Check with the [Kalibr wiki](https://github.com/ethz-asl/kalibr/wiki) on how to do that. The relevant tutorials for a VI sensor: [[1] camera calibration](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration), [[2] camera-IMU calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
3. Convert the Kalibr output into the maplab format. Run the following command:
```bash
rosrun kalibr kalibr_maplab_config --to-ncamera \
    --label <your_label> \
    --cam <your_cam_chain_yaml>
```
where `<your_label>` is an arbitrary string to identify your ncamera setup and `<your_cam_chain_yaml>` refers to the file you receive from Kalibr in step 2.
