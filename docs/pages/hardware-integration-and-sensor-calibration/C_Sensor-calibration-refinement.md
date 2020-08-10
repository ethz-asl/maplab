## Sensor calibration refinement

### Refining the calibration using the optimization

An accurate calibration is necessary to obtain high-quality odometry estimates and to successfully refine the maps during the subsequent batch optimization. In most cases, maplab uses an (initial) sensor calibration estimated using external calibration tools such as [Kalibr](Initial-sensor-calibration-with-Kalibr). 

Optionally, the visual-inertial batch optimization can jointly estimate the intrinsics and extrinsics of the camera(s) together with the trajectory and the structure. 

When running optvi, you can unfix certain parts of the calibration using the following flags:
* ``--ba_fix_ncamera_intrinsics=false``
* ``--ba_fix_ncamera_extrinsics_rotation=false``
* ``--ba_fix_ncamera_extrinsics_translation=false``

These flags are all set to ``true`` by default. When starting from an inaccurate initial guess you may want to to unfix only some of them (e.g. extrinsics first), perform the optimization, unfix some others, optimize, etc. Unfortunately, this process requires some experimentation.

**Note:** Usually, you will need a good initial estimate for the calibration refinement to work. Additionally, your map should be long enough, contain enough excitation and preferably loopclose (using the ``lc`` command) to yield a good calibration.

### Inspecting and saving the calibration

After the optimization, you can inspect the calibration using the command:
```
print_camera_calibrations
```
This will print an output similar to:
```
Camera(54812562fa109c40fe90b29a59dd7798): cam0
  line delay: 0
  image (cols,rows): 752, 480
  focal length (cols,rows): 458.655, 457.297
  optical center (cols,rows): 367.216, 248.375
  distortion: 
Distortion: (RadTanDistortion) 
  k1 (radial):     -0.283408
  k2 (radial):     0.0739591
  p1 (tangential): 0.000193595
  p2 (tangential): 1.76187e-05
T_C_B:
 0.0148655   0.999557 -0.0257744  0.0652229
 -0.999881  0.0149672 0.00375619 -0.0207064
 0.0041403  0.0257155   0.999661 -0.0080546
         0          0          0          1
```
While it's impossible to tell if the calibration is accurate based on this output, you can easily detect that something went wrong as the calibration typically:
* has similar x-y focal length values, e.g. 458.655 vs 457.297),
* has the optical center located roughly in the middle of the image, e.g. (367.216, 248.375) for a (752, 480) image size,
* has a reasonable translation between the camera and the base/IMU frame (last column of T_C_B), compare it with manually measured values.

If you're happy with the results, you can export the calibration using the command:
```
export_ncamera_calibration --ncamera_calibration_export_folder your_folder
```
and use it in the future.
