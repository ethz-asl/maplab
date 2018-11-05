#include "rovioli/datasource.h"

DEFINE_int64(
    rovioli_imu_to_camera_time_offset_ns, 0,
    "Fixed time offset of IMU to the camera, such that: t_imu - offset = "
    "t_cam");
