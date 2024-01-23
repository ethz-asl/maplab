# Connect to VPN so that github can be accessed from a terminal
# download and build maplab in a catkin workspace, ubuntu 18 ROS melodic
```
catkin clean
catkin config --merge-devel # for error 1

catkin build maplab
```

## Error 1
```
Errors     << aslam_cv_cameras:make /media/jhuai/docker/maplab2_ws/logs/aslam_cv_cameras/build.make.004.log                                                                                                
In file included from /media/jhuai/docker/maplab2_ws/src/maplab/aslam_cv2/aslam_cv_common/include/aslam/common/unique-id.h:12:0,
                 from /media/jhuai/docker/maplab2_ws/src/maplab/aslam_cv2/aslam_cv_common/include/aslam/common/sensor.h:7,
                 from /media/jhuai/docker/maplab2_ws/src/maplab/aslam_cv2/aslam_cv_cameras/include/aslam/cameras/camera.h:16,
                 from /media/jhuai/docker/maplab2_ws/src/maplab/aslam_cv2/aslam_cv_cameras/include/aslam/cameras/camera-unified-projection.h:4,
                 from /media/jhuai/docker/maplab2_ws/src/maplab/aslam_cv2/aslam_cv_cameras/src/camera-unified-projection.cc:3:
/media/jhuai/docker/maplab2_ws/src/maplab/aslam_cv2/aslam_cv_common/include/aslam/common/internal/unique-id.h:7:10: fatal error: aslam/common/id.pb.h: No such file or directory
 #include "aslam/common/id.pb.h"
          ^~~~~~~~~~~~~~~~~~~~~~
compilation terminated.

```
solution: catkin config --merge-devel 

source devel/setup.bash
rosrun rovioli tutorial_euroc /home/jhuai/Desktop/temp/maplab2 /media/jhuai/BackupPlus/jhuai/data/euroc/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
