## maplab_launch

This package contains the robot/sensor specific launch files and calibrations for:
 - maplab_node
 - rovioli
 - maplab_server_node

The intended folder layout is as follows:

```bash
 - config:
   - {robot-A}:
      - calib:
        - {robot-A}-maplab-calib-v1.yaml
        - {robot-A}-maplab-calib-v1.yaml
        - {robot-A}-okvis-calib-v1.yaml
      - rviz:
        - {robot-A}-application-1.yaml
        - {robot-A}-application-2.yaml
      - ros:
        - {robot-A}-msf-config.yaml
        - {robot-A}-maplab-node-rosparams.yaml
        - {robot-A}-rovioli-rosparam.yaml
   - {robot-B}:
     - ...
   - {sensor-X}:
     - ...
- launch:
  - {robot-A}:
    - {robot-A}-maplab-node.launch
    - {robot-A}-maplab-w-rovioli.launch
    - {robot-A}-maplab-w-okvis.launch
  - {robot-B}:
    - ...
  - {sensor-X}:
    - ...
```
