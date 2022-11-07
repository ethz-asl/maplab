## External Features

The maplab framework now supports the addition of external visual features. This is done through the interfaces defined in this [message](https://github.com/ethz-asl/maplab_msgs/blob/487fea2827ecabc30092586d7d3c295f9159a742/msg/Features.msg).


### Setting up

To notify the maplab node to expect external features, one has to modify the calibration file to add a new type of external features sensor. An example of how this is done for the HITLI datasets is shown [here](https://github.com/ethz-asl/maplab/blob/master/maplab-launch/config/hilti/calib/stick-sensors-2021.yaml).

In parallel with maplab one can then run the external features detection module which is implemented [here](https://github.com/ethz-asl/maplab_features/tree/devel/extract_all). There are example launch files for various supported feature types:

- [Surf features with LK tracking](https://github.com/ethz-asl/maplab_features/blob/devel/extract_all/launch/lk_surf.launch).
- [SuperPoint with SuperGlue](https://github.com/ethz-asl/maplab_features/blob/devel/extract_all/launch/super.launch).
