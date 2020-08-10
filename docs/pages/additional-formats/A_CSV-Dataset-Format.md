## CSV Dataset Format

It is often useful to interface the visual-inertial maps with other research tools (such as Python scripts, Matlab, deep learning frameworks). A consistent map constructed from multiple recordings and spanning a large area and a timeframe is very valuable for lifelong mapping, place recognition and other experiments. Maplab contains a CSV exporter that outputs the data stored in the map to a an easily readable format that can be universally accessed.

This page describes the output of `csv_export` command in the maplab console.
The command operates per mission and creates a folder named after the mission id in the export path. For each mission, the following data is exported:

### Vertices
File name: `vertices.csv`

| Column | Description | Unit | Frame |
|--:|:--|:--:|:--:|
1 | vertex index | - | -
2 | timestamp | ns | -
3 | position x | m | global
4 | position y | m | global
5 | position z | m | global
6 | orientation quaternion x | - | global
7 | orientation quaternion y | - | global
8 | orientation quaternion z | - | global
9 | orientation quaternion w | - | global
10 | velocity x | m/s | mission
11 | velocity y | m/s | mission
12 | velocity z | m/s | mission
13 | accelerometer bias x | m/s^2 | imu/sensor
14 | accelerometer bias y | m/s^2 | imu/sensor
15 | accelerometer bias z | m/s^2 | imu/sensor
16 | gyroscope bias x | rad/s | imu/sensor
17 | gyroscope bias y | rad/s | imu/sensor
18 | gyroscope bias z | rad/s | imu/sensor

### IMU data
File name: `imu.csv`

| Column | Description | Unit | Frame |
|--:|:--|:--:|:--:|
1 | IMU timestamp | ns | -
2-7  | IMU data 1 (6 values) | rad/s and m/s^2 | imu/sensor

### Tracks/keypoints
File name: `tracks.csv`

| Column | Description | Unit | Frame |
|--:|:--|:--:|:--:|
1 | timestamp | ns | -
2 | vertex index | - | -
3 | frame index | - | -
4 | keypoint index | - | -
5 | keypoint measurement 0 | px | distorted img
6 | keypoint measurement 1 | px | distorted img
7 | keypoint measurement uncertainty | - | -
8 | keypoint scale | - | -
9 | keypoint track id | - | -

### Descriptors
File name: `descriptors.csv`

| Column | Description | Unit | Frame |
|--:|:--|:--:|:--:|
1-N | descriptor byte as integer | - | -

The rows are ordered the same as in the tracks file.

### Landmarks
File name: `landmarks.csv`

| Column | Description | Unit | Frame |
|--:|:--|:--:|:--:|
1 | landmark index | - | -
2 | landmark position x | m | global
3 | landmark position y | m | global
4 | landmark position z | m | global

### Observations
File name: `observations.csv`

| Column | Description | Unit | Frame |
|--:|:--|:--:|:--:|
1 | vertex index | - | -
2 | frame index | - | -
3 | keypoint index | - | -
4 | landmark index | - | -
