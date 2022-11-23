<img src="https://github.com/ethz-asl/maplab/wiki/logos/maplab_new.png" width="500">


*Ubuntu 18.04+ROS melodic*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=maplab_nightly)](https://jenkins.asl.ethz.ch/job/maplab_nightly)
[![Documentation Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=maplab_docs&subject=docs)](https://jenkins.asl.ethz.ch/job/maplab_docs)

## News

 * **May 2018:** maplab was presented at [ICRA](https://icra2018.org/) in Brisbane. [Paper](https://arxiv.org/abs/1711.10250) / [Initial Release](https://github.com/ethz-asl/maplab/releases/tag/initial_release)
 * **July 2018:** Check out our release candidate with improved localization and lots of new features! [Release 1.3](https://github.com/ethz-asl/maplab/releases/tag/1.3)
 * **November 2022:** maplab 2.0 initial release with new features and sensors

## Description

This repository contains **maplab 2.0**, an open research-oriented mapping framework, written in C++,  for multi-session and multi-robot mapping. For the original maplab release from 2018 the source code and documentation is available [here](https://github.com/ethz-asl/maplab/releases/tag/1.3).

**For documentation, tutorials and datasets, please visit the [wiki](https://maplab.asl.ethz.ch/index.html).**

## Features

### Robust visual-inertial odometry with localization
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/rovio_stairs.gif" width="400"> <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/rviz_cla_vs.gif" width="400">

### Large-scale multisession mapping and optimization
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/largescale.gif" width="400"> <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/cla.png" width="400">

### Dense reconstruction
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/stereo.png" width="400"> <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/pmvs.png" width="400">

### A research platform extensively tested on real robots
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/topomap.png" width="400"> <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/robots.jpg" width="400">

## Installation and getting started

The following articles help you with getting started with maplab and ROVIOLI:

- [Installation on Ubuntu 18.04 or 20.04](https://maplab.asl.ethz.ch/docs/master/pages/installation/A_Installation-Ubuntu.html)
- [Introduction to the maplab framework](https://maplab.asl.ethz.ch/docs/master/pages/overview_and_introduction/A_The-Maplab-Framework.html)
- [Running ROVIOLI in VIO mode](https://maplab.asl.ethz.ch/docs/master/pages/tutorials-rovioli/B_Running-ROVIOLI-in-VIO-mode.html)
- [Basic console usage](https://maplab.asl.ethz.ch/docs/master/pages/tutorials-maplab/basics/A_Basic-Console-Usage.html)
- [Console map management](https://maplab.asl.ethz.ch/docs/master/pages/tutorials-maplab/basics/C_Console-map-management.html)

**More detailed information can be found in the [wiki pages](https://maplab.asl.ethz.ch/index.html).**

## Research Results

The maplab framework has been used as an experimental platform for numerous scientific publications. For a complete list of publications please refer to [Research based on maplab](https://maplab.asl.ethz.ch/docs/master/pages/overview_and_introduction/C_Related-Research.html).

## Citing

Please cite the [following paper](https://arxiv.org/abs/1711.10250) when using maplab for your research:

```bibtex
@article{schneider2018maplab,
  title={maplab: An Open Framework for Research in Visual-inertial Mapping and Localization},
  author={T. Schneider and M. T. Dymczyk and M. Fehr and K. Egger and S. Lynen and I. Gilitschenski and R. Siegwart},
  journal={IEEE Robotics and Automation Letters},
  year={2018},
  doi={10.1109/LRA.2018.2800113}
}
```

### Additional Citations

Certain components of maplab are directly based on [other publications](https://maplab.asl.ethz.ch/docs/master/pages/overview_and_introduction/B_Citing-Maplab).


## Credits

 * Thomas Schneider
 * Marcin Dymczyk
 * Marius Fehr
 * Kevin Egger
 * Simon Lynen
 * Mathias Bürki
 * Titus Cieslewski
 * Timo Hinzmann
 * Mathias Gehrig
 * Florian Tschopp
 * Andrei Cramariuc
 * Lukas Bernreiter

For a complete list of contributors, have a look at [CONTRIBUTORS.md](https://github.com/ethz-asl/maplab/blob/master/CONTRIBUTORS.md)
