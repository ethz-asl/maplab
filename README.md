<img src="https://github.com/ethz-asl/maplab/wiki/logos/maplab_new.png" width="500">

*Ubuntu 14.04+ROS indigo* and *Ubuntu 16.04+ROS kinetic*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=maplab_nightly)](https://jenkins.asl.ethz.ch/job/maplab_nightly)

## News

 * **May 2018:** maplab was presented at [ICRA](https://icra2018.org/) in Brisbane. ([paper](https://arxiv.org/abs/1711.10250))
 * **March 2018:** Check out our release candidate with improved localization and lots of new features! [PR](https://github.com/ethz-asl/maplab/pull/55)
 
## Description

This repository contains **maplab**,  an  open,  research-oriented visual-inertial  mapping  framework, written  in  C++,  for  creating, processing  and  manipulating  multi-session  maps.
On  the  one  hand, maplab  can  be  considered  as  a  ready-to-use  visual-inertial mapping  and  localization  system.
On  the  other  hand,  maplab provides  the  research  community  with  a  collection  of  multi-session mapping tools that include map merging, visual-inertial batch optimization, and loop closure.

Furthermore, it includes an online frontend, **ROVIOLI**, that can create visual-inertial maps and also track a global drift-free pose within a localization map.

For documentation, tutorials and datasets, please visit the [wiki](https://github.com/ethz-asl/maplab/wiki).

Please also check out our video:

<a href="https://www.youtube.com/watch?v=CrfG4v25B8k"> <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/maplab_video_thumbnail.png" alt="https://www.youtube.com/watch?v=CrfG4v25B8k" width="600">

## Features

### Robust visual-inertial odometry with localization
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/rovio_stairs.gif" width="400">    <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/rviz_cla_vs.gif" width="400">

### Large-scale multisession mapping and optimization
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/largescale.gif" width="400">    <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/cla.png" width="400">

### Dense reconstruction
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/stereo.png" width="400">    <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/pmvs.png" width="400">

### A research platform extensively tested on real robots
<img src="https://github.com/ethz-asl/maplab/wiki/readme_images/topomap.png" width="400">    <img src="https://github.com/ethz-asl/maplab/wiki/readme_images/robots.jpg" width="400">

## Installation and getting started

The following articles help you with getting started with maplab and ROVIOLI:

- [Installation on Ubuntu 14.04 or 16.04](https://github.com/ethz-asl/maplab/wiki/Installation-Ubuntu)
- [Introduction to the maplab framework](https://github.com/ethz-asl/maplab/wiki/Introduction-to-the-Maplab-Framework)
- [Structure of the framework](https://github.com/ethz-asl/maplab/wiki/Structure-of-the-framework)
- [Running ROVIOLI in VIO mode](https://github.com/ethz-asl/maplab/wiki/Running-ROVIOLI-in-VIO-mode)
- [Basic console usage](https://github.com/ethz-asl/maplab/wiki/Basic-Console-Usage)
- [Console map management](https://github.com/ethz-asl/maplab/wiki/Console-map-management)

More detailed information can be found in the [wiki pages](https://github.com/ethz-asl/maplab/wiki).

## Research Results

The maplab framework has been used as an experimental platform for numerous scientific publications. For a complete list of publications please refer to [Research based on maplab](https://github.com/ethz-asl/maplab/wiki/Related-Research).

## Citing

Please cite the [following paper](https://arxiv.org/abs/1711.10250) when using maplab or ROVIOLI for your research:

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

Certain components of maplab are directly using the code of the following publications:

 * Localization:
   ```bibtex
   @inproceedings{lynen2015get,
     title={Get Out of My Lab: Large-scale, Real-Time Visual-Inertial Localization.},
     author={Lynen, Simon and Sattler, Torsten and Bosse, Michael and Hesch, Joel A and Pollefeys, Marc and Siegwart, Roland},
     booktitle={Robotics: Science and Systems},
     year={2015}
   }
   ```
  * ROVIOLI which is composed of ROVIO + maplab for map building and localization:
    ```bibtex
    @article{bloesch2017iterated,
      title={Iterated extended Kalman filter based visual-inertial odometry using direct photometric feedback},
      author={Bloesch, Michael and Burri, Michael and Omari, Sammy and Hutter, Marco and Siegwart, Roland},
      journal={The International Journal of Robotics Research},
      volume={36},
      number={10},
      pages={1053--1072},
      year={2017},
      publisher={SAGE Publications Sage UK: London, England}
    }
    ```

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
 
For a complete list of contributors, have a look at [CONTRIBUTORS.md](https://github.com/ethz-asl/maplab/blob/master/CONTRIBUTORS.md)
