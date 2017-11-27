# Welcome to maplab

Maplab                     |  Rovioli
:-------------------------:|:-------------------------:
![sheep](https://user-images.githubusercontent.com/5071588/32980512-eefc7176-cc67-11e7-953b-677231abf480.png)  |  ![rovioli](https://user-images.githubusercontent.com/5337083/30172628-ab8c8a16-93f5-11e7-975f-168c1f62e342.png)

This repository contains **maplab**,  an  open,  research-oriented visual-inertial  mapping  framework, written  in  C++,  for  processing  and  manipulating  multi-session  maps.
On  the  one  hand, maplab  can  be  considered  as  a  ready-to-use  visual-inertial mapping  and  localization  system.
On  the  other  hand,  maplab provides  the  research  community  with  a  collection  of  multi-session mapping tools that include map merging, visual-inertial batch optimization, and loop closure.
Furthermore, it includes an online frontend, **ROVIOLI**, that can create visual-inertial maps and also track a global drift-free pose within a localization map.

For documentation, tutorials and datasets, please visit the [wiki](https://github.com/ethz-asl/maplab/wiki).

Please cite the following paper when using maplab for your research:

```bibtex
@article{schneider2018maplab,
  title={maplab: An Open Framework for Research in Visual-inertial Mapping and Localization},
  author={Schneider, Thomas, Dymczyk, Marcin, Marius, Fehr, Egger, Kevin,
          Lynen, Simon, Gilitschenski, Igor, and Siegwart, Roland
  year={2018}
}
```

## Credits

### Authors

 * Thomas Schneider
 * Marcin Dymczyk
 * Marius Fehr
 * Kevin Egger
 * Simon Lynen
 * Mathias Bürki
 * Titus Cieslewski
 * Timo Hinzmann
 * Mathias Gehrig
 
For a complete list of contributors, have a look at CONTRIBUTORS.md


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

### Topomap: Topological Mapping and Navigation Based on Visual SLAM Maps
bibtex:
```bibtex
@article{blochliger2017topomap,
  title={Topomap: Topological Mapping and Navigation Based on Visual SLAM Maps},
  author={Bl{\"o}chliger, Fabian and Fehr, Marius and Dymczyk, Marcin and Schneider, Thomas and Siegwart, Roland},
  journal={arXiv preprint arXiv:1709.05533},
  year={2017}
}
```  
Video:

[!["Topomap: Topological Mapping and Navigation Based on Visual SLAM Maps by Fabian Bloechliger](https://img.youtube.com/vi/UokjxSLTcd0/hqdefault.jpg)](https://www.youtube.com/watch?v=UokjxSLTcd0)

---

### Collaborative Navigation for Flying and Walking Robots
bibtex:
```bibtex
@inproceedings{fankhauser2016collaborative,
  title={Collaborative navigation for flying and walking robots},
  author={Fankhauser, P{\'e}ter and Bloesch, Michael and Kr{\"u}si, Philipp and Diethelm, Remo and Wermelinger, Martin and Schneider, Thomas and Dymczyk, Marcin and Hutter, Marco and Siegwart, Roland},
  booktitle={Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on},
  pages={2859--2866},
  year={2016},
  organization={IEEE}
}
```
Video:

[!["Collaborative Navigation for Flying and Walking Robots" by Peter Fankhauser](https://img.youtube.com/vi/9PprNdIKRaw/hqdefault.jpg)](https://www.youtube.com/watch?v=9PprNdIKRaw)

---

### Real-time visual-inertial mapping, re-localization and planning onboard mavs in unknown environments
bibtex:
```bibtex
@inproceedings{burri2015real,
  title={Real-time visual-inertial mapping, re-localization and planning onboard mavs in unknown environments},
  author={Burri, Michael and Oleynikova, Helen and Achtelik, Markus W and Siegwart, Roland},
  booktitle={Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on},
  pages={1872--1878},
  year={2015},
  organization={IEEE}
}
```
Video:

[!["Real-time visual-inertial mapping, re-localization and planning onboard mavs in unknown environments" by Michael Burri](https://img.youtube.com/vi/tuUMwcTJx8s/hqdefault.jpg)](https://www.youtube.com/watch?v=tuUMwcTJx8s)

---

### Appearance-based landmark selection for efficient long-term visual localization
bibtex:
```bibtex
@inproceedings{burki2016appearance,
  title={Appearance-based landmark selection for efficient long-term visual localization},
  author={B{\"u}rki, Mathias and Gilitschenski, Igor and Stumm, Elena and Siegwart, Roland and Nieto, Juan},
  booktitle={Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on},
  pages={4137--4143},
  year={2016},
  organization={IEEE}
}
```
Video:

[!["Appearance-based landmark selection for efficient long-term visual localization" by Mathias Buerki](https://img.youtube.com/vi/JL_5zMEQKYc/hqdefault.jpg)](https://www.youtube.com/watch?v=JL_5zMEQKYc)

---

### Reshaping our model of the world over time
bibtex:
```
@inproceedings{fehr2016reshaping,
  title={Reshaping our model of the world over time},
  author={Fehr, Marius and Dymczyk, Marcin and Lynen, Simon and Siegwart, Roland},
  booktitle={Robotics and Automation (ICRA), 2016 IEEE International Conference on},
  pages={2449--2455},
  year={2016},
  organization={IEEE}
}
```
Video:

[!["Reshaping our model of the world over time" by Marius Fehr](https://img.youtube.com/vi/ZX-LLnv_vzA/hqdefault.jpg)](https://www.youtube.com/watch?v=ZX-LLnv_vzA)

---

### Keep it brief: Scalable creation of compressed localization maps
bibtex:
```bibtex
@inproceedings{dymczyk2015keep,
  title={Keep it brief: Scalable creation of compressed localization maps},
  author={Dymczyk, Marcin and Lynen, Simon and Bosse, Michael and Siegwart, Roland},
  booktitle={Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on},
  pages={2536--2542},
  year={2015},
  organization={IEEE}
}
```
Video:

[!["Keep it brief: Scalable creation of compressed localization maps" by Marcin Dymczyk](https://img.youtube.com/vi/QrXav6oaacE/hqdefault.jpg)](https://www.youtube.com/watch?v=QrXav6oaacE)

---

### The gist of maps-summarizing experience for lifelong localization
bibtex:
```bibtex
@inproceedings{dymczyk2015gist,
  title={The gist of maps-summarizing experience for lifelong localization},
  author={Dymczyk, Marcin and Lynen, Simon and Cieslewski, Titus and Bosse, Michael and Siegwart, Roland and Furgale, Paul},
  booktitle={Robotics and Automation (ICRA), 2015 IEEE International Conference on},
  pages={2767--2773},
  year={2015},
  organization={IEEE}
}
```
Video:

[!["The gist of maps-summarizing experience for lifelong localization" by Marcin Dymczyk](https://img.youtube.com/vi/5I2SPZJi2NE/hqdefault.jpg)](https://www.youtube.com/watch?v=5I2SPZJi2NE)

---

### Real-time visual-inertial localization for aerial and ground robots
bibtex:
```bibtex
@inproceedings{oleynikova2015real,
  title={Real-time visual-inertial localization for aerial and ground robots},
  author={Oleynikova, Helen and Burri, Michael and Lynen, Simon and Siegwart, Roland},
  booktitle={Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on},
  pages={3079--3085},
  year={2015},
  organization={IEEE}
}
```
Video:

[!["Real-time visual-inertial localization for aerial and ground robots" by Helen Oleynikova](https://img.youtube.com/vi/Mb5PF4eNBvs/hqdefault.jpg)](https://www.youtube.com/watch?v=Mb5PF4eNBvs)

---

### Map API-scalable decentralized map building for robots
bibtex:
```bibtex
@inproceedings{cieslewski2015map,
  title={Map API-scalable decentralized map building for robots},
  author={Cieslewski, Titus and Lynen, Simon and Dymczyk, Marcin and Magnenat, St{\'e}phane and Siegwart, Roland},
  booktitle={Robotics and Automation (ICRA), 2015 IEEE International Conference on},
  pages={6241--6247},
  year={2015},
  organization={IEEE}
}
```
Video:

[!["Map API-scalable decentralized map building for robots" by Titus Cieslewski](https://img.youtube.com/vi/XNyGzxexxm0/hqdefault.jpg)](https://www.youtube.com/watch?v=XNyGzxexxm0)

---

### Additional Citations

The following list contains a list of paper on which specific components of maplab are built on:

 * Localization:
   ```bibtex
   @inproceedings{lynen2015get,
     title={Get Out of My Lab: Large-scale, Real-Time Visual-Inertial Localization.},
     author={Lynen, Simon and Sattler, Torsten and Bosse, Michael and Hesch, Joel A and Pollefeys, Marc and Siegwart, Roland},
     booktitle={Robotics: Science and Systems},
     year={2015}
   }
   ```
 * 3D reconstruction using the PMVS/CMVS exporter:
   ```
   @inproceedings{furukawa2010towards,
      title={Towards internet-scale multi-view stereo},
      author={Furukawa, Yasutaka and Curless, Brian and Seitz, Steven M and Szeliski, Richard},
      booktitle={Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on},
      pages={1434--1441},
      year={2010},
      organization={IEEE}
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
