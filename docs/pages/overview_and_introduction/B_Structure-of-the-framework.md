## Structure of the framework

The maplab framework consists of two major parts: 

<img src="https://github.com/ethz-asl/maplab/wiki/logos/rovioli.png" width="250"><br>
<img src="https://github.com/ethz-asl/maplab/wiki/logos/maplab.png" width="250">

an online frontend ROVIOLI and the offline maplab console.

### ROVIOLI frontend

* online frontend for mapping and localization,
* extends [ROVIO](https://github.com/ethz-asl/rovio) with 6DoF localization constraints,
* standalone operation for single-sessions,
* multi-session map support using the console (see below).

### Offline maplab console 

It permits to refine the map, merge multiple maps together and use them e.g. for the dense reconstruction. The features include, among others, the following:
* a convenient console user interface and a map manager to access the maps,
* a plug-in architecture to easily extend it with new commands and algorithms,
* visual-inertial least-squares optimization that can be extended with additional sensors,
* robust pose-graph relaxation using switchable constraints,
* BRISK/FREAK-based loopclosure,
* map summarization for lifelong mapping,
* dense reconstruction and an interface to [Voxblox](https://github.com/ethz-asl/voxblox).

### Typical workflow with maplab
<img src="https://github.com/ethz-asl/maplab/wiki/images/diagrams/maplab_dataflow.png" width="500">

* (a) Build maps using ROVIOLI in the VIO mode.
* (b) Refine the maps using the maplab console. Merge multiple maps together. Perform experiments and apply your algorithms of choice.
* (c) optional: Use the resulting map to localize using ROVIOLI in the Localization Mode. The localization increases the accuracy of the visual-inertial pose estimation.

