## Introduction to the Maplab Framework

### Terminology ###
* **Map:** A representation of the environment generated through mapping and used for localization.
* **Mapping:** The process in which the robot builds a model of the environment by fusing the output from one or multiple sensors.
* **Localization:** The process in which the robot infers its position relative wrt. a model of the environment.
* **SLAM**: Simultaneous Localization and Mapping is the process which the robot alternates between localization and mapping to build a map in a previously unknown environment while at the same time localizing from it.
* **Landmark** A point which was visible from multiple consecutive camera images. Given this information the point's position can be triangulated.
* **Loop-Closure**: The process of recognizing previously visited places which allows the robot to correct for any error accumulated during exploration of the environment. Similar to us humans recognizing places which then tells us sth. about where we are in the world.
* **Dense-Reconstruction**: The process of building a 3d-model with a high number of 3d-points from images.

### Overview ###
A modern autonomous robotic system is composed of many software building blocks (SLAM, path planning, scene interpretation), most of them depending on some form of localization and mapping. Therefore, there is a need for an API that allows these different elements to communicate, with as little interdependency as possible.

#### The Long-Term Localization Loop ####
In order for a robot to be able to perform long-term operations the above algorithms need to be tightly integrated s.t. they can complement one another. We call this combination the **Lifelong Localization Loop**:

![](https://cloud.githubusercontent.com/assets/966785/6546069/2a4ce0b2-c5a4-11e4-8613-ddcf5e8b0591.png)

* **Online localization and registration:** Real-time ego-motion estimation (SLAM) from the robot's sensors is combined with updates from a previously known map (Localization). This allows the robot to continuously estimate it's pose in a global frame of reference and thus execute it's tasks.
* **Offline geometric alignment:** The data captured online by the agents needs to be aligned and registered to previously known reference data such as models of buildings or maps created by other agents. This is most commonly done by using loop-closure algorithms. The combined data is then jointly optimized to achieve the best (maximum likelihood) estimate for all involved quantities (3d-structure, trajectories etc.)
* **Map summarization:** In order to generate a compact representation of the environment we need to remove redundant information from the maps and focus on the most informative parts. The resulting model should contain only the part of the data which is most informative for localization (step 1).
* **3d-reconstruction:** In order for the robot to be able to avoid obstacles and navigate the environment it is important to build a richer representation of the environment than what is given by the SLAM algorithm from step 1.
