![alt tag](images/banner_light.jpeg)


---

Tutorials
=========

This page lists the available tutorials for libpointmatcher. The [Beginner Section](#beginner) is aimed at the more casual user and contains high-level information on the various steps of point cloud registration. The [Advanced Section](#advanced) is targeted at those with existing experience with point cloud registration and proficiency in C++ development.  Those who wish to contribute to libpointmatcher can follow the guidelines in the [Developer](#developer) section.

Beginner<a name="beginner"></a>
---------

- [What is libpointmatcher about?](Introduction.md)
- [What can I do with libpointmatcher?](ApplicationsAndPub.md)
- [Ubuntu: How to compile libpointmatcher](Compilation.md)
- [Windows: How to compile libpointmatcher](CompilationWindows.md)
- [Mac OS X: How to compile libpointmatcher](CompilationMac.md)
- [What the different data filters do?](Datafilters.md)
- [Example: Applying a chain of data filters](ApplyingDatafilters.md)
- [Example: An introduction to ICP](ICPIntro.md)
- [The ICP chain configuration and its variants](DefaultICPConfig.md)
- [Configuring libpointmatcher using YAML](Configuration.md)
- [Supported file types and importing/exporting point clouds](ImportExport.md)

Advanced<a name="advanced"></a>
-------
- [How to link a project to libpointmatcher?](LinkingProjects.md)
- [How to use libpointmatcher in ROS?](UsingInRos.md)
- [How are point clouds represented?](Pointclouds.md)
- [Example: Writing a program which performs ICP](BasicRegistration.md)
- [How to move a point cloud using a rigid transformation?](Transformations.md)
- [Example: Configure an ICP solution without yaml](icpWithoutYaml.md)
- Measuring Hausdorff distance, Haussdorff quantile and mean residual error? [See this discussion for code examples.](https://github.com/ethz-asl/libpointmatcher/issues/125)
- How to compute the residual error with `ErrorMinimizer::getResidualError(...)` [See the example code provided here.](https://github.com/ethz-asl/libpointmatcher/issues/193#issue-203885636)
- How to I build a global map from a sequence of scans? [See the example align_sequence.cpp](../examples/align_sequence.cpp ).
- How to minimize the error with translation, rotation and scale? [See this example.](https://github.com/ethz-asl/libpointmatcher/issues/188#issuecomment-270960696)
- How to do a nearest neighbor search between two point clouds without an ICP object? [See the comments here.](https://github.com/ethz-asl/libpointmatcher/issues/193#issuecomment-276093785)
- How to construct a `DataPoints` from my own point cloud? [See the unit test on `Datapoints` here.](https://github.com/ethz-asl/libpointmatcher/blob/master/utest/ui/DataFilters.cpp#L52)

Developer<a name="developer"></a>
---------
- [Creating a DataPointsFilter](DataPointsFilterDev.md)
- [Creating a Transformation](TransformationDev.md)
- [Creating unit tests](UnitTestDev.md)

**Note**: if you don't find what you need, don't hesitate to propose or participate to new tutorials. 

---


![alt tag](images/banner_dark.jpeg)
