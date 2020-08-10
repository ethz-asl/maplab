# What is libpointmatcher about? 

libpointmatcher is a library that implements the Iterative Closest
Point (ICP) algorithm for alignment of point clouds. It supports both
point-to-point and point-to-plane ICP. With the former, it is able to
solve not only for a rigid transform, but also for a scale change
between the clouds (that is, a similarity transform).

More precisely, given two point clouds, R (the reference) and S (the
source), ICP tries to find the best rigid (or similarity) transform T
so that T * S = R.

The [Wikipedia article on ICP](https://en.wikipedia.org/wiki/Iterative_closest_point) has more
information.

libpointmatcher implements a [set of filters](Datafilters.md) to help
denoise and subsample the input point clouds. It supports a [variety
of file types](ImportExport.md) and it can be configured via both
[YAML files](Configuration.md) and an [in-memory API](icpWithoutYaml.md).

libpointmatcher is written in C++.


