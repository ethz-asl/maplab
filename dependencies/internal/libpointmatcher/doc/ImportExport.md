| [Tutorials Home](index.md)    | [Previous](Configuration.md) | [Next](LinkingProjects.md) |
| ------------- |:-------------:| -----:|
# Importing and Exporting Point Clouds

## Overview
There exists a myriad of [graphics file formats](http://en.wikipedia.org/wiki/Category:Graphics_file_formats) which can be used to represent point clouds.  Nevertheless most contain superfluous functionality which isn't necessary for ICP.  Consequently, libpointmatcher only supports a small number of file formats which are detailed in the following sections.

## Table of Supported File Formats
| File Type | Extension | Versions Supported | Descriptors Supported | Additional Information |
| --------- |:---------:|:------------------:|:---------------------:|---------|
| Comma Separated Values | .csv | NA | yes (see [table of descriptor labels](#descmaptable)) | |
| Visualization Toolkit Files | .vtk | Legacy format versions 3.0 and lower (ASCII only) | yes | Only polydata and unstructured grid VTK Datatypes supported.  More information can be found  [here](http://www.vtk.org/VTK/img/file-formats.pdf).|
| Polygon File Format | .ply | 1.0 (ASCII only) | yes (see [table of descriptor labels](#descmaptable)) | | 
| Point Cloud Library Format | .pcd | 0.7 (ASCII only) | yes (see [table of descriptor labels](#descmaptable)) | |

## Comma Separated Values (CSV) Files
The most simple file format supported to store clouds is a plain text file containing comma separated values.  Data is structured in a table format and is separated into rows or lines and columns.  Columns are delimited by commas, tabs, or semicolons.  2D and 3D point features are supported as well as a limited number of point descriptors.

It is common and best practice that the first line of the CSV file be a header identifying each data column.  Features are identified by the following names: "x", "y", and "z" if the point cloud in 3D.  Descriptors are divided into columns and these must be appropriately identified.  

If the first line does not include a header, the user will be asked to identify which columns correspond to the "x", "y", and "z" feature fields.  Any additional content including descriptors is ignored.

## Visualization Toolkit (VTK Legacy) Files
So as to allow the visualization of point clouds in [Paraview](http://www.paraview.org/), libpointmatcher supports importing and exporting of Paraview's native VTK format.  VTK files can be used to represent a wide variety of 3D graphics including point clouds.  The VTK standard comprises of a simpler legacy format and a newer XML based format.  As of now libpointmatcher only supports the legacy format.

VTK files can contain different geometrical topologies which are represented in different dataset types.  These types each have different file structures and can be one of the following :
    
* STRUCTURED_POINTS
* STRUCTURE_GRID
* UNSTRUCTURED_GRID
* POLYDATA
* RECTILINEAR_GRID
* FIELD

As of writing, libpointmatcher only supports the POLYDATA (used to represent polygonal data) and UNSTRUCTURED_GRID (used to describe any unstructured combination of geometrical cells) types.  While VTK files can be encoded in plain text (ASCII) or binary, only ASCII files are supported.

Descriptors are encoded in VTK files as dataset attributes which can be any of the following all of which are supported in libpointmatcher:

* SCALARS
* COLOR_SCALARS
* VECTORS
* NORMALS
* TENSORS 

Data contained in these attributes are loaded into the Datapoints feature matrices.  In the same way, Dataset descriptors are converted to the appropriate VTK attributes when exporting a point cloud.  The following table details which descriptors are exported to VTK files and the corresponding VTK attribute that is used to encode them.

*Mapping Between libpointmatcher Descriptors and VTK attributes*

| libpointmatcher Descriptor Label | VTK Dataset Attribute |
| ----------------------------- | --------------------- | 
| normals                       | NORMALS               |
| eigVectors                    | TENSOR                | 
| color                         | COLOR_SCALARS         | 
| Any other 1D descriptor       | SCALARS               |
| Any other 3D descriptor       | VECTORS               |  

## Polygon File Format (PLY or Stanford Triangle) Files
The PLY file format was developed at the Stanford Graphics Lab for storing 3D graphics in a relatively straight-forward fashion.  PLY files are flexible and data is structured by defining elements and properties.  Elements usually represent some geometrical construct such as a vertex or triangular face.  Elements are associated to scalar properties such as the x, y, z coordinate in the case of vertices.  Properties can contain any information however including colors, normal vector components, densities etc...

PLY files contain a header section at the top of the file which defines the elements and properties that are used in the file.  The rest of the file contains numerical data.  The PLY format exists in plain text (ASCII) and in binary, however libpointmatcher only supports the plain text version.

The PLY format does not prescribe labels to elements or properties, and therefore files must be encoded with appropriate labels in order to be read by libpointmatcher.  For information on which properties are supported by libpointmatcher, refer to [this table](#descmaptable).

## Point Cloud Library File Format (PCD) Files
The [Point Cloud Library](http://pointclouds.org/)(PCL) is an alternative library for handling 2D and 3D point clouds.  While libpointmatcher only performs the task of and is optimized for point cloud registration, PCL is widespread in its functionality.

The developers of PCL have developed their [own file format](http://pointclouds.org/documentation/tutorials/pcd_file_format.php) for storing point clouds.  libpointmatcher is compatible with this format and can import and export PCD files in the latest format (v 0.7).

The PCD format also exists in binary, however only the plain text (ASCII) version is supported.  Because PCD does not prescribe standards for descriptors, libpointmatcher utilizes the [same identifier mapping](#descmaptable) for identifying descriptors.   

## Descriptor Property Identifiers (PLY, CSV, PCD) <a name="descmaptable"></a>

| Property Label | Description | Feature or Descriptor | libpointmatcher Descriptor Label |
| -------------- | -------------| --------------------- | ---------------- |
| x              | x component of point | feature               |   NA        |
| y              | y component of point | feature               |     NA      |
| z              | z component of  point | feature               |      NA     |
| nx             | x component of normal vector at point | descriptor            | normals          |
| ny             | y component of normal vector at point | descriptor            | normals          |
| nz             | z component of normal vector at point | descriptor            | normals          |
| normal_x             | x component of normal vector at point | descriptor            | normals          |
| normal_y             | y component of normal vector at point | descriptor            | normals          |
| normal_z             | z component of normal vector at point | descriptor            | normals          |
| red            | red value (0.0-1.0) of RGB color code | descriptor |  color |
| green          | green value (0.0-1.0) of RGB color code | descriptor |  color |
| blue           | blue value (0.0-1.0) of RGB color code | descriptor |  color |
| alpha          | alpha value (0.0-1.0) of RGBA color code | descriptor | color | 
| intensity    | laser scan intensity at point | descriptor | intensity |
| eigValues0-2           | eigen values of nearest neighbors at point. Format is eigValues followed by the number of the eigen value (2 for 2D and 3 for 3D) | descriptor | eigValue |
| eigVectors0-2X-Z       | eigen vectors of nearest neighbors at point.  Format is eigVectors followed by the number of the eigen vector (2 for 2D and 3 for 3D) followed by the axis identifier (X, Y or Z) | descriptor | eigVectors |
| densities | point density at point | descriptor | densities |

Several identifiers may synonymously point to the same libpointmatcher descriptor.  For example, some standards may define the x normal component as *nx* and others as *normal_x*.  **Each file should only use one definition for each descriptor.  For example, it should never contain both "nx" and "normal_x" fields.**

While most files should contain data structured in a natural order ie ("x", "y" then "z", or "red", "green", then "blue") this cannot be guaranteed.  Therefore libpointmatcher will attempt to reorder the descriptor components when loading a file, and will always export them in a natural order.

---
### Note For libpointmatcher Developers
The association between descriptor properties identifiers and libpointmatcher descriptor labels is set in the `getDescAssocationMap` function in [pointmatcher/IO.cpp](/pointmatcher/IO.cpp).  To extend IO support to additional descriptors, you can modify this function.

The `getDescAssocationMap` returns a map which associates a property identifier and a pair consisting of a row number and a point matcher descriptor name.  For example, the descriptor identifier *nx* maps to row 0 of the *normals* libpointmatcher descriptor. Ie:

"nx" -> (0, "normals") <br>
"ny" -> (1, "normals") <br>
"nz" -> (2, "normals")

For converting libpointmatcher descriptors back to a property identifier, you must modify the  `getColLabel` function in [pointmatcher/IO.cpp](/pointmatcher/IO.cpp).
