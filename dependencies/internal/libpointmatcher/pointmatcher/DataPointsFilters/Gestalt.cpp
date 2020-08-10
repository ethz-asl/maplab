// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "Gestalt.h"

#include "utils.h"

#include <cmath>

// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "PointMatcherPrivate.h"

#include <vector>

//////////////////////////////////////////////////////////////////////////////////////

// GestaltDataPointsFilter

// Constructor
template<typename T>
GestaltDataPointsFilter<T>::GestaltDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("GestaltDataPointsFilter", 
		GestaltDataPointsFilter::availableParameters(), params),
	ratio(Parametrizable::get<T>("ratio")),
	radius(Parametrizable::get<T>("radius")),
	knn(Parametrizable::get<int>("knn")),
	vSizeX(Parametrizable::get<T>("vSizeX")),
	vSizeY(Parametrizable::get<T>("vSizeY")),
	vSizeZ(Parametrizable::get<T>("vSizeZ")),
	maxBoxDim(Parametrizable::get<T>("maxBoxDim")),
	maxTimeWindow(Parametrizable::get<T>("maxTimeWindow")),
	keepMeans(Parametrizable::get<bool>("keepMeans")),
	averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors")),
	keepNormals(Parametrizable::get<bool>("keepNormals")),
	keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
	keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors")),
	keepCovariances(Parametrizable::get<bool>("keepCovariances")),
	keepGestaltFeatures(Parametrizable::get<bool>("keepGestaltFeatures"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
GestaltDataPointsFilter<T>::filter(const DataPoints& input)
{
  DataPoints output(input);
  inPlaceFilter(output);
  return output;
}

// In-place filter
template<typename T>
void GestaltDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
  typedef typename DataPoints::View View;
  typedef typename DataPoints::Label Label;
  typedef typename DataPoints::Labels Labels;
  typedef typename DataPoints::TimeView TimeView;

  const int pointsCount(cloud.features.cols());
  const int featDim(cloud.features.rows());
  const int descDim(cloud.descriptors.rows());
  const unsigned int labelDim(cloud.descriptorLabels.size());

  int insertDim(0);
  if (averageExistingDescriptors)
  {
    // TODO: this should be in the form of an assert
    // Validate descriptors and labels
    for(unsigned int i = 0; i < labelDim; ++i)
      insertDim += cloud.descriptorLabels[i].span;
    if (insertDim != descDim)
      throw InvalidField("GestaltDataPointsFilter: Error, descriptor labels do not match descriptor data");
  }

  // Compute space requirement for new descriptors
  const int dimNormals(featDim-1);
  const int dimMeans(featDim-1);
  const int dimEigValues(featDim-1);
  const int dimEigVectors((featDim-1)*(featDim-1));
  const int dimCovariances((featDim-1)*(featDim-1));
  const int dimGestalt = 32;

  // Allocate space for new descriptors
  Labels cloudLabels, timeLabels;

  if (keepNormals)
    cloudLabels.push_back(Label("normals", dimNormals));
  if (keepMeans)
    cloudLabels.push_back(Label("means", dimMeans));
  if (keepEigenValues)
    cloudLabels.push_back(Label("eigValues", dimEigValues));
  if (keepEigenVectors)
    cloudLabels.push_back(Label("eigVectors", dimEigVectors));
  if (keepCovariances)
    cloudLabels.push_back(Label("covariance", dimCovariances));
  if (keepGestaltFeatures) 
  {
    cloudLabels.push_back(Label("gestaltMeans", dimGestalt));
    cloudLabels.push_back(Label("gestaltVariances", dimGestalt));
    cloudLabels.push_back(Label("warpedXYZ", 3));
    cloudLabels.push_back(Label("gestaltShapes", 2));
  }
  timeLabels.push_back(Label("time", 3));

  cloud.allocateDescriptors(cloudLabels);
  cloud.allocateTimes(timeLabels);

  // we keep build data on stack for reentrant behaviour
  View cloudExistingDescriptors(cloud.descriptors.block(0,0,cloud.descriptors.rows(),cloud.descriptors.cols()));
  TimeView cloudExistingTimes(cloud.times.block(0,0,cloud.times.rows(),cloud.times.cols()));
  BuildData buildData(cloud.features, cloud.descriptors, cloud.times);

  // get views
  if (keepNormals)
    buildData.normals = cloud.getDescriptorViewByName("normals");
  if(keepMeans)
    buildData.means = cloud.getDescriptorViewByName("means");
  if (keepEigenValues)
    buildData.eigenValues = cloud.getDescriptorViewByName("eigValues");
  if (keepEigenVectors)
    buildData.eigenVectors = cloud.getDescriptorViewByName("eigVectors");
  if (keepCovariances)
    buildData.covariance = cloud.getDescriptorViewByName("covariance");
  if (keepGestaltFeatures) 
  {
    buildData.gestaltMeans = cloud.getDescriptorViewByName("gestaltMeans");
    buildData.gestaltVariances = cloud.getDescriptorViewByName("gestaltVariances");
    buildData.warpedXYZ = cloud.getDescriptorViewByName("warpedXYZ");
    buildData.gestaltShapes = cloud.getDescriptorViewByName("gestaltShapes");
  }
  // build the new point cloud
  buildNew(
      buildData,
      0,
      pointsCount,
      cloud.features.rowwise().minCoeff(),
      cloud.features.rowwise().maxCoeff()
  );

  // buildData.indicesToKeep contains all the indices where we want Gestalt features at
  fuseRange(buildData, cloud, 0, pointsCount);

  // Bring the data we keep to the front of the arrays then
  // wipe the leftover unused space.
  std::sort(buildData.indicesToKeep.begin(), buildData.indicesToKeep.end());
  const int ptsOut = buildData.indicesToKeep.size();
  for (int i = 0; i < ptsOut; ++i)
  {
    const int k = buildData.indicesToKeep[i];
    assert(i <= k);
    cloud.features.col(i) = cloud.features.col(k);
    cloud.times.col(i) = cloud.times.col(k);
    if (cloud.descriptors.rows() != 0)
      cloud.descriptors.col(i) = cloud.descriptors.col(k);
    if(keepNormals)
      buildData.normals->col(i) = buildData.normals->col(k);
    if(keepMeans)
      buildData.means->col(i) = buildData.means->col(k);
    if(keepEigenValues)
      buildData.eigenValues->col(i) = buildData.eigenValues->col(k);
    if(keepEigenVectors)
      buildData.eigenVectors->col(i) = buildData.eigenVectors->col(k);
    if(keepCovariances)
      buildData.covariance->col(i) = buildData.covariance->col(k);
    if(keepGestaltFeatures) 
    {
      buildData.gestaltMeans->col(i) = buildData.gestaltMeans->col(k);
      buildData.gestaltVariances->col(i) = buildData.gestaltVariances->col(k);
      buildData.warpedXYZ->col(i) = buildData.warpedXYZ->col(k);
      buildData.gestaltShapes->col(i) = buildData.gestaltShapes->col(k);
    }
  }
  cloud.features.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.descriptors.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.times.conservativeResize(Eigen::NoChange, ptsOut);
  // warning if some points were dropped
  if(buildData.unfitPointsCount != 0)
    LOG_INFO_STREAM("  GestaltDataPointsFilter - Could not compute normal for " << buildData.unfitPointsCount << " pts.");
}

#include "VoxelGrid.h"
template<typename T>
void GestaltDataPointsFilter<T>::buildNew(
	BuildData& data, const int first, const int last, 
	Vector&& minValues, Vector&& maxValues) const
{
	using Voxel = typename VoxelGridDataPointsFilter<T>::Voxel;

  const T minBoundX = minValues.x() / vSizeX;
  const T maxBoundX = maxValues.x() / vSizeX;
  const T minBoundY = minValues.y() / vSizeY;
  const T maxBoundY = maxValues.y() / vSizeY;
  const T minBoundZ = minValues.z() / vSizeZ;
  const T maxBoundZ = maxValues.z() / vSizeZ;

  // number of divisions is total size / voxel size voxels of equal length + 1
  // with remaining space
  const unsigned int numDivX = 1 + maxBoundX - minBoundX;
  const unsigned int numDivY = 1 + maxBoundY - minBoundY;;
  const unsigned int numDivZ = 1 + maxBoundZ - minBoundZ;
  const unsigned int numVox = numDivX * numDivY * numDivZ;

  // Assume point cloud is randomly ordered
  // compute a linear index of the following type
  // i, j, k are the component indices
  // nx, ny number of divisions in x and y components
  // idx = i + j * nx + k * nx * ny
  const int numPoints = last - first;
  std::vector<unsigned int> indices(numPoints);

  // vector to hold the first point in a voxel
  // this point will be ovewritten in the input cloud with
  // the output value
  std::vector<Voxel> voxels;

  // try allocating vector. If too big return error
  try 
  {
    voxels = std::vector<Voxel>(numVox);
  } 
  catch (std::bad_alloc&) 
  {
    throw InvalidParameter((boost::format("GestaltDataPointsFilter: Memory allocation error with %1% voxels.  Try increasing the voxel dimensions.") % numVox).str());
  }

  const int featDim(data.features.rows());

  for (int p = 0; p < numPoints; ++p)
  {
    const unsigned int i = floor(data.features(0,p)/vSizeX - minBoundX);
    const unsigned int j = floor(data.features(1,p)/vSizeY- minBoundY);
    unsigned int k = 0;
    unsigned int idx;
    if ( featDim == 4 )
    {
      k = floor(data.features(2,p)/vSizeZ - minBoundZ);
      idx = i + j * numDivX + k * numDivX * numDivY;
    }
    else
    {
      idx = i + j * numDivX;
    }

    const unsigned int pointsInVox = voxels[idx].numPoints + 1;

    if (pointsInVox == 1)
    {
      voxels[idx].firstPoint = p;
    }

    voxels[idx].numPoints = pointsInVox;

    indices[p] = idx;

  }

  // store which points contain voxel position
  std::vector<unsigned int> pointsToKeep;

  // take centers of voxels for now
  // Todo revert to random point selection within cell
  for (int p = 0; p < numPoints ; ++p)
  {
    const unsigned int idx = indices[p];
    const unsigned int firstPoint = voxels[idx].firstPoint;

    // Choose random point in voxel
    const int randomIndex = std::rand() % numPoints;
    for (int f = 0; f < (featDim - 1); ++f)
    {
      data.features(f,firstPoint) = data.features(f,randomIndex);
    }
  }

  for (unsigned int idx = 0; idx < numVox; ++idx)
  {
    const unsigned int numPoints = voxels[idx].numPoints;
    const unsigned int firstPoint = voxels[idx].firstPoint;

    if (numPoints > 0)
    {
      // get back voxel indices in grid format
      // If we are in the last division, the voxel is smaller in size
      // We adjust the center as from the end of the last voxel to the bounding area

      pointsToKeep.push_back(firstPoint);
    }
  }

	const unsigned int nbPointsToKeep(pointsToKeep.size());
  // now the keypoints are in pointsToKeep
  // downsample with ratio
  for(unsigned int i=0; i<nbPointsToKeep; ++i)
  {
    const float r = (float)std::rand()/(float)RAND_MAX;
    if(r < ratio)
    {
      // Keep points with their descriptors
      const int k = pointsToKeep[i];
      // Mark the indices which will be part of the final data
      data.indicesToKeep.push_back(k);
    }
  }
}

template<typename T>
void GestaltDataPointsFilter<T>::fuseRange(
	BuildData& data, DataPoints& input, const int first, const int last) const
{
  using namespace PointMatcherSupport;
  
  typedef typename Eigen::Matrix<std::int64_t, Eigen::Dynamic, Eigen::Dynamic> Int64Matrix;

  const unsigned int nbIdxToKeep(data.indicesToKeep.size());
  const int inputFeatDim(input.features.cols());
  
  std::vector<int> indicesToKeepStrict;
  for (unsigned int i = 0; i < nbIdxToKeep ; ++i) 
  {
    Eigen::Matrix<T,3,1> keyPoint;
    keyPoint = input.features.col(data.indicesToKeep[i]);

    // Define a search box around each keypoint to search for nearest neighbours.
    const T minBoundX = keyPoint(0,0) - radius;
    const T maxBoundX = keyPoint(0,0) + radius;
    const T minBoundY = keyPoint(1,0) - radius;
    const T maxBoundY = keyPoint(1,0) + radius;
    const T minBoundZ = keyPoint(2,0) - radius;
    const T maxBoundZ = keyPoint(2,0) + radius;
    // iterate over data and find in- / outliers
    Eigen::Matrix<T,3,1> feature;
    std::vector<int> goodIndices;
    for (int j = 0; j < inputFeatDim; ++j) 
    {
      feature = input.features.col(j);
      if(feature(0,0) <= maxBoundX && feature(0,0) >= minBoundX &&
          feature(1,0) <= maxBoundY && feature(1,0) >= minBoundY &&
          feature(2,0) <= maxBoundZ && feature(2,0) >= minBoundZ &&
          keyPoint != feature) 
      {
        goodIndices.push_back(j);
      }
    }
    const int colCount = goodIndices.size();
    // if empty neighbourhood unfit the point
    if (colCount == 0) 
    {
      ++(data.unfitPointsCount);
      continue;
    }
    
    const int featDim(data.features.rows());
    
    Matrix d(featDim-1, colCount);
    Int64Matrix t(1, colCount);

    for (int j = 0; j < colCount; ++j) 
    {
      d.col(j) = data.features.block(0,data.indices[goodIndices[j]],featDim-1, 1);
      t.col(j) = data.times.col(data.indices[goodIndices[j]]);
    }

    const Vector mean = d.rowwise().sum() / T(colCount);
    const Matrix NN = d.colwise() - mean;
    const std::int64_t minTime = t.minCoeff();
    const std::int64_t maxTime = t.maxCoeff();
    const std::int64_t meanTime = t.sum() / T(colCount);
    // compute covariance
    const Matrix C(NN * NN.transpose());
    Vector eigenVa = Vector::Identity(featDim-1, 1);
    Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
    // Ensure that the matrix is suited for eigenvalues calculation
    if(keepNormals || keepEigenValues || keepEigenVectors || keepCovariances || keepGestaltFeatures)
    {
      if(C.fullPivHouseholderQr().rank()+1 >= featDim-1)
      {
        const Eigen::EigenSolver<Matrix> solver(C);
        eigenVa = solver.eigenvalues().real();
        eigenVe = solver.eigenvectors().real();
      }
      else
      {
        data.unfitPointsCount += colCount;
        continue;
      }
    }
    Eigen::Matrix<T,3,1> normal, newX, newY;
    Eigen::Matrix<T,3,3> newBasis;
    double planarity = 0.;
		double cylindricality = 0.;

    if(keepNormals || keepGestaltFeatures) 
    {
      // calculate orientation of NN
      normal = computeNormal<T>(eigenVa, eigenVe);

      if(keepGestaltFeatures) 
      {
        Vector eigenVaSort = sortEigenValues<T>(eigenVa);
        planarity = 2 * (eigenVaSort(1) - eigenVaSort(0))/eigenVaSort.sum();
        cylindricality = (eigenVaSort(2) - eigenVaSort(1))/eigenVaSort.sum();
        // project normal on horizontal plane
        Eigen::Matrix<T,3,1> up, base;
        up << 0,0,1;
        base << 1,0,0;
        newX << normal(0), normal(1), 0;
        newX.normalize();
        newY = up.cross(newX);
        newY = newY / newY.norm();
        // form a new basis with world z-axis and projected x & y-axis
        newBasis << newX(0), newY(0), up(0),
            newX(1), newY(1), up(1),
            newX(2), newY(2), up(2);

        // discard keypoints with high planarity
        if(planarity > 0.9) 
        {
          data.unfitPointsCount += colCount;
          continue;
        }
        // discard keypoints with normal too close to vertical
        if(acos(normal.dot(up)) < abs(10 * M_PI/180)) 
        {
          data.unfitPointsCount += colCount;
          continue;
        }

        // define features in new basis that is oriented with the covariance
        for (int j = 0; j < colCount; ++j) 
        {
          data.warpedXYZ->col(j) = ((data.features.block(0,j,3,1) - keyPoint).transpose() * newBasis).transpose();
        }
      }
    }
    Vector angles(colCount), radii(colCount), heights(colCount);
    Matrix gestaltMeans(4, 8), gestaltVariances(4, 8), numOfValues(4, 8);
    if(keepGestaltFeatures) 
    {
      // calculate the polar coordinates of points
      angles = GestaltDataPointsFilter::calculateAngles(*data.warpedXYZ, keyPoint);
      radii = GestaltDataPointsFilter::calculateRadii(*data.warpedXYZ, keyPoint);
      heights = data.warpedXYZ->row(2);

      // sort points into Gestalt bins
      const T angularBinWidth = M_PI/4;
      const T radialBinWidth = radius/4;
      Eigen::MatrixXi indices(2, colCount);
      gestaltMeans = Matrix::Zero(4, 8);
      gestaltVariances = Matrix::Zero(4, 8);
      numOfValues = Matrix::Zero(4, 8);

      for (int it=0; it < colCount; ++it) 
      {
        indices(0,it) = floor(radii(it)/radialBinWidth);
        // if value exceeds borders of bin -> put in outmost bin
        if(indices(0,it) > 3)
          // this case should never happen - just in case
          indices(0,it) = 3;
        indices(1,it) = floor(angles(it)/angularBinWidth);
        if(indices(1,it) > 7)
          indices(1,it) = 7;
        gestaltMeans(indices(0,it), indices(1,it)) += heights(it);
        ++(numOfValues(indices(0,it), indices(1,it)));
      }

      for (int radial=0; radial < 4; ++radial) 
      {
        for (int angular = 0; angular < 8; ++angular) 
        {
          if (numOfValues(radial, angular) > 0) 
          {
            gestaltMeans(radial, angular) = gestaltMeans(radial, angular)/numOfValues(radial, angular);
          }
        }
      }
      for (int it=0; it < colCount; ++it) 
      {
        gestaltVariances(indices(0,it), indices(1,it)) += (heights(it)-gestaltMeans(indices(0,it), indices(1,it))) * (heights(it)-gestaltMeans(indices(0,it), indices(1,it)));
      }
      for (int radial=0; radial < 4; ++radial) 
      {
        for (int angular = 0; angular < 8; ++angular) 
        {
          // if bins are == 0 -> propagate with value in bin closer to keypoint
          if (gestaltMeans(radial,angular) == 0 && radial > 0) 
          {
            gestaltMeans(radial, angular) = gestaltMeans(radial-1, angular);
            gestaltVariances(radial, angular) = gestaltVariances(radial-1, angular);
          } 
          else if (numOfValues(radial, angular) > 0) 
          {
            gestaltVariances(radial, angular) = gestaltVariances(radial, angular)/numOfValues(radial, angular);
          }
        }
      }
    }
    
    Vector serialEigVector;
    if(keepEigenVectors)
      serialEigVector = serializeEigVec<T>(eigenVe);
    Vector serialCovVector;
    if(keepCovariances)
      serialCovVector = serializeEigVec<T>(C);
    Vector serialGestaltMeans;
    Vector serialGestaltVariances;
    if(keepGestaltFeatures) 
    {
      serialGestaltMeans = GestaltDataPointsFilter::serializeGestaltMatrix(gestaltMeans);
      serialGestaltVariances = GestaltDataPointsFilter::serializeGestaltMatrix(gestaltVariances);
    }
    // some safety check
    if(data.descriptors.rows() != 0)
      assert(data.descriptors.cols() != 0);

    // write the updated times: min, max, mean
    data.times(0, data.indicesToKeep[i]) = minTime;
    data.times(1, data.indicesToKeep[i]) = maxTime;
    data.times(2, data.indicesToKeep[i]) = meanTime;

    // Build new descriptors
    if(keepNormals)
      data.normals->col(data.indicesToKeep[i]) = normal;
    if(keepMeans)
      data.means->col(data.indicesToKeep[i]) = mean;
    if(keepEigenValues)
      data.eigenValues->col(data.indicesToKeep[i]) = eigenVa;
    if(keepEigenVectors)
      data.eigenVectors->col(data.indicesToKeep[i]) = serialEigVector;
    if(keepCovariances)
      data.covariance->col(data.indicesToKeep[i]) = serialCovVector;
    if(keepGestaltFeatures) 
    {
      // preserve gestalt features
      data.gestaltMeans->col(data.indicesToKeep[i]) = serialGestaltMeans;
      data.gestaltVariances->col(data.indicesToKeep[i]) = serialGestaltVariances;
      (*data.gestaltShapes)(0,data.indicesToKeep[i]) = planarity;
      (*data.gestaltShapes)(1,data.indicesToKeep[i]) = cylindricality;
    }
    // all went well so far - so keep this keypoint
    indicesToKeepStrict.push_back(data.indicesToKeep[i]);
  }
  data.indicesToKeep = indicesToKeepStrict;
}

template<typename T>
typename PointMatcher<T>::Vector
GestaltDataPointsFilter<T>::serializeGestaltMatrix(const Matrix& gestaltFeatures) const
{
  // serialize the gestalt descriptors
  const int dim = gestaltFeatures.rows() * gestaltFeatures.cols();
  Vector output(dim);
  for(int k=0; k < gestaltFeatures.rows(); ++k)
  {
    output.segment(k*gestaltFeatures.cols(), gestaltFeatures.cols()) =
        gestaltFeatures.row(k).transpose();
  }
  return output;
}

template<typename T>
typename PointMatcher<T>::Vector
GestaltDataPointsFilter<T>::calculateAngles(
	const Matrix& points, const Eigen::Matrix<T,3,1>& keyPoint) const
{
	const unsigned int dim(points.cols());
  Vector angles(dim);
  
  for (unsigned int i = 0; i < dim; ++i) 
  {
    angles(i) = atan2(points(0,i), points(1,i));
    if (angles(i) < 0)
      angles(i) += (2 * M_PI);
  }

  return angles;
}

template<typename T>
typename PointMatcher<T>::Vector
GestaltDataPointsFilter<T>::calculateRadii(
	const Matrix& points, const Eigen::Matrix<T,3,1>& keyPoint) const
{
	const unsigned int dim(points.cols());
  Vector radii(dim);

  for (unsigned int i = 0; i < dim; ++i) 
  {
    radii(i) = sqrt((points(0,i)) * (points(0,i)) + (points(1,i)) * (points(1,i)));
  }
  return radii;
}

template struct GestaltDataPointsFilter<float>;
template struct GestaltDataPointsFilter<double>;
