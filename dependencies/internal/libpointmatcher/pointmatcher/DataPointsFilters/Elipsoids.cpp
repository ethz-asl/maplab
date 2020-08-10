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
#include "Elipsoids.h"

#include "utils.h"
// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "PointMatcherPrivate.h"

// ElipsoidsDataPointsFilter

// Constructor
template<typename T>
ElipsoidsDataPointsFilter<T>::ElipsoidsDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("ElipsoidsDataPointsFilter", 
		ElipsoidsDataPointsFilter::availableParameters(), params),
	ratio(Parametrizable::get<T>("ratio")),
	knn(Parametrizable::get<int>("knn")),
	samplingMethod(Parametrizable::get<int>("samplingMethod")),
	maxBoxDim(Parametrizable::get<T>("maxBoxDim")),
	maxTimeWindow(Parametrizable::get<T>("maxTimeWindow")),
	minPlanarity(Parametrizable::get<T>("minPlanarity")),
	averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors")),
	keepNormals(Parametrizable::get<bool>("keepNormals")),
	keepDensities(Parametrizable::get<bool>("keepDensities")),
	keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
	keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors")),
	keepCovariances(Parametrizable::get<bool>("keepCovariances")),
	keepWeights(Parametrizable::get<bool>("keepWeights")),
	keepMeans(Parametrizable::get<bool>("keepMeans")),
	keepShapes(Parametrizable::get<bool>("keepShapes")),
	keepIndices(Parametrizable::get<bool>("keepIndices"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
ElipsoidsDataPointsFilter<T>::filter(const DataPoints& input)
{
  DataPoints output(input);
  inPlaceFilter(output);
  return output;
}

// In-place filter
template<typename T>
void ElipsoidsDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
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
      throw InvalidField("ElipsoidsDataPointsFilter: Error, descriptor labels do not match descriptor data");
  }

  // Compute space requirement for new descriptors
  const int dimNormals(featDim-1);
  const int dimDensities(1);
  const int dimEigValues(featDim-1);
  const int dimEigVectors((featDim-1)*(featDim-1));
  const int dimWeights(1);
  const int dimMeans(featDim-1);
  const int dimCovariances((featDim-1)*(featDim-1));
  const int dimShapes(featDim-1);
  const int dimPointIds(knn);

  // Allocate space for new descriptors
  Labels cloudLabels, timeLabels;
  if (keepIndices) 
  {
    cloudLabels.push_back(Label("pointIds", dimPointIds));
    cloudLabels.push_back(Label("pointX", dimPointIds));
    cloudLabels.push_back(Label("pointY", dimPointIds));
    cloudLabels.push_back(Label("pointZ", dimPointIds));
    cloudLabels.push_back(Label("numOfNN", 1));
  }
  if (keepNormals)
    cloudLabels.push_back(Label("normals", dimNormals));
  if (keepDensities)
    cloudLabels.push_back(Label("densities", dimDensities));
  if (keepEigenValues)
    cloudLabels.push_back(Label("eigValues", dimEigValues));
  if (keepEigenVectors)
    cloudLabels.push_back(Label("eigVectors", dimEigVectors));
  if (keepCovariances)
    cloudLabels.push_back(Label("covariance", dimCovariances));
  if (keepWeights)
    cloudLabels.push_back(Label("weights", dimWeights));
  if (keepMeans)
    cloudLabels.push_back(Label("means", dimMeans));
  if (keepShapes) 
  {
    assert(featDim == 3);
    cloudLabels.push_back(Label("shapes", dimShapes)); // Planarity, Cylindricality, Sphericality
  }
  timeLabels.push_back(Label("time", 2));

  cloud.allocateDescriptors(cloudLabels);
  cloud.allocateTimes(timeLabels);

  // we keep build data on stack for reentrant behaviour
  View cloudExistingDescriptors(cloud.descriptors.block(0,0,cloud.descriptors.rows(),cloud.descriptors.cols()));
  TimeView cloudExistingTimes(cloud.times.block(0,0,cloud.times.rows(),cloud.times.cols()));
  BuildData buildData(cloud.features, cloud.descriptors, cloud.times);

  // get views
  if (keepIndices) 
  {
    buildData.pointIds = cloud.getDescriptorViewByName("pointIds");
    buildData.pointX = cloud.getDescriptorViewByName("pointX");
    buildData.pointY = cloud.getDescriptorViewByName("pointY");
    buildData.pointZ = cloud.getDescriptorViewByName("pointZ");
    buildData.numOfNN = cloud.getDescriptorViewByName("numOfNN");
  }
  if (keepNormals)
    buildData.normals = cloud.getDescriptorViewByName("normals");
  if (keepDensities)
    buildData.densities = cloud.getDescriptorViewByName("densities");
  if (keepEigenValues)
    buildData.eigenValues = cloud.getDescriptorViewByName("eigValues");
  if (keepEigenVectors)
    buildData.eigenVectors = cloud.getDescriptorViewByName("eigVectors");
  if (keepCovariances)
    buildData.covariance = cloud.getDescriptorViewByName("covariance");
  if (keepWeights)
    buildData.weights = cloud.getDescriptorViewByName("weights");
  if (keepMeans)
    buildData.means = cloud.getDescriptorViewByName("means");
  if (keepShapes)
    buildData.shapes = cloud.getDescriptorViewByName("shapes");

  // build the new point cloud
  buildNew(
      buildData,
      0,
      pointsCount,
      cloud.features.rowwise().minCoeff(),
      cloud.features.rowwise().maxCoeff()
  );

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
    if(keepIndices) 
    {
      buildData.pointIds->col(i) = buildData.pointIds->col(k);
      buildData.pointX->col(i) = buildData.pointX->col(k);
      buildData.pointY->col(i) = buildData.pointY->col(k);
      buildData.pointZ->col(i) = buildData.pointZ->col(k);
      buildData.numOfNN->col(i) = buildData.numOfNN->col(k);
    }
    if(keepNormals)
      buildData.normals->col(i) = buildData.normals->col(k);
    if(keepDensities)
      (*buildData.densities)(0,i) = (*buildData.densities)(0,k);
    if(keepEigenValues)
      buildData.eigenValues->col(i) = buildData.eigenValues->col(k);
    if(keepEigenVectors)
      buildData.eigenVectors->col(i) = buildData.eigenVectors->col(k);
    if(keepWeights)
      buildData.weights->col(i) = buildData.weights->col(k);
    if(keepCovariances)
      buildData.covariance->col(i) = buildData.covariance->col(k);
    if(keepMeans)
      buildData.means->col(i) = buildData.means->col(k);
    if(keepShapes)
      buildData.shapes->col(i) = buildData.shapes->col(k);
  }
  cloud.features.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.descriptors.conservativeResize(Eigen::NoChange, ptsOut);
  cloud.times.conservativeResize(Eigen::NoChange, ptsOut);

  // warning if some points were dropped
  if(buildData.unfitPointsCount != 0)
    LOG_INFO_STREAM("  ElipsoidsDataPointsFilter - Could not compute normal for " << buildData.unfitPointsCount << " pts.");
}

template<typename T>
void ElipsoidsDataPointsFilter<T>::buildNew(
	BuildData& data, const int first, const int last, 
	Vector&& minValues, Vector&& maxValues) const
{
  using namespace PointMatcherSupport;
  
  const int count(last - first);
  if (count <= int(knn))
  {
    // compute for this range
    fuseRange(data, first, last);
    // typically by stopping recursion after the median of the bounding cuboid
    // is below a threshold, or that the number of points falls under a threshold
    return;
  }
  // find the largest dimension of the box
  const int cutDim = argMax<T>(maxValues - minValues);

  // compute number of elements
  const int rightCount(count/2);
  const int leftCount(count - rightCount);
  assert(last - rightCount == first + leftCount);

  // sort, hack std::nth_element
  std::nth_element(
      data.indices.begin() + first,
      data.indices.begin() + first + leftCount,
      data.indices.begin() + last,
      CompareDim(cutDim, data)
  );

  // get value
  const int cutIndex(data.indices[first+leftCount]);
  const T cutVal(data.features(cutDim, cutIndex));

  // update bounds for left
  Vector leftMaxValues(maxValues);
  leftMaxValues[cutDim] = cutVal;
  // update bounds for right
  Vector rightMinValues(minValues);
  rightMinValues[cutDim] = cutVal;

  // recurse
  buildNew(data, first, first + leftCount, std::forward<Vector>(minValues), std::move(leftMaxValues));
  buildNew(data, first + leftCount, last, std::move(rightMinValues), std::forward<Vector>(maxValues));
}

template<typename T>
void ElipsoidsDataPointsFilter<T>::fuseRange(
	BuildData& data, const int first, const int last) const
{
  using namespace PointMatcherSupport;
  
  typedef typename Eigen::Matrix<std::int64_t, Eigen::Dynamic, Eigen::Dynamic> Int64Matrix;

  const int colCount(last-first);
  const int featDim(data.features.rows());

  // build nearest neighbors list
  Matrix d(featDim-1, colCount);
  Int64Matrix t(1, colCount);
  for (int i = 0; i < colCount; ++i) 
  {
    d.col(i) = data.features.block(0,data.indices[first+i],featDim-1, 1);
    t.col(i) = data.times.col(data.indices[first + i]); //, 0);
  }
  const Vector box = d.rowwise().maxCoeff() - d.rowwise().minCoeff();
  const std::int64_t timeBox = t.maxCoeff() - t.minCoeff();

  const T boxDim(box.maxCoeff());
  // drop box if it is too large or max timeframe is exceeded
  if (boxDim > maxBoxDim || timeBox > maxTimeWindow)
  {
    data.unfitPointsCount += colCount;
    return;
  }
  const Vector mean = d.rowwise().sum() / T(colCount);
  const Matrix NN = (d.colwise() - mean);

  const std::int64_t minTime = t.minCoeff();
  const std::int64_t maxTime = t.maxCoeff();
  const std::int64_t meanTime = t.sum() / T(colCount);

  // compute covariance
  const Matrix C(NN * NN.transpose());
  Vector eigenVa = Vector::Identity(featDim-1, 1);
  Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
  // Ensure that the matrix is suited for eigenvalues calculation
  if(keepNormals || keepEigenValues || keepEigenVectors || keepCovariances || keepShapes || minPlanarity > 0)
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
      return;
    }
    if(minPlanarity > 0) 
    {
      Eigen::Matrix<T, 3, 1> vals;
      (vals << eigenVa(0),eigenVa(1),eigenVa(2));
      vals = vals/eigenVa.sum();
      const T planarity = 2 * vals(1)-2*vals(2);
      // throw out surfel if it does not meet planarity criteria
      if (planarity < minPlanarity)
      {
        data.unfitPointsCount += colCount;
        return;
      }
    }
  }

  // keep the indices of each ellipsoid
  Vector pointIds(1,colCount);
  Matrix points(3,colCount);

  if(keepIndices) 
  {
    for (int i = 0; i < colCount; ++i) 
    {
      pointIds(i) = data.indices[first+i];
      points.col(i) = data.features.block(0,data.indices[first+i],2, 1);
    }
  }

  Vector normal;
  if(keepNormals)
    normal = computeNormal<T>(eigenVa, eigenVe);

  T density = 0;
  if(keepDensities)
    density = computeDensity<T>(NN);
  Vector serialEigVector;
  if(keepEigenVectors)
    serialEigVector = serializeEigVec<T>(eigenVe);
  Vector serialCovVector;
  if(keepCovariances)
    serialCovVector = serializeEigVec<T>(C);

  // some safety check
  if(data.descriptors.rows() != 0)
    assert(data.descriptors.cols() != 0);

  // Filter points randomly
  if(samplingMethod == 0)
  {
    for(int i=0; i<colCount; ++i)
    {
      const float r = (float)std::rand()/(float)RAND_MAX;
      if(r < ratio)
      {
        // Keep points with their descriptors
        const int k = data.indices[first+i];
        // Mark the indices which will be part of the final data
        data.indicesToKeep.push_back(k);

        // write the updated times: min, max, mean
        data.times(0, k) = minTime;
        data.times(1, k) = maxTime;
        data.times(2, k) = meanTime;

        // Build new descriptors
        if(keepIndices) 
        {
          data.pointIds->col(k) = pointIds;
          data.pointX->col(k) = points.row(0);
          data.pointY->col(k) = points.row(1);
          data.pointZ->col(k) = points.row(2);
          (*data.numOfNN)(0,k) = NN.cols();
        }
        if(keepNormals)
          data.normals->col(k) = normal;
        if(keepDensities)
          (*data.densities)(0,k) = density;
        if(keepEigenValues)
          data.eigenValues->col(k) = eigenVa;
        if(keepEigenVectors)
          data.eigenVectors->col(k) = serialEigVector;
        if(keepCovariances)
          data.covariance->col(k) = serialCovVector;
        if(keepMeans)
          data.means->col(k) = mean;    
        // a 3d vecetor of shape parameters: planarity (P), cylindricality (C), sphericality (S)
        if(keepShapes) 
        {
          Eigen::Matrix<T, 3, 3> shapeMat;
          (shapeMat << 0, 2, -2, 1, -1, 0, 0, 0, 3);
          Eigen::Matrix<T, 3, 1> vals;
          (vals << eigenVa(0),eigenVa(1),eigenVa(2));
          vals = vals/eigenVa.sum();
          data.shapes->col(k) = shapeMat * vals;

        }
        if(keepWeights) 
        {
          (*data.weights)(0,k) = colCount;
        }
      }
    }
  }
  else
  {
    const int k = data.indices[first];
    // Mark the indices which will be part of the final data
    data.indicesToKeep.push_back(k);
    data.features.col(k).topRows(featDim-1) = mean;
    // write the updated times: min, max, mean
    data.times(0, k) = minTime;
    data.times(1, k) = maxTime;
    data.times(2, k) = meanTime;

    data.features(featDim-1, k) = 1;

    if(data.descriptors.rows() != 0)
    {
      // average the existing descriptors
      if (averageExistingDescriptors)
      {
        Vector mergedDesc(Vector::Zero(data.descriptors.rows()));
        for (int i = 0; i < colCount; ++i)
          mergedDesc += data.descriptors.col(data.indices[first+i]);
        mergedDesc /= T(colCount);
        data.descriptors.col(k) = mergedDesc;
      }
      // else just keep the first one
    }

    // Build new descriptors
    if(keepIndices) 
    {
      data.pointIds->col(k) = pointIds;
      data.pointX->col(k) = points.row(0);
      data.pointY->col(k) = points.row(1);
      data.pointZ->col(k) = points.row(2);
    }
    if(keepNormals)
      data.normals->col(k) = normal;
    if(keepDensities)
      (*data.densities)(0,k) = density;
    if(keepEigenValues)
      data.eigenValues->col(k) = eigenVa;
    if(keepEigenVectors)
      data.eigenVectors->col(k) = serialEigVector;
    if(keepCovariances)
      data.covariance->col(k) = serialCovVector;
    if(keepMeans)
      data.means->col(k) = mean;
    if(keepShapes) 
    {
      Eigen::Matrix<T, 3, 3> shapeMat;
      (shapeMat << 0, 2, -2, 1, -1, 0, 0, 0, 3);
      Eigen::Matrix<T, 3, 1> vals;
      (vals << eigenVa(0),eigenVa(1),eigenVa(2));
      vals = vals/eigenVa.sum();
      data.shapes->col(k) = shapeMat * vals; //eigenVa;
    }
    if(keepWeights)
      (*data.weights)(0,k) = colCount;
  }

}

template struct ElipsoidsDataPointsFilter<float>;
template struct ElipsoidsDataPointsFilter<double>;

