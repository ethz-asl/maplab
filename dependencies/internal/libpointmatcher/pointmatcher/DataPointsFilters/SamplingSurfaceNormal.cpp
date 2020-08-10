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
#include "SamplingSurfaceNormal.h"

#include "utils.h"

// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "PointMatcherPrivate.h"

#include <utility>
#include <algorithm>

// SamplingSurfaceNormalDataPointsFilter

// Constructor
template<typename T>
SamplingSurfaceNormalDataPointsFilter<T>::SamplingSurfaceNormalDataPointsFilter(
	const Parameters& params):
	PointMatcher<T>::DataPointsFilter("SamplingSurfaceNormalDataPointsFilter",
		SamplingSurfaceNormalDataPointsFilter::availableParameters(), params),
	ratio(Parametrizable::get<T>("ratio")),
	knn(Parametrizable::get<int>("knn")),
	samplingMethod(Parametrizable::get<int>("samplingMethod")),
	maxBoxDim(Parametrizable::get<T>("maxBoxDim")),
	averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors")),
	keepNormals(Parametrizable::get<bool>("keepNormals")),
	keepDensities(Parametrizable::get<bool>("keepDensities")),
	keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
	keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
SamplingSurfaceNormalDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void SamplingSurfaceNormalDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;

	const int pointsCount(cloud.features.cols());
	const int featDim(cloud.features.rows());
	const int descDim(cloud.descriptors.rows());
	const unsigned int labelDim(cloud.descriptorLabels.size());

	int insertDim(0);
	if (averageExistingDescriptors)
	{
		// TODO: this should be in the form of an assert
		// Validate descriptors and labels
		for(unsigned int i = 0; i < labelDim ; ++i)
			insertDim += cloud.descriptorLabels[i].span;
		if (insertDim != descDim)
			throw InvalidField("SamplingSurfaceNormalDataPointsFilter: Error, descriptor labels do not match descriptor data");
	}

	// Compute space requirement for new descriptors
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));

	// Allocate space for new descriptors
	Labels cloudLabels;
	if (keepNormals)
		cloudLabels.push_back(Label("normals", dimNormals));
	if (keepDensities)
		cloudLabels.push_back(Label("densities", dimDensities));
	if (keepEigenValues)
		cloudLabels.push_back(Label("eigValues", dimEigValues));
	if (keepEigenVectors)
		cloudLabels.push_back(Label("eigVectors", dimEigVectors));
	cloud.allocateDescriptors(cloudLabels);

	// we keep build data on stack for reentrant behaviour
	View cloudExistingDescriptors(cloud.descriptors.block(0,0,cloud.descriptors.rows(),cloud.descriptors.cols()));
	BuildData buildData(cloud.features, cloud.descriptors);

	// get views
	if (keepNormals)
		buildData.normals = cloud.getDescriptorViewByName("normals");
	if (keepDensities)
		buildData.densities = cloud.getDescriptorViewByName("densities");
	if (keepEigenValues)
		buildData.eigenValues = cloud.getDescriptorViewByName("eigValues");
	if (keepEigenVectors)
		buildData.eigenVectors = cloud.getDescriptorViewByName("eigVectors");
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
		if (cloud.descriptors.rows() != 0)
			cloud.descriptors.col(i) = cloud.descriptors.col(k);
		if(keepNormals)
			buildData.normals->col(i) = buildData.normals->col(k);
		if(keepDensities)
			(*buildData.densities)(0,i) = (*buildData.densities)(0,k);
		if(keepEigenValues)
			buildData.eigenValues->col(i) = buildData.eigenValues->col(k);
		if(keepEigenVectors)
			buildData.eigenVectors->col(i) = buildData.eigenVectors->col(k);
	}
	cloud.features.conservativeResize(Eigen::NoChange, ptsOut);
	cloud.descriptors.conservativeResize(Eigen::NoChange, ptsOut);

	// warning if some points were dropped
	if(buildData.unfitPointsCount != 0)
		LOG_INFO_STREAM("  SamplingSurfaceNormalDataPointsFilter - Could not compute normal for " << buildData.unfitPointsCount << " pts.");
}

template<typename T>
void SamplingSurfaceNormalDataPointsFilter<T>::buildNew(
	BuildData& data, const int first, const int last, 
	Vector&& minValues, Vector&& maxValues) const
{
	using namespace PointMatcherSupport;
	
	const int count(last - first);
	if (count <= int(knn))
	{
		// compute for this range
		fuseRange(data, first, last);
		// TODO: make another filter that creates constant-density clouds,
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
	buildNew(data, first, first + leftCount, 
		std::forward<Vector>(minValues), std::move(leftMaxValues));
	buildNew(data, first + leftCount, last, 
		std::move(rightMinValues), std::forward<Vector>(maxValues));
}

template<typename T>
void SamplingSurfaceNormalDataPointsFilter<T>::fuseRange(
	BuildData& data, const int first, const int last) const
{
	using namespace PointMatcherSupport;
	
	const int colCount(last-first);
	const int featDim(data.features.rows());

	// build nearest neighbors list
	Matrix d(featDim-1, colCount);
	for (int i = 0; i < colCount; ++i)
		d.col(i) = data.features.block(0,data.indices[first+i],featDim-1, 1);
	const Vector box = d.rowwise().maxCoeff() - d.rowwise().minCoeff();
	const T boxDim(box.maxCoeff());
	// drop box if it is too large
	if (boxDim > maxBoxDim)
	{
		data.unfitPointsCount += colCount;
		return;
	}
	const Vector mean = d.rowwise().sum() / T(colCount);
	const Matrix NN = (d.colwise() - mean);

	// compute covariance
	const Matrix C(NN * NN.transpose());
	Vector eigenVa = Vector::Identity(featDim-1, 1);
	Matrix eigenVe = Matrix::Identity(featDim-1, featDim-1);
	// Ensure that the matrix is suited for eigenvalues calculation
	if(keepNormals || keepEigenValues || keepEigenVectors)
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
	}

	Vector normal;
	if(keepNormals)
		normal = computeNormal<T>(eigenVa, eigenVe);

	T densitie = 0;
	if(keepDensities)
		densitie = computeDensity<T>(NN);

	//if(keepEigenValues) nothing to do

	Vector serialEigVector;
	if(keepEigenVectors)
		serialEigVector = serializeEigVec<T>(eigenVe);

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

				// Build new descriptors
				if(keepNormals)
					data.normals->col(k) = normal;
				if(keepDensities)
					(*data.densities)(0,k) = densitie;
				if(keepEigenValues)
					data.eigenValues->col(k) = eigenVa;
				if(keepEigenVectors)
					data.eigenVectors->col(k) = serialEigVector;
			}
		}
	}
	else
	{
		const int k = data.indices[first];
		// Mark the indices which will be part of the final data
		data.indicesToKeep.push_back(k);
		data.features.col(k).topRows(featDim-1) = mean;
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
		if(keepNormals)
			data.normals->col(k) = normal;
		if(keepDensities)
			(*data.densities)(0,k) = densitie;
		if(keepEigenValues)
			data.eigenValues->col(k) = eigenVa;
		if(keepEigenVectors)
			data.eigenVectors->col(k) = serialEigVector;
	}
}

template struct SamplingSurfaceNormalDataPointsFilter<float>;
template struct SamplingSurfaceNormalDataPointsFilter<double>;

