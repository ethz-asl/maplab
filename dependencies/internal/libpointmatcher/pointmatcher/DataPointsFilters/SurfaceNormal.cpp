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
#include "SurfaceNormal.h"

// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

#include "PointMatcherPrivate.h"
#include "IO.h"
#include "MatchersImpl.h"

#include <boost/format.hpp>

#include "utils.h"

// SurfaceNormalDataPointsFilter
// Constructor
template<typename T>
SurfaceNormalDataPointsFilter<T>::SurfaceNormalDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("SurfaceNormalDataPointsFilter",
		SurfaceNormalDataPointsFilter::availableParameters(), params),
	knn(Parametrizable::get<int>("knn")),
	maxDist(Parametrizable::get<T>("maxDist")),
	epsilon(Parametrizable::get<T>("epsilon")),
	keepNormals(Parametrizable::get<bool>("keepNormals")),
	keepDensities(Parametrizable::get<bool>("keepDensities")),
	keepEigenValues(Parametrizable::get<bool>("keepEigenValues")),
	keepEigenVectors(Parametrizable::get<bool>("keepEigenVectors")),
	keepMatchedIds(Parametrizable::get<bool>("keepMatchedIds")),
	keepMeanDist(Parametrizable::get<bool>("keepMeanDist")),
	sortEigen(Parametrizable::get<bool>("sortEigen")),
	smoothNormals(Parametrizable::get<bool>("smoothNormals"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
SurfaceNormalDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void SurfaceNormalDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	typedef typename DataPoints::View View;
	typedef typename DataPoints::Label Label;
	typedef typename DataPoints::Labels Labels;
	typedef typename MatchersImpl<T>::KDTreeMatcher KDTreeMatcher;
	typedef typename PointMatcher<T>::Matches Matches;

	using namespace PointMatcherSupport;

	const int pointsCount(cloud.features.cols());
	const int featDim(cloud.features.rows());
	const int descDim(cloud.descriptors.rows());
	const unsigned int labelDim(cloud.descriptorLabels.size());

	// Validate descriptors and labels
	int insertDim(0);
	for(unsigned int i = 0; i < labelDim ; ++i)
		insertDim += cloud.descriptorLabels[i].span;
	if (insertDim != descDim)
		throw InvalidField("SurfaceNormalDataPointsFilter: Error, descriptor labels do not match descriptor data");

	// Reserve memory for new descriptors
	const int dimNormals(featDim-1);
	const int dimDensities(1);
	const int dimEigValues(featDim-1);
	const int dimEigVectors((featDim-1)*(featDim-1));
	//const int dimMatchedIds(knn);
	const int dimMeanDist(1);

	boost::optional<View> normals;
	boost::optional<View> densities;
	boost::optional<View> eigenValues;
	boost::optional<View> eigenVectors;
	boost::optional<View> matchedValues;
	boost::optional<View> matchIds;
	boost::optional<View> meanDists;

	Labels cloudLabels;
	if (keepNormals)
		cloudLabels.push_back(Label("normals", dimNormals));
	if (keepDensities)
		cloudLabels.push_back(Label("densities", dimDensities));
	if (keepEigenValues)
		cloudLabels.push_back(Label("eigValues", dimEigValues));
	if (keepEigenVectors)
		cloudLabels.push_back(Label("eigVectors", dimEigVectors));
	if (keepMatchedIds)
		cloudLabels.push_back(Label("matchedIds", knn));
	if (keepMeanDist)
		cloudLabels.push_back(Label("meanDists", dimMeanDist));

	// Reserve memory
	cloud.allocateDescriptors(cloudLabels);

	if (keepNormals)
		normals = cloud.getDescriptorViewByName("normals");
	if (keepDensities)
		densities = cloud.getDescriptorViewByName("densities");
	if (keepEigenValues)
		eigenValues = cloud.getDescriptorViewByName("eigValues");
	if (keepEigenVectors)
		eigenVectors = cloud.getDescriptorViewByName("eigVectors");
	if (keepMatchedIds)
		matchIds = cloud.getDescriptorViewByName("matchedIds");
	if (keepMeanDist)
		meanDists = cloud.getDescriptorViewByName("meanDists");

	using namespace PointMatcherSupport;
	// Build kd-tree
	Parametrizable::Parameters param;
	boost::assign::insert(param) ( "knn", toParam(knn) );
	boost::assign::insert(param) ( "epsilon", toParam(epsilon) );
	boost::assign::insert(param) ( "maxDist", toParam(maxDist) );

	KDTreeMatcher matcher(param);
	matcher.init(cloud);

	Matches matches(typename Matches::Dists(knn, pointsCount), typename Matches::Ids(knn, pointsCount));
	matches = matcher.findClosests(cloud);

	// Search for surrounding points and compute descriptors
	int degenerateCount(0);
	for (int i = 0; i < pointsCount; ++i)
	{
		bool isDegenerate = false;
		// Mean of nearest neighbors (NN)
		Matrix d(featDim-1, knn);
		int realKnn = 0;

		for(int j = 0; j < int(knn); ++j)
		{
			if (matches.dists(j,i) != Matches::InvalidDist)
			{
				const int refIndex(matches.ids(j,i));
				d.col(realKnn) = cloud.features.block(0, refIndex, featDim-1, 1);
				++realKnn;
			}
		}
		d.conservativeResize(Eigen::NoChange, realKnn);

		const Vector mean = d.rowwise().sum() / T(realKnn);
		const Matrix NN = d.colwise() - mean;

		const Matrix C(NN * NN.transpose());
		Vector eigenVa = Vector::Zero(featDim-1, 1);
		Matrix eigenVe = Matrix::Zero(featDim-1, featDim-1);
		// Ensure that the matrix is suited for eigenvalues calculation
		if(keepNormals || keepEigenValues || keepEigenVectors)
		{
			if(C.fullPivHouseholderQr().rank()+1 >= featDim-1)
			{
				const Eigen::EigenSolver<Matrix> solver(C);
				eigenVa = solver.eigenvalues().real();
				eigenVe = solver.eigenvectors().real();

				if(sortEigen)
				{
					const std::vector<size_t> idx = sortIndexes<T>(eigenVa);
					const size_t idxSize = idx.size();
					Vector tmp_eigenVa = eigenVa;
					Matrix tmp_eigenVe = eigenVe;
					for(size_t i=0; i<idxSize; ++i)
					{
						eigenVa(i,0) = tmp_eigenVa(idx[i], 0);
						eigenVe.col(i) = tmp_eigenVe.col(idx[i]);
					}
				}
			}
			else
			{
				//std::cout << "WARNING: Matrix C needed for eigen decomposition is degenerated. Expected cause: no noise in data" << std::endl;
				++degenerateCount;
				isDegenerate = true;
			}
		}

		if(keepNormals)
		{
			if(sortEigen)
				normals->col(i) = eigenVe.col(0);
			else
				normals->col(i) = computeNormal<T>(eigenVa, eigenVe);
			
			// clamp normals to [-1,1] to handle approximation errors
			normals->col(i) = normals->col(i).cwiseMax(-1.0).cwiseMin(1.0);
		}
		if(keepDensities)
		{
			if(isDegenerate)
				(*densities)(0, i) = 0.;
			else
				(*densities)(0, i) = computeDensity<T>(NN);
		}
		if(keepEigenValues)
			eigenValues->col(i) = eigenVa;
		if(keepEigenVectors)
			eigenVectors->col(i) = serializeEigVec<T>(eigenVe);
		if(keepMeanDist)
		{
			if(isDegenerate)
				(*meanDists)(0, i) = std::numeric_limits<std::size_t>::max();
			else
			{
				const Vector point = cloud.features.block(0, i, featDim-1, 1);
				(*meanDists)(0, i) = (point - mean).norm();
			}
		}

	}

	if(keepMatchedIds)
	{
		matchIds.get() = matches.ids.template cast<T>();
	}

	if(smoothNormals)
	{
		for (int i = 0; i < pointsCount; ++i)
		{
			const Vector currentNormal = normals->col(i);
			Vector mean = Vector::Zero(featDim-1);
			int n=0;
			for(int j = 0; j < int(knn); ++j)
			{
				if (matches.dists(j,i) != Matches::InvalidDist)
				{
					const int refIndex(matches.ids(j,i));
					const Vector normal = normals->col(refIndex);
					if(currentNormal.dot(normal) > 0.)
						mean += normal;
					else // flip normal vector
						mean -= normal;

					++n;
				}
			}

			normals->col(i) = mean / T(n);
		}
	}

	if (degenerateCount)
	{
		LOG_WARNING_STREAM("WARNING: Matrix C needed for eigen decomposition was degenerated in " << degenerateCount << " points over " << pointsCount << " (" << float(degenerateCount)*100.f/float(pointsCount) << " %)");
	}

}

template struct SurfaceNormalDataPointsFilter<float>;
template struct SurfaceNormalDataPointsFilter<double>;

