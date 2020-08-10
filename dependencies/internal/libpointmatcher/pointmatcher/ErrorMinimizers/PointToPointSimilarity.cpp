// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Eigen/SVD"

using namespace Eigen;

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPointSimilarityErrorMinimizer<T>::compute(const ErrorElements& mPts_const)
{
	
	// Copy error element to use as storage later
	// TODO: check that, might worth it to only copy useful parts
	ErrorElements mPts = mPts_const;
	
	// now minimize on kept points
	const int dimCount(mPts.reading.features.rows());
	//const int ptsCount(mPts.reading.features.cols()); //Both point clouds have now the same number of (matched) point
	
	
	// Compute the (weighted) mean of each point cloud
	//TODO: change the member weights to be a Vector
	const Vector w = mPts.weights.row(0);
	const T w_sum_inv = T(1.)/w.sum();
	const Vector meanReading =
		(mPts.reading.features.topRows(dimCount-1).array().rowwise() * w.array().transpose()).rowwise().sum() * w_sum_inv;
	const Vector meanReference =
		(mPts.reference.features.topRows(dimCount-1).array().rowwise() * w.array().transpose()).rowwise().sum() * w_sum_inv;
	
	
	// Remove the mean from the point clouds
	mPts.reading.features.topRows(dimCount-1).colwise() -= meanReading;
	mPts.reference.features.topRows(dimCount-1).colwise() -= meanReference;
	
	const T sigma = mPts.reading.features.topRows(dimCount-1).colwise().squaredNorm().cwiseProduct(w.transpose()).sum();
	
	// Singular Value Decomposition
	const Matrix m(mPts.reference.features.topRows(dimCount-1) * w.asDiagonal()
		       * mPts.reading.features.topRows(dimCount-1).transpose());
	const JacobiSVD<Matrix> svd(m, ComputeThinU | ComputeThinV);
	Matrix rotMatrix(svd.matrixU() * svd.matrixV().transpose());
	typedef typename JacobiSVD<Matrix>::SingularValuesType SingularValuesType;
	SingularValuesType singularValues = svd.singularValues();
	// It is possible to get a reflection instead of a rotation. In this case, we
	// take the second best solution, guaranteed to be a rotation. For more details,
	// read the tech report: "Least-Squares Rigid Motion Using SVD", Olga Sorkine
	// http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
	if (rotMatrix.determinant() < 0.)
	{
		Matrix tmpV = svd.matrixV().transpose();
		tmpV.row(dimCount-2) *= -1.;
		rotMatrix = svd.matrixU() * tmpV;
		singularValues(dimCount-2) *= -1.;
	}
	T scale = singularValues.sum() / sigma;
	if (sigma < 0.0001) scale = T(1);
	const Vector trVector(meanReference - scale * rotMatrix * meanReading);
	
	Matrix result(Matrix::Identity(dimCount, dimCount));
	result.topLeftCorner(dimCount-1, dimCount-1) = scale * rotMatrix;
	result.topRightCorner(dimCount-1, 1) = trVector;
	
	return result;
}

template<typename T>
T PointToPointSimilarityErrorMinimizer<T>::getResidualError(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches) const
{
	assert(matches.ids.rows() > 0);
	
	// Fetch paired points
	typename ErrorMinimizer::ErrorElements mPts(filteredReading, filteredReference, outlierWeights, matches);
	
	return PointToPointErrorMinimizer<T>::computeResidualError(mPts);
}

template<typename T>
T PointToPointSimilarityErrorMinimizer<T>::getOverlap() const
{
	//NOTE: computing overlap of 2 point clouds can be complicated due to
	// the sparse nature of the representation. Here is only an estimate
	// of the true overlap.
	const int nbPoints = this->lastErrorElements.reading.features.cols();
	const int dim = this->lastErrorElements.reading.features.rows();
	if(nbPoints == 0)
	{
		throw std::runtime_error("Error, last error element empty. Error minimizer needs to be called at least once before using this method.");
	}
	
	if (!this->lastErrorElements.reading.descriptorExists("simpleSensorNoise"))
	{
		LOG_INFO_STREAM("PointToPointSimilarityErrorMinimizer - warning, no sensor noise found. Using best estimate given outlier rejection instead.");
		return this->getWeightedPointUsedRatio();
	}
	
	const BOOST_AUTO(noises, this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise"));
	
	const Vector dists = (this->lastErrorElements.reading.features.topRows(dim-1) - this->lastErrorElements.reference.features.topRows(dim-1)).colwise().norm();
	const T mean = dists.sum()/nbPoints;
	
	int count = 0;
	for(int i=0; i < nbPoints; i++)
	{
		if(dists(i) < (mean + noises(0,i)))
		{
			count++;
		}
	}
	
	return (T)count/(T)nbPoints;
}

template struct PointToPointSimilarityErrorMinimizer<float>;
template struct PointToPointSimilarityErrorMinimizer<double>;
