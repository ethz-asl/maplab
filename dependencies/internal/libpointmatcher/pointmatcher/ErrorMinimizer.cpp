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

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

///////////////////////////////////////
// ErrorElements
///////////////////////////////////////

//! Constructor without data
template<typename T>
PointMatcher<T>::ErrorMinimizer::ErrorElements::ErrorElements():
	reading(DataPoints()),
	reference(DataPoints()),
	weights(OutlierWeights()),
	matches(Matches()),
	nbRejectedMatches(-1),
	nbRejectedPoints(-1),
	pointUsedRatio(-1.0),
	weightedPointUsedRatio(-1.0)
{
}

//! Constructor from existing data. This will align the data.
template<typename T>
PointMatcher<T>::ErrorMinimizer::ErrorElements::ErrorElements(const DataPoints& requestedPts, const DataPoints& sourcePts, const OutlierWeights& outlierWeights, const Matches& matches)
{
	typedef typename Matches::Ids Ids;
	typedef typename Matches::Dists Dists;
	
	assert(matches.ids.rows() > 0);
	assert(matches.ids.cols() > 0);
	assert(matches.ids.cols() == requestedPts.features.cols()); //nbpts
	assert(outlierWeights.rows() == matches.ids.rows());  // knn
	
	const int knn = outlierWeights.rows();
	const int dimFeat = requestedPts.features.rows();
	const int dimReqDesc = requestedPts.descriptors.rows();
	const int dimReqTime = requestedPts.times.rows();

	// Count points with no weights
	const int pointsCount = (outlierWeights.array() != 0.0).count();
	if (pointsCount == 0)
		throw ConvergenceError("ErrorMnimizer: no point to minimize");

	Matrix keptFeat(dimFeat, pointsCount);
	
	Matrix keptDesc;
	if(dimReqDesc > 0)
		keptDesc = Matrix(dimReqDesc, pointsCount);
	
	Int64Matrix keptTime;
	if(dimReqTime > 0)
		keptTime = Int64Matrix(dimReqTime, pointsCount);
	
	Matches keptMatches (Dists(1,pointsCount), Ids(1, pointsCount));
	OutlierWeights keptWeights(1, pointsCount);

	int j = 0;
	int rejectedMatchCount = 0;
	int rejectedPointCount = 0;
	bool matchExist = false;
	this->weightedPointUsedRatio = 0;
	
	for (int i = 0; i < requestedPts.features.cols(); ++i) //nb pts
	{
		matchExist = false;
		for(int k = 0; k < knn; k++) // knn
		{
			const auto matchDist = matches.dists(k, i);
			if (matchDist == Matches::InvalidDist){
				continue;
			}

			if (outlierWeights(k,i) != 0.0)
			{
				if(dimReqDesc > 0)
					keptDesc.col(j) = requestedPts.descriptors.col(i);

				if(dimReqTime > 0)
					keptTime.col(j) = requestedPts.times.col(i);

				
				keptFeat.col(j) = requestedPts.features.col(i);
				keptMatches.ids(0, j) = matches.ids(k, i);
				keptMatches.dists(0, j) = matchDist;
				keptWeights(0,j) = outlierWeights(k,i);
				++j;
				this->weightedPointUsedRatio += outlierWeights(k,i);
				matchExist = true;
			}
			else
			{
				rejectedMatchCount++;
			}
		}

		if(matchExist == false)
		{
			rejectedPointCount++;
		}
	}

	assert(j == pointsCount);

	this->pointUsedRatio = T(j)/T(knn*requestedPts.features.cols());
	this->weightedPointUsedRatio /= T(knn*requestedPts.features.cols());
	
	assert(dimFeat == sourcePts.features.rows());
	const int dimSourDesc = sourcePts.descriptors.rows();
	const int dimSourTime = sourcePts.times.rows();
	
	Matrix associatedFeat(dimFeat, pointsCount);
	
	Matrix associatedDesc;
	if(dimSourDesc > 0)
		associatedDesc = Matrix(dimSourDesc, pointsCount);
	
	Int64Matrix associatedTime;
	if(dimSourTime> 0)
		associatedTime = Int64Matrix(dimSourTime, pointsCount);
	
	// Fetch matched points
	for (int i = 0; i < pointsCount; ++i)
	{
		const int refIndex(keptMatches.ids(i));
		associatedFeat.col(i) = sourcePts.features.block(0, refIndex, dimFeat, 1);
		
		if(dimSourDesc > 0)
			associatedDesc.col(i) = sourcePts.descriptors.block(0, refIndex, dimSourDesc, 1);

		if(dimSourTime> 0)
			associatedTime.col(i) = sourcePts.times.block(0, refIndex, dimSourTime, 1);

	}

	// Copy final data to structure
	this->reading = DataPoints(
		keptFeat, 
		requestedPts.featureLabels,
		keptDesc,
		requestedPts.descriptorLabels,
		keptTime,
		requestedPts.timeLabels
	);

	this->reference = DataPoints(
		associatedFeat,
		sourcePts.featureLabels,
		associatedDesc,
		sourcePts.descriptorLabels,
		associatedTime,
		sourcePts.timeLabels
	);

	this->weights = keptWeights;
	this->matches = keptMatches;
	this->nbRejectedMatches = rejectedMatchCount;
	this->nbRejectedPoints = rejectedPointCount;
}


///////////////////////////////////////
// ErrorMinimizer
///////////////////////////////////////

//! Construct without parameter
template<typename T>
PointMatcher<T>::ErrorMinimizer::ErrorMinimizer()
{}

//! Construct with parameters
template<typename T>
PointMatcher<T>::ErrorMinimizer::ErrorMinimizer(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	Parametrizable(className,paramsDoc,params)
{}

//! virtual destructor
template<typename T>
PointMatcher<T>::ErrorMinimizer::~ErrorMinimizer()
{}

//! Find the transformation that minimizes the error
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ErrorMinimizer::compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches)
{
	
	// generates pairs of matching points
	typename ErrorMinimizer::ErrorElements matchedPoints(filteredReading, filteredReference, outlierWeights, matches);
	
	// calls specific instantiation for a given ErrorMinimizer
	TransformationParameters transform = this->compute(matchedPoints);
	
	// saves paired points for future introspection
	this->lastErrorElements = matchedPoints;
	
	// returns transforme parameters
	return transform;
}

//! Return the ratio of how many points were used for error minimization
template<typename T>
T PointMatcher<T>::ErrorMinimizer::getPointUsedRatio() const
{
	return lastErrorElements.pointUsedRatio;
}

//! Return the last the ErrorElements structure that was used for error minimization.
template<typename T>
typename PointMatcher<T>::ErrorMinimizer::ErrorElements PointMatcher<T>::ErrorMinimizer::getErrorElements() const
{
	//Warning: the use of the variable lastErrorElements is not standardized yet.
	return lastErrorElements;
}

//! Return the ratio of how many points were used (with weight) for error minimization
template<typename T>
T PointMatcher<T>::ErrorMinimizer::getWeightedPointUsedRatio() const
{
	return lastErrorElements.weightedPointUsedRatio;
}

//! If not redefined by child class, return the ratio of how many points were used (with weight) for error minimization
template<typename T>
T PointMatcher<T>::ErrorMinimizer::getOverlap() const
{
	LOG_WARNING_STREAM("ErrorMinimizer - warning, no specific method to compute overlap was provided for the ErrorMinimizer used.");
	return lastErrorElements.weightedPointUsedRatio;
}

//! If not redefined by child class, return zero matrix
template<typename T>
typename PointMatcher<T>::Matrix PointMatcher<T>::ErrorMinimizer::getCovariance() const
{
	LOG_WARNING_STREAM("ErrorMinimizer - warning, no specific method to compute covariance was provided for the ErrorMinimizer used.");
	return Matrix::Zero(6,6);
}

//! If not redefined by child class, return max value for T
template<typename T>
T PointMatcher<T>::ErrorMinimizer::getResidualError(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches) const
{
	LOG_WARNING_STREAM("ErrorMinimizer - warning, no specific method to compute residual was provided for the ErrorMinimizer used.");
	return std::numeric_limits<T>::max();
}

//! Helper funtion doing the cross product in 3D and a pseudo cross product in 2D
template<typename T>
typename PointMatcher<T>::Matrix PointMatcher<T>::ErrorMinimizer::crossProduct(const Matrix& A, const Matrix& B)
{
	//Note: A = [x, y, z, 1] and B = [x, y, z] for convenience

	// Expecting matched points
	assert(A.cols() == B.cols());

	// Expecting homogenous coord X eucl. coord
	assert(A.rows() -1 == B.rows());

	// Expecting homogenous coordinates
	assert(A.rows() == 4 || A.rows() == 3);
	
	const unsigned int x = 0;
	const unsigned int y = 1;
	const unsigned int z = 2;

	Matrix cross;
	if(A.rows() == 4)
	{
		cross = Matrix(B.rows(), B.cols());
				
		cross.row(x) = A.row(y).array() * B.row(z).array() - A.row(z).array() * B.row(y).array();
		cross.row(y) = A.row(z).array() * B.row(x).array() - A.row(x).array() * B.row(z).array();
		cross.row(z) = A.row(x).array() * B.row(y).array() - A.row(y).array() * B.row(x).array();
	}
	else
	{
		//pseudo-cross product for 2D vectors
		cross = Vector(B.cols());
		cross = A.row(x).array() * B.row(y).array() - A.row(y).array() * B.row(x).array();
	}
	return cross;
}




template struct PointMatcher<float>::ErrorMinimizer;
template struct PointMatcher<double>::ErrorMinimizer;
