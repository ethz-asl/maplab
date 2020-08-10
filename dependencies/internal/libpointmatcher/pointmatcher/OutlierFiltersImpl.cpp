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

#include "OutlierFiltersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"
#include "MatchersImpl.h"

#include <algorithm>
#include <vector>
#include <iostream>
#include <limits>
#include <numeric>
#include <ciso646>

using namespace std;
using namespace PointMatcherSupport;

// NullOutlierFilter
template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::NullOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights::Constant(input.ids.rows(), input.ids.cols(), 1);
}

template struct OutlierFiltersImpl<float>::NullOutlierFilter;
template struct OutlierFiltersImpl<double>::NullOutlierFilter;


// MaxDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MaxDistOutlierFilter::MaxDistOutlierFilter(const Parameters& params):
	OutlierFilter("MaxDistOutlierFilter", MaxDistOutlierFilter::availableParameters(), params),
	maxDist(pow(Parametrizable::get<T>("maxDist"),2)) // we use the square distance later
{
}


template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MaxDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights((input.dists.array() <= maxDist).template cast<T>());
}

template struct OutlierFiltersImpl<float>::MaxDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MaxDistOutlierFilter;

// MinDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MinDistOutlierFilter::MinDistOutlierFilter(const Parameters& params):
	OutlierFilter("MinDistOutlierFilter", MinDistOutlierFilter::availableParameters(), params),
	minDist(pow(Parametrizable::get<T>("minDist"),2))// Note: we use the square distance later
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MinDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	return OutlierWeights((input.dists.array() >= minDist).template cast<T>());
}

template struct OutlierFiltersImpl<float>::MinDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MinDistOutlierFilter;



// MedianDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::MedianDistOutlierFilter::MedianDistOutlierFilter(const Parameters& params):
	OutlierFilter("MedianDistOutlierFilter", MedianDistOutlierFilter::availableParameters(), params),
	factor(Parametrizable::get<T>("factor"))
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::MedianDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const T median = input.getDistsQuantile(0.5);
	const T limit = factor * median;
	return OutlierWeights((input.dists.array() <= limit).template cast<T>());
}

template struct OutlierFiltersImpl<float>::MedianDistOutlierFilter;
template struct OutlierFiltersImpl<double>::MedianDistOutlierFilter;


// TrimmedDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::TrimmedDistOutlierFilter::TrimmedDistOutlierFilter(const Parameters& params):
	OutlierFilter("TrimmedDistOutlierFilter", TrimmedDistOutlierFilter::availableParameters(), params),
	ratio(Parametrizable::get<T>("ratio"))
{
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::TrimmedDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const T limit = input.getDistsQuantile(ratio);
	return OutlierWeights((input.dists.array() <= limit).template cast<T>());
}

template struct OutlierFiltersImpl<float>::TrimmedDistOutlierFilter;
template struct OutlierFiltersImpl<double>::TrimmedDistOutlierFilter;

// VarTrimmedDistOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter::VarTrimmedDistOutlierFilter(const Parameters& params):
	OutlierFilter("VarTrimmedDistOutlierFilter", VarTrimmedDistOutlierFilter::availableParameters(), params),
	minRatio(Parametrizable::get<T>("minRatio")),
	maxRatio(Parametrizable::get<T>("maxRatio")),
	lambda(Parametrizable::get<T>("lambda"))
{
	if (this->minRatio >= this->maxRatio)
	{
		throw InvalidParameter((boost::format("VarTrimmedDistOutlierFilter: minRatio (%1%) should be smaller than maxRatio (%2%)") % minRatio % maxRatio).str());
	}
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const T tunedRatio = optimizeInlierRatio(input);
	LOG_INFO_STREAM("Optimized ratio: " << tunedRatio);

	const T limit = input.getDistsQuantile(tunedRatio);
	return OutlierWeights((input.dists.array() <= limit).template cast<T>());
}

template<typename T>
T OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter::optimizeInlierRatio(const Matches& matches)
{
	typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;
	typedef typename Eigen::Array<T, Eigen::Dynamic, 1> LineArray;
	
	const int points_nbr = matches.dists.rows() * matches.dists.cols();
	
	// vector containing the squared distances of the matches
	std::vector<T> tmpSortedDist;
	tmpSortedDist.reserve(points_nbr);
	for (int x = 0; x < matches.dists.cols(); ++x)
		for (int y = 0; y < matches.dists.rows(); ++y)
			if ((matches.dists(y, x) != numeric_limits<T>::infinity()) && (matches.dists(y, x) > 0))
				tmpSortedDist.push_back(matches.dists(y, x));
	if (tmpSortedDist.size() == 0)
		throw ConvergenceError("no outlier to filter");
			
	std::sort(tmpSortedDist.begin(), tmpSortedDist.end());
	std::vector<T> tmpCumSumSortedDist;
	tmpCumSumSortedDist.reserve(points_nbr);
	std::partial_sum(tmpSortedDist.begin(), tmpSortedDist.end(), tmpCumSumSortedDist.begin());

	const int minEl = floor(this->minRatio*points_nbr);
	const int maxEl = floor(this->maxRatio*points_nbr);

	// Return std::vector to an eigen::vector
	Eigen::Map<LineArray> sortedDist(&tmpCumSumSortedDist[0], points_nbr);

	const LineArray trunkSortedDist = sortedDist.segment(minEl, maxEl-minEl);

	const LineArray ids = LineArray::LinSpaced(trunkSortedDist.rows(), minEl+1, maxEl);
	const LineArray ratio = ids / points_nbr; // ratio for each of element between minEl and maxEl
	const LineArray deno = ratio.pow(this->lambda); // f^λ
	// frms = cumSumDists[minEl:maxEl] / id / (f^λ)²
	const LineArray FRMS = trunkSortedDist * ids.inverse() * deno.inverse().square() ;
	int minIndex(0);// = FRMS.minCoeff();
	FRMS.minCoeff(&minIndex);
	const T optRatio = (float)(minIndex + minEl)/ (float)points_nbr;
	
	return optRatio;
}

template struct OutlierFiltersImpl<float>::VarTrimmedDistOutlierFilter;
template struct OutlierFiltersImpl<double>::VarTrimmedDistOutlierFilter;

// SurfaceNormalOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::SurfaceNormalOutlierFilter::SurfaceNormalOutlierFilter(const Parameters& params):
	OutlierFilter("SurfaceNormalOutlierFilter", SurfaceNormalOutlierFilter::availableParameters(), params),
	eps(cos(Parametrizable::get<T>("maxAngle"))),
	warningPrinted(false)
{
	//warning: eps is change to cos(maxAngle)!
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::SurfaceNormalOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	const BOOST_AUTO(normalsReading, filteredReading.getDescriptorViewByName("normals"));
	const BOOST_AUTO(normalsReference, filteredReference.getDescriptorViewByName("normals"));
	
	// select weight from median
	OutlierWeights w(input.dists.rows(), input.dists.cols());

	if(normalsReading.cols() != 0 && normalsReference.cols() != 0)
	{
		for (int x = 0; x < w.cols(); ++x) // pts in reading
		{
			const Vector normalRead = normalsReading.col(x).normalized();

			for (int y = 0; y < w.rows(); ++y) // knn 
			{
				const int idRef = input.ids(y, x);

				if (idRef == MatchersImpl<T>::NNS::InvalidIndex) {
					w(y, x) = 0;
					continue;
				}

				const Vector normalRef = normalsReference.col(idRef).normalized();

				const T value = anyabs(normalRead.dot(normalRef));

				if(value < eps) // test to keep the points
					w(y, x) = 0;
				else
					w(y, x) = 1;
			}
		}
	}
	else
	{
		if(warningPrinted == false)
		{
			LOG_INFO_STREAM("SurfaceNormalOutlierFilter: surface normals not available. Skipping filtering");
			warningPrinted = true;
		}

		w = Matrix::Ones(input.dists.rows(), input.dists.cols());
	}
	//abort();
	return w;
}

template struct OutlierFiltersImpl<float>::SurfaceNormalOutlierFilter;
template struct OutlierFiltersImpl<double>::SurfaceNormalOutlierFilter;

// GenericDescriptorOutlierFilter
template<typename T>
OutlierFiltersImpl<T>::GenericDescriptorOutlierFilter::GenericDescriptorOutlierFilter(const Parameters& params):
	OutlierFilter("GenericDescriptorOutlierFilter", GenericDescriptorOutlierFilter::availableParameters(), params),
	source(Parametrizable::getParamValueString("source")),
	descName(Parametrizable::getParamValueString("descName")),
	useSoftThreshold(Parametrizable::get<bool>("useSoftThreshold")),
	useLargerThan(Parametrizable::get<bool>("useLargerThan")),
	threshold(Parametrizable::get<T>("threshold"))
{
	if(source != "reference" && source != "reading")
	{
		throw InvalidParameter(
		(boost::format("GenericDescriptorOutlierFilter: Error, the parameter named 'source' can only be set to 'reference' or 'reading' but was set to %1%") % source).str());
	}
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::GenericDescriptorOutlierFilter::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	typedef typename DataPoints::ConstView ConstView;

	const int knn = input.dists.rows();
	const int readPtsCount = input.dists.cols();
	
	OutlierWeights w(knn, readPtsCount);

	const DataPoints *cloud;

	if(source == "reference")
		cloud = &filteredReference;
	else
		cloud = &filteredReference;

	ConstView desc(cloud->getDescriptorViewByName(descName));

	if(desc.rows() != 1)
	{
		throw InvalidParameter(
		(boost::format("GenericDescriptorOutlierFilter: Error, the parameter named 'descName' must be a 1D descriptor but the field %1% is %2%D") % descName % desc.rows()).str());
	}

	for(int k=0; k < knn; k++)
	{
		for(int i=0; i < readPtsCount; i++)
		{
			const int idRead = input.ids(k, i);
			if (idRead == MatchersImpl<T>::NNS::InvalidIndex){
				w(k,i) = 0;
				continue;
			}
			if(useSoftThreshold == false)
			{
				if(useLargerThan == true)
				{
					if (desc(0, idRead) > threshold)
						w(k,i) = 1;
					else
						w(k,i) = 0;
				}
				else
				{
					if (desc(0, idRead) < threshold)
						w(k,i) = 1;
					else
						w(k,i) = 0;
				}
			}
			else
			{
				// use soft threshold by assigning the weight using the descriptor
				w(k,i) = desc(0, idRead);
			}
		}
	}

	//Normalize
	if(useSoftThreshold)
		w = w/w.maxCoeff();

	return w;
}

template struct OutlierFiltersImpl<float>::GenericDescriptorOutlierFilter;
template struct OutlierFiltersImpl<double>::GenericDescriptorOutlierFilter;

// RobustOutlierFilter
template<typename T>
typename OutlierFiltersImpl<T>::RobustOutlierFilter::RobustFctMap
OutlierFiltersImpl<T>::RobustOutlierFilter::robustFcts = {
	{"cauchy",  RobustFctId::Cauchy},
	{"welsch",  RobustFctId::Welsch},
	{"sc",      RobustFctId::SwitchableConstraint},
	{"gm",      RobustFctId::GM},
	{"tukey",   RobustFctId::Tukey},
	{"huber",   RobustFctId::Huber},
	{"L1",      RobustFctId::L1},
	{"student", RobustFctId::Student}
};

template<typename T>
OutlierFiltersImpl<T>::RobustOutlierFilter::RobustOutlierFilter(const std::string& className,
		const ParametersDoc paramsDoc,
		const Parameters& params):
	OutlierFilter(className, paramsDoc, params),
	robustFctName(Parametrizable::get<string>("robustFct")),
	tuning(Parametrizable::get<T>("tuning")),
	squaredApproximation(pow(Parametrizable::get<T>("approximation"), 2)),
	scaleEstimator(Parametrizable::get<string>("scaleEstimator")),
	nbIterationForScale(Parametrizable::get<int>("nbIterationForScale")),
	distanceType(Parametrizable::get<string>("distanceType")),
	robustFctId(-1),
	iteration(1),
	scale(0.0),
	berg_target_scale(0)
{
	const set<string> validScaleEstimator = {"none", "mad", "berg", "std"};
	if (validScaleEstimator.find(scaleEstimator) == validScaleEstimator.end()) {
		throw InvalidParameter("Invalid scale estimator name.");
	}
	const set<string> validDistanceType = {"point2point", "point2plane"};
	if (validDistanceType.find(distanceType) == validDistanceType.end()) {
		throw InvalidParameter("Invalid distance type name.");
	}

	resolveEstimatorName();

	if (scaleEstimator == "berg") {
		berg_target_scale = tuning;

		// See \cite{Bergstrom2014}
		if (robustFctId == RobustFctId::Cauchy)
		{
			tuning = 4.3040;
		} else if (robustFctId == RobustFctId::Tukey)
		{
			tuning = 7.0589;
		} else if (robustFctId == RobustFctId::Huber)
		{
			tuning = 2.0138;
		}
	}
}

template<typename T>
OutlierFiltersImpl<T>::RobustOutlierFilter::RobustOutlierFilter(const Parameters& params):
	RobustOutlierFilter("RobustOutlierFilter", RobustOutlierFilter::availableParameters(), params)
{
}


template<typename T>
void OutlierFiltersImpl<T>::RobustOutlierFilter::resolveEstimatorName(){
	if (robustFcts.find(robustFctName) == robustFcts.end())
	{
		throw InvalidParameter("Invalid robust function name.");
	}
	robustFctId = robustFcts[robustFctName];
}

	template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::RobustOutlierFilter::compute(
		const DataPoints& filteredReading,
		const DataPoints& filteredReference,
		const Matches& input)
{
	return this->robustFiltering(filteredReading, filteredReference, input);
}



template<typename T>
typename PointMatcher<T>::Matrix
OutlierFiltersImpl<T>::RobustOutlierFilter::computePointToPlaneDistance(
		const DataPoints& reading,
		const DataPoints& reference,
		const Matches& input) {

	int nbr_read_point = input.dists.cols();
	int nbr_match = input.dists.rows();

	Matrix normals = reference.getDescriptorViewByName("normals");

	Vector reading_point(Vector::Zero(3));
	Vector reference_point(Vector::Zero(3));
	Vector normal(3);

	Matrix dists(Matrix::Zero(nbr_match, nbr_read_point));

	for(int i = 0; i < nbr_read_point; ++i)
	{
		reading_point = reading.features.block(0, i, 3, 1);
		for(int j = 0; j < nbr_match; ++j)
		{
			const int reference_idx = input.ids(j, i);
			if (reference_idx != Matches::InvalidId) {
				reference_point = reference.features.block(0, reference_idx, 3, 1);

				normal = normals.col(reference_idx).normalized();
				// distance_point_to_plan = dot(n, p-q)²
				dists(j, i) = pow(normal.dot(reading_point-reference_point), 2);
			}
		}
	}

	return dists;
}

template<typename T>
typename PointMatcher<T>::OutlierWeights OutlierFiltersImpl<T>::RobustOutlierFilter::robustFiltering(
		const DataPoints& filteredReading,
		const DataPoints& filteredReference,
		const Matches& input) {

	if (scaleEstimator == "mad")
	{
		if (iteration <= nbIterationForScale or nbIterationForScale == 0)
		{
			scale = sqrt(input.getMedianAbsDeviation());
		}
	} else if (scaleEstimator == "std")
	{
		if (iteration <= nbIterationForScale or nbIterationForScale == 0)
		{
			scale = sqrt(input.getStandardDeviation());
		}
	} else if (scaleEstimator == "berg")
	{
		if (iteration <= nbIterationForScale or nbIterationForScale == 0)
		{
			// The tuning constant is the target scale that we want to reach
			// It's a bit confusing to use the tuning constant for scaling...
			if (iteration == 1)
			{
				scale = 1.9 * sqrt(input.getDistsQuantile(0.5));
			}
			else
			{ // TODO: maybe add it has another parameter or make him a function of the max iteration
				const T CONVERGENCE_RATE = 0.85;
				scale = CONVERGENCE_RATE * (scale - berg_target_scale) + berg_target_scale;
			}
		}
	}
	else
	{
		scale = 1.0; // We don't rescale
	}
	iteration++;

	Matrix dists = distanceType == "point2point" ? input.dists : computePointToPlaneDistance(filteredReading, filteredReference, input);

	// e² = scaled squared distance
	Array e2 = dists.array() / (scale * scale);

	T k = tuning;
	const T k2 = k * k;
	Array w, aboveThres, belowThres;
	switch (robustFctId) {
		case RobustFctId::Cauchy: // 1/(1 + e²/k²)
			w = (1 + e2 / k2).inverse();
			break;
		case RobustFctId::Welsch: // exp(-e²/k²)
			w = (-e2 / k2).exp();
			break;
		case RobustFctId::SwitchableConstraint: // if e² > k then 4 * k²/(k + e²)²
			aboveThres = 4.0 * k2 * ((k + e2).square()).inverse();
			w = (e2 >= k).select(aboveThres, 1.0);
			break;
		case RobustFctId::GM:    // k²/(k + e²)²
			w = k2*((k + e2).square()).inverse();
			break;
		case RobustFctId::Tukey: // if e² < k² then (1-e²/k²)²
			belowThres = (1 - e2 / k2).square();
			w = (e2 >= k2).select(0.0, belowThres);
			break;
		case RobustFctId::Huber: // if |e| >= k then k/|e| = k/sqrt(e²)
			aboveThres = k * (e2.sqrt().inverse());
			w = (e2 >= k2).select(aboveThres, 1.0);
			break;
		case RobustFctId::L1: // 1/|e| = 1/sqrt(e²)
			w = e2.sqrt().inverse();
			break;
		case RobustFctId::Student: { // ....
			const T d = 3;
			auto p = (1 + e2 / k).pow(-(k + d) / 2);
			w = p * (k + d) * (k + e2).inverse();
			break;
		}
		default:
			break;
	}

	// In the minimizer, zero weight are ignored, we want them to be notice by having the smallest value
	// The value can not be a numeric limit, since they might cause a nan/inf.
	const double ARBITRARY_SMALL_VALUE = 1e-50;
	w = (w.array() <= ARBITRARY_SMALL_VALUE).select(ARBITRARY_SMALL_VALUE, w);


	if(squaredApproximation != std::numeric_limits<T>::infinity())
	{
		//Note from Eigen documentation: (if statement).select(then matrix, else matrix)
		w = (e2 >= squaredApproximation).select(0.0, w);
	}

	return w;
}


template struct OutlierFiltersImpl<float>::RobustOutlierFilter;
template struct OutlierFiltersImpl<double>::RobustOutlierFilter;
