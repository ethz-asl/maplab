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

#include "MatchersImpl.h"
#include "PointMatcherPrivate.h"

// NullMatcher
template<typename T>
void MatchersImpl<T>::NullMatcher::init(
	const DataPoints& filteredReference)
{
	
}

template<typename T>
typename PointMatcher<T>::Matches MatchersImpl<T>::NullMatcher::findClosests(
	const DataPoints& filteredReading)
{
	return Matches();
}

template struct MatchersImpl<float>::NullMatcher;
template struct MatchersImpl<double>::NullMatcher;



// KDTreeMatcher
template<typename T>
MatchersImpl<T>::KDTreeMatcher::KDTreeMatcher(const Parameters& params):
	Matcher("KDTreeMatcher", KDTreeMatcher::availableParameters(), params),
	knn(Parametrizable::get<int>("knn")),
	epsilon(Parametrizable::get<T>("epsilon")),
	searchType(NNSearchType(Parametrizable::get<int>("searchType"))),
	maxDist(Parametrizable::get<T>("maxDist"))
{
	LOG_INFO_STREAM("* KDTreeMatcher: initialized with knn=" << knn << ", epsilon=" << epsilon << ", searchType=" << searchType << " and maxDist=" << maxDist);
}

template<typename T>
MatchersImpl<T>::KDTreeMatcher::~KDTreeMatcher()
{

}

template<typename T>
void MatchersImpl<T>::KDTreeMatcher::init(
	const DataPoints& filteredReference)
{
	// build and populate NNS
	featureNNS.reset( NNS::create(filteredReference.features, filteredReference.features.rows() - 1, searchType, NNS::TOUCH_STATISTICS));
}

template<typename T>
typename PointMatcher<T>::Matches MatchersImpl<T>::KDTreeMatcher::findClosests(
	const DataPoints& filteredReading)
{
	
	const int pointsCount(filteredReading.features.cols());
	Matches matches(
		typename Matches::Dists(knn, pointsCount),
		typename Matches::Ids(knn, pointsCount)
	);
	
	static_assert(NNS::InvalidIndex == Matches::InvalidId, "");
	static_assert(NNS::InvalidValue == Matches::InvalidDist, "");
	this->visitCounter += featureNNS->knn(filteredReading.features, matches.ids, matches.dists, knn, epsilon, NNS::ALLOW_SELF_MATCH, maxDist);

	return matches;
}

template struct MatchersImpl<float>::KDTreeMatcher;
template struct MatchersImpl<double>::KDTreeMatcher;

// KDTreeVarDistMatcher
template<typename T>
MatchersImpl<T>::KDTreeVarDistMatcher::KDTreeVarDistMatcher(const Parameters& params):
	Matcher("KDTreeVarDistMatcher", KDTreeVarDistMatcher::availableParameters(), params),
	knn(Parametrizable::get<int>("knn")),
	epsilon(Parametrizable::get<T>("epsilon")),
	searchType(NNSearchType(Parametrizable::get<int>("searchType"))),
	maxDistField(Parametrizable::getParamValueString("maxDistField"))
{
	LOG_INFO_STREAM("* KDTreeVarDsitMatcher: initialized with knn=" << knn << ", epsilon=" << epsilon << ", searchType=" << searchType << " and maxDistField=" << maxDistField);
}

template<typename T>
MatchersImpl<T>::KDTreeVarDistMatcher::~KDTreeVarDistMatcher()
{

}

template<typename T>
void MatchersImpl<T>::KDTreeVarDistMatcher::init(
	const DataPoints& filteredReference)
{
	// build and populate NNS
	featureNNS.reset( NNS::create(filteredReference.features, filteredReference.features.rows() - 1, searchType, NNS::TOUCH_STATISTICS));
}

template<typename T>
typename PointMatcher<T>::Matches MatchersImpl<T>::KDTreeVarDistMatcher::findClosests(
	const DataPoints& filteredReading)
{
	
	const int pointsCount(filteredReading.features.cols());
	Matches matches(
		typename Matches::Dists(knn, pointsCount),
		typename Matches::Ids(knn, pointsCount)
	);
	
	const BOOST_AUTO(maxDists, filteredReading.getDescriptorViewByName(maxDistField));
	
	static_assert(NNS::InvalidIndex == Matches::InvalidId, "");
	static_assert(NNS::InvalidValue == Matches::InvalidDist, "");
	this->visitCounter += featureNNS->knn(filteredReading.features, matches.ids, matches.dists, maxDists.transpose(), knn, epsilon, NNS::ALLOW_SELF_MATCH);

	return matches;
}

template struct MatchersImpl<float>::KDTreeVarDistMatcher;
template struct MatchersImpl<double>::KDTreeVarDistMatcher;
