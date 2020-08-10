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

#ifndef __POINTMATCHER_MATCHERS_H
#define __POINTMATCHER_MATCHERS_H

#include "PointMatcher.h"

#include "nabo/nabo.h"
#if NABO_VERSION_INT < 10007
	#error "You need libnabo version 1.0.7 or greater"
#endif

template<typename T>
struct MatchersImpl
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	
	typedef typename Nabo::NearestNeighbourSearch<T> NNS;
	typedef typename NNS::SearchType NNSearchType;
	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::Matcher Matcher;
	typedef typename PointMatcher<T>::Matches Matches;
	
	struct NullMatcher: public Matcher
	{
		inline static const std::string description()
		{
			return "Does nothing, returns no match.";
		}

		NullMatcher() : Matcher("NullMatcher",  ParametersDoc(), Parameters()) {}
		virtual void init(const DataPoints& filteredReference);
		virtual Matches findClosests(const DataPoints& filteredReading);
	};

	struct KDTreeMatcher: public Matcher
	{
		inline static const std::string description()
		{
			return "This matcher matches a point from the reading to its closest neighbors in the reference.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return {
				{"knn", "number of nearest neighbors to consider it the reference", "1", "1", "2147483647", &P::Comp<unsigned>},
				{"epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T>},
				{"searchType", "Nabo search type. 0: brute force, check distance to every point in the data (very slow), 1: kd-tree with linear heap, good for small knn (~up to 30) and 2: kd-tree with tree heap, good for large knn (~from 30)", "1", "0", "2", &P::Comp<unsigned>},
				{"maxDist", "maximum distance to consider for neighbors", "inf", "0", "inf", &P::Comp<T>}
			};
		}
		
		const int knn;
		const T epsilon;
		const NNSearchType searchType;
		const T maxDist;

	protected:
		std::shared_ptr<NNS> featureNNS;

	public:
		KDTreeMatcher(const Parameters& params = Parameters());
		virtual ~KDTreeMatcher();
		virtual void init(const DataPoints& filteredReference);
		virtual Matches findClosests(const DataPoints& filteredReading);
	};

	struct KDTreeVarDistMatcher: public Matcher
	{
		inline static const std::string description()
		{
			return "This matcher matches a point from the reading to its closest neighbors in the reference. A maximum search radius per point can be defined.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return {
				{"knn", "number of nearest neighbors to consider it the reference", "1", "1", "2147483647", &P::Comp<unsigned>},
				{"epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T>},
				{"searchType", "Nabo search type. 0: brute force, check distance to every point in the data (very slow), 1: kd-tree with linear heap, good for small knn (~up to 30) and 2: kd-tree with tree heap, good for large knn (~from 30)", "1", "0", "2", &P::Comp<unsigned>},
				{"maxDistField", "descriptor field name used to set a maximum distance to consider for neighbors per point", "maxSearchDist"}
			};
		}
		
		const int knn;
		const T epsilon;
		const NNSearchType searchType;
		const std::string maxDistField;

	protected:
		std::shared_ptr<NNS> featureNNS;

	public:
		KDTreeVarDistMatcher(const Parameters& params = Parameters());
		virtual ~KDTreeVarDistMatcher();
		virtual void init(const DataPoints& filteredReference);
		virtual Matches findClosests(const DataPoints& filteredReading);
	};

}; // MatchersImpl

#endif // __POINTMATCHER_MATCHERS_H
