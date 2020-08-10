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
#pragma once

#include "PointMatcher.h"
#include "utils/octree.h"

#include <unordered_map>

/*!
 * \class OctreeGridDataPointsFilter
 * \brief Data Filter based on Octree representation
 *
 * \author Mathieu Labussiere (<mathieu dot labu at gmail dot com>)
 * \date 24/05/2018
 * \version 0.1
 *
 * Processings are applyed via a Visitor through Depth-first search in the Octree (DFS)
 * i.e. for each node, the Visitor/Callback is called
 */
template<typename T>
struct OctreeGridDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
  	// Type definitions
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPointsFilter DataPointsFilter;

	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename DataPoints::Index Index;

	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

	inline static const std::string description()
	{
		return "Construct an Octree grid representation of the point cloud. Constructed either by limiting the number of point in each octant or by limiting the size of the bounding box. Down-sample by taking either the first or a random point, or compute the centroid.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
			{"buildParallel", "If 1 (true), use threads to build the octree.", "1", "0", "1", P::Comp<bool>},
			{"maxPointByNode", "Number of point under which the octree stop dividing.", "1", "1", "4294967295", &P::Comp<std::size_t>},
			{"maxSizeByNode", "Size of the bounding box under which the octree stop dividing.", "0", "0", "+inf", &P::Comp<T>},
			{"samplingMethod", "Method to sample the Octree: First Point (0), Random (1), Centroid (2) (more accurate but costly), Medoid (3) (more accurate but costly)", "0", "0", "3", &P::Comp<int>}
		//FIXME: add seed parameter for the random sampling
		};
	}

public:
//Visitors class to apply processing
	struct FirstPtsSampler
	{
		std::size_t idx;
		DataPoints&	pts;

		//Build map of (old index to new index), 
		// in case we sample pts at the begining of the pointcloud
		std::unordered_map<std::size_t, std::size_t> mapidx;

		FirstPtsSampler(DataPoints& dp);
		virtual ~FirstPtsSampler(){}
		
		template<std::size_t dim>
		bool operator()(Octree_<T,dim>& oc);
		
		virtual bool finalize();
	};
	struct RandomPtsSampler : public FirstPtsSampler
	{
		using FirstPtsSampler::idx;
		using FirstPtsSampler::pts;
		using FirstPtsSampler::mapidx;
		
		const std::size_t seed;
	
		RandomPtsSampler(DataPoints& dp);
		RandomPtsSampler(DataPoints& dp, const std::size_t seed_);
		virtual ~RandomPtsSampler(){}
	
		template<std::size_t dim>
		bool operator()(Octree_<T,dim>& oc);
		
		virtual bool finalize();
	};
	struct CentroidSampler : public FirstPtsSampler
	{
		using FirstPtsSampler::idx;
		using FirstPtsSampler::pts;
		using FirstPtsSampler::mapidx;
		
		CentroidSampler(DataPoints& dp);
	
		virtual ~CentroidSampler(){}
	
		template<std::size_t dim>
		bool operator()(Octree_<T,dim>& oc);
	};
	//Nearest point from the centroid (contained in the cloud)
	struct MedoidSampler : public FirstPtsSampler
	{
		using FirstPtsSampler::idx;
		using FirstPtsSampler::pts;
		using FirstPtsSampler::mapidx;
		
		MedoidSampler(DataPoints& dp);
	
		virtual ~MedoidSampler(){}
	
		template<std::size_t dim>
		bool operator()(Octree_<T,dim>& oc);		
	};

//-------	
	enum SamplingMethod : int { FIRST_PTS=0, RAND_PTS=1, CENTROID=2, MEDOID=3 };

//Atributes
	bool buildParallel;
	
	std::size_t maxPointByNode;
	T           maxSizeByNode;
	
	SamplingMethod samplingMethod;

//Methods	
	//Constructor, uses parameter interface
	OctreeGridDataPointsFilter(const Parameters& params = Parameters());

	OctreeGridDataPointsFilter();
	// Destr
	virtual ~OctreeGridDataPointsFilter() {};

	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);

private:
	template<std::size_t dim> void sample(DataPoints& cloud);
};
