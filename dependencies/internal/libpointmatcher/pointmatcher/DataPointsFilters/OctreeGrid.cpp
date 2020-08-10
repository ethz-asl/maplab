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
#include "OctreeGrid.h"

#include <limits>

//Define Visitor classes to apply processing
template<typename T>
OctreeGridDataPointsFilter<T>::FirstPtsSampler::FirstPtsSampler(DataPoints& dp) 
	: idx{0}, pts(dp) 
{
}

template <typename T>
template<std::size_t dim>
bool OctreeGridDataPointsFilter<T>::FirstPtsSampler::operator()(Octree_<T,dim>& oc)
{
	if(oc.isLeaf() and not oc.isEmpty())
	{			
		auto* data = oc.getData();	
		const auto& d = (*data)[0];
		
		std::size_t j = d;
		
		//retrieve index from lookup table if sampling in already switched element
		if(std::size_t(d)<idx)
			j = mapidx[d];
			
		//Switch columns j and idx
		pts.swapCols(idx, j);
				
		//Maintain new index position	
		mapidx[idx] = j;
		//Update index
		++idx;		
	}
	
	return true;
}
 
template <typename T>
bool OctreeGridDataPointsFilter<T>::FirstPtsSampler::finalize()
{
	//Resize point cloud
	pts.conservativeResize(idx);
	//Reset param
	idx=0;
	return true;
}


template<typename T>
OctreeGridDataPointsFilter<T>::RandomPtsSampler::RandomPtsSampler(DataPoints& dp) 
	: OctreeGridDataPointsFilter<T>::FirstPtsSampler{dp}, seed{1}
{
	std::srand(seed);
}
template<typename T>
OctreeGridDataPointsFilter<T>::RandomPtsSampler::RandomPtsSampler(
	DataPoints& dp, const std::size_t seed_
): OctreeGridDataPointsFilter<T>::FirstPtsSampler{dp}, seed{seed_}
{
	std::srand(seed);
}
template<typename T>
template<std::size_t dim>
bool OctreeGridDataPointsFilter<T>::RandomPtsSampler::operator()(Octree_<T,dim>& oc)
{
	if(oc.isLeaf() and not oc.isEmpty())
	{			
		auto* data = oc.getData();
		const std::size_t nbData = (*data).size() - 1;
		const std::size_t randId = 
			static_cast<std::size_t>( nbData * 
				(static_cast<float>(std::rand()/static_cast<float>(RAND_MAX))));
				
		const auto& d = (*data)[randId];
		
		std::size_t j = d;
		
		//retrieve index from lookup table if sampling in already switched element
		if(std::size_t(d)<idx)
			j = mapidx[d];
			
		//Switch columns j and idx
		pts.swapCols(idx, j);	
		
		//Maintain new index position	
		mapidx[idx] = j;
		//Update index
		++idx;		
	}
	
	return true;
}
	
template<typename T>
bool OctreeGridDataPointsFilter<T>::RandomPtsSampler::finalize()
{
	bool ret = FirstPtsSampler::finalize();
	//Reset seed
	std::srand(seed);
	
	return ret;			
}

template<typename T>
OctreeGridDataPointsFilter<T>::CentroidSampler::CentroidSampler(DataPoints& dp)  
	: OctreeGridDataPointsFilter<T>::FirstPtsSampler{dp}
{
}
	
template<typename T>
template<std::size_t dim>
bool OctreeGridDataPointsFilter<T>::CentroidSampler::operator()(Octree_<T,dim>& oc)
{
	if(oc.isLeaf() and not oc.isEmpty())
	{			
		const int featDim(pts.features.rows());
		const int descDim(pts.descriptors.rows());
		const int timeDim(pts.times.rows());
		
		auto* data = oc.getData();
		const std::size_t nbData = (*data).size();
			
		const auto& d = (*data)[0]; //get first data
		std::size_t j = d; //j contains real index of first point
		
		//retrieve index from lookup table if sampling in already switched element
		if(std::size_t(d)<idx)
			j = mapidx[d];
		
		//We sum all the data in the first data
		for(std::size_t id=1;id<nbData;++id)
		{
			//get current idx
			const auto& curId = (*data)[id];
			std::size_t i = curId; //i contains real index
			
			//retrieve index from lookup table if sampling in already switched element
			if(std::size_t(curId)<idx)
				i = mapidx[curId];
			
			for (int f = 0; f < (featDim - 1); ++f)
				pts.features(f,j) += pts.features(f,i);
			
			if (pts.descriptors.cols() > 0)
				for (int d = 0; d < descDim; ++d)
					pts.descriptors(d,j) += pts.descriptors(d,i);
			
			if (pts.times.cols() > 0)
				for (int t = 0; t < timeDim; ++t)
					pts.times(t,j) += pts.times(t,i);	
		}
		
		// Normalize sums to get centroid (average)
		for (int f = 0; f < (featDim - 1); ++f)
			pts.features(f,j) /= T(nbData);
		
		if (pts.descriptors.cols() > 0)
			for (int d = 0; d < descDim; ++d)
				pts.descriptors(d,j) /= T(nbData);
		
		if (pts.times.cols() > 0)
			for (int t = 0; t < timeDim; ++t)
				pts.times(t,j) /= T(nbData);	
								
		//Switch columns j and idx
		pts.swapCols(idx, j);
		
		//Maintain new index position	
		mapidx[idx] = j;
		//Update index
		++idx;		
	}
	
	return true;
}
template<typename T>
OctreeGridDataPointsFilter<T>::MedoidSampler::MedoidSampler(DataPoints& dp)  
	: OctreeGridDataPointsFilter<T>::FirstPtsSampler{dp}
{
}
	
template<typename T>
template<std::size_t dim>
bool OctreeGridDataPointsFilter<T>::MedoidSampler::operator()(Octree_<T,dim>& oc)
{
	if(oc.isLeaf() and not oc.isEmpty())
	{		
		auto* data = oc.getData();
		const std::size_t nbData = (*data).size();

		auto dist = [](const typename Octree_<T,dim>::Point& p1, const typename Octree_<T,dim>::Point& p2) -> T {		
				return (p1 - p2).norm();		
			};
			
		//Build centroid
		typename Octree_<T,dim>::Point center;
		for(std::size_t i=0;i<dim;++i) center(i)=T(0.);
		
		for(std::size_t id=0;id<nbData;++id)
		{
			//get current idx
			const auto& curId = (*data)[id];
			std::size_t i = curId; //i contains real index
			
			//retrieve index from lookup table if sampling in already switched element
			if(std::size_t(curId)<idx)
				i = mapidx[curId];
			
			for (std::size_t f = 0; f < dim; ++f)
				center(f) += pts.features(f,i);	
		}
		for(std::size_t i=0;i<dim;++i) center(i)/=T(nbData);
		
		//Get the closest point from the center 
		T minDist = std::numeric_limits<T>::max();
		std::size_t medId = 0;
			
		for(std::size_t id=0;id<nbData;++id)
		{
			//get current idx
			const auto curId = (*data)[id];
			std::size_t i = curId; //i contains real index
			
			//retrieve index from lookup table if sampling in already switched element
			if(std::size_t(curId)<idx)
				i = mapidx[curId];
				
			const T curDist = dist(pts.features.col(i).head(dim), center);
			if(curDist<minDist)
			{
				minDist = curDist;
				medId=i;
			}
		}
				
		//Switch columns j and idx
		pts.swapCols(idx, medId);
	
		//Maintain new index position	
		mapidx[idx] = medId;
	
		//Update index
		++idx;		
	}

	return true;
}

// OctreeGridDataPointsFilter
template <typename T>
OctreeGridDataPointsFilter<T>::OctreeGridDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("OctreeGridDataPointsFilter", 
		OctreeGridDataPointsFilter::availableParameters(), params),
	buildParallel{Parametrizable::get<bool>("buildParallel")},
	maxPointByNode{Parametrizable::get<std::size_t>("maxPointByNode")},
	maxSizeByNode{Parametrizable::get<T>("maxSizeByNode")}
{
	try 
	{
		const int sm = this->template get<int>("samplingMethod");
		samplingMethod = SamplingMethod(sm);
	}
	catch (const InvalidParameter& e) 
	{
		samplingMethod = SamplingMethod::FIRST_PTS;
	}
}

template <typename T>
typename PointMatcher<T>::DataPoints
OctreeGridDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void OctreeGridDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	const std::size_t featDim = cloud.features.rows();
	
	assert(featDim == 4 or featDim == 3);

	if(featDim==3) //2D case
		this->sample<2>(cloud);
		
	else if(featDim==4) //3D case
		this->sample<3>(cloud);
}

template<typename T>
template<std::size_t dim>
void OctreeGridDataPointsFilter<T>::sample(DataPoints& cloud)
{
	Octree_<T,dim> oc;
	
	oc.build(cloud, maxPointByNode, maxSizeByNode, buildParallel);
	
	switch(samplingMethod)
	{
		case SamplingMethod::FIRST_PTS:
		{
			FirstPtsSampler sampler(cloud);
			oc.visit(sampler);
			sampler.finalize();
			break;
		}
		case SamplingMethod::RAND_PTS:
		{
			RandomPtsSampler sampler(cloud); //FIXME: add seed parameter
			oc.visit(sampler);
			sampler.finalize();
			break;
		}
		case SamplingMethod::CENTROID:
		{
			CentroidSampler sampler(cloud);
			oc.visit(sampler);
			sampler.finalize();
			break;
		}
		case SamplingMethod::MEDOID:
		{
			MedoidSampler sampler(cloud);
			oc.visit(sampler);
			sampler.finalize();
			break;
		}
	}
}


template struct OctreeGridDataPointsFilter<float>;
template struct OctreeGridDataPointsFilter<double>;
