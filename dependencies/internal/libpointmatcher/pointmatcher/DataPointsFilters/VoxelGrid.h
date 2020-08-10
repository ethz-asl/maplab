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

template<typename T>
struct VoxelGridDataPointsFilter : public PointMatcher<T>::DataPointsFilter
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

	typedef typename PointMatcher<T>::Matrix Matrix;
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename Eigen::Matrix<T,2,1> Vector2;
	typedef typename Eigen::Matrix<T,3,1> Vector3;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;


	inline static const std::string description()
	{
		return "Construct Voxel grid of the point cloud. Down-sample by taking centroid or center of grid cells.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
			{"vSizeX", "Dimension of each voxel cell in x direction", "1.0", "0.001", "+inf", &P::Comp<T>},
			{"vSizeY", "Dimension of each voxel cell in y direction", "1.0", "0.001", "+inf", &P::Comp<T>},
			{"vSizeZ", "Dimension of each voxel cell in z direction", "1.0", "0.001", "+inf", &P::Comp<T>},
			{"useCentroid", "If 1 (true), down-sample by using centroid of voxel cell.  If false (0), use center of voxel cell.", "1", "0", "1", P::Comp<bool>},
			{"averageExistingDescriptors", "whether the filter keep the existing point descriptors and average them or should it drop them", "1", "0", "1", P::Comp<bool>}
		};
	}

	const T vSizeX;
	const T vSizeY;
	const T vSizeZ;
	const bool useCentroid;
	const bool averageExistingDescriptors;

	struct Voxel {
		unsigned int    numPoints;
		unsigned int    firstPoint;
		Voxel() : numPoints(0), firstPoint(0) {}
	};

	//Constructor, uses parameter interface
	VoxelGridDataPointsFilter(const Parameters& params = Parameters());

	VoxelGridDataPointsFilter();
  // Destr
	virtual ~VoxelGridDataPointsFilter() {};

	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};
