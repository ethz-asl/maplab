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

#include <vector>

//! Surface normals estimation. Find the normal for every point using eigen-decomposition of neighbour points
template<typename T>
struct SurfaceNormalDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::Matrix Matrix;	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;

	inline static const std::string description()
	{
		return "This filter extracts the surface normal vector and other statistics to each point by taking the eigenvector corresponding to the smallest eigenvalue of its nearest neighbors.\n\n"
		       "Required descriptors: none.\n"
		       "Produced descritors:  normals(optional), densities(optional), eigValues(optional), eigVectors(optional), matchedIds (optional), meanDists(optional).\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     none.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned>},
			{"maxDist", "maximum distance to consider for neighbors", "inf", "0", "inf", &P::Comp<T>},
			{"epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T>},
			{"keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1"},
			{"keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0"},
			{"keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", "0"},
			{"keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", "0"},
			{"keepMatchedIds" , "whether the identifiers of matches points should be added as descriptors to the resulting cloud", "0"},
			{"keepMeanDist" , "whether the distance to the nearest neighbor mean should be added as descriptors to the resulting cloud", "0"},
			{"sortEigen" , "whether the eigenvalues and eigenvectors should be sorted (ascending) based on the eigenvalues", "0"},
			{"smoothNormals", "whether the normal vector should be average with the nearest neighbors", "0"}
		};
	}
	
	const unsigned knn;
	const T maxDist;
	const T epsilon;
	const bool keepNormals;
	const bool keepDensities;
	const bool keepEigenValues;
	const bool keepEigenVectors;
	const bool keepMatchedIds;
	const bool keepMeanDist;
	const bool sortEigen;
	const bool smoothNormals;

	SurfaceNormalDataPointsFilter(const Parameters& params = Parameters());
	virtual ~SurfaceNormalDataPointsFilter() {};
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};
