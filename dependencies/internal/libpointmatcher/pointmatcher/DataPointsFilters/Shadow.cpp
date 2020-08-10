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
#include "Shadow.h"

#include "Functions.h"

// ShadowDataPointsFilter
// Constructor
template<typename T>
ShadowDataPointsFilter<T>::ShadowDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("ShadowDataPointsFilter", 
		ShadowDataPointsFilter::availableParameters(), params),
	eps(sin(Parametrizable::get<T>("eps")))
{
	//waring: maxAngle is change to sin(maxAngle)!
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
ShadowDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void ShadowDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	using namespace PointMatcherSupport;
	
	// Check if normals are present
	if (!cloud.descriptorExists("normals"))
	{
		throw InvalidField("ShadowDataPointsFilter, Error: cannot find normals in descriptors");
	}

	const int dim(cloud.features.rows());
	const int featDim(cloud.features.cols());

	const BOOST_AUTO(normals, cloud.getDescriptorViewByName("normals"));
	int j = 0;

	for(int i=0; i < featDim; ++i)
	{
		const Vector normal = normals.col(i).normalized();
		const Vector point = cloud.features.block(0, i, dim-1, 1).normalized();

		const T value = anyabs(normal.dot(point));

		if(value > eps) // test to keep the points
		{
			cloud.features.col(j) = cloud.features.col(i);
			cloud.descriptors.col(j) = cloud.descriptors.col(i);
			++j;
		}
	}

	cloud.conservativeResize(j);
}

template struct ShadowDataPointsFilter<float>;
template struct ShadowDataPointsFilter<double>;

