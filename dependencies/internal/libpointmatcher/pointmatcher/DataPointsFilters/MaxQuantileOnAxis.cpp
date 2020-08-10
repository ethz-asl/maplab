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
#include "MaxQuantileOnAxis.h"

#include <algorithm>
#include <vector>

// MaxQuantileOnAxisDataPointsFilter
// Constructor
template<typename T>
MaxQuantileOnAxisDataPointsFilter<T>::MaxQuantileOnAxisDataPointsFilter(
	const Parameters& params): 
	PointMatcher<T>::DataPointsFilter("MaxQuantileOnAxisDataPointsFilter",
		MaxQuantileOnAxisDataPointsFilter::availableParameters(), params),
	dim(Parametrizable::get<unsigned>("dim")),
	ratio(Parametrizable::get<T>("ratio"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
MaxQuantileOnAxisDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void MaxQuantileOnAxisDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	if (int(dim) >= cloud.features.rows())
		throw InvalidParameter((boost::format("MaxQuantileOnAxisDataPointsFilter: Error, filtering on dimension number %1%, larger than feature dimensionality %2%") % dim % cloud.features.rows()).str());

	const int nbPointsIn = cloud.features.cols();
	const int nbPointsOut = nbPointsIn * ratio;

	// build array
	std::vector<T> values;
	values.reserve(nbPointsIn);
	for (int x = 0; x < nbPointsIn; ++x)
		values.push_back(cloud.features(dim, x));

	// get quartiles value
	std::nth_element(values.begin(), values.begin() + (values.size() * ratio), values.end());
	const T limit = values[nbPointsOut];

	// copy towards beginning the elements we keep
	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		if (cloud.features(dim, i) < limit)
		{
			assert(j <= i);
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}
	assert(j <= nbPointsOut);

	cloud.conservativeResize(j);

}

template struct MaxQuantileOnAxisDataPointsFilter<float>;
template struct MaxQuantileOnAxisDataPointsFilter<double>;


