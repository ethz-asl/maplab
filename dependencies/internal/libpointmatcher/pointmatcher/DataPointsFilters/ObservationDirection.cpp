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
#include "ObservationDirection.h"

// ObservationDirectionDataPointsFilter
// Constructor
template<typename T>
ObservationDirectionDataPointsFilter<T>::ObservationDirectionDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("ObservationDirectionDataPointsFilter", 
		ObservationDirectionDataPointsFilter::availableParameters(), params),
	centerX(Parametrizable::get<T>("x")),
	centerY(Parametrizable::get<T>("y")),
	centerZ(Parametrizable::get<T>("z"))
{
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints
ObservationDirectionDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void ObservationDirectionDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	const int dim(cloud.features.rows() - 1);
	const int featDim(cloud.features.cols());
	if (dim != 2 && dim != 3)
	{
		throw InvalidField(
			(boost::format("ObservationDirectionDataPointsFilter: Error, works only in 2 or 3 dimensions, cloud has %1% dimensions.") % dim).str()
		);
	}

	Vector center(dim);
	center[0] = centerX;
	center[1] = centerY;
	if (dim == 3)
		center[2] = centerZ;

	cloud.allocateDescriptor("observationDirections", dim);
	BOOST_AUTO(observationDirections, cloud.getDescriptorViewByName("observationDirections"));

	for (int i = 0; i < featDim; ++i)
	{
		// Check normal orientation
		const Vector p(cloud.features.block(0, i, dim, 1));
		observationDirections.col(i) = center - p;
	}

}

template struct ObservationDirectionDataPointsFilter<float>;
template struct ObservationDirectionDataPointsFilter<double>;

