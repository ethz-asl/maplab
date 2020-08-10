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
#include "SimpleSensorNoise.h"

#include "PointMatcherPrivate.h"

#include <string>
#include <vector>

#include <boost/format.hpp>

// SimpleSensorNoiseDataPointsFilter
// Constructor
template<typename T>
SimpleSensorNoiseDataPointsFilter<T>::SimpleSensorNoiseDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("SimpleSensorNoiseDataPointsFilter",
	SimpleSensorNoiseDataPointsFilter::availableParameters(), params),
	sensorType(Parametrizable::get<unsigned>("sensorType")),
	gain(Parametrizable::get<T>("gain"))
{
	std::vector<std::string> sensorNames = {"Sick LMS-1xx",
											"Hokuyo URG-04LX",
											"Hokuyo UTM-30LX",
											"Kinect / Xtion","Sick Tim3xx"};
	if (sensorType >= sensorNames.size())
	{
		throw InvalidParameter(
			(boost::format("SimpleSensorNoiseDataPointsFilter: Error, sensorType id %1% does not exist.") % sensorType).str());
	}

	LOG_INFO_STREAM("SimpleSensorNoiseDataPointsFilter - using sensor noise model: " << sensorNames[sensorType]);
}


// SimpleSensorNoiseDataPointsFilter
// Compute
template<typename T>
typename PointMatcher<T>::DataPoints 
SimpleSensorNoiseDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void SimpleSensorNoiseDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	cloud.allocateDescriptor("simpleSensorNoise", 1);
	BOOST_AUTO(noise, cloud.getDescriptorViewByName("simpleSensorNoise"));

	switch(sensorType)
	{
	case 0: // Sick LMS-1xx
	{
		noise = computeLaserNoise(0.012, 0.0068, 0.0008, cloud.features);
		break;
	}
	case 1: // Hokuyo URG-04LX
	{
		noise = computeLaserNoise(0.028, 0.0013, 0.0001, cloud.features);
		break;
	}
	case 2: // Hokuyo UTM-30LX
	{
		noise = computeLaserNoise(0.018, 0.0006, 0.0015, cloud.features);
		break;
	}
	case 3: // Kinect / Xtion
	{
		const int dim = cloud.features.rows();
		const Matrix squaredValues(cloud.features.topRows(dim-1).colwise().norm().array().square());
		noise = squaredValues*(0.5*0.00285);
		break;
	}
  case 4: // Sick Tim3xx
  {
    noise = computeLaserNoise(0.004, 0.0053, -0.0092, cloud.features);
    break;
  }
	default:
		throw InvalidParameter(
			(boost::format("SimpleSensorNoiseDataPointsFilter: Error, cannot compute noise for sensorType id %1% .") % sensorType).str());
	}

}

template<typename T>
typename PointMatcher<T>::Matrix 
SimpleSensorNoiseDataPointsFilter<T>::computeLaserNoise(
	const T minRadius, const T beamAngle, const T beamConst, const Matrix& features)
{
	typedef typename Eigen::Array<T, 2, Eigen::Dynamic> Array2rows;

	const int nbPoints = features.cols();
	const int dim = features.rows();

	Array2rows evalNoise = Array2rows::Constant(2, nbPoints, minRadius);
	evalNoise.row(0) =  beamAngle * features.topRows(dim-1).colwise().norm();
	evalNoise.row(0) += beamConst;

	return evalNoise.colwise().maxCoeff();
}


template struct SimpleSensorNoiseDataPointsFilter<float>;
template struct SimpleSensorNoiseDataPointsFilter<double>;


