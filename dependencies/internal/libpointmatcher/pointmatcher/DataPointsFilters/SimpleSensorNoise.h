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

//! Sick LMS-xxx noise model
template<typename T>
struct SimpleSensorNoiseDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;

	typedef typename PointMatcher<T>::Matrix Matrix;	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	inline static const std::string description()
	{
		return "Add a 1D descriptor named <sensorNoise> that would represent the noise radius expressed in meter based on SICK LMS specifications \\cite{Pomerleau2012Noise}.";
	}
	
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"sensorType", "Type of the sensor used. Choices: 0=Sick LMS-1xx, 1=Hokuyo URG-04LX, 2=Hokuyo UTM-30LX, 3=Kinect/Xtion", "0", "0", "2147483647", &P::Comp<unsigned>},
			{"gain", "If the point cloud is coming from an untrusty source, you can use the gain to augment the uncertainty", "1", "1", "inf", &P::Comp<T>}
		};
	}

	const unsigned sensorType;
	const T gain;
	
	//! Constructor, uses parameter interface
	SimpleSensorNoiseDataPointsFilter(const Parameters& params = Parameters());
	
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);

private:
	/// @param minRadius in meter, noise level of depth measurements
	/// @param beamAngle in rad, half of the total laser beam
	/// @param beamConst in meter, minimum size of the laser beam
	/// @param features points from the sensor
	Matrix computeLaserNoise(const T minRadius, const T beamAngle, const T beamConst, const Matrix& features);

};
