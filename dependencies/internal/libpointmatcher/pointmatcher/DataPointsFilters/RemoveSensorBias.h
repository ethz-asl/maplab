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
#include <array>

template<typename T>
struct RemoveSensorBiasDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPointsFilter DataPointsFilter;
	typedef typename PM::Matrix Matrix;
	typedef typename PM::Vector Vector;
	
	typedef typename DataPoints::InvalidField InvalidField;
	
	inline static const std::string description()
	{
		return "Remove the bias induced by the angle of incidence\n\n"
			   "Required descriptors: incidenceAngles, observationDirections.\n"
		       "Produced descritors:  none.\n"
			   "Altered descriptors:  none.\n"
			   "Altered features:     points coordinates and number of points.";
	}
	
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"sensorType", "Type of the sensor used. Choices: 0=Sick LMS-1xx, 1=Velodyne HDL-32E", "0", "0", "1", &P::Comp < int >},
			{"angleThreshold", "Threshold at which angle the correction is not applied, in degrees", "88.", "0.", "90.", &P::Comp < T >}
		};
	}
	
	RemoveSensorBiasDataPointsFilter(const Parameters& params = Parameters());
	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);

private:
	enum SensorType : int { LMS_1XX=0, HDL_32E=1 };
	// doubles are used since large and tiny numbers are handled in this filter
	struct SensorParameters{
	private:
		SensorParameters(double aperture_, double k1_, double k2_): aperture(aperture_), k1(k1_), k2(k2_) {}
	public:
		const double aperture;
		const double k1;
		const double k2;
		
		static const SensorParameters LMS_1XX;
		static const SensorParameters HDL_32E;
	};
	
	static constexpr double tau = 50e-9; //s - pulse length
	static constexpr double pulse_intensity = 0.39; //w.m^-2 - pulse intensity
	static constexpr double lambda_light = 905e-9; //m - wavelength of the laser
	static constexpr double c = 299792458.0; //m.s^-1 - celerity of light
	
	const SensorType sensorType;
	const T angleThreshold;

	std::array<double,4> getCoefficients(const double depth, const T theta, const double aperture) const;
	double diffDist(const double depth, const T theta, const double aperture) const;
	double ratioCurvature(const double depth, const T theta, const double aperture) const;
};

template<typename T>
const typename RemoveSensorBiasDataPointsFilter<T>::SensorParameters RemoveSensorBiasDataPointsFilter<T>::SensorParameters::LMS_1XX =
	RemoveSensorBiasDataPointsFilter<T>::SensorParameters(0.0075049, 6.08040951e+00, 3.17921789e-03 );

template<typename T>
const typename RemoveSensorBiasDataPointsFilter<T>::SensorParameters RemoveSensorBiasDataPointsFilter<T>::SensorParameters::HDL_32E =
	RemoveSensorBiasDataPointsFilter<T>::SensorParameters(0.0014835, 1.03211569e+01, 7.07893371e-03);


