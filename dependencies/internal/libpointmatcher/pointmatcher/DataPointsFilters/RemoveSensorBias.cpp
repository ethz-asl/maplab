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
#include "RemoveSensorBias.h"

#include "PointMatcherPrivate.h"

#include <string>
#include <vector>

#include <boost/format.hpp>
#include <ciso646>
#include <cmath>


template<typename T>
RemoveSensorBiasDataPointsFilter<T>::RemoveSensorBiasDataPointsFilter(const Parameters& params):
	PointMatcher<T>::DataPointsFilter("RemoveSensorBiasDataPointsFilter", 
		RemoveSensorBiasDataPointsFilter::availableParameters(), params),
	sensorType(SensorType(Parametrizable::get<int>("sensorType"))),
	angleThreshold(Parametrizable::get<T>("angleThreshold")/180.*M_PI)
{
}

template<typename T>
typename PointMatcher<T>::DataPoints 
RemoveSensorBiasDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template<typename T>
void RemoveSensorBiasDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	//Check if there is normals info
	if (!cloud.descriptorExists("incidenceAngles"))
		throw InvalidField("RemoveSensorBiasDataPointsFilter: Error, cannot find incidence angles in descriptors.");
	//Check if there is normals info
	if (!cloud.descriptorExists("observationDirections"))
		throw InvalidField("RemoveSensorBiasDataPointsFilter: Error, cannot find observationDirections in descriptors.");
		
	const auto& incidenceAngles = cloud.getDescriptorViewByName("incidenceAngles");
	const auto& observationDirections = cloud.getDescriptorViewByName("observationDirections");

	double aperture, k1, k2;

	switch(sensorType)
	{
		case LMS_1XX: 
		{
			aperture = SensorParameters::LMS_1XX.aperture;
			k1 = SensorParameters::LMS_1XX.k1;
			k2 = SensorParameters::LMS_1XX.k2;
			break;
		}
		case HDL_32E: 
		{
			aperture = SensorParameters::HDL_32E.aperture;
			k1 = SensorParameters::HDL_32E.k1;
			k2 = SensorParameters::HDL_32E.k2;
			break;
		}
		default:
			throw InvalidParameter(
				(boost::format("RemoveSensorBiasDataPointsFilter: Error, cannot remove bias for sensorType id %1% .") % sensorType).str());
	}

	const std::size_t nbPts = cloud.getNbPoints();
	const std::size_t dim = cloud.features.rows();
	

	assert(dim == 3 or dim == 4); //check 2D or 3D

	std::size_t j = 0;
	for(std::size_t i = 0; i < nbPts; ++i)
	{
		const Vector vObs = observationDirections.col(i);

		const double depth = vObs.norm();
		const T incidence = incidenceAngles(0, i);

		//check if the incidence angle could be estimated.
		//For angles very close to 90 degrees, a small error of estimation could change drastically the correction, so we skip those points.
		if(not std::isnan(incidence) and incidence >= 0. and incidence < angleThreshold)
		{
			const double correction = k1 * diffDist(depth, incidence, aperture) + k2 * ratioCurvature(depth, incidence, aperture);

			Vector p = cloud.features.col(i);
			p.head(dim-1) += correction * vObs.normalized(); 
			cloud.features.col(i) = p;
			cloud.setColFrom(j, cloud, i);
			++j;
		}		
	}
	cloud.conservativeResize(j);
}

template<typename T>
std::array<double, 4> RemoveSensorBiasDataPointsFilter<T>::getCoefficients(const double depth, const T theta, const double aperture) const
{
	const double sigma = tau / std::sqrt(2. * M_PI);
	const double w0 = lambda_light / (M_PI * aperture);

	const double A  = 2. * std::pow(depth * std::tan(theta), 2) / std::pow(sigma * c, 2) + 2. / std::pow(aperture, 2);
	const double K1 = std::pow(std::cos(theta), 3);
	const double K2 = 3. * std::pow(std::cos(theta), 2) * std::sin(theta);
	const double L1 = pulse_intensity * std::pow(w0 / (aperture * depth * std::cos(theta)), 2) *
		std::sqrt(M_PI) * std::erf(aperture * std::sqrt(A)) / (2. * std::pow(A, 3. / 2.));
	const double L2 = pulse_intensity * std::pow(w0 / (aperture * depth * std::cos(theta)), 2) * K2 / (2. * A);

	const double a0 = 2. * A * K1 * L1;
	const double a1 = -(2. * std::tan(theta) * depth * 
			(L1 * K2 - 2. * L2 * aperture * std::exp(-A * std::pow(aperture, 2)))) / (std::pow(sigma, 2) * c);
	const double a2 = -L1 * 2. * A * K1 * (std::pow(sigma * c * std::cos(theta), 2) * A + 2. * std::pow(std::cos(theta) * depth, 2) - 2. * std::pow(depth, 2)) / 
		(2 * std::pow(c * std::cos(theta), 2) * std::pow(sigma, 4) * A);
	const double a3 = L1 * K2 * depth * std::tan(theta) * (std::pow(sigma * c, 2) * A - 2. * std::pow(depth * std::tan(theta), 2)) / 
		(std::pow(sigma, 6) * std::pow(c, 3) * A);

	return {a0, a1, a2, a3};
}

template<typename T>
double RemoveSensorBiasDataPointsFilter<T>::diffDist(const double depth, const T theta, const double aperture) const
{
	static constexpr double epsilon = 1e-5;
	
	const std::array<double, 4> a = getCoefficients(depth, theta, aperture);

	double Tmax;

	if(theta < epsilon) // approx. 0
		Tmax = 0.;
	else // theta >0
		Tmax = (-2 * a[2] - std::sqrt(4 * pow(a[2], 2) - 12 * a[1] * a[3])) / (6 * a[3]);
	
	return Tmax * c / 2.;
}

template<typename T>
double RemoveSensorBiasDataPointsFilter<T>::ratioCurvature(const double depth, const T theta, const double aperture) const
{
	static constexpr double epsilon = 1e-5;
	const std::array<double,4> a = getCoefficients(depth, theta, aperture);
	const std::array<double,4> b = getCoefficients(depth, 0., aperture);

	double Tmax;

	if(theta < epsilon) // approx. 0
		Tmax = 0.;
	else // theta >0
		Tmax = ( -2 * a[2] - std::sqrt(4 * std::pow(a[2], 2) - 12 * a[1] * a[3])) / (6 * a[3]);
	
	return 1. - 2 * b[2] / (2 * a[2] + 6 * Tmax * a[3]);
}

template struct RemoveSensorBiasDataPointsFilter<float>;
template struct RemoveSensorBiasDataPointsFilter<double>;
