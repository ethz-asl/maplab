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
struct CovarianceSamplingDataPointsFilter : public PointMatcher<T>::DataPointsFilter
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
	
	typedef typename DataPoints::Index Index;

	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::Matrix Matrix;	
	
	using Matrix66 = Eigen::Matrix<T, 6, 6>;
	using Vector6  = Eigen::Matrix<T, 6, 1>;
	using Vector3  = Eigen::Matrix<T, 3, 1>;

	inline static const std::string description()
	{
		return "Covariance Sampling (CovS) \\cite{Gelfand2003}. Performs stability analysis to select geometrically stable points that can bind the rotational components as well as the translational. Uses an estimate of the covariance matrix to detect pair of points which will not be constrained.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
			{"nbSample", "Number of point to select.", "5000", "1", "4294967295", &P::Comp<std::size_t>},
			{"torqueNorm", "Method for torque normalization: (0) L=1 (no normalization, more t-normals), (1) L=Lavg (average distance, torque is scale-independent), (2) L=Lmax (scale in unit ball, more r-normals)", "1", "0", "2", &P::Comp<std::uint8_t>}
		};
	}

	enum TorqueNormMethod : std::uint8_t { L1=0, Lavg=1, Lmax=2 };

	std::size_t nbSample;
	TorqueNormMethod normalizationMethod;
	
	//Ctor, uses parameter interface
	CovarianceSamplingDataPointsFilter(const Parameters& params = Parameters());
	//CovarianceSamplingDataPointsFilter();
	
	//Dtor
	virtual ~CovarianceSamplingDataPointsFilter() {};

	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);

	static T computeConditionNumber(const Matrix66 &cov);
};
	

