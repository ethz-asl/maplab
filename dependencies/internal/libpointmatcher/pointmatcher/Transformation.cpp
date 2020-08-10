// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

//! Construct without parameter
template<typename T>
PointMatcher<T>::Transformation::Transformation()
{}

//! Construct with parameters
template<typename T>
PointMatcher<T>::Transformation::Transformation(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	Parametrizable(className,paramsDoc,params)
{}

//! virtual destructor
template<typename T>
PointMatcher<T>::Transformation::~Transformation()
{}

template struct PointMatcher<float>::Transformation;
template struct PointMatcher<double>::Transformation;


//! Apply this chain to cloud, using parameters, mutates cloud
template<typename T>
void PointMatcher<T>::Transformations::apply(DataPoints& cloud, const TransformationParameters& parameters) const
{
	// There is no chain per se, the current API is very confusing. Normally,
	// if this object is initialized correctly, there is one single iteration
	// to be done, just the type of the transform changes. 
	// TODO: This API must be re-written to not even have the concept
	// of chain for this classs. 
	int num_iter = 0;

	DataPoints transformedCloud;
	for (TransformationsConstIt it = this->begin(); it != this->end(); ++it)
	{
		transformedCloud = (*it)->compute(cloud, parameters);
		swapDataPoints(cloud, transformedCloud);
		num_iter++;
	}
	if (num_iter != 1)
		throw std::runtime_error("Transformations: Error, the transform should have been applied just once.");
}

template struct PointMatcher<float>::Transformations;
template struct PointMatcher<double>::Transformations;
