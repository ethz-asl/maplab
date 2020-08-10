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
PointMatcher<T>::TransformationChecker::TransformationChecker()
{} 

//! Construct with parameters
template<typename T>
PointMatcher<T>::TransformationChecker::TransformationChecker(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	Parametrizable(className,paramsDoc,params)
{}

//! Destructor
template<typename T>
PointMatcher<T>::TransformationChecker::~TransformationChecker()
{} 

//! Return the value of limits involved in conditions to stop ICP loop
template<typename T>
const typename PointMatcher<T>::Vector& PointMatcher<T>::TransformationChecker::getLimits() const
{
	return limits;
}

//! Return the values of variables involved in conditions to stop ICP loop
template<typename T>
const typename PointMatcher<T>::Vector& PointMatcher<T>::TransformationChecker::getConditionVariables() const
{
	return conditionVariables;
}

//! Return the names of limits involved in conditions to stop ICP loop
template<typename T>
const typename PointMatcher<T>::TransformationChecker::StringVector& PointMatcher<T>::TransformationChecker::getLimitNames() const
{
	return limitNames;
}

//! Return the names of variables involved in conditions to stop ICP loop
template<typename T>
const typename PointMatcher<T>::TransformationChecker::StringVector& PointMatcher<T>::TransformationChecker::getConditionVariableNames() const
{
	return conditionVariableNames;
}

//! Extract the Euler angles from a rigid-transformation matrix 
template<typename T>
typename PointMatcher<T>::Vector PointMatcher<T>::TransformationChecker::matrixToAngles(const TransformationParameters& parameters)
{
	Vector angles;
	if(parameters.rows() == 4)
	{
		angles = Vector::Zero(3);

		angles(0) = atan2(parameters(2,0), parameters(2,1));
		angles(1) = acos(parameters(2,2));
		angles(2) = -atan2(parameters(0,2), parameters(1,2));
	}
	else
	{
		angles = Vector::Zero(1);

		angles(0) = acos(parameters(0,0));
	}

	return angles;
}

template struct PointMatcher<float>::TransformationChecker;
template struct PointMatcher<double>::TransformationChecker;


//! Init all transformation checkers, set iterate to false if iteration should stop
template<typename T>
void PointMatcher<T>::TransformationCheckers::init(const TransformationParameters& parameters, bool& iterate)
{
	for (TransformationCheckersIt it = this->begin(); it != this->end(); ++it)
		(*it)->init(parameters, iterate);
}

//! Check using all transformation checkers, set iterate to false if iteration should stop
template<typename T>
void PointMatcher<T>::TransformationCheckers::check(const TransformationParameters& parameters, bool& iterate)
{
	for (TransformationCheckersIt it = this->begin(); it != this->end(); ++it)
		(*it)->check(parameters, iterate);
}

template struct PointMatcher<float>::TransformationCheckers;
template struct PointMatcher<double>::TransformationCheckers;
