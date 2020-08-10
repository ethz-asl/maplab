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

#include "TransformationCheckersImpl.h"
#include "Functions.h"
#include <boost/math/special_functions/fpclassify.hpp> 

using namespace std;
using namespace PointMatcherSupport;

//--------------------------------------
// max iteration counter
template<typename T>
TransformationCheckersImpl<T>::CounterTransformationChecker::CounterTransformationChecker(const Parameters& params):
	TransformationChecker("CounterTransformationChecker", CounterTransformationChecker::availableParameters(), params),
	maxIterationCount(Parametrizable::get<unsigned>("maxIterationCount"))
{
	this->limits.setZero(1);
	this->limits(0) = maxIterationCount;

	this->conditionVariableNames.push_back("Iteration");
	this->limitNames.push_back("Max iteration");
}

template<typename T>
void TransformationCheckersImpl<T>::CounterTransformationChecker::init(const TransformationParameters& parameters, bool& iterate)
{
	this->conditionVariables.setZero(1);
}

template<typename T>
void TransformationCheckersImpl<T>::CounterTransformationChecker::check(const TransformationParameters& parameters, bool& iterate)
{
	this->conditionVariables(0)++;
	
	//std::cout << "Iter: " << this->conditionVariables(0) << " / " << this->limits(0) << std::endl;
	//cerr << parameters << endl;
	
	if (this->conditionVariables(0) >= this->limits(0))
	{
		iterate = false;
		throw MaxNumIterationsReached();
	}
}

template struct TransformationCheckersImpl<float>::CounterTransformationChecker;
template struct TransformationCheckersImpl<double>::CounterTransformationChecker;


//--------------------------------------
// error
template<typename T>
TransformationCheckersImpl<T>::DifferentialTransformationChecker::DifferentialTransformationChecker(const Parameters& params):
	TransformationChecker("DifferentialTransformationChecker", DifferentialTransformationChecker::availableParameters(), params),
	minDiffRotErr(Parametrizable::get<T>("minDiffRotErr")),
	minDiffTransErr(Parametrizable::get<T>("minDiffTransErr")),
	smoothLength(Parametrizable::get<unsigned>("smoothLength"))
{
	this->limits.setZero(2);
	this->limits(0) = minDiffRotErr;
	this->limits(1) = minDiffTransErr;
	
	this->conditionVariableNames.push_back("Mean abs differential rot err");
	this->conditionVariableNames.push_back("Mean abs differential trans err");
	this->limitNames.push_back("Min differential rotation err");
	this->limitNames.push_back("Min differential translation err");

}

template<typename T>
void TransformationCheckersImpl<T>::DifferentialTransformationChecker::init(const TransformationParameters& parameters, bool& iterate)
{
	this->conditionVariables.setZero(2);
	
	rotations.clear();
	translations.clear();
	
	if (parameters.rows() == 4)
	{
		rotations.push_back(Quaternion(Eigen::Matrix<T,3,3>(parameters.topLeftCorner(3,3))));
	}
	else
	{
		// Handle the 2D case
		Eigen::Matrix<T,3,3> m(Matrix::Identity(3,3));
		m.topLeftCorner(2,2) = parameters.topLeftCorner(2,2);
		rotations.push_back(Quaternion(m));
	}
	
	const unsigned int nbRows = parameters.rows()-1;
	translations.push_back(parameters.topRightCorner(nbRows,1));
}

template<typename T>
void TransformationCheckersImpl<T>::DifferentialTransformationChecker::check(const TransformationParameters& parameters, bool& iterate)
{
	typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;
	
	rotations.push_back(Quaternion(Eigen::Matrix<T,3,3>(parameters.topLeftCorner(3,3))));
	const unsigned int nbRows = parameters.rows()-1;
	translations.push_back(parameters.topRightCorner(nbRows,1));
	
	this->conditionVariables.setZero(2);
	if(rotations.size() > smoothLength)
	{
		for(size_t i = rotations.size()-1; i >= rotations.size()-smoothLength; i--)
		{
			//Compute the mean derivative
			this->conditionVariables(0) += anyabs(rotations[i].angularDistance(rotations[i-1]));
			this->conditionVariables(1) += anyabs((translations[i] - translations[i-1]).norm());
		}

		this->conditionVariables /= smoothLength;

		if(this->conditionVariables(0) < this->limits(0) && this->conditionVariables(1) < this->limits(1))
			iterate = false;
	}
	
	//std::cout << "Abs Rotation: " << this->conditionVariables(0) << " / " << this->limits(0) << std::endl;
	//std::cout << "Abs Translation: " << this->conditionVariables(1) << " / " << this->limits(1) << std::endl;
	
	if (boost::math::isnan(this->conditionVariables(0)))
		throw ConvergenceError("abs rotation norm not a number");
	if (boost::math::isnan(this->conditionVariables(1)))
		throw ConvergenceError("abs translation norm not a number");
}

template struct TransformationCheckersImpl<float>::DifferentialTransformationChecker;
template struct TransformationCheckersImpl<double>::DifferentialTransformationChecker;

//--------------------------------------
// bound

template<typename T>
TransformationCheckersImpl<T>::BoundTransformationChecker::BoundTransformationChecker(const Parameters& params):
	TransformationChecker("BoundTransformationChecker", BoundTransformationChecker::availableParameters(), params),
	maxRotationNorm(Parametrizable::get<T>("maxRotationNorm")),
	maxTranslationNorm(Parametrizable::get<T>("maxTranslationNorm"))
{
	this->limits.setZero(2);
	this->limits(0) = maxRotationNorm;
	this->limits(1) = maxTranslationNorm;

	this->limitNames.push_back("Max rotation angle");
	this->limitNames.push_back("Max translation norm");
	this->conditionVariableNames.push_back("Rotation angle");
	this->conditionVariableNames.push_back("Translation norm");
}

template<typename T>
void TransformationCheckersImpl<T>::BoundTransformationChecker::init(const TransformationParameters& parameters, bool& iterate)
{
	this->conditionVariables.setZero(2);
	if (parameters.rows() == 4)
		initialRotation3D = Quaternion(Eigen::Matrix<T,3,3>(parameters.topLeftCorner(3,3)));
	else if (parameters.rows() == 3)
		initialRotation2D = acos(parameters(0,0));
	else
		throw runtime_error("BoundTransformationChecker only works in 2D or 3D");
		
	const unsigned int nbRows = parameters.rows()-1;
	initialTranslation = parameters.topRightCorner(nbRows,1);
}

template<typename T>
void TransformationCheckersImpl<T>::BoundTransformationChecker::check(const TransformationParameters& parameters, bool& iterate)
{
	typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;
	
	if (parameters.rows() == 4)
	{
		const Quaternion currentRotation = Quaternion(Eigen::Matrix<T,3,3>(parameters.topLeftCorner(3,3)));
		this->conditionVariables(0) = currentRotation.angularDistance(initialRotation3D);
	}
	else if (parameters.rows() == 3)
	{
		const T currentRotation(acos(parameters(0,0)));
		this->conditionVariables(0) = normalizeAngle(currentRotation - initialRotation2D);
	}
	else
		assert(false);
	const unsigned int nbRows = parameters.rows()-1;
	const Vector currentTranslation = parameters.topRightCorner(nbRows,1);
	this->conditionVariables(1) = (currentTranslation - initialTranslation).norm();
	if (this->conditionVariables(0) > this->limits(0) || this->conditionVariables(1) > this->limits(1))
	{
		ostringstream oss;
		oss << "limit out of bounds: ";
		oss << "rot: " << this->conditionVariables(0) << "/" << this->limits(0) << " ";
		oss << "tr: " << this->conditionVariables(1) << "/" << this->limits(1);
		throw ConvergenceError(oss.str());
	}
}

template struct TransformationCheckersImpl<float>::BoundTransformationChecker;
template struct TransformationCheckersImpl<double>::BoundTransformationChecker;
