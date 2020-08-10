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

#ifndef __POINTMATCHER_TRANSFORMATIONCHECKERS_H
#define __POINTMATCHER_TRANSFORMATIONCHECKERS_H

#include "PointMatcher.h"

template<typename T>
struct TransformationCheckersImpl
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	
	typedef typename PointMatcher<T>::TransformationChecker TransformationChecker;
	typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::VectorVector VectorVector;
	typedef typename PointMatcher<T>::Quaternion Quaternion;
	typedef typename PointMatcher<T>::QuaternionVector QuaternionVector;
	typedef typename PointMatcher<T>::Matrix Matrix;
	
	struct CounterTransformationChecker: public TransformationChecker
	{
		//! Struct used to inform through an exeption that ICP reached max number of iterations
		struct MaxNumIterationsReached {};

		inline static const std::string description()
		{
			return "This checker stops the ICP loop after a certain number of iterations.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return {
				{"maxIterationCount", "maximum number of iterations ", "40", "0", "2147483647", &P::Comp<unsigned>}
			};
		}
		
		const unsigned maxIterationCount;
		
		CounterTransformationChecker(const Parameters& params = Parameters());
		virtual void init(const TransformationParameters& parameters, bool& iterate);
		virtual void check(const TransformationParameters& parameters, bool& iterate);
	};

	struct DifferentialTransformationChecker: public TransformationChecker
	{
		inline static const std::string description()
		{
			return "This checker stops the ICP loop when the relative motions (i.e. abs(currentIter - lastIter)) of rotation and translation components are below a fix thresholds. This allows to stop the iteration when the point cloud is stabilized. Smoothing can be applied to avoid oscillations. Inspired by \\cite{Chetverikov2002Trimmed}.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return {
				{"minDiffRotErr", "threshold for rotation error (radian)", "0.001", "0.", "6.2831854", &P::Comp<T>},
				{"minDiffTransErr", "threshold for translation error", "0.001", "0.", "inf", &P::Comp<T>},
				{"smoothLength", "number of iterations over which to average the differencial error", "3", "0", "2147483647", &P::Comp<unsigned>}
			};
		}
		
		const T minDiffRotErr;
		const T minDiffTransErr;
		const unsigned int smoothLength;

	protected:
		QuaternionVector rotations;
		VectorVector translations;

	public:
		DifferentialTransformationChecker(const Parameters& params = Parameters());
		
		virtual void init(const TransformationParameters& parameters, bool& iterate);
		virtual void check(const TransformationParameters& parameters, bool& iterate);
	};

	struct BoundTransformationChecker: public TransformationChecker
	{
		inline static const std::string description()
		{
			return "This checker stops the ICP loop with an exception when the transformation values exceed bounds.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return {
				{"maxRotationNorm",    "rotation bound",    "1", "0", "inf", &P::Comp < T > },
				{"maxTranslationNorm", "translation bound", "1", "0", "inf", &P::Comp < T > }
			};
		}
			
		const T maxRotationNorm;
		const T maxTranslationNorm;
		
	protected:
		Quaternion initialRotation3D;
		T initialRotation2D;
		Vector initialTranslation;
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		BoundTransformationChecker(const Parameters& params = Parameters());
		virtual void init(const TransformationParameters& parameters, bool& iterate);
		virtual void check(const TransformationParameters& parameters, bool& iterate);
	};
}; // TransformationCheckersImpl

#endif // __POINTMATCHER_TRANSFORMATIONCHECKERS_H
