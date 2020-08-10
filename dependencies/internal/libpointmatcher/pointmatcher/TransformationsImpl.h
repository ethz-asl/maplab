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

#ifndef __POINTMATCHER_TRANSFORMATIONS_H
#define __POINTMATCHER_TRANSFORMATIONS_H

#include "PointMatcher.h"

template<typename T>
struct TransformationsImpl
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
	typedef typename PointMatcher<T>::Transformation Transformation;
	
	struct RigidTransformation: public Transformation
	{
		inline static const std::string description()
		{
			return "Rigid transformation.";
		}

		RigidTransformation() : Transformation("RigidTransformation",  ParametersDoc(), Parameters()) {}
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const;
		virtual bool checkParameters(const TransformationParameters& parameters) const;
		virtual TransformationParameters correctParameters(const TransformationParameters& parameters) const;
	};

	struct SimilarityTransformation: public Transformation
	{
		inline static const std::string description()
		{
			return "Similarity transformation (rotation + translation + scale).";
		}
		
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const;
		virtual bool checkParameters(const TransformationParameters& parameters) const;
		virtual TransformationParameters correctParameters(const TransformationParameters& parameters) const;
	};

	struct PureTranslation : public Transformation
	{
		inline static const std::string description()
		{
			return "Pure translation transformation\nA rigid transformation with no rotation.";
		}

		PureTranslation() : Transformation("PureTranslation",  ParametersDoc(), Parameters()) {}
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const;
		virtual bool checkParameters(const TransformationParameters& parameters) const;
		virtual TransformationParameters correctParameters(const TransformationParameters& parameters) const;
	};

}; // TransformationsImpl

#endif // __POINTMATCHER_TRANSFORMATION_H
