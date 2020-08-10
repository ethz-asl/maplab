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

#include "LoggerImpl.h"
#include "TransformationsImpl.h"
#include "DataPointsFiltersImpl.h"
#include "MatchersImpl.h"
#include "OutlierFiltersImpl.h"
#include "ErrorMinimizersImpl.h"
#include "TransformationCheckersImpl.h"
#include "InspectorsImpl.h"

#include <cassert>
#include <iostream>
#include <limits>

using namespace std;
using namespace PointMatcherSupport;

//! Construct an invalid-element exception
InvalidElement::InvalidElement(const std::string& reason):
	runtime_error(reason)
{}

//! Constructor, populates the registrars
template<typename T>
PointMatcher<T>::PointMatcher()
{
	ADD_TO_REGISTRAR_NO_PARAM(Transformation, RigidTransformation, typename TransformationsImpl<T>::RigidTransformation)
	ADD_TO_REGISTRAR_NO_PARAM(Transformation, PureTranslation, typename TransformationsImpl<T>::PureTranslation)
	
	ADD_TO_REGISTRAR_NO_PARAM(DataPointsFilter, IdentityDataPointsFilter, typename DataPointsFiltersImpl<T>::IdentityDataPointsFilter)
	ADD_TO_REGISTRAR_NO_PARAM(DataPointsFilter, RemoveNaNDataPointsFilter, typename DataPointsFiltersImpl<T>::RemoveNaNDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, MaxDistDataPointsFilter, typename DataPointsFiltersImpl<T>::MaxDistDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, MinDistDataPointsFilter, typename DataPointsFiltersImpl<T>::MinDistDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, BoundingBoxDataPointsFilter, typename DataPointsFiltersImpl<T>::BoundingBoxDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, MaxQuantileOnAxisDataPointsFilter, typename DataPointsFiltersImpl<T>::MaxQuantileOnAxisDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, MaxDensityDataPointsFilter, typename DataPointsFiltersImpl<T>::MaxDensityDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, SurfaceNormalDataPointsFilter, typename DataPointsFiltersImpl<T>::SurfaceNormalDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, SamplingSurfaceNormalDataPointsFilter, typename DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, OrientNormalsDataPointsFilter, typename DataPointsFiltersImpl<T>::OrientNormalsDataPointsFilter)
	ADD_TO_REGISTRAR_NO_PARAM(DataPointsFilter, IncidenceAngleDataPointsFilter, typename DataPointsFiltersImpl<T>::IncidenceAngleDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, RandomSamplingDataPointsFilter, typename DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, MaxPointCountDataPointsFilter, typename DataPointsFiltersImpl<T>::MaxPointCountDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, FixStepSamplingDataPointsFilter, typename DataPointsFiltersImpl<T>::FixStepSamplingDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, ShadowDataPointsFilter, typename DataPointsFiltersImpl<T>::ShadowDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, SimpleSensorNoiseDataPointsFilter, typename DataPointsFiltersImpl<T>::SimpleSensorNoiseDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, ObservationDirectionDataPointsFilter, typename DataPointsFiltersImpl<T>::ObservationDirectionDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, VoxelGridDataPointsFilter, typename DataPointsFiltersImpl<T>::VoxelGridDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, CutAtDescriptorThresholdDataPointsFilter, typename DataPointsFiltersImpl<T>::CutAtDescriptorThresholdDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, ElipsoidsDataPointsFilter, typename DataPointsFiltersImpl<T>::ElipsoidsDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, GestaltDataPointsFilter, typename DataPointsFiltersImpl<T>::GestaltDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, OctreeGridDataPointsFilter, typename DataPointsFiltersImpl<T>::OctreeGridDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, NormalSpaceDataPointsFilter, typename DataPointsFiltersImpl<T>::NormalSpaceDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, CovarianceSamplingDataPointsFilter, typename DataPointsFiltersImpl<T>::CovarianceSamplingDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, DistanceLimitDataPointsFilter, typename DataPointsFiltersImpl<T>::DistanceLimitDataPointsFilter)
	ADD_TO_REGISTRAR(DataPointsFilter, RemoveSensorBiasDataPointsFilter, typename DataPointsFiltersImpl<T>::RemoveSensorBiasDataPointsFilter)
	
	ADD_TO_REGISTRAR_NO_PARAM(Matcher, NullMatcher, typename MatchersImpl<T>::NullMatcher)
	ADD_TO_REGISTRAR(Matcher, KDTreeMatcher, typename MatchersImpl<T>::KDTreeMatcher)
	ADD_TO_REGISTRAR(Matcher, KDTreeVarDistMatcher, typename MatchersImpl<T>::KDTreeVarDistMatcher)
	
	ADD_TO_REGISTRAR_NO_PARAM(OutlierFilter, NullOutlierFilter, typename OutlierFiltersImpl<T>::NullOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, MaxDistOutlierFilter, typename OutlierFiltersImpl<T>::MaxDistOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, MinDistOutlierFilter, typename OutlierFiltersImpl<T>::MinDistOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, MedianDistOutlierFilter, typename OutlierFiltersImpl<T>::MedianDistOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, TrimmedDistOutlierFilter, typename OutlierFiltersImpl<T>::TrimmedDistOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, VarTrimmedDistOutlierFilter, typename OutlierFiltersImpl<T>::VarTrimmedDistOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, SurfaceNormalOutlierFilter, typename OutlierFiltersImpl<T>::SurfaceNormalOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, GenericDescriptorOutlierFilter, typename OutlierFiltersImpl<T>::GenericDescriptorOutlierFilter)
	ADD_TO_REGISTRAR(OutlierFilter, RobustOutlierFilter, typename OutlierFiltersImpl<T>::RobustOutlierFilter)
	
	ADD_TO_REGISTRAR_NO_PARAM(ErrorMinimizer, IdentityErrorMinimizer, typename ErrorMinimizersImpl<T>::IdentityErrorMinimizer)
	ADD_TO_REGISTRAR_NO_PARAM(ErrorMinimizer, PointToPointErrorMinimizer, typename ErrorMinimizersImpl<T>::PointToPointErrorMinimizer)
	ADD_TO_REGISTRAR_NO_PARAM(ErrorMinimizer, PointToPointSimilarityErrorMinimizer, typename ErrorMinimizersImpl<T>::PointToPointSimilarityErrorMinimizer)
	ADD_TO_REGISTRAR(ErrorMinimizer, PointToPlaneErrorMinimizer, typename ErrorMinimizersImpl<T>::PointToPlaneErrorMinimizer)
	ADD_TO_REGISTRAR(ErrorMinimizer, PointToPointWithCovErrorMinimizer, typename ErrorMinimizersImpl<T>::PointToPointWithCovErrorMinimizer)
	ADD_TO_REGISTRAR(ErrorMinimizer, PointToPlaneWithCovErrorMinimizer, typename ErrorMinimizersImpl<T>::PointToPlaneWithCovErrorMinimizer)
	
	ADD_TO_REGISTRAR(TransformationChecker, CounterTransformationChecker, typename TransformationCheckersImpl<T>::CounterTransformationChecker)
	ADD_TO_REGISTRAR(TransformationChecker, DifferentialTransformationChecker, typename TransformationCheckersImpl<T>::DifferentialTransformationChecker)
	ADD_TO_REGISTRAR(TransformationChecker, BoundTransformationChecker, typename TransformationCheckersImpl<T>::BoundTransformationChecker)
	
	ADD_TO_REGISTRAR_NO_PARAM(Inspector, NullInspector, typename InspectorsImpl<T>::NullInspector)
	ADD_TO_REGISTRAR(Inspector, PerformanceInspector, typename InspectorsImpl<T>::PerformanceInspector)
	ADD_TO_REGISTRAR(Inspector, VTKFileInspector, typename InspectorsImpl<T>::VTKFileInspector)
	
	ADD_TO_REGISTRAR_NO_PARAM(Logger, NullLogger, NullLogger)
	ADD_TO_REGISTRAR(Logger, FileLogger, FileLogger)
}

// static instances plus wrapper function to get them from templatized static method in PointMatcher

PointMatcher<float> _PointMatcherFloat;
PointMatcher<double> _PointMatcherDouble;

template<typename T>
const PointMatcher<T>& _getPM();

template<>
const PointMatcher<float>& _getPM<float>() { return _PointMatcherFloat; }
template<>
const PointMatcher<double>& _getPM<double>() { return _PointMatcherDouble; }

//! Return instances
template<typename T>
const PointMatcher<T>& PointMatcher<T>::get()
{
	return _getPM<T>();
}

template const PointMatcher<float>& PointMatcher<float>::get();
template const PointMatcher<double>& PointMatcher<double>::get();
