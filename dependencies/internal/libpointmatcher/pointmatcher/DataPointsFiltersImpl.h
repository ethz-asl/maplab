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

#ifndef __POINTMATCHER_DATAPOINTSFILTERS_H
#define __POINTMATCHER_DATAPOINTSFILTERS_H

#include "DataPointsFilters/Identity.h"
#include "DataPointsFilters/RemoveNaN.h"
#include "DataPointsFilters/MaxDist.h"
#include "DataPointsFilters/MinDist.h"
#include "DataPointsFilters/BoundingBox.h"
#include "DataPointsFilters/MaxQuantileOnAxis.h"
#include "DataPointsFilters/MaxDensity.h"
#include "DataPointsFilters/SurfaceNormal.h"
#include "DataPointsFilters/SamplingSurfaceNormal.h"
#include "DataPointsFilters/OrientNormals.h"
#include "DataPointsFilters/IncidenceAngle.h"
#include "DataPointsFilters/RandomSampling.h"
#include "DataPointsFilters/MaxPointCount.h"
#include "DataPointsFilters/FixStepSampling.h"
#include "DataPointsFilters/Shadow.h"
#include "DataPointsFilters/SimpleSensorNoise.h"
#include "DataPointsFilters/ObservationDirection.h"
#include "DataPointsFilters/VoxelGrid.h"
#include "DataPointsFilters/CutAtDescriptorThreshold.h"
#include "DataPointsFilters/Elipsoids.h"
#include "DataPointsFilters/Gestalt.h"
#include "DataPointsFilters/OctreeGrid.h"
#include "DataPointsFilters/NormalSpace.h"
#include "DataPointsFilters/CovarianceSampling.h"
#include "DataPointsFilters/DistanceLimit.h"
#include "DataPointsFilters/RemoveSensorBias.h"

template<typename T>
struct DataPointsFiltersImpl
{
	typedef ::IdentityDataPointsFilter<T>   IdentityDataPointsFilter;
	typedef ::RemoveNaNDataPointsFilter<T>  RemoveNaNDataPointsFilter;
	typedef ::MaxDistDataPointsFilter<T>	MaxDistDataPointsFilter;
	typedef ::MinDistDataPointsFilter<T>	MinDistDataPointsFilter;
	typedef ::BoundingBoxDataPointsFilter<T> BoundingBoxDataPointsFilter;
	typedef ::MaxQuantileOnAxisDataPointsFilter<T> MaxQuantileOnAxisDataPointsFilter;
	typedef ::MaxDensityDataPointsFilter<T> MaxDensityDataPointsFilter;
	typedef ::SurfaceNormalDataPointsFilter<T> SurfaceNormalDataPointsFilter;
	typedef ::SamplingSurfaceNormalDataPointsFilter<T> SamplingSurfaceNormalDataPointsFilter;
	typedef ::OrientNormalsDataPointsFilter<T>  OrientNormalsDataPointsFilter;
	typedef ::IncidenceAngleDataPointsFilter<T> IncidenceAngleDataPointsFilter;
	typedef ::RandomSamplingDataPointsFilter<T> RandomSamplingDataPointsFilter;
	typedef ::MaxPointCountDataPointsFilter<T> MaxPointCountDataPointsFilter;
	typedef ::FixStepSamplingDataPointsFilter<T> FixStepSamplingDataPointsFilter;
	typedef ::ShadowDataPointsFilter<T> ShadowDataPointsFilter;
	typedef ::SimpleSensorNoiseDataPointsFilter<T> SimpleSensorNoiseDataPointsFilter;
	typedef ::ObservationDirectionDataPointsFilter<T> ObservationDirectionDataPointsFilter;
	typedef ::VoxelGridDataPointsFilter<T> VoxelGridDataPointsFilter;
	typedef ::CutAtDescriptorThresholdDataPointsFilter<T> CutAtDescriptorThresholdDataPointsFilter;
	typedef ::ElipsoidsDataPointsFilter<T> ElipsoidsDataPointsFilter;
	typedef ::GestaltDataPointsFilter<T> GestaltDataPointsFilter;
	typedef ::OctreeGridDataPointsFilter<T> OctreeGridDataPointsFilter;
	typedef ::NormalSpaceDataPointsFilter<T> NormalSpaceDataPointsFilter;
	typedef ::CovarianceSamplingDataPointsFilter<T> CovarianceSamplingDataPointsFilter;
	typedef ::DistanceLimitDataPointsFilter<T> DistanceLimitDataPointsFilter;
	typedef ::RemoveSensorBiasDataPointsFilter<T> RemoveSensorBiasDataPointsFilter;

}; // DataPointsFiltersImpl

#endif // __POINTMATCHER_DATAPOINTSFILTERS_H
