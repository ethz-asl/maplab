#pragma once

// nabo
#include <nabo/nabo.h>

// pointmatcher
#include <pointmatcher/PointMatcher.h>

namespace PointMatcher_ros {

// nabo
using NNS = Nabo::NearestNeighbourSearch<float>;
using NNSearchType = NNS::SearchType;

// pointmatcher
using Pm = PointMatcher<float>;
using PmDataPoints = Pm::DataPoints;
using PmDataPointsView = PmDataPoints::View;
using PmDataPointsConstView = PmDataPoints::ConstView;
using PmIcp = Pm::ICP;
using PmPointCloudFilter = Pm::DataPointsFilter;
using PmPointCloudFilters = Pm::DataPointsFilters;
using PmTransformator = Pm::Transformation;
using PmTfParameters = Pm::TransformationParameters;
using PmMatrix = Pm::Matrix;
using PmMatches = Pm::Matches;

}  // namespace PointMatcher_ros
