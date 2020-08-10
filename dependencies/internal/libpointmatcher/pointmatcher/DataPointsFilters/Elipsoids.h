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

//! Subsampling Surfels (Elipsoids) filter. First decimate the space until there is at most knn points, then find the center of mass and use the points to estimate nromal using eigen-decomposition
template<typename T>
struct ElipsoidsDataPointsFilter: public PointMatcher<T>::DataPointsFilter
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;
	
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename PointMatcher<T>::Matrix Matrix;	
	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
	
  inline static const std::string description()
  {
    return "Subsampling, Surfels (Elipsoids). This filter decomposes the point-cloud space in boxes, by recursively splitting the cloud through axis-aligned hyperplanes such as to maximize the evenness of the aspect ratio of the box. When the number of points in a box reaches a value knn or lower, the filter computes the center of mass of these points and its normal by taking the eigenvector corresponding to the smallest eigenvalue of all points in the box.";
  }
  inline static const ParametersDoc availableParameters()
  {
    return {
		{"ratio", "ratio of points to keep with random subsampling. Matrix (normal, density, etc.) will be associated to all points in the same bin.", "0.5", "0.0000001", "0.9999999", &P::Comp<T> },
		{"knn", "determined how many points are used to compute the normals. Direct link with the rapidity of the computation (large = fast). Technically, limit over which a box is splitted in two", "7", "3", "2147483647", &P::Comp<unsigned> },
		{"samplingMethod", "if set to 0, random subsampling using the parameter ratio. If set to 1, bin subsampling with the resulting number of points being 1/knn.", "0", "0", "1", &P::Comp<unsigned> },
		{"maxBoxDim", "maximum length of a box above which the box is discarded", "inf" },
		{"averageExistingDescriptors", "whether the filter keep the existing point descriptors and average them or should it drop them", "1" },
		{"maxTimeWindow", "maximum spread of times in a surfel", "inf" },
		{"minPlanarity", "to what extend planarity of surfels needs to be enforced", "0"},
		{"keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1" },
		{"keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0" },
		{"keepEigenValues", "whether the eigen values should be added as descriptors to the resulting cloud", "0" },
		{"keepEigenVectors", "whether the eigen vectors should be added as descriptors to the resulting cloud", "0" },
		{"keepMeans", "whether the means should be added as descriptors to the resulting cloud", "0" },
		{"keepCovariances", "whether the covariances should be added as descriptors to the resulting cloud", "0" },
		{"keepWeights", "whether the original number of points should be added as descriptors to the resulting cloud", "0" },
		{"keepShapes", "whether the shape parameters of cylindricity (C), sphericality (S) and planarity (P) shall be calculated", "0" },
		{"keepIndices", "whether the indices of points an ellipsoid is constructed of shall be kept", "0" }
    }
    ;
  }

  const T ratio;
  const unsigned knn;
  const unsigned samplingMethod;
  const T maxBoxDim;
  const T maxTimeWindow;
  const T minPlanarity;
  const bool averageExistingDescriptors;
  const bool keepNormals;
  const bool keepDensities;
  const bool keepEigenValues;
  const bool keepEigenVectors;
  const bool keepCovariances;
  const bool keepWeights;
  const bool keepMeans;
  const bool keepShapes;
  const bool keepIndices;


 public:
  ElipsoidsDataPointsFilter(const Parameters& params = Parameters());
  virtual ~ElipsoidsDataPointsFilter() {}
  virtual DataPoints filter(const DataPoints& input);
  virtual void inPlaceFilter(DataPoints& cloud);

 protected:
  struct BuildData
  {
    typedef std::vector<int> Indices;
    typedef typename DataPoints::View View;
    typedef typename Eigen::Matrix<std::int64_t, Eigen::Dynamic, Eigen::Dynamic> Int64Matrix;
    typedef typename Eigen::Matrix<std::int64_t, 1, Eigen::Dynamic> Int64Vector;

    Indices indices;
    Indices indicesToKeep;
    Matrix& features;
    Matrix& descriptors;
    Int64Matrix& times;
    boost::optional<View> normals;
    boost::optional<View> densities;
    boost::optional<View> eigenValues;
    boost::optional<View> eigenVectors;
    boost::optional<View> weights;
    boost::optional<View> covariance;
    boost::optional<View> means;
    boost::optional<View> shapes;
    boost::optional<View> pointIds;
    boost::optional<View> pointX;
    boost::optional<View> pointY;
    boost::optional<View> pointZ;
    boost::optional<View> numOfNN;
    int outputInsertionPoint;
    int unfitPointsCount;

    BuildData(Matrix& features, Matrix& descriptors, Int64Matrix& times):
      features(features),
      descriptors(descriptors),
      times(times),
      unfitPointsCount(0)
    {
      const int pointsCount(features.cols());
      indices.reserve(pointsCount);
      for (int i = 0; i < pointsCount; ++i)
        indices.push_back(i);
    }
  };

  struct CompareDim
  {
    const int dim;
    const BuildData& buildData;
    CompareDim(const int dim, const BuildData& buildData):dim(dim),buildData(buildData){}
    bool operator() (const int& p0, const int& p1)
    {
      return buildData.features(dim, p0) <
          buildData.features(dim, p1);
    }
  };

 protected:
  void buildNew(BuildData& data, const int first, const int last, Vector&& minValues, Vector&& maxValues) const;
  void fuseRange(BuildData& data, const int first, const int last) const;
};
