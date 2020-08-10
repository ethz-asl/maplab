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

#include "TransformationsImpl.h"
#include "Functions.h"

#include <iostream>

using namespace PointMatcherSupport;

//! Construct a transformation exception
TransformationError::TransformationError(const std::string& reason):
	runtime_error(reason)
{}

//! RigidTransformation
template<typename T>
typename PointMatcher<T>::DataPoints TransformationsImpl<T>::RigidTransformation::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());

	const unsigned int nbRows = parameters.rows()-1;
	const unsigned int nbCols = parameters.cols()-1;

	const TransformationParameters R(parameters.topLeftCorner(nbRows, nbCols));

	if(this->checkParameters(parameters) == false)	
		throw TransformationError("RigidTransformation: Error, rotation matrix is not orthogonal.");	
	
	//DataPoints transformedCloud(input.featureLabels, input.descriptorLabels, input.timeLabels, input.features.cols());
	DataPoints transformedCloud = input;
	
	// Apply the transformation to features
	transformedCloud.features = parameters * input.features;
	
	// Apply the transformation to descriptors
	int row(0);
	const int descCols(input.descriptors.cols());
	for (size_t i = 0; i < input.descriptorLabels.size(); ++i)
	{
		const int span(input.descriptorLabels[i].span);
		const std::string& name(input.descriptorLabels[i].text);
		const BOOST_AUTO(inputDesc, input.descriptors.block(row, 0, span, descCols));
		BOOST_AUTO(outputDesc, transformedCloud.descriptors.block(row, 0, span, descCols));
		if (name == "normals" || name == "observationDirections")
			outputDesc = R * inputDesc;
		
		row += span;
	}

	return transformedCloud;
}

//! Ensure orthogonality of the rotation matrix
template<typename T>
bool TransformationsImpl<T>::RigidTransformation::checkParameters(const TransformationParameters& parameters) const
{
	//FIXME: FP - should we put that as function argument?
	const T epsilon = 0.001;
	const unsigned int nbRows = parameters.rows()-1;
	const unsigned int nbCols = parameters.cols()-1;

	const TransformationParameters R(parameters.topLeftCorner(nbRows, nbCols));
	
	if(anyabs(1 - R.determinant()) > epsilon)
		return false;
	else
		return true;

}

//! Force orthogonality of the rotation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters TransformationsImpl<T>::RigidTransformation::correctParameters(const TransformationParameters& parameters) const
{
	TransformationParameters ortho = parameters;
	if(ortho.cols() == 4)
	{
		//const Eigen::Matrix<T, 3, 1> col0 = parameters.block(0, 0, 3, 1).normalized();
		const Eigen::Matrix<T, 3, 1> col1 = parameters.block(0, 1, 3, 1).normalized();
		const Eigen::Matrix<T, 3, 1> col2 = parameters.block(0, 2, 3, 1).normalized();

		const Eigen::Matrix<T, 3, 1> newCol0 = col1.cross(col2);
		const Eigen::Matrix<T, 3, 1> newCol1 = col2.cross(newCol0);
		const Eigen::Matrix<T, 3, 1> newCol2 = col2;

		ortho.block(0, 0, 3, 1) = newCol0;
		ortho.block(0, 1, 3, 1) = newCol1;
		ortho.block(0, 2, 3, 1) = newCol2;
	}
	else if(ortho.cols() == 3)
	{
		const T epsilon = 0.001;

		// R = [ a b]
		//     [-b a]
		if(anyabs(parameters(0,0) - parameters(1,1)) > epsilon || anyabs(parameters(1,0) + parameters(0,1)) > epsilon)
		{
			throw TransformationError("RigidTransformation: Error, only proper rigid transformations are supported.");
		}
		
		// mean of a and b
		T a = (parameters(0,0) + parameters(1,1))/2; 	
		T b = (-parameters(1,0) + parameters(0,1))/2;
		T sum = sqrt(pow(a,2) + pow(b,2));

		a = a/sum;
		b = b/sum;

		ortho(0,0) =  a; ortho(0,1) = b;
		ortho(1,0) = -b; ortho(1,1) = a;
	}


	return ortho;
}

template struct TransformationsImpl<float>::RigidTransformation;
template struct TransformationsImpl<double>::RigidTransformation;

//! SimilarityTransformation
template<typename T>
typename PointMatcher<T>::DataPoints TransformationsImpl<T>::SimilarityTransformation::compute(
	const DataPoints& input,
	const TransformationParameters& parameters) const
{
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());

	const unsigned int nbRows = parameters.rows()-1;
	const unsigned int nbCols = parameters.cols()-1;

	const TransformationParameters R(parameters.topLeftCorner(nbRows, nbCols));

	if(this->checkParameters(parameters) == false)
		throw TransformationError("SimilarityTransformation: Error, invalid similarity transform.");
	
	//DataPoints transformedCloud(input.featureLabels, input.descriptorLabels, input.timeLabels, input.features.cols());
	DataPoints transformedCloud = input;
	
	// Apply the transformation to features
	transformedCloud.features = parameters * input.features;
	
	// Apply the transformation to descriptors
	int row(0);
	const int descCols(input.descriptors.cols());
	for (size_t i = 0; i < input.descriptorLabels.size(); ++i)
	{
		const int span(input.descriptorLabels[i].span);
		const std::string& name(input.descriptorLabels[i].text);
		const BOOST_AUTO(inputDesc, input.descriptors.block(row, 0, span, descCols));
		BOOST_AUTO(outputDesc, transformedCloud.descriptors.block(row, 0, span, descCols));
		if (name == "normals" || name == "observationDirections")
			outputDesc = R * inputDesc;
		
		row += span;
	}

	return transformedCloud;
}

//! Nothing to check for a similarity transform
template<typename T>
bool TransformationsImpl<T>::SimilarityTransformation::checkParameters(const TransformationParameters& parameters) const
{
	//FIXME: FP - should we put that as function argument?
	return true;
}

//! Nothing to correct for a similarity transform
template<typename T>
typename PointMatcher<T>::TransformationParameters TransformationsImpl<T>::SimilarityTransformation::correctParameters(const TransformationParameters& parameters) const
{
	return parameters;
}

template struct TransformationsImpl<float>::SimilarityTransformation;
template struct TransformationsImpl<double>::SimilarityTransformation;

template<typename T>
typename PointMatcher<T>::DataPoints TransformationsImpl<T>::PureTranslation::compute(const DataPoints& input,
		const TransformationParameters& parameters) const {
	assert(input.features.rows() == parameters.rows());
	assert(parameters.rows() == parameters.cols());

	if(this->checkParameters(parameters) == false)
		throw PointMatcherSupport::TransformationError("PureTranslation: Error, left part  not identity.");

	//DataPoints transformedCloud(input.featureLabels, input.descriptorLabels, input.features.cols());
	DataPoints transformedCloud = input;

	// Apply the transformation to features
	transformedCloud.features = parameters * input.features;

	return transformedCloud;
}

template<typename T>
typename PointMatcher<T>::TransformationParameters TransformationsImpl<T>::PureTranslation::correctParameters(
		const TransformationParameters& parameters) const {
	const int rows = parameters.rows();
	const int cols = parameters.cols();

	// make a copy of the parameters to perform corrections on
	TransformationParameters correctedParameters(parameters);

	// set the top left block to the identity matrix
	correctedParameters.block(0,0,rows-1,cols-1).setIdentity();

	// fix the bottom row
	correctedParameters.block(rows-1,0,1,cols-1).setZero();
	correctedParameters(rows-1,cols-1) = 1;

	return correctedParameters;
}

template<typename T>
bool TransformationsImpl<T>::PureTranslation::checkParameters(
		const TransformationParameters& parameters) const {
	const int rows = parameters.rows();
	const int cols = parameters.cols();

	// make a copy of parameters to perform the check
	TransformationParameters parameters_(parameters);

	// set the translation components of the transformation matrix to 0
	parameters_.block(0,cols-1,rows-1,1).setZero();

	// If we have the identity matrix, than this is indeed a pure translation
	if (parameters_.isApprox(TransformationParameters::Identity(rows,cols)))
		return true;
	else
		return false;
}

template struct TransformationsImpl<float>::PureTranslation;
template struct TransformationsImpl<double>::PureTranslation;
