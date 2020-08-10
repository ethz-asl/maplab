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
#include <iostream>

using namespace std;

//! Construct a label from a given name and number of data dimensions it spans
template<typename T>
PointMatcher<T>::DataPoints::Label::Label(const std::string& text, const size_t span):
	text(text),
	span(span)
{}

//! Construct the exception with an error message
template<typename T>
PointMatcher<T>::DataPoints::InvalidField::InvalidField(const std::string& reason):
	runtime_error(reason)
{}

//! Return whether two labels are equals
template<typename T>
bool PointMatcher<T>::DataPoints::Label::operator ==(const Label& that) const
{
	return (this->text == that.text) && (this->span == that.span);
}

//! Construct empty Labels
template<typename T>
PointMatcher<T>::DataPoints::Labels::Labels()
{}

//! Construct Labels with a single Label in it
template<typename T>
PointMatcher<T>::DataPoints::Labels::Labels(const Label& label):
	std::vector<Label>(1, label)
{}

//! Return whether there is a label named text
template<typename T>
bool PointMatcher<T>::DataPoints::Labels::contains(const std::string& text) const
{
	for (const_iterator it(this->begin()); it != this->end(); ++it)
	{
		if (it->text == text)
			return true;
	}
	return false;
}

//! Return the sum of the spans of each label
template<typename T>
size_t PointMatcher<T>::DataPoints::Labels::totalDim() const
{
	size_t dim(0);
	for (const_iterator it(this->begin()); it != this->end(); ++it)
		dim += it->span;
	return dim;
}

//! Construct an empty point cloud
template<typename T>
PointMatcher<T>::DataPoints::DataPoints()
{}

//! Construct a point cloud from existing descriptions
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Labels& featureLabels, const Labels& descriptorLabels, const size_t pointCount):
	featureLabels(featureLabels),
	descriptorLabels(descriptorLabels)
{
	features.resize(featureLabels.totalDim(), pointCount);
	if(descriptorLabels.totalDim())
		descriptors.resize(descriptorLabels.totalDim(), pointCount);
}

//! Construct a point cloud from existing descriptions
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Labels& featureLabels,
                                        const Labels& descriptorLabels,
										const Labels& timeLabels,
										const size_t pointCount):
	featureLabels(featureLabels),
	descriptorLabels(descriptorLabels),
	timeLabels(timeLabels)
{
	features.resize(featureLabels.totalDim(), pointCount);

	if(descriptorLabels.totalDim())
		descriptors.resize(descriptorLabels.totalDim(), pointCount);
	if(timeLabels.totalDim())
		times.resize(timeLabels.totalDim(), pointCount);
}

//! Construct a point cloud from existing features without any descriptor
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Matrix& features, const Labels& featureLabels):
	features(features),
	featureLabels(featureLabels)
{}

//! Construct a point cloud from existing features and descriptors
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Matrix& features, const Labels& featureLabels, const Matrix& descriptors, const Labels& descriptorLabels):
	features(features),
	featureLabels(featureLabels),
	descriptors(descriptors),
	descriptorLabels(descriptorLabels)
{}

//! Construct a point cloud from existing features, descriptors and times
template<typename T>
PointMatcher<T>::DataPoints::DataPoints(const Matrix& features, const Labels& featureLabels, const Matrix& descriptors, const Labels& descriptorLabels, const Int64Matrix& times, const Labels& timeLabels):
	features(features),
	featureLabels(featureLabels),
	descriptors(descriptors),
	descriptorLabels(descriptorLabels),
	times(times),
	timeLabels(timeLabels)
{}

//! Return the number of points contained in the point cloud
template<typename T>
unsigned PointMatcher<T>::DataPoints::getNbPoints() const
{
	return features.cols();
}

//! Return the dimension of the point cloud
template<typename T>
unsigned PointMatcher<T>::DataPoints::getEuclideanDim() const
{
	return (features.rows() - 1);
}

//! Return the dimension of the point cloud in homogeneous coordinates (one more than Euclidean dimension)
template<typename T>
unsigned PointMatcher<T>::DataPoints::getHomogeneousDim() const
{
	return features.rows();
}

//! Return the number of grouped descriptors (e.g., normals can have 3 components but would count as only one)
template<typename T>
unsigned PointMatcher<T>::DataPoints::getNbGroupedDescriptors() const
{
	return descriptorLabels.size();
}

//! Return the total number of descriptors
template<typename T>
unsigned PointMatcher<T>::DataPoints::getDescriptorDim() const
{
	return descriptors.rows();
}

//! Return the total number of times
template<typename T>
unsigned PointMatcher<T>::DataPoints::getTimeDim() const
{
	return times.rows();
}

//! Return whether two point-clouds are identical (same data and same labels)
template<typename T>
bool PointMatcher<T>::DataPoints::operator ==(const DataPoints& that) const
{
	// Note comparing matrix without the same dimensions trigger an assert
	bool isEqual = false;
	if((features.rows() == that.features.rows()) &&
		(features.cols() == that.features.cols()) &&
		(descriptors.rows() == that.descriptors.rows()) &&
		(descriptors.cols() == that.descriptors.cols())	&&
		(times.rows() == that.times.rows()) &&
		(times.cols() == that.times.cols()))
	{
		isEqual = (features == that.features) &&
			(featureLabels == that.featureLabels) &&
			(descriptors == that.descriptors) &&
			(descriptorLabels == that.descriptorLabels)&&
			(times == that.times) &&
			(timeLabels == that.timeLabels);
	}

	return isEqual;
		
}

//! Add another point cloud after the current one. Faster merge will be achieved if all descriptor and time labels are the same. If not, only the commun labels with be kept.
template<typename T>
void PointMatcher<T>::DataPoints::concatenate(const DataPoints& dp)
{
	const int nbPoints1 = this->features.cols();
	const int nbPoints2 = dp.features.cols();
	const int nbPointsTotal = nbPoints1 + nbPoints2;

	const int dimFeat = this->features.rows();
	if(dimFeat != dp.features.rows())
	{
		stringstream errorMsg;
		errorMsg << "Cannot concatenate DataPoints because the dimension of the features are not the same. Actual dimension: " << dimFeat << " New dimension: " << dp.features.rows(); 
		throw InvalidField(errorMsg.str());
	}
	
	// concatenate features
	this->features.conservativeResize(Eigen::NoChange, nbPointsTotal);
	this->features.rightCols(nbPoints2) = dp.features;
	
	// concatenate descriptors
	concatenateLabelledMatrix(&descriptorLabels, &descriptors, dp.descriptorLabels, dp.descriptors);
	assertDescriptorConsistency();
	
	// concatenate time
	concatenateLabelledMatrix(&timeLabels, &times, dp.timeLabels, dp.times);
	assertTimesConsistency();
	
}


//! Add another matrix after the current one. Faster merge will be achieved if all labels are the same. If not, only the commun labels with be kept.
template<typename T>
template<typename MatrixType>
void PointMatcher<T>::DataPoints::concatenateLabelledMatrix(Labels* labels, MatrixType* data, const Labels extraLabels, const MatrixType extraData)
{
	const int nbPoints1 = data->cols();
	const int nbPoints2 = extraData.cols();
	const int nbPointsTotal = nbPoints1 + nbPoints2;


	if (*labels == extraLabels)
	{
		if (labels->size() > 0)
		{
			// same descriptors, fast merge
			data->conservativeResize(Eigen::NoChange, nbPointsTotal);
			data->rightCols(nbPoints2) = extraData;
		}
	}
	else
	{
		// different descriptors, slow merge
		
		// collect labels to be kept
		Labels newLabels;
		for(BOOST_AUTO(it, labels->begin()); it != labels->end(); ++it)
		{
			for(BOOST_AUTO(jt, extraLabels.begin()); jt != extraLabels.end(); ++jt)
			{
				if (it->text == jt->text)
				{
					if (it->span != jt->span)
						throw InvalidField(
						(boost::format("The field %1% has dimension %2% in this, different than dimension %3% in that") % it->text % it->span % jt->span).str()
					);
					newLabels.push_back(*it);
					break;
				}
			}
		}
		
		// allocate new descriptors
		if (newLabels.size() > 0)
		{
			MatrixType newData;
			Labels filledLabels;
			this->allocateFields(newLabels, filledLabels, newData);
			assert(newLabels == filledLabels);
			
			// fill
			unsigned row(0);
			for(BOOST_AUTO(it, newLabels.begin()); it != newLabels.end(); ++it)
			{
				Eigen::Block<MatrixType> view(newData.block(row, 0, it->span, newData.cols()));
				view.leftCols(nbPoints1) = getViewByName(it->text, *labels, *data);
				view.rightCols(nbPoints2) = getViewByName(it->text, extraLabels, extraData);
				row += it->span;
			}
			
			// swap descriptors
			data->swap(newData);
			*labels = newLabels;
		}
		else
		{
			*data = MatrixType();
			*labels = Labels();
		}
	}
}

//! Resize the cloud to pointCount points, conserving existing ones
template<typename T>
void PointMatcher<T>::DataPoints::conservativeResize(Index pointCount)
{
	features.conservativeResize(Eigen::NoChange, pointCount);
	if (descriptors.cols() > 0)
		descriptors.conservativeResize(Eigen::NoChange, pointCount);

	if (times.cols() > 0)
		times.conservativeResize(Eigen::NoChange, pointCount);
}

//! Create an empty DataPoints of similar dimensions and labels  for features, descriptors and times
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::DataPoints::createSimilarEmpty() const
{
	const int nbPoints(features.cols());
	DataPoints output(
		Matrix(features.rows(), nbPoints),
		featureLabels
	);

	assertDescriptorConsistency();
	if (descriptors.cols() > 0)
	{
		output.descriptors = Matrix(descriptors.rows(), nbPoints);
		output.descriptorLabels = descriptorLabels;
	}

	assertTimesConsistency();
	if (times.cols() > 0)
	{
		output.times = Int64Matrix(times.rows(), nbPoints);
		output.timeLabels = timeLabels;
	}

	return output;
}

//! Create an empty DataPoints with pointCount points of similar dimensions and labels, both for features and descriptors
template<typename T>
typename PointMatcher<T>::DataPoints PointMatcher<T>::DataPoints::createSimilarEmpty(Index pointCount) const
{
	DataPoints output(
		Matrix(features.rows(), pointCount),
		featureLabels
	);

	assertDescriptorConsistency();
	if (descriptors.cols() > 0)
	{
		output.descriptors = Matrix(descriptors.rows(), pointCount);
		output.descriptorLabels = descriptorLabels;
	}

	assertTimesConsistency();
	if (times.cols() > 0)
	{
		output.times = Int64Matrix(times.rows(), pointCount);
		output.timeLabels = timeLabels;
	}

	return output;
}

//! Set column thisCol equal to column thatCol of that, copy features and descriptors if any. Assumes sizes are similar
template<typename T>
void PointMatcher<T>::DataPoints::setColFrom(Index thisCol, const DataPoints& that, Index thatCol)
{
	features.col(thisCol) = that.features.col(thatCol);
	
	if (descriptors.cols() > 0)
		descriptors.col(thisCol) = that.descriptors.col(thatCol);

	if (times.cols() > 0)
		times.col(thisCol) = that.times.col(thatCol);

}
//! Swap column i and j in the point cloud, swap also features and descriptors if any. Assumes sizes are similar
template<typename T>
void PointMatcher<T>::DataPoints::swapCols(Index iCol,Index jCol)
{
	features.col(iCol).swap(features.col(jCol));
	if (descriptors.cols() > 0)
		descriptors.col(iCol).swap(descriptors.col(jCol));
	if (times.cols() > 0)
		times.col(iCol).swap(times.col(jCol));
}

//------------------------------------
// Methods related to features
//------------------------------------


//! Makes sure a feature of a given name exists, if present, check its dimensions
template<typename T>
void PointMatcher<T>::DataPoints::allocateFeature(const std::string& name, const unsigned dim)
{
	allocateField(name, dim, featureLabels, features);
}

//! Make sure a vector of features of given names exist
template<typename T>
void PointMatcher<T>::DataPoints::allocateFeatures(const Labels& newLabels)
{
	allocateFields(newLabels, featureLabels, features);
}

//! Add a feature by name, remove first if already exists. The 'pad' field will stay at the end for homogeneous transformation 
template<typename T>
void PointMatcher<T>::DataPoints::addFeature(const std::string& name, const Matrix& newFeature)
{
	removeFeature("pad");
	addField<Matrix>(name, newFeature, featureLabels, features);
	addField<Matrix>("pad", Matrix::Ones(1, features.cols()), featureLabels, features);
}

//! Remove a feature by name, the whole matrix will be copied
template<typename T>
void PointMatcher<T>::DataPoints::removeFeature(const std::string& name)
{
	removeField(name, featureLabels, features);
}

//! Get feature by name, return a matrix containing a copy of the requested feature
template<typename T>
typename PointMatcher<T>::Matrix PointMatcher<T>::DataPoints::getFeatureCopyByName(const std::string& name) const
{
	return Matrix(getFeatureViewByName(name));
}

//! Get a const view on a feature by name, throw an exception if it does not exist
template<typename T>
const typename PointMatcher<T>::DataPoints::ConstView PointMatcher<T>::DataPoints::getFeatureViewByName(const std::string& name) const
{
	return getConstViewByName(name, featureLabels, features);
}

//! Get a view on a feature by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::View PointMatcher<T>::DataPoints::getFeatureViewByName(const std::string& name)
{
	return getViewByName(name, featureLabels, features);
}

//! Get a const view on a feature row by name and number, throw an exception if it does not exist
template<typename T>
const typename PointMatcher<T>::DataPoints::ConstView PointMatcher<T>::DataPoints::getFeatureRowViewByName(const std::string& name, const unsigned row) const
{
	return getConstViewByName(name, featureLabels, features, int(row));
}

//! Get a view on a feature by row name and number, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::View PointMatcher<T>::DataPoints::getFeatureRowViewByName(const std::string& name, const unsigned row)
{
	return getViewByName(name, featureLabels, features, int(row));
}

//! Look if a feature with a given name exist
template<typename T>
bool PointMatcher<T>::DataPoints::featureExists(const std::string& name) const
{
	return fieldExists(name, 0, featureLabels);
}

//! Look if a feature with a given name and dimension exist
template<typename T>
bool PointMatcher<T>::DataPoints::featureExists(const std::string& name, const unsigned dim) const
{
	return fieldExists(name, dim, featureLabels);
}

//! Return the dimension of a feature with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFeatureDimension(const std::string& name) const
{
	return getFieldDimension(name, featureLabels);
}

//! Return the starting row of a feature with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFeatureStartingRow(const std::string& name) const
{
	return getFieldStartingRow(name, featureLabels);
}

//------------------------------------
// Methods related to descriptors
//------------------------------------

//! Makes sure a descriptor of a given name exists, if present, check its dimensions
template<typename T>
void PointMatcher<T>::DataPoints::allocateDescriptor(const std::string& name, const unsigned dim)
{
	allocateField(name, dim, descriptorLabels, descriptors);
}

//! Make sure a vector of descriptors of given names exist
template<typename T>
void PointMatcher<T>::DataPoints::allocateDescriptors(const Labels& newLabels)
{
	allocateFields(newLabels, descriptorLabels, descriptors);
}

//! Add a descriptor by name, remove first if already exists
template<typename T>
void PointMatcher<T>::DataPoints::addDescriptor(const std::string& name, const Matrix& newDescriptor)
{
	addField(name, newDescriptor, descriptorLabels, descriptors);
}

//! Remove a descriptor by name, the whole matrix will be copied
template<typename T>
void PointMatcher<T>::DataPoints::removeDescriptor(const std::string& name)
{
	removeField(name, descriptorLabels, descriptors);
}


//! Get descriptor by name, return a matrix containing a copy of the requested descriptor
template<typename T>
typename PointMatcher<T>::Matrix PointMatcher<T>::DataPoints::getDescriptorCopyByName(const std::string& name) const
{
	return Matrix(getDescriptorViewByName(name));
}

//! Get a const view on a descriptor by name, throw an exception if it does not exist
template<typename T>
const typename PointMatcher<T>::DataPoints::ConstView PointMatcher<T>::DataPoints::getDescriptorViewByName(const std::string& name) const
{
	return getConstViewByName(name, descriptorLabels, descriptors);
}

//! Get a view on a descriptor by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::View PointMatcher<T>::DataPoints::getDescriptorViewByName(const std::string& name)
{
	return getViewByName(name, descriptorLabels, descriptors);
}

//! Get a const view on a descriptor row by name and number, throw an exception if it does not exist
template<typename T>
const typename PointMatcher<T>::DataPoints::ConstView PointMatcher<T>::DataPoints::getDescriptorRowViewByName(const std::string& name, const unsigned row) const
{
	return getConstViewByName(name, descriptorLabels, descriptors, int(row));
}

//! Get a view on a descriptor by row name and number, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::View PointMatcher<T>::DataPoints::getDescriptorRowViewByName(const std::string& name, const unsigned row)
{
	return getViewByName(name, descriptorLabels, descriptors, int(row));
}

//! Look if a descriptor with a given name exist
template<typename T>
bool PointMatcher<T>::DataPoints::descriptorExists(const std::string& name) const
{
	return fieldExists(name, 0, descriptorLabels);
}

//! Look if a descriptor with a given name and dimension exist
template<typename T>
bool PointMatcher<T>::DataPoints::descriptorExists(const std::string& name, const unsigned dim) const
{
	return fieldExists(name, dim, descriptorLabels);
}

//! Return the dimension of a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getDescriptorDimension(const std::string& name) const
{
	return getFieldDimension(name, descriptorLabels);
}

//! Return the starting row of a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getDescriptorStartingRow(const std::string& name) const
{
	return getFieldStartingRow(name, descriptorLabels);
}

//! Assert if descriptors are not consistent with features
template<typename T>
void PointMatcher<T>::DataPoints::assertDescriptorConsistency() const
{
	assertConsistency("descriptors", descriptors.rows(), descriptors.cols(), descriptorLabels);
}

//------------------------------------
// Methods related to time
//------------------------------------

//! Makes sure a time of a given name exists, if present, check its dimensions
template<typename T>
void PointMatcher<T>::DataPoints::allocateTime(const std::string& name, const unsigned dim)
{
	allocateField(name, dim, timeLabels, times);
}

//! Make sure a vector of time of given names exist
template<typename T>
void PointMatcher<T>::DataPoints::allocateTimes(const Labels& newLabels)
{
	allocateFields(newLabels, timeLabels, times);
}


//! Add a time by name, remove first if already exists
template<typename T>
void PointMatcher<T>::DataPoints::addTime(const std::string& name, const Int64Matrix& newTime)
{
	addField(name, newTime, timeLabels, times);
}

//! Remove a descriptor by name, the whole matrix will be copied
template<typename T>
void PointMatcher<T>::DataPoints::removeTime(const std::string& name)
{
	removeField(name, timeLabels, times);
}

//! Get time by name, return a matrix containing a copy of the requested time 
template<typename T>
typename PointMatcher<T>::Int64Matrix PointMatcher<T>::DataPoints::getTimeCopyByName(const std::string& name) const
{
	return Int64Matrix(getTimeViewByName(name));
}


//! Get a const view on a time by name, throw an exception if it does not exist
template<typename T>
const typename PointMatcher<T>::DataPoints::TimeConstView PointMatcher<T>::DataPoints::getTimeViewByName(const std::string& name) const
{
	return getConstViewByName(name, timeLabels, times);
}

//! Get a view on a time by name, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::TimeView PointMatcher<T>::DataPoints::getTimeViewByName(const std::string& name)
{
	return getViewByName(name, timeLabels, times);
}

//! Get a const view on a time row by name and number, throw an exception if it does not exist
template<typename T>
const typename PointMatcher<T>::DataPoints::TimeConstView PointMatcher<T>::DataPoints::getTimeRowViewByName(const std::string& name, const unsigned row) const
{
	return getConstViewByName(name, timeLabels, times, int(row));
}

//! Get a view on a time by row name and number, throw an exception if it does not exist
template<typename T>
typename PointMatcher<T>::DataPoints::TimeView PointMatcher<T>::DataPoints::getTimeRowViewByName(const std::string& name, const unsigned row)
{
	return getViewByName(name, timeLabels, times, int(row));
}


//! Look if a time with a given name exist
template<typename T>
bool PointMatcher<T>::DataPoints::timeExists(const std::string& name) const
{
	return fieldExists(name, 0, timeLabels);
}

//! Look if a time with a given name and dimension exist
template<typename T>
bool PointMatcher<T>::DataPoints::timeExists(const std::string& name, const unsigned dim) const
{
	return fieldExists(name, dim, timeLabels);
}

//! Return the dimension of a time with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getTimeDimension(const std::string& name) const
{
	return getFieldDimension(name, timeLabels);
}

//! Return the starting row of a time with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getTimeStartingRow(const std::string& name) const
{
	return getFieldStartingRow(name, timeLabels);
}

//! Assert if times are not consistent with features
template<typename T>
void PointMatcher<T>::DataPoints::assertTimesConsistency() const
{
	assertConsistency("times", times.rows(), times.cols(), timeLabels);
}

//! Assert if a matrix is not consistent with features
template<typename T>
void PointMatcher<T>::DataPoints::assertConsistency(const std::string& dataName,  const int dataRows, const int dataCols, const Labels& labels) const
{
	if (dataRows == 0)
	{
		if (dataCols != 0)
			throw InvalidField(
				(boost::format("Point cloud has degenerate %2% dimensions of rows=0, cols=%1%") % dataCols % dataName).str()
			);
		if (labels.size() > 0)
			throw InvalidField(
				(boost::format("Point cloud has no %2% data but %1% descriptor labels") % labels.size() % dataName).str()
			);
	}
	else
	{
		if (dataCols != features.cols())
		{
			throw InvalidField(
				(boost::format("Point cloud has %1% points in features but %2% points in %3%") % features.cols() % dataCols % dataName).str()
			);
		}

		int descDim(0);
		for (BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
			descDim += it->span;
		
		if (dataRows != descDim)
			throw InvalidField(
				(boost::format("Labels from %3% return %1% total dimensions but there are %2% in the %3% matrix") % descDim % dataRows % dataName).str()
			);
	
	}

}

//! Make sure a field of a given name exists, if present, check its dimensions
template<typename T>
template<typename MatrixType>
void PointMatcher<T>::DataPoints::allocateField(const std::string& name, const unsigned dim, Labels& labels, MatrixType& data) const
{
	if (fieldExists(name, 0, labels))
	{
		const unsigned descDim(getFieldDimension(name, labels));
		if (descDim != dim)
		{
			throw InvalidField(
				(boost::format("The existing field %1% has dimension %2%, different than requested dimension %3%") % name % descDim % dim).str()
			);
		}
	}
	else
	{
		const int oldDim(data.rows());
		const int totalDim(oldDim + dim);
		const int pointCount(features.cols());
		data.conservativeResize(totalDim, pointCount);
		labels.push_back(Label(name, dim));
	}
}

//! Make sure a vector of fields of given names exist
template<typename T>
template<typename MatrixType>
void PointMatcher<T>::DataPoints::allocateFields(const Labels& newLabels, Labels& labels, MatrixType& data) const
{
	typedef vector<bool> BoolVector;
	BoolVector present(newLabels.size(), false);
	
	// for fields that exist, take note and check dimension
	size_t additionalDim(0);
	for (size_t i = 0; i < newLabels.size(); ++i)
	{
		const string& newName(newLabels[i].text);
		const size_t newSpan(newLabels[i].span);
		for(BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
		{
			if (it->text == newName)
			{
				if (it->span != newSpan)
					throw InvalidField(
						(boost::format("The existing field %1% has dimension %2%, different than requested dimension %3%") % newName % it->span % newSpan).str()
					);
				present[i] = true;
				break;
			}
		}
		if (!present[i])
			additionalDim += newSpan;
	}
	
	// for new fields allocate
	const int oldDim(data.rows());
	const int totalDim(oldDim + additionalDim);
	const int pointCount(features.cols());
	data.conservativeResize(totalDim, pointCount);
	for (size_t i = 0; i < newLabels.size(); ++i)
	{
		if (!present[i])
			labels.push_back(newLabels[i]);
	}
}

//! Add a descriptor or feature by name, remove first if already exists
template<typename T>
template<typename MatrixType>
void PointMatcher<T>::DataPoints::addField(const std::string& name, const MatrixType& newField, Labels& labels, MatrixType& data) const
{
	const int newFieldDim = newField.rows();
	const int newPointCount = newField.cols();
	const int pointCount = features.cols();

	if (newField.rows() == 0)
		return;

	// Replace if the field exists
	if (fieldExists(name, 0, labels))
	{
		const int fieldDim = getFieldDimension(name, labels);
		
		if(fieldDim == newFieldDim)
		{
			// Ensure that the number of points in the point cloud and in the field are the same
			if(pointCount == newPointCount)
			{
				const int row = getFieldStartingRow(name, labels);
				data.block(row, 0, fieldDim, pointCount) = newField;
			}
			else
			{
				stringstream errorMsg;
				errorMsg << "The field " << name << " cannot be added because the number of points is not the same. Old point count: " << pointCount << "new: " << newPointCount;
				throw InvalidField(errorMsg.str());
			}
		}
		else
		{
			stringstream errorMsg;
			errorMsg << "The field " << name << " already exists but could not be added because the dimension is not the same. Old dim: " << fieldDim << " new: " << newFieldDim;
			throw InvalidField(errorMsg.str());
		}
	}
	else // Add at the end if it is a new field
	{
		if(pointCount == newPointCount || pointCount == 0)
		{
			const int oldFieldDim(data.rows());
			const int totalDim = oldFieldDim + newFieldDim;
			data.conservativeResize(totalDim, newPointCount);
			data.bottomRows(newFieldDim) = newField;
			labels.push_back(Label(name, newFieldDim));
		}
		else
		{
			stringstream errorMsg;
			errorMsg << "The field " << name << " cannot be added because the number of points is not the same. Old point count: " << pointCount << " new: " << newPointCount;
			throw InvalidField(errorMsg.str());
		}
	}
}
//! Remove a descriptor or feature by name, no copy is done.
template<typename T>
template<typename MatrixType>
void PointMatcher<T>::DataPoints::removeField(const std::string& name, Labels& labels, MatrixType& data) const
{

	const unsigned deleteId = getFieldStartingRow(name, labels);
	const unsigned span = getFieldDimension(name, labels);
	const unsigned keepAfterId = deleteId + span;
	const unsigned lastId = data.rows() - 1;
	const unsigned sizeKeep = data.rows() - keepAfterId;
	const unsigned nbPoints = data.cols();


	// check if the data to be removed at the end
	if(keepAfterId <= lastId)
	{
		data.block(deleteId, 0, sizeKeep, nbPoints) = data.block(keepAfterId, 0, sizeKeep, nbPoints);
	}

	//Remove the last rows
	data.conservativeResize(data.rows()-span, nbPoints);

	// remove label from the label list
	for(BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
		{
			labels.erase(it);
			break;
		}
	}

}


//! Get a const view on a matrix by name, throw an exception if it does not exist.
//! If viewRow is given, only return this row, otherwise return the full view
template<typename T>
template<typename MatrixType>
const typename Eigen::Block<const MatrixType> PointMatcher<T>::DataPoints::getConstViewByName(const std::string& name, const Labels& labels, const MatrixType& data, const int viewRow) const
{
	unsigned row(0);
	for(BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
		{
			if (viewRow >= 0)
			{
				if (viewRow >= int(it->span))
					throw InvalidField(
						(boost::format("Requesting row %1% of field %2% that only has %3% rows") % viewRow % name % it->span).str()
					);
				return data.block(row + viewRow, 0, 1, data.cols());
			}
			else
				return data.block(row, 0, it->span, data.cols());
		}
		row += it->span;
	}
	throw InvalidField("Field " + name + " not found");
}

//! Get a view on a matrix by name, throw an exception if it does not exist
//! If viewRow is given, only return this row, otherwise return the full view
template<typename T>
template<typename MatrixType>
typename Eigen::Block<MatrixType> PointMatcher<T>::DataPoints::getViewByName(const std::string& name, const Labels& labels, MatrixType& data, const int viewRow) const
{
	unsigned row(0);
	for(BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
		{
			if (viewRow >= 0)
			{
				if (viewRow >= int(it->span))
					throw InvalidField(
						(boost::format("Requesting row %1% of field %2% that only has %3% rows") % viewRow % name % it->span).str()
					);
				return data.block(row + viewRow, 0, 1, data.cols());
			}
			else
				return data.block(row, 0, it->span, data.cols());
		}
		row += it->span;
	}
	throw InvalidField("Field " + name + " not found");
}

//! Look if a descriptor or a feature with a given name and dimension exist
template<typename T>
bool PointMatcher<T>::DataPoints::fieldExists(const std::string& name, const unsigned dim, const Labels& labels) const
{
	for(BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
		{
			if (dim == 0 || it->span == dim)
				return true;
			else
				return false;
		}
	}
	return false;
}


//! Return the dimension of a feature or a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFieldDimension(const std::string& name, const Labels& labels) const
{
	for(BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
			return it->span;
	}
	return 0;
}


//! Return the starting row of a feature or a descriptor with a given name. Return 0 if the name is not found
template<typename T>
unsigned PointMatcher<T>::DataPoints::getFieldStartingRow(const std::string& name, const Labels& labels) const
{
	unsigned row(0);
	for(BOOST_AUTO(it, labels.begin()); it != labels.end(); ++it)
	{
		if (it->text == name)
			return row;
		row += it->span;
	}
	return 0;
}

template struct PointMatcher<float>::DataPoints;
template struct PointMatcher<double>::DataPoints;


//! Exchange in place point clouds a and b, with no data copy
template<typename T>
void PointMatcher<T>::swapDataPoints(DataPoints& a, DataPoints& b)
{
	a.features.swap(b.features);
	swap(a.featureLabels, b.featureLabels);
	a.descriptors.swap(b.descriptors);
	swap(a.descriptorLabels, b.descriptorLabels);
	a.times.swap(b.times);
	swap(a.timeLabels, b.timeLabels);
}

template
void PointMatcher<float>::swapDataPoints(DataPoints& a, DataPoints& b);
template
void PointMatcher<double>::swapDataPoints(DataPoints& a, DataPoints& b);
