// kate: replace-tabs off; indent-width 4; indent-mode 
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

#include "InspectorsImpl.h"

#include "PointMatcherPrivate.h"
#include "IOFunctions.h"

#include <cassert>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <boost/type_traits/is_same.hpp>

using namespace std;
using namespace PointMatcherSupport;

template<typename T>
InspectorsImpl<T>::PerformanceInspector::PerformanceInspector(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	Inspector(className,paramsDoc,params),
	baseFileName(Parametrizable::get<string>("baseFileName")),
	bDumpPerfOnExit(Parametrizable::get<bool>("dumpPerfOnExit")),
	bDumpStats(Parametrizable::get<bool>("dumpStats"))

{//FIXME: do we need that constructor?
}

template<typename T>
InspectorsImpl<T>::PerformanceInspector::PerformanceInspector(const Parameters& params):
	Inspector("PerformanceInspector", PerformanceInspector::availableParameters(), params),
	baseFileName(Parametrizable::get<string>("baseFileName")),
	bDumpPerfOnExit(Parametrizable::get<bool>("dumpPerfOnExit")),
	bDumpStats(Parametrizable::get<bool>("dumpStats"))
{}

template<typename T>
void InspectorsImpl<T>::PerformanceInspector::addStat(const std::string& name, double data)
{
	if (!bDumpStats) return;
	
	HistogramMap::iterator it(stats.find(name));
	if (it == stats.end()) {
		LOG_INFO_STREAM("Adding new stat: " << name);
		it = stats.insert(HistogramMap::value_type(name, Histogram(16, name, baseFileName, bDumpPerfOnExit))).first;
	}
	it->second.push_back(data);
}

template<typename T>
void InspectorsImpl<T>::PerformanceInspector::dumpStats(std::ostream& stream)
{
	// Note: this dump format will most probably change in the future
	for (BOOST_AUTO(it, stats.begin()); it != stats.end(); ++it)
	{
		it->second.dumpStats(stream);
		BOOST_AUTO(jt, it);
		++jt;
		if (jt != stats.end())
			stream << ", ";
	}
}

template<typename T>
void InspectorsImpl<T>::PerformanceInspector::dumpStatsHeader(std::ostream& stream)
{
	for (BOOST_AUTO(it, stats.begin()); it != stats.end(); ++it)
	{
		it->second.dumpStatsHeader(stream);
		BOOST_AUTO(jt, it);
		++jt;
		if (jt != stats.end())
			stream << ", ";
	}
}

template struct InspectorsImpl<float>::PerformanceInspector;
template struct InspectorsImpl<double>::PerformanceInspector;

/*
	// This code creates a header according to information from the datapoints
	std::ofstream ofs(fileName.c_str());
	size_t labelIndex(0);
	size_t labelComponent(0);
	const typename DataPoints::Labels labels(data.featureLabels);
	for (int i = 0; i < features.rows(); ++i)
	{
		// display label for row
		if (labelIndex <= labels.size())
		{
			const typename DataPoints::Label label(labels[labelIndex]);
			ofs << label.text;
			if (label.span > 1)
				ofs << "_" << labelComponent;
			++labelComponent;
			if (labelComponent >= label.span)
				++labelIndex;
		}
		else
		{
			ofs << "?";
		}
		ofs << " ";
	}*/
template<typename T>
InspectorsImpl<T>::AbstractVTKInspector::AbstractVTKInspector(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	PerformanceInspector(className,paramsDoc,params),
	streamIter(0),
	bDumpIterationInfo(Parametrizable::get<bool>("dumpIterationInfo")),
	bDumpDataLinks(Parametrizable::get<bool>("dumpDataLinks")),
	bDumpReading(Parametrizable::get<bool>("dumpReading")),
	bDumpReference(Parametrizable::get<bool>("dumpReference")),
	bWriteBinary(Parametrizable::get<bool>("writeBinary"))
{
}

template<typename T>
std::string getTypeName() {
	if (boost::is_same<double, T>::value) {
		return "double";
	} else {
		return "float";
	}
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::dumpDataPoints(const DataPoints& data, std::ostream& stream)
{
	const Matrix& features(data.features);
	//const Matrix& descriptors(data.descriptors);
	
	stream << "# vtk DataFile Version 3.0\n";
	stream << "File created by libpointmatcher\n";
	stream << (bWriteBinary ? "BINARY":"ASCII") << "\n";
	stream << "DATASET POLYDATA\n";

	stream << "POINTS " << features.cols() << " " << getTypeName<T>() << "\n";

	if(features.rows() == 4)
	{
		writeVtkData(bWriteBinary, features.topLeftCorner(3, features.cols()).transpose(), stream) << "\n";
	}
	else
	{
		writeVtkData(bWriteBinary, features.transpose(), stream)  << "\n";
	}
	
	stream << "VERTICES "  << features.cols() << " "<< features.cols() * 2 << "\n";
	for (int i = 0; i < features.cols(); ++i){
		if(bWriteBinary){
			stream.write(reinterpret_cast<const char*>(&oneBigEndian), sizeof(int));
			ConverterToAndFromBytes<int> converter(i);
			if(!isBigEndian){
				converter.swapBytes();
			}
			stream.write(converter.bytes, sizeof(int));
		}else {
			stream << "1 " << i << "\n";
		}
	}
	

	// Save points
	stream << "POINT_DATA " << features.cols() << "\n";

	// Loop through all descriptor and dispatch appropriate VTK tags
	for(BOOST_AUTO(it, data.descriptorLabels.begin()); it != data.descriptorLabels.end(); it++)
	{
		// handle specific cases
		if(it->text == "normals")
		{
			buildNormalStream(stream, "normals", data);
		}
		else if(it->text == "eigVectors")
		{
			buildTensorStream(stream, "eigVectors", data);
		}
		else if(it->text == "color")
		{
			buildColorStream(stream, "color", data);
		}
		// handle generic cases
		else if(it->span == 1)
		{
			buildScalarStream(stream, it->text, data);
		}
		else if(it->span == 3 || it->span == 2)
		{
			buildVectorStream(stream, it->text, data);
		}
		else
		{
			LOG_WARNING_STREAM("Could not save label named " << it->text << " (dim=" << it->span << ").");
		}
	}
	
	// Loop through all time fields, split in high 32 bits and low 32 bits and export as two scalar
	for(BOOST_AUTO(it, data.timeLabels.begin()); it != data.timeLabels.end(); it++)
	{
		buildTimeStream(stream, it->text, data);
	}

}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::dumpMeshNodes(const DataPoints& data, std::ostream& stream)
{
	//const Matrix& features(data.features);
	const Matrix& descriptors(data.descriptors.transpose());
	
	assert(descriptors.cols() >= 15);

	stream << "# vtk DataFile Version 3.0\n";
	stream << "Triangle mesh\n";
	stream << "ASCII\n";
	stream << "DATASET POLYDATA\n";

	stream << "POINTS " << descriptors.rows() * 3 << " float\n"; // not optimal: points and edges are drawn several times!
	for (int i = 0; i < descriptors.rows(); i++)
	{
		// TODO: use getDescriptorByName(nameTag) to access blocks 
		stream << descriptors.block(i, 3, 1, 3) << "\n";
		stream << descriptors.block(i, 6, 1, 3) << "\n";
		stream << descriptors.block(i, 9, 1, 3) << "\n";
	}

	//TODO: add centroids...
	/*
	stream << features.transpose() << "\n";
	*/

	stream << "POLYGONS " << descriptors.rows() << " " << descriptors.rows() * 4 << "\n";
	for (int i = 0; i < descriptors.rows(); i++)
	{
		stream << "3 " << (i*3) << " " << (i*3 + 1) << " " << (i*3 + 2) << "\n";
	}
	
	stream << "CELL_DATA " << descriptors.rows() << "\n";

	stream << "NORMALS triangle_normals float\n";
	stream << descriptors.block(0, 0, descriptors.rows(), 3) << "\n";

	// TODO: use descriptor labels, particularly to represent normals
	//stream << "POINT_DATA " << descriptors.rows() << "\n";
	//buildScalarStream(stream, "densities", data);
	//buildNormalStream(stream, "normals", data);
	//buildVectorStream(stream, "eigValues", data);
	//buildTensorStream(stream, "eigVectors", data);
	//buildVectorStream(stream, "observationDirections", data);
}

// FIXME:rethink how we dump stuff (accumulate in a correctly-referenced table, and then dump?) and unify with previous
template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::dumpDataLinks(
	const DataPoints& ref, 
	const DataPoints& reading, 
	const Matches& matches, 
	const OutlierWeights& featureOutlierWeights, 
	std::ostream& stream)
{

	const Matrix& refFeatures(ref.features);
	const int refPtCount(refFeatures.cols());
	//const int featDim(refFeatures.rows());
	const Matrix& readingFeatures(reading.features);
	const int readingPtCount(readingFeatures.cols());
	const int totalPtCount(refPtCount+readingPtCount);
	
	stream << "# vtk DataFile Version 3.0\n";
	stream << "comment\n";
	stream << "ASCII\n";
	stream << "DATASET POLYDATA\n";
	
	stream << "POINTS " << totalPtCount << " float\n";
	if(refFeatures.rows() == 4)
	{
		// reference pt
		stream << refFeatures.topLeftCorner(3, refFeatures.cols()).transpose() << "\n";
		// reading pt
		stream << readingFeatures.topLeftCorner(3, readingFeatures.cols()).transpose() << "\n";
	}
	else
	{
		// reference pt
		stream << refFeatures.transpose() << "\n";
		// reading pt
		stream << readingFeatures.transpose() << "\n";
	}
	const int knn = matches.ids.rows();
	
	size_t matchCount = readingPtCount*knn;
	for (int k = 0; k < knn; ++k)
	{
		for (int i = 0; i < readingPtCount; ++i)
		{
			if (matches.ids(k, i) == Matches::InvalidId){
				matchCount --;
			}
		}
	}

	stream << "LINES " << matchCount << " "  << matchCount * 3 << "\n";
	//int j = 0;
	for (int k = 0; k < knn; k++) // knn
	{
		for (int i = 0; i < readingPtCount; ++i)
		{
			const auto id = matches.ids(k, i);
			if (id != Matches::InvalidId){
				stream << "2 " << refPtCount + i << " " << id << "\n";
			}
		}
	}

	stream << "CELL_DATA " << matchCount << "\n";
	stream << "SCALARS outlier float 1\n";
	stream << "LOOKUP_TABLE default\n";
	//stream << "LOOKUP_TABLE alphaOutlier\n";
	for (int k = 0; k < knn; k++) // knn
	{
		for (int i = 0; i < readingPtCount; ++i) //nb pts
		{
			const auto id = matches.ids(k, i);
			if (id != Matches::InvalidId){
				stream << featureOutlierWeights(k, i) << "\n";
			}
		}
	}

	//stream << "LOOKUP_TABLE alphaOutlier 2\n";
	//stream << "1 0 0 0.5\n";
	//stream << "0 1 0 1\n";

}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::dumpDataPoints(const DataPoints& filteredReference, const std::string& name)
{
	ostream* stream(openStream(name));
	dumpDataPoints(filteredReference, *stream);
	closeStream(stream);
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::dumpMeshNodes(const DataPoints& filteredReference, const std::string& name)
{
	ostream* stream(openStream(name));
	dumpMeshNodes(filteredReference, *stream);
	closeStream(stream);
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::dumpIteration(
	const size_t iterationNumber,
	const TransformationParameters& parameters,
	const DataPoints& filteredReference,
	const DataPoints& reading,
	const Matches& matches,
	const OutlierWeights& outlierWeights, 
	const TransformationCheckers& transCheck)
{

	if (bDumpDataLinks){
		ostream* streamLinks(openStream("link", iterationNumber));
		dumpDataLinks(filteredReference, reading, matches, outlierWeights, *streamLinks);
		closeStream(streamLinks);
	}
	
	if (bDumpReading){
		ostream* streamRead(openStream("reading", iterationNumber));
		dumpDataPoints(reading, *streamRead);
		closeStream(streamRead);
	}
	
	if (bDumpReference){
		ostream* streamRef(openStream("reference", iterationNumber));
		dumpDataPoints(filteredReference, *streamRef);
		closeStream(streamRef);
	}
        
	if (!bDumpIterationInfo) return;

	// streamIter must be define by children
	assert(streamIter);

	if(iterationNumber == 0)
	{
		//Build header
		for(unsigned int j = 0; j < transCheck.size(); j++)
		{
			for(unsigned int i=0; i < transCheck[j]->getConditionVariableNames().size(); i++)
			{
				if (!(j == 0 && i == 0))
					*streamIter << ", ";
				*streamIter << transCheck[j]->getConditionVariableNames()[i] << ", "; 
				*streamIter << transCheck[j]->getLimitNames()[i]; 
			}
		}
		
		*streamIter << "\n";
	}

	
	for(unsigned int j = 0; j < transCheck.size(); j++)
	{
		for(unsigned int i=0; i < transCheck[j]->getConditionVariables().size(); i++)
		{
		
			if (!(j == 0 && i == 0))
				*streamIter << ", ";

			*streamIter << transCheck[j]->getConditionVariables()[i] << ", ";
			*streamIter << transCheck[j]->getLimits()[i]; 
		}
	}

	*streamIter << "\n";
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildGenericAttributeStream(std::ostream& stream, const std::string& attribute, const std::string& nameTag, const DataPoints& cloud, const int forcedDim)
{
	if (!cloud.descriptorExists(nameTag))
		return;
		
	const BOOST_AUTO(desc, cloud.getDescriptorViewByName(nameTag));
	assert(desc.rows() <= forcedDim);

	if(desc.rows() != 0)
	{
		if(attribute.compare("COLOR_SCALARS") == 0)
		{
			stream << attribute << " " << nameTag << " " << forcedDim << "\n";
			if(bWriteBinary){
				std::vector<unsigned char> buffer(forcedDim, 0);
				for (int i = 0; i < desc.cols(); ++i){
					for(int r=0; r < desc.rows(); ++r){
						buffer[r] = static_cast<unsigned int>(desc(r, i) * static_cast<T>(255) + static_cast<T>(0.5)); // this is how libvtk implements it ( vtkScalarsToColors::ColorToUChar )
					}
					stream.write(reinterpret_cast<char *>(&buffer.front()), forcedDim);
				}
			}
			else {
				stream << padWithOnes(desc, forcedDim, desc.cols()).transpose();
			}
		}
		else
		{
			stream << attribute << " " << nameTag << " " << getTypeName<T>() << "\n";
			if(attribute.compare("SCALARS") == 0)
				stream << "LOOKUP_TABLE default\n";

			writeVtkData(bWriteBinary, padWithZeros(desc, forcedDim, desc.cols()).transpose(), stream);
		}
		stream << "\n";
	}
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildScalarStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& cloud)
{
	buildGenericAttributeStream(stream, "SCALARS", name, cloud, 1);
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildNormalStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& cloud)
{
	buildGenericAttributeStream(stream, "NORMALS", name, cloud, 3);
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildVectorStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& cloud)
{
	buildGenericAttributeStream(stream, "VECTORS", name, cloud, 3);
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildTensorStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& cloud)
{
	buildGenericAttributeStream(stream, "TENSORS", name, cloud, 9);
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildColorStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& cloud)
{
	buildGenericAttributeStream(stream, "COLOR_SCALARS", name, cloud, 4);
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildScalarStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& ref, 
	const DataPoints& reading)
{
			
	const Matrix descRef(ref.getDescriptorByName(name));	
	const Matrix descRead(reading.getDescriptorByName(name));

	if(descRef.rows() != 0 && descRead.rows() != 0)
	{
		stream << "SCALARS " << name << " float\n";
		stream << "LOOKUP_TABLE default\n";
		
		stream << padWithZeros(
				descRef, 1, ref.descriptors.cols()).transpose();
		stream << "\n";
		stream << padWithZeros(
				descRead, 1, reading.descriptors.cols()).transpose();
		stream << "\n";
	}
}


template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildNormalStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& ref, 
	const DataPoints& reading)
{
			
	const Matrix descRef(ref.getDescriptorByName(name));	
	const Matrix descRead(reading.getDescriptorByName(name));

	if(descRef.rows() != 0 && descRead.rows() != 0)
	{
		stream << "NORMALS " << name << " float\n";

		stream << padWithZeros(
				descRef, 3, ref.descriptors.cols()).transpose();
		stream << "\n";
		stream << padWithZeros(
				descRead, 3, reading.descriptors.cols()).transpose();
		stream << "\n";
	}
}


template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildVectorStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& ref, 
	const DataPoints& reading)
{
			
	const Matrix descRef(ref.getDescriptorByName(name));	
	const Matrix descRead(reading.getDescriptorByName(name));

	if(descRef.rows() != 0 && descRead.rows() != 0)
	{
		stream << "VECTORS " << name << " " << getTypeName<T>() << "\n";

		stream << padWithZeros(
				descRef, 3, ref.descriptors.cols()).transpose();
		stream << "\n";
		stream << padWithZeros(
				descRead, 3, reading.descriptors.cols()).transpose();
		stream << "\n";
	}
}


template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildTensorStream(std::ostream& stream,
	const std::string& name,
	const DataPoints& ref, 
	const DataPoints& reading)
{
			
	const Matrix descRef(ref.getDescriptorByName(name));	
	const Matrix descRead(reading.getDescriptorByName(name));

	if(descRef.rows() != 0 && descRead.rows() != 0)
	{
		stream << "TENSORS " << name << " float\n";

		stream << padWithZeros(
				descRef, 9, ref.descriptors.cols()).transpose();
		stream << "\n";
		stream << padWithZeros(
				descRead, 9, reading.descriptors.cols()).transpose();
		stream << "\n";
	}
}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::buildTimeStream(std::ostream& stream, const std::string& name, const DataPoints& cloud)
{
	//TODO: this check is a reminder of the old implementation. Check
	// if we still need that. FP
	if (!cloud.timeExists(name))
		return;
		
	const BOOST_AUTO(time, cloud.getTimeViewByName(name));
	assert(time.rows() == 1);

	// Loop through the array to split the lower and higher part of int64_t
	// TODO: if an Eigen matrix operator can do it without loop, change that

	Eigen::Matrix<uint32_t, 1, Eigen::Dynamic> high32(time.cols());
	Eigen::Matrix<uint32_t, 1, Eigen::Dynamic> low32(time.cols());

	for(int i=0; i<time.cols(); i++)
	{
		high32(0, i) = (uint32_t)(time(0, i) >> 32);
		low32(0, i) = (uint32_t)time(0, i);
	}
	
	stream << "SCALARS" << " " << name << "_splitTime_high32" << " " << "unsigned_int" << "\n";
	stream << "LOOKUP_TABLE default\n";

	writeVtkData(bWriteBinary, high32.transpose(), stream);

	stream << "\n";

	stream << "SCALARS" << " " << name << "_splitTime_low32" << " " << "unsigned_int" << "\n";
	stream << "LOOKUP_TABLE default\n";

	writeVtkData(bWriteBinary, low32.transpose(), stream);

	stream << "\n";
}

template<typename T>
typename PointMatcher<T>::Matrix InspectorsImpl<T>::AbstractVTKInspector::padWithZeros(
	const Matrix m,
	const int expectedRow,
	const int expectedCols)
{
	assert(m.cols() <= expectedCols || m.rows() <= expectedRow);
	if(m.cols() == expectedCols && m.rows() == expectedRow)
	{
		return m;
	}
	else
	{
		Matrix tmp = Matrix::Zero(expectedRow, expectedCols); 
		tmp.topLeftCorner(m.rows(), m.cols()) = m;
		return tmp;
	}

}

template<typename T>
typename PointMatcher<T>::Matrix InspectorsImpl<T>::AbstractVTKInspector::padWithOnes(
	const Matrix m,
	const int expectedRow,
	const int expectedCols)
{
	assert(m.cols() <= expectedCols || m.rows() <= expectedRow);
	if(m.cols() == expectedCols && m.rows() == expectedRow)
	{
		return m;
	}
	else
	{
		Matrix tmp = Matrix::Ones(expectedRow, expectedCols); 
		tmp.topLeftCorner(m.rows(), m.cols()) = m;
		return tmp;
	}

}

template<typename T>
void InspectorsImpl<T>::AbstractVTKInspector::finish(const size_t iterationCount)
{
}


//-----------------------------------
// VTK File inspector

template<typename T>
InspectorsImpl<T>::VTKFileInspector::VTKFileInspector(const Parameters& params):
	AbstractVTKInspector("VTKFileInspector", VTKFileInspector::availableParameters(), params),
	baseFileName(Parametrizable::get<string>("baseFileName")),
	bDumpIterationInfo(Parametrizable::get<bool>("dumpIterationInfo")),
	bDumpDataLinks(Parametrizable::get<bool>("dumpDataLinks")),
	bDumpReading(Parametrizable::get<bool>("dumpReading")),
	bDumpReference(Parametrizable::get<bool>("dumpReference"))
{
}

template<typename T>
void InspectorsImpl<T>::VTKFileInspector::init()
{

	if (!bDumpIterationInfo) return;
 
	ostringstream oss;
	oss << baseFileName << "-iterationInfo.csv";
	//std::cerr << "writing to " << oss.str() << std::endl;
	LOG_INFO_STREAM("writing to " << oss.str());

	this->streamIter = new ofstream(oss.str().c_str());
	if (this->streamIter->fail())
		throw std::runtime_error("Couldn't open the file \"" + oss.str() + "\". Check if directory exist.");
	
}

template<typename T>
void InspectorsImpl<T>::VTKFileInspector::finish(const size_t iterationCount)
{
	if (!bDumpIterationInfo) return;
	closeStream(this->streamIter);
}

template<typename T>
std::ostream* InspectorsImpl<T>::VTKFileInspector::openStream(const std::string& role)
{
	string filteredStr = role;
	if(role.substr(role.size()-4,4) == ".vtk")
		filteredStr = role.substr(0, role.size()-4);

	ostringstream oss;
	if(baseFileName != "")
		oss << baseFileName << "-" << filteredStr << ".vtk";
	else
		oss << filteredStr << ".vtk";

	//std::cerr << "writing to " << oss.str() << std::endl;
	LOG_INFO_STREAM("writing to " << oss.str());
	ofstream* file = new ofstream(oss.str().c_str(), std::ios::binary);
	if (file->fail())
		throw std::runtime_error("Couldn't open the file \"" + oss.str() + "\". Check if directory exist.");
	return file;
}

template<typename T>
std::ostream* InspectorsImpl<T>::VTKFileInspector::openStream(const std::string& role, const size_t iterationNumber)
{
	ostringstream oss;
	oss << baseFileName << "-" << role << "-" << iterationNumber << ".vtk";
	ofstream* file = new ofstream(oss.str().c_str());
	if (file->fail())
		throw std::runtime_error("Couldn't open the file \"" + oss.str() + "\". Check if directory exist.");
	return file;
}

template<typename T>
void InspectorsImpl<T>::VTKFileInspector::closeStream(std::ostream* stream)
{
	delete stream;
}



template struct InspectorsImpl<float>::VTKFileInspector;
template struct InspectorsImpl<double>::VTKFileInspector;
