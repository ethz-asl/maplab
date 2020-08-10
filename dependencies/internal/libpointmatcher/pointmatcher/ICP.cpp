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
#include "Timer.h"

#include "LoggerImpl.h"
#include "TransformationsImpl.h"
#include "DataPointsFiltersImpl.h"
#include "MatchersImpl.h"
#include "OutlierFiltersImpl.h"
#include "ErrorMinimizersImpl.h"
#include "TransformationCheckersImpl.h"
#include "InspectorsImpl.h"

#ifdef SYSTEM_YAML_CPP
    #include "yaml-cpp/yaml.h"
#else
	#include "yaml-cpp-pm/yaml.h"
#endif // HAVE_YAML_CPP

using namespace std;
using namespace PointMatcherSupport;

//! Construct an invalid--module-type exception
InvalidModuleType::InvalidModuleType(const std::string& reason):
	runtime_error(reason)
{}

//! Protected contstructor, to prevent the creation of this object
template<typename T>
PointMatcher<T>::ICPChainBase::ICPChainBase():
	prefilteredReadingPtsCount(0),
	prefilteredReferencePtsCount(0),
	maxNumIterationsReached(false)
{}

//! virtual desctructor
template<typename T>
PointMatcher<T>::ICPChainBase::~ICPChainBase()
{
}

//! Clean chain up, empty all filters and delete associated objects
template<typename T>
void PointMatcher<T>::ICPChainBase::cleanup()
{
	transformations.clear();
	readingDataPointsFilters.clear();
	readingStepDataPointsFilters.clear();
	referenceDataPointsFilters.clear();
	matcher.reset();
	outlierFilters.clear();
	errorMinimizer.reset();
	transformationCheckers.clear();
	inspector.reset();
}

//! Hook to load addition subclass-specific content from the YAML file
template<typename T>
void PointMatcher<T>::ICPChainBase::loadAdditionalYAMLContent(YAML::Node& doc)
{
}

//! Construct an ICP algorithm that works in most of the cases
template<typename T>
void PointMatcher<T>::ICPChainBase::setDefault()
{
	this->cleanup();
	
	this->transformations.push_back(std::make_shared<typename TransformationsImpl<T>::RigidTransformation>());
	this->readingDataPointsFilters.push_back(std::make_shared<typename DataPointsFiltersImpl<T>::RandomSamplingDataPointsFilter>());
	this->referenceDataPointsFilters.push_back(std::make_shared<typename DataPointsFiltersImpl<T>::SamplingSurfaceNormalDataPointsFilter>());
	this->outlierFilters.push_back(std::make_shared<typename OutlierFiltersImpl<T>::TrimmedDistOutlierFilter>());
	this->matcher = std::make_shared<typename MatchersImpl<T>::KDTreeMatcher>();
	this->errorMinimizer = std::make_shared<PointToPlaneErrorMinimizer<T> >();
	this->transformationCheckers.push_back(std::make_shared<typename TransformationCheckersImpl<T>::CounterTransformationChecker>());
	this->transformationCheckers.push_back(std::make_shared<typename TransformationCheckersImpl<T>::DifferentialTransformationChecker>());
	this->inspector = std::make_shared<typename InspectorsImpl<T>::NullInspector>();
}

//! Construct an ICP algorithm from a YAML file
template<typename T>
void PointMatcher<T>::ICPChainBase::loadFromYaml(std::istream& in)
{
	this->cleanup();
	
	YAML::Parser parser(in);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	typedef set<string> StringSet;
	StringSet usedModuleTypes;
	
	// Fix for issue #6: compilation on gcc 4.4.4
	//PointMatcher<T> pm;
	const PointMatcher & pm = PointMatcher::get();

	{
		// NOTE: The logger needs to be initialize first to allow ouput from other contructors
		boost::mutex::scoped_lock lock(loggerMutex);
		usedModuleTypes.insert(createModuleFromRegistrar("logger", doc, pm.REG(Logger), logger));
	}
	usedModuleTypes.insert(createModulesFromRegistrar("readingDataPointsFilters", doc, pm.REG(DataPointsFilter), readingDataPointsFilters));
	usedModuleTypes.insert(createModulesFromRegistrar("readingStepDataPointsFilters", doc, pm.REG(DataPointsFilter), readingStepDataPointsFilters));
	usedModuleTypes.insert(createModulesFromRegistrar("referenceDataPointsFilters", doc, pm.REG(DataPointsFilter), referenceDataPointsFilters));
	//usedModuleTypes.insert(createModulesFromRegistrar("transformations", doc, pm.REG(Transformation), transformations));
	usedModuleTypes.insert(createModuleFromRegistrar("matcher", doc, pm.REG(Matcher), matcher));
	usedModuleTypes.insert(createModulesFromRegistrar("outlierFilters", doc, pm.REG(OutlierFilter), outlierFilters));
	usedModuleTypes.insert(createModuleFromRegistrar("errorMinimizer", doc, pm.REG(ErrorMinimizer), errorMinimizer));

	// See if to use a rigid transformation
	if (nodeVal("errorMinimizer", doc) != "PointToPointSimilarityErrorMinimizer")
		this->transformations.push_back(std::make_shared<typename TransformationsImpl<T>::RigidTransformation>());
	else
		this->transformations.push_back(std::make_shared<typename TransformationsImpl<T>::SimilarityTransformation>());
	
	usedModuleTypes.insert(createModulesFromRegistrar("transformationCheckers", doc, pm.REG(TransformationChecker), transformationCheckers));
	usedModuleTypes.insert(createModuleFromRegistrar("inspector", doc, pm.REG(Inspector),inspector));
	
	
	// FIXME: this line cause segfault when there is an error in the yaml file...
	//loadAdditionalYAMLContent(doc);
	
	// check YAML entries that do not correspend to any module
	for(YAML::Iterator moduleTypeIt = doc.begin(); moduleTypeIt != doc.end(); ++moduleTypeIt)
	{
		string moduleType;
		moduleTypeIt.first() >> moduleType;
		if (usedModuleTypes.find(moduleType) == usedModuleTypes.end())
			throw InvalidModuleType(
				(boost::format("Module type %1% does not exist") % moduleType).str()
			);
	}
}

//! Return the remaining number of points in reading after prefiltering but before the iterative process
template<typename T>
unsigned PointMatcher<T>::ICPChainBase::getPrefilteredReadingPtsCount() const
{
	return prefilteredReadingPtsCount;
}

//! Return the remaining number of points in the reference after prefiltering but before the iterative process
template<typename T>
unsigned PointMatcher<T>::ICPChainBase::getPrefilteredReferencePtsCount() const
{
	return prefilteredReferencePtsCount;
}

//! Return the flag that informs if we reached the maximum number of iterations during the last iterative process
template<typename T>
bool PointMatcher<T>::ICPChainBase::getMaxNumIterationsReached() const
{
	return maxNumIterationsReached;
}

//! Instantiate modules if their names are in the YAML file
template<typename T>
template<typename R>
const std::string& PointMatcher<T>::ICPChainBase::createModulesFromRegistrar(const std::string& regName, const YAML::Node& doc, const R& registrar, std::vector<std::shared_ptr<typename R::TargetType> >& modules)
{
	const YAML::Node *reg = doc.FindValue(regName);
	if (reg)
	{
		//cout << regName << endl;
		for(YAML::Iterator moduleIt = reg->begin(); moduleIt != reg->end(); ++moduleIt)
		{
			const YAML::Node& module(*moduleIt);
			modules.push_back(registrar.createFromYAML(module));
		}
	}
	return regName;
}

//! Instantiate a module if its name is in the YAML file
template<typename T>
template<typename R>
const std::string& PointMatcher<T>::ICPChainBase::createModuleFromRegistrar(const std::string& regName, const YAML::Node& doc, const R& registrar, std::shared_ptr<typename R::TargetType>& module)
{
	const YAML::Node *reg = doc.FindValue(regName);
	if (reg)
	{
		//cout << regName << endl;
		module = registrar.createFromYAML(*reg);
	}
	else
		module.reset();
	return regName;
}

template<typename T>
std::string PointMatcher<T>::ICPChainBase::nodeVal(const std::string& regName, const PointMatcherSupport::YAML::Node& doc)
{
	const YAML::Node *reg = doc.FindValue(regName);
	if (reg)
	{
		std::string name;
		Parametrizable::Parameters params;
		PointMatcherSupport::getNameParamsFromYAML(*reg, name, params);
		return name;
	}
	return "";
}

template struct PointMatcher<float>::ICPChainBase;
template struct PointMatcher<double>::ICPChainBase;


//! Perform ICP and return optimised transformation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::operator ()(
	const DataPoints& readingIn,
	const DataPoints& referenceIn)
{
	const int dim = readingIn.features.rows();
	const TransformationParameters identity = TransformationParameters::Identity(dim, dim);
	return this->compute(readingIn, referenceIn, identity);
}

//! Perform ICP from initial guess and return optimised transformation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::operator ()(
	const DataPoints& readingIn,
	const DataPoints& referenceIn,
	const TransformationParameters& initialTransformationParameters)
{
	return this->compute(readingIn, referenceIn, initialTransformationParameters);
}

//! Perform ICP from initial guess and return optimised transformation matrix
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::compute(
	const DataPoints& readingIn,
	const DataPoints& referenceIn,
	const TransformationParameters& T_refIn_dataIn)
{
	// Ensuring minimum definition of components
	if (!this->matcher)
		throw runtime_error("You must setup a matcher before running ICP");
	if (!this->errorMinimizer)
		throw runtime_error("You must setup an error minimizer before running ICP");
	if (!this->inspector)
		throw runtime_error("You must setup an inspector before running ICP");
	
	this->inspector->init();
	
	timer t; // Print how long take the algo
	const int dim(referenceIn.features.rows());
	
	// Apply reference filters
	// reference is express in frame <refIn>
	DataPoints reference(referenceIn);
	this->referenceDataPointsFilters.init();
	this->referenceDataPointsFilters.apply(reference);
	
	// Create intermediate frame at the center of mass of reference pts cloud
	//  this help to solve for rotations
	const int nbPtsReference = reference.features.cols();
	const Vector meanReference = reference.features.rowwise().sum() / nbPtsReference;
	TransformationParameters T_refIn_refMean(Matrix::Identity(dim, dim));
	T_refIn_refMean.block(0,dim-1, dim-1, 1) = meanReference.head(dim-1);
	
	// Reajust reference position: 
	// from here reference is express in frame <refMean>
	// Shortcut to do T_refIn_refMean.inverse() * reference
	reference.features.topRows(dim-1).colwise() -= meanReference.head(dim-1);
	
	// Init matcher with reference points center on its mean
	this->matcher->init(reference);
	
	// statistics on last step
	this->inspector->addStat("ReferencePreprocessingDuration", t.elapsed());
	this->inspector->addStat("ReferenceInPointCount", referenceIn.features.cols());
	this->inspector->addStat("ReferencePointCount", reference.features.cols());
	LOG_INFO_STREAM("PointMatcher::icp - reference pre-processing took " << t.elapsed() << " [s]");
	this->prefilteredReferencePtsCount = reference.features.cols();
	
	return computeWithTransformedReference(readingIn, reference, T_refIn_refMean, T_refIn_dataIn);
	
}

//! Perferm ICP using an already-transformed reference and with an already-initialized matcher
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICP::computeWithTransformedReference(
	const DataPoints& readingIn, 
	const DataPoints& reference, 
	const TransformationParameters& T_refIn_refMean,
	const TransformationParameters& T_refIn_dataIn)
{
	const int dim(reference.features.rows());

	if (T_refIn_dataIn.cols() != T_refIn_dataIn.rows()) {
		throw runtime_error("The initial transformation matrix must be squared.");
	}
	if (dim != T_refIn_dataIn.cols()) {
		throw runtime_error("The shape of initial transformation matrix must be NxN. "
											  "Where N is the number of rows in the read/reference scans.");
	}

	timer t; // Print how long take the algo
	
	// Apply readings filters
	// reading is express in frame <dataIn>
	DataPoints reading(readingIn);
	//const int nbPtsReading = reading.features.cols();
	this->readingDataPointsFilters.init();
	this->readingDataPointsFilters.apply(reading);
	readingFiltered = reading;

	// Reajust reading position: 
	// from here reading is express in frame <refMean>
	TransformationParameters 
		T_refMean_dataIn = T_refIn_refMean.inverse() * T_refIn_dataIn;
	this->transformations.apply(reading, T_refMean_dataIn);
	
	// Prepare reading filters used in the loop 
	this->readingStepDataPointsFilters.init();
	
	// Since reading and reference are express in <refMean>
	// the frame <refMean> is equivalent to the frame <iter(0)>
	TransformationParameters T_iter = Matrix::Identity(dim, dim);
	
	bool iterate(true);
	this->maxNumIterationsReached = false;
	this->transformationCheckers.init(T_iter, iterate);

	size_t iterationCount(0);
	
	// statistics on last step
	this->inspector->addStat("ReadingPreprocessingDuration", t.elapsed());
	this->inspector->addStat("ReadingInPointCount", readingIn.features.cols());
	this->inspector->addStat("ReadingPointCount", reading.features.cols());
	LOG_INFO_STREAM("PointMatcher::icp - reading pre-processing took " << t.elapsed() << " [s]");
	this->prefilteredReadingPtsCount = reading.features.cols();
	t.restart();
	
	// iterations
	while (iterate)
	{
		DataPoints stepReading(reading);
		
		//-----------------------------
		// Apply step filter
		this->readingStepDataPointsFilters.apply(stepReading);
		
		//-----------------------------
		// Transform Readings
		this->transformations.apply(stepReading, T_iter);
		
		//-----------------------------
		// Match to closest point in Reference
		const Matches matches(
			this->matcher->findClosests(stepReading)
		);
		
		//-----------------------------
		// Detect outliers
		const OutlierWeights outlierWeights(
			this->outlierFilters.compute(stepReading, reference, matches)
		);
		
		assert(outlierWeights.rows() == matches.ids.rows());
		assert(outlierWeights.cols() == matches.ids.cols());
		
		//cout << "outlierWeights: " << outlierWeights << "\n";
	
		
		//-----------------------------
		// Dump
		this->inspector->dumpIteration(
			iterationCount, T_iter, reference, stepReading, matches, outlierWeights, this->transformationCheckers
		);
		
		//-----------------------------
		// Error minimization
		// equivalent to: 
		//   T_iter(i+1)_iter(0) = T_iter(i+1)_iter(i) * T_iter(i)_iter(0)
		T_iter = this->errorMinimizer->compute(
			stepReading, reference, outlierWeights, matches) * T_iter;
		
		// Old version
		//T_iter = T_iter * this->errorMinimizer->compute(
		//	stepReading, reference, outlierWeights, matches);
		
		// in test
		try
		{
			this->transformationCheckers.check(T_iter, iterate);
		}
		catch(const typename TransformationCheckersImpl<T>::CounterTransformationChecker::MaxNumIterationsReached & e)
		{
			iterate = false;
			this->maxNumIterationsReached = true;
		}
	
		++iterationCount;
	}
	
	this->inspector->addStat("IterationsCount", iterationCount);
	this->inspector->addStat("PointCountTouched", this->matcher->getVisitCount());
	this->matcher->resetVisitCount();
	this->inspector->addStat("OverlapRatio", this->errorMinimizer->getWeightedPointUsedRatio());
	this->inspector->addStat("ConvergenceDuration", t.elapsed());
	this->inspector->finish(iterationCount);
	
	LOG_INFO_STREAM("PointMatcher::icp - " << iterationCount << " iterations took " << t.elapsed() << " [s]");
	
	// Move transformation back to original coordinate (without center of mass)
	// T_iter is equivalent to: T_iter(i+1)_iter(0)
	// the frame <iter(0)> equals <refMean>
	// so we have: 
	//   T_iter(i+1)_dataIn = T_iter(i+1)_iter(0) * T_refMean_dataIn
	//   T_iter(i+1)_dataIn = T_iter(i+1)_iter(0) * T_iter(0)_dataIn
	// T_refIn_refMean remove the temperary frame added during initialization
	return (T_refIn_refMean * T_iter * T_refMean_dataIn);
}

template struct PointMatcher<float>::ICP;
template struct PointMatcher<double>::ICP;


//! Return whether the object currently holds a valid map
template<typename T>
bool PointMatcher<T>::ICPSequence::hasMap() const
{
	return (mapPointCloud.features.cols() != 0);
}

//! Set the map using inputCloud
template<typename T>
bool PointMatcher<T>::ICPSequence::setMap(const DataPoints& inputCloud)
{
	// Ensuring minimum definition of components
	if (!this->matcher)
		throw runtime_error("You must setup a matcher before running ICP");
	if (!this->inspector)
		throw runtime_error("You must setup an inspector before running ICP");
	
	timer t; // Print how long take the algo
	const int dim(inputCloud.features.rows());
	const int ptCount(inputCloud.features.cols());
	
	// update keyframe
	if (ptCount == 0)
	{
		LOG_WARNING_STREAM("Ignoring attempt to create a map from an empty cloud");
		return false;
	}
	
	this->inspector->addStat("MapPointCount", inputCloud.features.cols());
	
	// Set map
	mapPointCloud = inputCloud;

	// Create intermediate frame at the center of mass of reference pts cloud
	//  this help to solve for rotations
	const Vector meanMap = mapPointCloud.features.rowwise().sum() / ptCount;
	T_refIn_refMean = Matrix::Identity(dim, dim);
	T_refIn_refMean.block(0,dim-1, dim-1, 1) = meanMap.head(dim-1);
	
	// Reajust reference position (only translations): 
	// from here reference is express in frame <refMean>
	// Shortcut to do T_refIn_refMean.inverse() * reference
	mapPointCloud.features.topRows(dim-1).colwise() -= meanMap.head(dim-1);

	// Apply reference filters
	this->referenceDataPointsFilters.init();
	this->referenceDataPointsFilters.apply(mapPointCloud);
	
	this->matcher->init(mapPointCloud);
	
	this->inspector->addStat("SetMapDuration", t.elapsed());
	
	return true;
}

//! Clear the map (reset to same state as after the object is created)
template<typename T>
void PointMatcher<T>::ICPSequence::clearMap()
{
	const int dim(mapPointCloud.features.rows());
	T_refIn_refMean = Matrix::Identity(dim, dim);
	mapPointCloud = DataPoints();
}

template<typename T>
void PointMatcher<T>::ICPSequence::setDefault()
{
	ICPChainBase::setDefault();
	
	if(mapPointCloud.getNbPoints() > 0)
	{
		this->matcher->init(mapPointCloud);
	}
}

template<typename T>
void PointMatcher<T>::ICPSequence::loadFromYaml(std::istream& in)
{
	ICPChainBase::loadFromYaml(in);
	
	if(mapPointCloud.getNbPoints() > 0)
	{
		this->matcher->init(mapPointCloud);
	}
}

//! Return the map, in global coordinates (slow)
template<typename T>
const typename PointMatcher<T>::DataPoints PointMatcher<T>::ICPSequence::getPrefilteredMap() const
{
	DataPoints globalMap(mapPointCloud);
	if(this->hasMap())
	{
		const int dim(mapPointCloud.features.rows());
		const Vector meanMapNonHomo(T_refIn_refMean.block(0,dim-1, dim-1, 1));
		globalMap.features.topRows(dim-1).colwise() += meanMapNonHomo;
	}

	return globalMap;
}

//! Return the map, in global coordinates (slow). Deprecated in favor of getPrefilteredMap()
template<typename T>
const typename PointMatcher<T>::DataPoints PointMatcher<T>::ICPSequence::getMap() const {
	return PointMatcher<T>::ICPSequence::getPrefilteredMap();
}

//! Return the map, in internal coordinates (fast)
template<typename T>
const typename PointMatcher<T>::DataPoints& PointMatcher<T>::ICPSequence::getPrefilteredInternalMap() const
{
	return mapPointCloud;
}

//! Return the map, in internal coordinates (fast). Deprecated in favor of getPrefilteredInternalMap().
template<typename T>
const typename PointMatcher<T>::DataPoints& PointMatcher<T>::ICPSequence::getInternalMap() const {
	return PointMatcher<T>::ICPSequence::getPrefilteredInternalMap();
}

//! Apply ICP to cloud cloudIn, with identity as initial guess
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICPSequence::operator ()(
	const DataPoints& cloudIn)
{
	const int dim = cloudIn.features.rows();
	const TransformationParameters identity = TransformationParameters::Identity(dim, dim);
	return this->compute(cloudIn, identity);
}

//! Apply ICP to cloud cloudIn, with initial guess
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICPSequence::operator ()(
	const DataPoints& cloudIn, const TransformationParameters& T_dataInOld_dataInNew)
{
	return this->compute(cloudIn, T_dataInOld_dataInNew);
}

//! Apply ICP to cloud cloudIn, with initial guess
template<typename T>
typename PointMatcher<T>::TransformationParameters PointMatcher<T>::ICPSequence::compute(
	const DataPoints& cloudIn, const TransformationParameters& T_refIn_dataIn)
{
	// initial keyframe
	if (!hasMap())
	{
		const int dim(cloudIn.features.rows());
		LOG_WARNING_STREAM("Ignoring attempt to perform ICP with an empty map");
		return Matrix::Identity(dim, dim);
	}
	
	this->inspector->init();
	
	return this->computeWithTransformedReference(cloudIn, mapPointCloud, T_refIn_refMean, T_refIn_dataIn);
}

template struct PointMatcher<float>::ICPSequence;
template struct PointMatcher<double>::ICPSequence;
