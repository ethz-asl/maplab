// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
Francois Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
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

#ifndef __POINTMATCHER_CORE_H
#define __POINTMATCHER_CORE_H

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR
//#define EIGEN2_SUPPORT

//TODO: investigate if that causes more problems down the road
#define EIGEN_NO_DEBUG

#include "Eigen/StdVector"
#include "Eigen/Core"
#include "Eigen/Geometry"


#include <boost/thread/mutex.hpp>

#include <stdexcept>
#include <limits>
#include <iostream>
#include <ostream>
#include <memory>
//#include <cstdint>
#include <boost/cstdint.hpp>

#include "DeprecationWarnings.h"
#include "Parametrizable.h"
#include "Registrar.h"
 
/*! 
	\file PointMatcher.h
	\brief public interface
*/


//! version of the Pointmatcher library as string
#define POINTMATCHER_VERSION "1.3.1"
//! version of the Pointmatcher library as an int
#define POINTMATCHER_VERSION_INT 10301

//! Functions and classes that are not dependant on scalar type are defined in this namespace
namespace PointMatcherSupport
{
	// TODO: gather all exceptions

	//! An exception thrown when one tries to use a module type that does not exist
	struct InvalidModuleType: std::runtime_error
	{
		InvalidModuleType(const std::string& reason);
	};

	//! An expection thrown when a transformation has invalid parameters
	struct TransformationError: std::runtime_error
	{
		//! return an exception when a transformation has invalid parameters
		TransformationError(const std::string& reason);
	};
	
	//! The logger interface, used to output warnings and informations
	struct Logger: public Parametrizable
	{
		Logger();
		Logger(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		
		virtual ~Logger();
		virtual bool hasInfoChannel() const;
		virtual void beginInfoEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* infoStream();
		virtual void finishInfoEntry(const char *file, unsigned line, const char *func);
		virtual bool hasWarningChannel() const;
		virtual void beginWarningEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* warningStream();
		virtual void finishWarningEntry(const char *file, unsigned line, const char *func);
	};
	
	void setLogger(std::shared_ptr<Logger> newLogger);
	
	void validateFile(const std::string& fileName);
	
	//! Data from a CSV file
	typedef std::map<std::string, std::vector<std::string> > CsvElements;
}

//! Functions and classes that are dependant on scalar type are defined in this templatized class
template<typename T>
struct PointMatcher
{
	// ---------------------------------
	// macros for constants
	// ---------------------------------
	
	//! The smallest value larger than 0
	#define ZERO_PLUS_EPS (0. + std::numeric_limits<double>::epsilon())
	//! The largest value smaller than 1
	#define ONE_MINUS_EPS (1. - std::numeric_limits<double>::epsilon())
	
	// ---------------------------------
	// exceptions
	// ---------------------------------

	//TODO: gather exceptions here and in Exceptions.cpp

	//! Point matcher did not converge
	struct ConvergenceError: std::runtime_error
	{
		ConvergenceError(const std::string& reason);
	};


	// ---------------------------------
	// eigen and nabo-based types
	// ---------------------------------
	
	//! The scalar type
	typedef T ScalarType;
	//! A vector over ScalarType
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	//! A vector of vector over ScalarType, not a matrix
	typedef std::vector<Vector, Eigen::aligned_allocator<Vector> > VectorVector;
	//! A quaternion over ScalarType
	typedef typename Eigen::Quaternion<T> Quaternion;
	//! A vector of quaternions over ScalarType
	typedef std::vector<Quaternion, Eigen::aligned_allocator<Quaternion> > QuaternionVector;
	//! A dense matrix over ScalarType
	typedef typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	//! A dense integer matrix
	typedef typename Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IntMatrix;
	//! A dense signed 64-bits matrix
	typedef typename Eigen::Matrix<std::int64_t, Eigen::Dynamic, Eigen::Dynamic> Int64Matrix;
	//! A dense array over ScalarType
	typedef typename Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic> Array;


	//! A matrix holding the parameters a transformation.
	/**
		The transformation lies in the special Euclidean group of dimension \f$n\f$, \f$SE(n)\f$, implemented as a dense matrix of size \f$n+1 \times n+1\f$ over ScalarType.
	*/
	typedef Matrix TransformationParameters;
	
	// alias for scope reasons
	typedef PointMatcherSupport::Parametrizable Parametrizable; //!< alias
	typedef Parametrizable::Parameters Parameters; //!< alias
	typedef Parametrizable::ParameterDoc ParameterDoc; //!< alias
	typedef Parametrizable::ParametersDoc ParametersDoc; //!< alias
	typedef Parametrizable::InvalidParameter InvalidParameter; //!< alias
	
	// ---------------------------------
	// input types
	// ---------------------------------
	
	//! A point cloud
	/**
		For every point, it has features and, optionally, descriptors.
		Features are typically the coordinates of the point in the space.
		Descriptors contain information attached to the point, such as its color, its normal vector, etc.
		In both features and descriptors, every point can have multiple channels.
		Every channel has a dimension and a name.
		For instance, a typical 3D cloud might have the channels \c x, \c y, \c z, \c w of dimension 1 as features (using homogeneous coordinates), and the channel \c normal of size 3 as descriptor.
		There are no sub-channels, such as \c normal.x, for the sake of simplicity.
		Moreover, the position of the points is in homogeneous coordinates because they need both translation and rotation, while the normals need only rotation.
		All channels contain scalar values of type ScalarType.
	*/
	struct DataPoints
	{
		//! A view on a feature or descriptor
		typedef Eigen::Block<Matrix> View;
		//! A view on a time
		typedef Eigen::Block<Int64Matrix> TimeView;
		//! A view on a const feature or const descriptor
		typedef const Eigen::Block<const Matrix> ConstView;
		//! a view on a const time
		typedef const Eigen::Block<const Int64Matrix> TimeConstView;
		//! An index to a row or a column
		typedef typename Matrix::Index Index;
		
		//! The name for a certain number of dim
		struct Label
		{
			std::string text; //!< name of the label
			size_t span; //!< number of data dimensions the label spans
			Label(const std::string& text = "", const size_t span = 0);
			bool operator ==(const Label& that) const;
		};
		//! A vector of Label
		struct Labels: std::vector<Label>
		{
			typedef typename std::vector<Label>::const_iterator const_iterator; //!< alias
			Labels();
			Labels(const Label& label);
			bool contains(const std::string& text) const;
			size_t totalDim() const;
			friend std::ostream& operator<< (std::ostream& stream, const Labels& labels)
			{
				for(size_t i=0; i<labels.size(); i++)
				{
					stream << labels[i].text;
					if(i != (labels.size() -1))
						stream << ", ";
				}

				return stream;
			};
		};
		
		//! An exception thrown when one tries to access features or descriptors unexisting or of wrong dimensions
		struct InvalidField: std::runtime_error
		{
			InvalidField(const std::string& reason);
		};
		
		// Constructors from descriptions (reserve memory)
		DataPoints();
		DataPoints(const Labels& featureLabels, const Labels& descriptorLabels, const size_t pointCount);
		DataPoints(const Labels& featureLabels, const Labels& descriptorLabels, const Labels& timeLabels, const size_t pointCount);

		// Copy constructors from partial data
		DataPoints(const Matrix& features, const Labels& featureLabels);
		DataPoints(const Matrix& features, const Labels& featureLabels, const Matrix& descriptors, const Labels& descriptorLabels);
		DataPoints(const Matrix& features, const Labels& featureLabels, const Matrix& descriptors, const Labels& descriptorLabels, const Int64Matrix& times, const Labels& timeLabels);
		
		bool operator ==(const DataPoints& that) const;
	
		unsigned getNbPoints() const;
		unsigned getEuclideanDim() const;
		unsigned getHomogeneousDim() const;
		unsigned getNbGroupedDescriptors() const;
		unsigned getDescriptorDim() const;
		unsigned getTimeDim() const;

		void save(const std::string& fileName, bool binary = false) const;
		static DataPoints load(const std::string& fileName);
		
		void concatenate(const DataPoints& dp);
		void conservativeResize(Index pointCount);
		DataPoints createSimilarEmpty() const;
		DataPoints createSimilarEmpty(Index pointCount) const;
		void setColFrom(Index thisCol, const DataPoints& that, Index thatCol);
		void swapCols(Index iCol,Index jCol);
		
		// methods related to features
		void allocateFeature(const std::string& name, const unsigned dim);
		void allocateFeatures(const Labels& newLabels);
		void addFeature(const std::string& name, const Matrix& newFeature);
		void removeFeature(const std::string& name);
		Matrix getFeatureCopyByName(const std::string& name) const;
		ConstView getFeatureViewByName(const std::string& name) const;
		View getFeatureViewByName(const std::string& name);
		ConstView getFeatureRowViewByName(const std::string& name, const unsigned row) const;
		View getFeatureRowViewByName(const std::string& name, const unsigned row);
		bool featureExists(const std::string& name) const;
		bool featureExists(const std::string& name, const unsigned dim) const;
		unsigned getFeatureDimension(const std::string& name) const;
		unsigned getFeatureStartingRow(const std::string& name) const;
		
		// methods related to descriptors
		void allocateDescriptor(const std::string& name, const unsigned dim);
		void allocateDescriptors(const Labels& newLabels);
		void addDescriptor(const std::string& name, const Matrix& newDescriptor);
		void removeDescriptor(const std::string& name);
		Matrix getDescriptorCopyByName(const std::string& name) const;
		ConstView getDescriptorViewByName(const std::string& name) const;
		View getDescriptorViewByName(const std::string& name);
		ConstView getDescriptorRowViewByName(const std::string& name, const unsigned row) const;
		View getDescriptorRowViewByName(const std::string& name, const unsigned row);
		bool descriptorExists(const std::string& name) const;
		bool descriptorExists(const std::string& name, const unsigned dim) const;
		unsigned getDescriptorDimension(const std::string& name) const;
		unsigned getDescriptorStartingRow(const std::string& name) const;
		void assertDescriptorConsistency() const;
		
		// methods related to times
		void allocateTime(const std::string& name, const unsigned dim);
		void allocateTimes(const Labels& newLabels);
		void addTime(const std::string& name, const Int64Matrix& newTime);
		void removeTime(const std::string& name);
		Int64Matrix getTimeCopyByName(const std::string& name) const;
		TimeConstView getTimeViewByName(const std::string& name) const;
		TimeView getTimeViewByName(const std::string& name);
		TimeConstView getTimeRowViewByName(const std::string& name, const unsigned row) const;
		TimeView getTimeRowViewByName(const std::string& name, const unsigned row);
		bool timeExists(const std::string& name) const;
		bool timeExists(const std::string& name, const unsigned dim) const;
		unsigned getTimeDimension(const std::string& name) const;
		unsigned getTimeStartingRow(const std::string& name) const;
		void assertTimesConsistency() const;

		Matrix features; //!< features of points in the cloud
		Labels featureLabels; //!< labels of features
		Matrix descriptors; //!< descriptors of points in the cloud, might be empty
		Labels descriptorLabels; //!< labels of descriptors
		Int64Matrix times; //!< time associated to each points, might be empty
		Labels timeLabels; //!< labels of times.
	
	private:
		void assertConsistency(const std::string& dataName, const int dataRows, const int dataCols, const Labels& labels) const;
		template<typename MatrixType> 
		void allocateFields(const Labels& newLabels, Labels& labels, MatrixType& data) const;
		template<typename MatrixType> 
		void allocateField(const std::string& name, const unsigned dim, Labels& labels, MatrixType& data) const;
		template<typename MatrixType> 
		void addField(const std::string& name, const MatrixType& newField, Labels& labels, MatrixType& data) const;
		template<typename MatrixType>
		void removeField(const std::string& name, Labels& labels, MatrixType& data) const;
		template<typename MatrixType>
		const Eigen::Block<const MatrixType> getConstViewByName(const std::string& name, const Labels& labels, const MatrixType& data, const int viewRow = -1) const;
		template<typename MatrixType>
		Eigen::Block<MatrixType> getViewByName(const std::string& name, const Labels& labels, MatrixType& data, const int viewRow = -1) const;
		bool fieldExists(const std::string& name, const unsigned dim, const Labels& labels) const;
		unsigned getFieldDimension(const std::string& name, const Labels& labels) const;
		unsigned getFieldStartingRow(const std::string& name, const Labels& labels) const;

		template<typename MatrixType>
		void concatenateLabelledMatrix(Labels* labels, MatrixType* data, const Labels extraLabels, const MatrixType extraData);
	};
	
	static void swapDataPoints(DataPoints& a, DataPoints& b);

	// ---------------------------------
	// intermediate types
	// ---------------------------------
	
	//! Result of the data-association step (Matcher::findClosests), before outlier rejection.
	/**
		This class holds a list of associated reference identifiers, along with the corresponding \e squared distance, for all points in the reading.
		A single point in the reading can have one or multiple matches.
	*/
	struct Matches
	{
		typedef Matrix Dists; //!< Squared distances to closest points, dense matrix of ScalarType
		typedef IntMatrix Ids; //!< Identifiers of closest points, dense matrix of integers
	

		static constexpr int InvalidId = -1; //! In case of too few matches the ids are filled with InvalidId
		static constexpr T InvalidDist = std::numeric_limits<T>::infinity(); //! In case of too few matches the dists are filled with InvalidDist

		Matches();
		Matches(const Dists& dists, const Ids ids);
		Matches(const int knn, const int pointsCount);
		
		Dists dists; //!< squared distances to closest points
		Ids ids; //!< identifiers of closest points
		
		T getDistsQuantile(const T quantile) const;
		T getMedianAbsDeviation() const;
		T getStandardDeviation() const;

	};

	//! Weights of the associations between the points in Matches and the points in the reference.
	/**
		A weight of 0 means no association, while a weight of 1 means a complete trust in association.
	*/
	typedef Matrix OutlierWeights;
	
	// ---------------------------------
	// types of processing bricks
	// ---------------------------------
	
	//! A function that transforms points and their descriptors given a transformation matrix
	struct Transformation: public Parametrizable
	{
		Transformation();
		Transformation(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~Transformation();
		
		//! Transform input using the transformation matrix
		virtual DataPoints compute(const DataPoints& input, const TransformationParameters& parameters) const = 0; 

		//! Return whether the given parameters respect the expected constraints
		virtual bool checkParameters(const TransformationParameters& parameters) const = 0;

		//! Return a valid version of the given transformation
		virtual TransformationParameters correctParameters(const TransformationParameters& parameters) const = 0;

	};
	
	//! A chain of Transformation
	struct Transformations: public std::vector<std::shared_ptr<Transformation> >
	{
		void apply(DataPoints& cloud, const TransformationParameters& parameters) const;
	};
	typedef typename Transformations::iterator TransformationsIt; //!< alias
	typedef typename Transformations::const_iterator TransformationsConstIt; //!< alias
	
	DEF_REGISTRAR(Transformation)
	
	// ---------------------------------
	
	//! A data filter takes a point cloud as input, transforms it, and produces another point cloud as output.
	/**
		The filter might add information, for instance surface normals, or might change the number of points, for instance by randomly removing some of them.
	*/
	struct DataPointsFilter: public Parametrizable
	{
		DataPointsFilter();
		DataPointsFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~DataPointsFilter();
		virtual void init();

		//! Apply filters to input point cloud.  This is the non-destructive version and returns a copy.
		virtual DataPoints filter(const DataPoints& input) = 0;

		//! Apply these filters to a point cloud without copying.
		virtual void inPlaceFilter(DataPoints& cloud) = 0;
	};
	
	//! A chain of DataPointsFilter
	struct DataPointsFilters: public std::vector<std::shared_ptr<DataPointsFilter> >
	{
		DataPointsFilters();
		DataPointsFilters(std::istream& in);
		void init();
		void apply(DataPoints& cloud);
	};
	typedef typename DataPointsFilters::iterator DataPointsFiltersIt; //!< alias
	typedef typename DataPointsFilters::const_iterator DataPointsFiltersConstIt; //!< alias
	
	DEF_REGISTRAR(DataPointsFilter)
	
	// ---------------------------------
	
	//! A matcher links points in the reading to points in the reference.
	/**
		This typically uses a space-partitioning structure such as a kd-tree for performance optimization.
	*/
	struct Matcher: public Parametrizable
	{
		unsigned long visitCounter; //!< number of points visited
		
		Matcher();
		Matcher(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~Matcher();
		
		void resetVisitCount();
		unsigned long getVisitCount() const;
		
		//! Init this matcher to find nearest neighbor in filteredReference
		virtual void init(const DataPoints& filteredReference) = 0;
		//! Find the closest neighbors of filteredReading in filteredReference passed to init()
		virtual Matches findClosests(const DataPoints& filteredReading) = 0;
	};
	
	DEF_REGISTRAR(Matcher)
	
	// ---------------------------------
	
	//! An outlier filter removes or weights links between points in reading and their matched points in reference, depending on some criteria.
	/**
		Criteria can be a fixed maximum authorized distance, a factor of the median distance, etc. 
		Points with zero weights are ignored in the subsequent minimization step.
	*/
	struct OutlierFilter: public Parametrizable
	{
		OutlierFilter();
		OutlierFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		
		virtual ~OutlierFilter();
		
		//! Detect outliers using features
		virtual OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input) = 0;
	};
	
	
	//! A chain of OutlierFilter
	struct OutlierFilters: public std::vector<std::shared_ptr<OutlierFilter> >
	{
		
		OutlierWeights compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const Matches& input);
		
	};
	
	typedef typename OutlierFilters::const_iterator OutlierFiltersConstIt; //!< alias
	typedef typename OutlierFilters::iterator OutlierFiltersIt; //!< alias
	
	DEF_REGISTRAR(OutlierFilter)

	// ---------------------------------
	
	//! An error minimizer will compute a transformation matrix such as to minimize the error between the reading and the reference. 
	/**
		Typical error minimized are point-to-point and point-to-plane.
	*/
	struct ErrorMinimizer: public Parametrizable
	{
		//! A structure holding data ready for minimization. The data are "normalized", for instance there are no points with 0 weight, etc.
		struct ErrorElements
		{
			DataPoints reading; //!< reading point cloud
			DataPoints reference; //!< reference point cloud
			OutlierWeights weights; //!< weights for every association
			Matches matches; //!< associations
			int nbRejectedMatches; //!< number of matches with zero weights
			int nbRejectedPoints; //!< number of points with all matches set to zero weights
			T pointUsedRatio;  //!< the ratio of how many points were used for error minimization
			T weightedPointUsedRatio;//!< the ratio of how many points were used (with weight) for error minimization

			ErrorElements();
			ErrorElements(const DataPoints& requestedPts, const DataPoints& sourcePts, const OutlierWeights& outlierWeights, const Matches& matches);
		};
		
		ErrorMinimizer();
		ErrorMinimizer(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~ErrorMinimizer();
		
		T getPointUsedRatio() const;
		T getWeightedPointUsedRatio() const;
		ErrorElements getErrorElements() const; //TODO: ensure that is return a usable value
		virtual T getOverlap() const;
		virtual Matrix getCovariance() const;
		virtual T getResidualError(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches) const;
		
		//! Find the transformation that minimizes the error
		virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches);
		//! Find the transformation that minimizes the error given matched pair of points. This function most be defined for all new instances of ErrorMinimizer.
		virtual TransformationParameters compute(const ErrorElements& matchedPoints) = 0;
		
		// helper functions
		static Matrix crossProduct(const Matrix& A, const Matrix& B);//TODO: this might go in pointmatcher_support namespace
		
	protected:
		//T pointUsedRatio; //!< the ratio of how many points were used for error minimization
		//T weightedPointUsedRatio; //!< the ratio of how many points were used (with weight) for error minimization
		//TODO: standardize the use of this variable
		ErrorElements lastErrorElements; //!< memory of the last computed error
	};
	
	DEF_REGISTRAR(ErrorMinimizer)
	
	// ---------------------------------
	
	//! A transformation checker can stop the iteration depending on some conditions.
	/**
		For example, a condition can be the number of times the loop was executed, or it can be related to the matching error.
		Because the modules can be chained, we defined that the relation between modules must agree through an OR-condition, while all AND-conditions are defined within a single module.
	*/
	struct TransformationChecker: public Parametrizable
	{
	protected:
		typedef std::vector<std::string> StringVector; //!< a vector of strings
		Vector limits; //!< values of limits involved in conditions to stop ICP loop
		Vector conditionVariables; //!< values of variables involved in conditions to stop ICP loop
		StringVector limitNames; //!< names of limits involved in conditions to stop ICP loop
		StringVector conditionVariableNames; //!< names of variables involved in conditions to stop ICP loop

	public:
		TransformationChecker();
		TransformationChecker(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		virtual ~TransformationChecker();
		//! Init, set iterate to false if iteration should stop
		virtual void init(const TransformationParameters& parameters, bool& iterate) = 0;
		//! Set iterate to false if iteration should stop
		virtual void check(const TransformationParameters& parameters, bool& iterate) = 0;
		
		const Vector& getLimits() const;
		const Vector& getConditionVariables() const;
		const StringVector& getLimitNames() const;
		const StringVector& getConditionVariableNames() const;
		
	protected:
		static Vector matrixToAngles(const TransformationParameters& parameters);
	};
	
	//! A chain of TransformationChecker
	struct TransformationCheckers: public std::vector<std::shared_ptr<TransformationChecker> >
	{
		void init(const TransformationParameters& parameters, bool& iterate);
		void check(const TransformationParameters& parameters, bool& iterate);
	};
	typedef typename TransformationCheckers::iterator TransformationCheckersIt; //!< alias
	typedef typename TransformationCheckers::const_iterator TransformationCheckersConstIt; //!< alias
	
	DEF_REGISTRAR(TransformationChecker)

	// ---------------------------------
	
	//! An inspector allows to log data at the different steps, for analysis.
	struct Inspector: public Parametrizable
	{
		
		Inspector();
		Inspector(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params);
		
		// 
		virtual ~Inspector();
		virtual void init();
		
		// performance statistics
		virtual void addStat(const std::string& name, double data);
		virtual void dumpStats(std::ostream& stream);
		virtual void dumpStatsHeader(std::ostream& stream);
		
		// data statistics 
		virtual void dumpIteration(const size_t iterationNumber, const TransformationParameters& parameters, const DataPoints& filteredReference, const DataPoints& reading, const Matches& matches, const OutlierWeights& outlierWeights, const TransformationCheckers& transformationCheckers);
		virtual void finish(const size_t iterationCount);
	};
	
	DEF_REGISTRAR(Inspector) 
	
	// ---------------------------------
	
	DEF_REGISTRAR_IFACE(Logger, PointMatcherSupport::Logger)

	// ---------------------------------
	
	// algorithms
	
	//! Stuff common to all ICP algorithms
	struct ICPChainBase
	{
	public:
		DataPointsFilters readingDataPointsFilters; //!< filters for reading, applied once
		DataPointsFilters readingStepDataPointsFilters; //!< filters for reading, applied at each step
		DataPointsFilters referenceDataPointsFilters; //!< filters for reference
		Transformations transformations; //!< transformations
		std::shared_ptr<Matcher> matcher; //!< matcher
		OutlierFilters outlierFilters; //!< outlier filters
		std::shared_ptr<ErrorMinimizer> errorMinimizer; //!< error minimizer
		TransformationCheckers transformationCheckers; //!< transformation checkers
		std::shared_ptr<Inspector> inspector; //!< inspector
		
		virtual ~ICPChainBase();

		virtual void setDefault();
		
		virtual void loadFromYaml(std::istream& in);
		unsigned getPrefilteredReadingPtsCount() const;
		unsigned getPrefilteredReferencePtsCount() const;

		bool getMaxNumIterationsReached() const;
		
	protected:
		unsigned prefilteredReadingPtsCount; //!< remaining number of points after prefiltering but before the iterative process
		unsigned prefilteredReferencePtsCount; //!< remaining number of points after prefiltering but before the iterative process
		bool maxNumIterationsReached; //!< store if we reached the maximum number of iterations last time compute was called

		ICPChainBase();
		
		void cleanup();
		
        virtual void loadAdditionalYAMLContent(PointMatcherSupport::YAML::Node& doc);
		
		//! Instantiate modules if their names are in the YAML file
		template<typename R>
        const std::string& createModulesFromRegistrar(const std::string& regName, const PointMatcherSupport::YAML::Node& doc, const R& registrar, std::vector<std::shared_ptr<typename R::TargetType> >& modules);
		
		//! Instantiate a module if its name is in the YAML file
		template<typename R>
        const std::string& createModuleFromRegistrar(const std::string& regName, const PointMatcherSupport::YAML::Node& doc, const R& registrar, std::shared_ptr<typename R::TargetType>& module);
		
		//! Get the value of a field in a node
        std::string nodeVal(const std::string& regName, const PointMatcherSupport::YAML::Node& doc);
	};
	
	//! ICP algorithm
	struct ICP: ICPChainBase
	{
		TransformationParameters operator()(
			const DataPoints& readingIn,
			const DataPoints& referenceIn);

		TransformationParameters operator()(
			const DataPoints& readingIn,
			const DataPoints& referenceIn,
			const TransformationParameters& initialTransformationParameters);
		
		TransformationParameters compute(
			const DataPoints& readingIn,
			const DataPoints& referenceIn,
			const TransformationParameters& initialTransformationParameters);

		//! Return the filtered point cloud reading used in the ICP chain
		const DataPoints& getReadingFiltered() const { return readingFiltered; }

	protected:
		TransformationParameters computeWithTransformedReference(
			const DataPoints& readingIn, 
			const DataPoints& reference, 
			const TransformationParameters& T_refIn_refMean,
			const TransformationParameters& initialTransformationParameters);

		DataPoints readingFiltered; //!< reading point cloud after the filters were applied
	};
	
	//! ICP alogrithm, taking a sequence of clouds and using a map
	//! Warning: used with caution, you need to set the map manually.
	struct ICPSequence: public ICP
	{
		TransformationParameters operator()(
			const DataPoints& cloudIn);
		TransformationParameters operator()(
			const DataPoints& cloudIn,
			const TransformationParameters& initialTransformationParameters);
		TransformationParameters compute(
			const DataPoints& cloudIn,
			const TransformationParameters& initialTransformationParameters);
		
		bool hasMap() const;
		bool setMap(const DataPoints& map);
		void clearMap();
		virtual void setDefault();
		virtual void loadFromYaml(std::istream& in);
		PM_DEPRECATED("Use getPrefilteredInternalMap instead. "
			            "Function now always returns map with filter chain applied. "
			            "This may have altered your program behavior."
		              "Reasons for this stated here and in associated PR: "
		              "https://github.com/ethz-asl/libpointmatcher/issues/209.")
		const DataPoints& getInternalMap() const;
		const DataPoints& getPrefilteredInternalMap() const;
		PM_DEPRECATED("Use getPrefilteredMap instead. "
									"Function now always returns map with filter chain applied. "
			            "This may have altered your program behavior."
			            "Reasons for this stated here and in associated PR: "
			            "https://github.com/ethz-asl/libpointmatcher/issues/209")
		const DataPoints getMap() const;
		const DataPoints getPrefilteredMap() const;
		
	protected:
		DataPoints mapPointCloud; //!< point cloud of the map, always in global frame (frame of first point cloud)
		TransformationParameters T_refIn_refMean; //!< offset for centered map
	};
	
	// ---------------------------------
	// Instance-related functions
	// ---------------------------------
	
	PointMatcher();
	
	static const PointMatcher& get();

	
}; // PointMatcher<T>

#endif // __POINTMATCHER_CORE_H

