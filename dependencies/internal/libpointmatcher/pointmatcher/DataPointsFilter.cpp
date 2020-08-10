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

#ifdef SYSTEM_YAML_CPP
    #include "yaml-cpp/yaml.h"
#else
	#include "yaml-cpp-pm/yaml.h"
    namespace YAML = YAML_PM;
#endif // HAVE_YAML_CPP

//! Construct without parameter
template<typename T>
PointMatcher<T>::DataPointsFilter::DataPointsFilter()
{} 

//! Construct with parameters
template<typename T>
PointMatcher<T>::DataPointsFilter::DataPointsFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	Parametrizable(className,paramsDoc,params)
{}

//! virtual destructor
template<typename T>
PointMatcher<T>::DataPointsFilter::~DataPointsFilter()
{} 

//! Init this filter
template<typename T>
void PointMatcher<T>::DataPointsFilter::init()
{}

template struct PointMatcher<float>::DataPointsFilter;
template struct PointMatcher<double>::DataPointsFilter;


//! Construct an empty chain
template<typename T>
PointMatcher<T>::DataPointsFilters::DataPointsFilters()
{}

//! Construct a chain from a YAML file
template<typename T>
PointMatcher<T>::DataPointsFilters::DataPointsFilters(std::istream& in)
{
	YAML::Parser parser(in);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	// Fix for issue #6: compilation on gcc 4.4.4
	//PointMatcher<T> pm;
	const PointMatcher & pm = PointMatcher::get();
	
	for(YAML::Iterator moduleIt = doc.begin(); moduleIt != doc.end(); ++moduleIt)
	{
		const YAML::Node& module(*moduleIt);
		this->push_back(pm.REG(DataPointsFilter).createFromYAML(module));
	}
}

//! Init the chain
template<typename T>
void PointMatcher<T>::DataPointsFilters::init()
{
	for (DataPointsFiltersIt it = this->begin(); it != this->end(); ++it)
	{
		(*it)->init();
	}
}

//! Apply this chain to cloud, mutates cloud
template<typename T>
void PointMatcher<T>::DataPointsFilters::apply(DataPoints& cloud)
{
	if (this->empty())
		return;

	cloud.assertDescriptorConsistency();
	const int nbPointsBeforeFilters(cloud.features.cols());
	LOG_INFO_STREAM("Applying " << this->size() << " DataPoints filters - " << nbPointsBeforeFilters << " points in");
	for (DataPointsFiltersIt it = this->begin(); it != this->end(); ++it)
	{
		const int nbPointsIn(cloud.features.cols());
		if (nbPointsIn == 0) {
			throw ConvergenceError("no points to filter");
		}

		(*it)->inPlaceFilter(cloud);
		cloud.assertDescriptorConsistency();

		const int nbPointsOut(cloud.features.cols());
		LOG_INFO_STREAM("* " << (*it)->className << " - " << nbPointsOut << " points out (-" << (100 - double(nbPointsOut*100.)/nbPointsIn) << "\%)");
	}
	
	const int nbPointsAfterFilters(cloud.features.cols());
	LOG_INFO_STREAM("Applied " << this->size() << " filters - " << nbPointsAfterFilters << " points out (-" << (100 - double(nbPointsAfterFilters*100.)/nbPointsBeforeFilters) << "\%)");
}

template struct PointMatcher<float>::DataPointsFilters;
template struct PointMatcher<double>::DataPointsFilters;
