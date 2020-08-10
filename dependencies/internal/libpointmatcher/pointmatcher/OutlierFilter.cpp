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
#include <limits>

using namespace std;

//! Construct without parameter
template<typename T>
PointMatcher<T>::OutlierFilter::OutlierFilter()
{}

//! Construct with parameters
template<typename T>
PointMatcher<T>::OutlierFilter::OutlierFilter(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
	Parametrizable(className,paramsDoc,params)
{}

//! virtual destructor
template<typename T>
PointMatcher<T>::OutlierFilter::~OutlierFilter()
{}

template struct PointMatcher<float>::OutlierFilter;
template struct PointMatcher<double>::OutlierFilter;


//! Apply outlier-detection chain
template<typename T>
typename PointMatcher<T>::OutlierWeights PointMatcher<T>::OutlierFilters::compute(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const Matches& input)
{
	//FIXME: Why we filter infinit distance only when no filter?
	if (this->empty())
	{
		// we do not have any filter, therefore we must put 0 weights for infinite distances
		OutlierWeights w(input.dists.rows(), input.dists.cols());
		for (int x = 0; x < w.cols(); ++x)
		{
			for (int y = 0; y < w.rows(); ++y)
			{
				if (input.dists(y, x) == numeric_limits<T>::infinity())
					w(y, x) = 0;
				else
					w(y, x) = 1;
			}
		}
		return w;
	}
	else
	{
		// apply filters, they should take care of infinite distances
		//LOG_INFO_STREAM("Applying " << this->size() << " Outlier filters" );
		OutlierWeights w = (*this->begin())->compute(filteredReading, filteredReference, input);
		//LOG_INFO_STREAM("* " << (*this->begin())->className );
		if (this->size() > 1)
		{
			for (OutlierFiltersConstIt it = (this->begin() + 1); it != this->end(); ++it)
			{
				w = w.array() * (*it)->compute(filteredReading, filteredReference, input).array();
				//LOG_INFO_STREAM("* " << (*it)->className );
			}
		}

		return w;
	}
}



template struct PointMatcher<float>::OutlierFilters;
template struct PointMatcher<double>::OutlierFilters;
