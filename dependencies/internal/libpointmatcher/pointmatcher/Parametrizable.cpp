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

#include "Parametrizable.h"
#include <boost/format.hpp>
#include <boost/typeof/typeof.hpp>
#include <string>

namespace PointMatcherSupport
{
	using namespace std;
	
	//! Construct an invalid-parameter exception
	Parametrizable::InvalidParameter::InvalidParameter(const std::string& reason):
		runtime_error(reason)
	{}
	
	//! Dump the documentation of this parameter to a stream
	std::ostream& operator<< (std::ostream& o, const Parametrizable::ParameterDoc& p)
	{
		o << p.name << " (default: " << p.defaultValue << ") - " << p.doc;
		if (!p.minValue.empty())
			o << " - min: " << p.minValue;
		if (!p.maxValue.empty())
			o << " - max: " << p.maxValue;
		return o;
	}
	
	//! Dump the documentation of this object to a stream
	std::ostream& operator<< (std::ostream& o, const Parametrizable& p)
	{
		o << p.parametersDoc;
		return o;
	}
	
	//! Dump the documentation of these parameters to a stream
	std::ostream& operator<< (std::ostream& o, const Parametrizable::ParametersDoc& p)
	{
		for (BOOST_AUTO(it,p.begin()); it != p.end(); ++it)
			o << "- " << *it << endl;
		return o;
	}
	
	//! Return always false
	bool FalseLexicalComparison(std::string, std::string)
	{
		return false;
	}
	
	/*template<typename S>
	bool ScalarLexicalComparison(std::string a, std::string b)
	{
		return boost::lexical_cast<S>(a) < boost::lexical_cast<S>(b);
	}
	
	Uncomment once most people have gcc >= 4.5
	Shame on bug  9050 (http://gcc.gnu.org/bugzilla/show_bug.cgi?id=9050)
	template<typename S>
	Parametrizable::ParameterDoc::ParameterDoc(const std::string& name, const std::string& doc, const S defaultValue, const S minValue, const S maxValue):
		name(name),
		doc(doc),
		defaultValue(boost::lexical_cast<string>(defaultValue)), 
		minValue(boost::lexical_cast<string>(minValue)), 
		maxValue(boost::lexical_cast<string>(maxValue)),
		comp(ScalarLexicalComparison<S>)
	{}
	
	template<typename S>
	Parametrizable::ParameterDoc::ParameterDoc(const std::string& name, const std::string& doc, const S defaultValue):
		name(name),
		doc(doc),
		defaultValue(boost::lexical_cast<string>(defaultValue)), 
		minValue(""), 
		maxValue(""),
		comp(TrueLexicalComparison)
	{}
	*/
	
	//! Construct a parameter documentation with bounds
	Parametrizable::ParameterDoc::ParameterDoc(const std::string& name, const std::string& doc, const std::string& defaultValue, const std::string& minValue, const std::string& maxValue, LexicalComparison comp):
		name(name),
		doc(doc),
		defaultValue(defaultValue), 
		minValue(minValue), 
		maxValue(maxValue),
		comp(comp)
	{}
	
	//! Construct a parameter documentation without bounds
	Parametrizable::ParameterDoc::ParameterDoc(const std::string& name, const std::string& doc, const std::string& defaultValue):
		name(name),
		doc(doc),
		defaultValue(defaultValue),
		minValue(""),
		maxValue(""),
		comp(FalseLexicalComparison)
	{}
	
	//! Construct a documentation of parameters from a description in the source
	/*ParametersDoc(const std::vector<ParameterDoc>& list):
		std::vector<ParameterDoc>(list)
	{}*/
	
	/*
	Again, not used because fo gcc bug 9050
	
	template<typename S>
	Parametrizable::Parameter::Parameter(const S value):
		std::string(boost::lexical_cast<string>(value))
	{}
	
	// force instantiation of constructors
	template Parametrizable::ParameterDoc::ParameterDoc<int>(const std::string, const std::string, const int);
	template Parametrizable::ParameterDoc::ParameterDoc<float>(const std::string, const std::string, const float);
	template Parametrizable::ParameterDoc::ParameterDoc<double>(const std::string, const std::string, const double);
	template Parametrizable::ParameterDoc::ParameterDoc<bool>(const std::string, const std::string, const bool);
	template Parametrizable::ParameterDoc::ParameterDoc<std::string>(const std::string, const std::string, const std::string);
	template Parametrizable::ParameterDoc::ParameterDoc<const char*>(const std::string, const std::string, const char*);
	
	template Parametrizable::ParameterDoc::ParameterDoc<int>(const std::string, const std::string, const int, const int, const int);
	template Parametrizable::ParameterDoc::ParameterDoc<float>(const std::string, const std::string, const float, const float, const float);
	template Parametrizable::ParameterDoc::ParameterDoc<double>(const std::string, const std::string, const double, const double, const double);
	
	template Parametrizable::Parameter::Parameter<int>(const int);
	template Parametrizable::Parameter::Parameter<float>(const float);
	template Parametrizable::Parameter::Parameter<double>(const double);
	template Parametrizable::Parameter::Parameter<bool>(const bool);
	template Parametrizable::Parameter::Parameter<std::string>(const std::string);

	*/
	
	//! Construct an unknown class without parameters
	Parametrizable::Parametrizable():
		className("unknown")
	{}
	
	//! Construct with documented parameters
	Parametrizable::Parametrizable(
		const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
		className(className),
		parametersDoc(paramsDoc)
	{
		// fill current parameters from either values passed as argument, or default value
		for (BOOST_AUTO(it, parametersDoc.begin()); it != parametersDoc.end(); ++it)
		{
			const string& paramName(it->name);
			Parameters::const_iterator paramIt(params.find(paramName));
			if (paramIt != params.end())
			{
				const string& val(paramIt->second);
				if (it->comp(val, it->minValue))
					throw InvalidParameter((boost::format("Value %1% of parameter %2% in class %3% is smaller than minimum admissible value %4%") % val % paramName % className % it->minValue).str());
				if (it->comp(it->maxValue, val))
					throw InvalidParameter((boost::format("Value %1% of parameter %2% in class %3% is larger than maximum admissible value %4%") % val % paramName % className % it->maxValue).str());
				parameters[paramName] = val;
			}
			else
				parameters[paramName] = it->defaultValue;
		}
	}
	
	//! Virtual destructor, do nothing
	Parametrizable::~Parametrizable()
	{}

	//! Get the value of a parameter, as a string
	std::string Parametrizable::getParamValueString(const std::string& paramName)
	{
		Parameters::const_iterator paramIt(parameters.find(paramName));
		if (paramIt == parameters.end())
			throw InvalidParameter((boost::format("Parameter %1% does not exist in class %2%") % paramName % className).str());
		// TODO: use string distance to propose close one, copy/paste code from Aseba
		this->parametersUsed.insert(paramIt->first);
		return paramIt->second;
	}
} // namespace PointMatcherSupport
