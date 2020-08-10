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

#ifndef __POINTMATCHER_REGISTRAR_H
#define __POINTMATCHER_REGISTRAR_H

#include "Parametrizable.h"
#include "PointMatcher.h"
#include <boost/format.hpp>
#include <boost/typeof/typeof.hpp>


#ifdef SYSTEM_YAML_CPP
	namespace YAML
	{
		class Node;
	}
#else
	namespace YAML_PM
	{
		class Node;
	}
#endif // HAVE_YAML_CPP

namespace PointMatcherSupport
{
#ifdef SYSTEM_YAML_CPP
	namespace YAML = ::YAML;
#else
	namespace YAML = ::YAML_PM;
#endif

	//! Retrieve name and parameters from a yaml node
	void getNameParamsFromYAML(const YAML::Node& module, std::string& name, Parametrizable::Parameters& params);

	//! An exception thrown when one tries to instanciate an element that does not exist in the registrar
	struct InvalidElement: std::runtime_error
	{
		InvalidElement(const std::string& reason);
	};
	
	//! A factor for subclasses of Interface
	template<typename Interface>
	struct Registrar
	{
	public:
		typedef Interface TargetType; //!< alias to recover the template parameter
		
		//! The interface for class descriptors
		struct ClassDescriptor
		{
			//! Virtual destructor, do nothing
			virtual ~ClassDescriptor() {}
			//! Create an instance of Interface using params
			virtual std::shared_ptr<Interface> createInstance(const std::string& className, const Parametrizable::Parameters& params) const = 0;
			//! Return the description of this class
			virtual const std::string description() const = 0;
			//! Return the available parameters for this class
			virtual const Parametrizable::ParametersDoc availableParameters() const = 0;
		};
		
		//! A descriptor for a class C that provides parameters
		template<typename C>
		struct GenericClassDescriptor: public ClassDescriptor
		{
			virtual std::shared_ptr<Interface> createInstance(const std::string& className, const Parametrizable::Parameters& params) const
			{
				std::shared_ptr<C> instance = std::make_shared<C>(params);
				
				// check that all parameters were set
				for (BOOST_AUTO(it, params.begin()); it != params.end() ;++it)
				{
					if (instance->parametersUsed.find(it->first) == instance->parametersUsed.end()){
						throw Parametrizable::InvalidParameter(
							(boost::format("Parameter %1% for module %2% was set but is not used") % it->first % className).str()
						);
					}
				}
				
				return instance;
			}
			virtual const std::string description() const
			{
				return C::description();
			}
			virtual const Parametrizable::ParametersDoc availableParameters() const
			{
				return C::availableParameters();
			}
		};
		
		//! A descriptor for a class C that does not provide any parameter
		template<typename C>
		struct GenericClassDescriptorNoParam: public ClassDescriptor
		{
			virtual std::shared_ptr<Interface> createInstance(const std::string& className, const Parametrizable::Parameters& params) const
			{
				for (BOOST_AUTO(it, params.begin()); it != params.end() ;++it)
					throw Parametrizable::InvalidParameter(
							(boost::format("Parameter %1% was set but module %2% dos not use any parameter") % it->first % className).str()
						);
				return std::make_shared<C>();
			}
			virtual const std::string description() const
			{
				return C::description();
			}
			virtual const Parametrizable::ParametersDoc availableParameters() const
			{
				return Parametrizable::ParametersDoc();
			}
		};
		
	protected:
		typedef std::map<std::string, ClassDescriptor*> DescriptorMap; //!< descriptors for sub-classes of Interface, indexed by their names
		DescriptorMap classes; //!< known classes that can be constructed
		
	public:
		//! Destructor, remove all classes descriptors 
		~Registrar()
		{
			for (BOOST_AUTO(it, classes.begin()); it != classes.end(); ++it)
				delete it->second;
		}
		//! Register a class by storing an instance of a descriptor helper class
		void reg(const std::string &name, ClassDescriptor* descriptor)
		{
			classes[name] = descriptor;
		}
		
		//! Return a descriptor following a name, throw an exception if name is invalid
		const ClassDescriptor* getDescriptor(const std::string& name) const
		{
			BOOST_AUTO(it, classes.find(name));
			if (it == classes.end())
			{
				std::cerr << "No element named " << name << " is registered. Known ones are:\n";
				dump(std::cerr);
				throw InvalidElement(
					(boost::format("Trying to instanciate unknown element %1% from registrar") % name).str()
				);
			}
			return it->second;
		}
		
		//! Create an instance
		std::shared_ptr<Interface> create(const std::string& name, const Parametrizable::Parameters& params = Parametrizable::Parameters()) const
		{
			return getDescriptor(name)->createInstance(name, params);
		}
				
		//! Create an instance from a YAML node
        	std::shared_ptr<Interface> createFromYAML(const YAML::Node& module) const
		{
			std::string name;
			Parametrizable::Parameters params;

			getNameParamsFromYAML(module, name, params);
			
			return create(name, params);
		}
				
		//! Get the description of a class
		const std::string getDescription(const std::string& name) const
		{
			return getDescriptor(name)->description();
		}
		
		//! Print the list of registered classes to stream
		void dump(std::ostream &stream) const
		{
			for (BOOST_AUTO(it, classes.begin()); it != classes.end(); ++it)
				stream << "- " << it->first << "\n";
		}
		
		//! begin for const iterator over classes descriptions
		typename DescriptorMap::const_iterator begin() const
		{	
			return classes.begin();
		}
		//! end for const iterator over classes descriptions
		typename DescriptorMap::const_iterator end() const
		{
			return classes.end();
		}
	};

	#define REG(name) name##Registrar
	#define DEF_REGISTRAR(name) PointMatcherSupport::Registrar< name > name##Registrar;
	#define DEF_REGISTRAR_IFACE(name, ifaceName) PointMatcherSupport::Registrar< ifaceName > name##Registrar;
	#define ADD_TO_REGISTRAR(name, elementName, element) { \
		typedef typename PointMatcherSupport::Registrar< name >::template GenericClassDescriptor< element > Desc; \
		name##Registrar.reg(# elementName, new Desc() ); \
	}
	#define ADD_TO_REGISTRAR_NO_PARAM(name, elementName, element) { \
		typedef typename PointMatcherSupport::Registrar< name >::template GenericClassDescriptorNoParam< element > Desc; \
		name##Registrar.reg(# elementName, new Desc() ); \
	}
} // namespace PointMatcherSupport

#endif // __POINTMATCHER_REGISTRAR_H
