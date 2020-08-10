#include "PointMatcher.h"

#ifdef SYSTEM_YAML_CPP
    #include "yaml-cpp/yaml.h"
#else
	#include "yaml-cpp-pm/yaml.h"
    namespace YAML = YAML_PM;
#endif // HAVE_YAML_CPP

namespace PointMatcherSupport
{
	void getNameParamsFromYAML(const YAML::Node& module, std::string& name, Parametrizable::Parameters& params)
	{
		if (module.size() != 1)
		{
			// parameter-less entry
			name = module.to<std::string>();
		}
		else
		{
			// get parameters
      YAML::Iterator mapIt(module.begin());
			mapIt.first() >> name;
      for(YAML::Iterator paramIt = mapIt.second().begin(); paramIt != mapIt.second().end(); ++paramIt)
			{
				std::string key, value;
				paramIt.first() >> key;
				paramIt.second() >> value;
				params[key] = value;
			}
		}
	}
}
