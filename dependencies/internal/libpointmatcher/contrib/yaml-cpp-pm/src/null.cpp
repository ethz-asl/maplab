#include "yaml-cpp-pm/null.h"
#include "yaml-cpp-pm/node.h"

namespace YAML_PM
{
	_Null Null;

	bool IsNull(const Node& node)
	{
		return node.Read(Null);
	}
}
