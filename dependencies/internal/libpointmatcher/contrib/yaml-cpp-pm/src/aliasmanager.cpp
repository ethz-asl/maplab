#include "yaml-cpp-pm/aliasmanager.h"
#include "yaml-cpp-pm/node.h"
#include <cassert>
#include <sstream>

namespace YAML_PM
{
	AliasManager::AliasManager(): m_curAnchor(0)
	{
	}

	void AliasManager::RegisterReference(const Node& node)
	{
		m_anchorByIdentity.insert(std::make_pair(&node, _CreateNewAnchor()));
	}

	anchor_t AliasManager::LookupAnchor(const Node& node) const
	{
		AnchorByIdentity::const_iterator it = m_anchorByIdentity.find(&node);
		if(it == m_anchorByIdentity.end())
			return 0;
		return it->second;
	}

	anchor_t AliasManager::_CreateNewAnchor()
	{
		return ++m_curAnchor;
	}
}
