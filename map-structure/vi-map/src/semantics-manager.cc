#include "vi-map/semantics-manager.h"

#include <unordered_map>

#include <maplab-common/accessors.h>

#include "vi-map/vi-map.h"

namespace vi_map {

SemanticsManager::SemanticsManager() {}

SemanticsManager::SemanticsManager(const MissionNames& mission_names)
    : mission_names_(mission_names) {}

std::string SemanticsManager::getNameOfMission(const MissionId& id) const {
  if (mission_names_.count(id) == 0u) {
    return "";
  } else {
    return common::getChecked(mission_names_, id);
  }
}

MissionId SemanticsManager::getMissionIdForName(
    const std::string& query_name) const {
  MissionId result;
  for (const MissionNames::value_type& pair : mission_names_) {
    if (pair.second == query_name) {
      return pair.first;
    }
  }
  return result;
}

void SemanticsManager::nameMission(
    const MissionId& id, const std::string& name) {
  CHECK(!getMissionIdForName(name).isValid()) << "Name " << name
                                              << " already assigned!";
  CHECK(mission_names_.emplace(id, name).second) << id << " already named!";
}

void SemanticsManager::getUnnamedMissionIds(
    const VIMap& map, MissionIdList* result) {
  CHECK_NOTNULL(result)->clear();

  MissionIdList named_mission_ids;
  for (const MissionNames::value_type& pair : mission_names_) {
    named_mission_ids.emplace_back(pair.first);
  }
  std::sort(named_mission_ids.begin(), named_mission_ids.end());

  MissionIdList map_mission_ids;
  map.getAllMissionIds(&map_mission_ids);
  std::sort(map_mission_ids.begin(), map_mission_ids.end());

  std::set_difference(
      map_mission_ids.begin(), map_mission_ids.end(), named_mission_ids.begin(),
      named_mission_ids.end(), std::inserter(*result, result->begin()));
}

bool SemanticsManager::getUniqueUnnamedMissionId(
    const VIMap& map, MissionId* id) {
  CHECK_NOTNULL(id);

  MissionIdList unnamed_mission_ids;
  getUnnamedMissionIds(map, &unnamed_mission_ids);

  if (unnamed_mission_ids.size() != 1u) {
    return false;
  } else {
    *id = unnamed_mission_ids.front();
    return true;
  }
}

}  // namespace vi_map
