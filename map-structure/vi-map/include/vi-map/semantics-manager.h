#ifndef VI_MAP_SEMANTICS_MANAGER_H_
#define VI_MAP_SEMANTICS_MANAGER_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "vi-map/unique-id.h"

namespace common {
template <typename IdType, typename DataType>
class MappedContainerBase;
}  // namespace common

namespace vi_map {
class VIMap;

class SemanticsManager {
 public:
  typedef std::unordered_map<MissionId, std::string> MissionNames;

  SemanticsManager();
  explicit SemanticsManager(const MissionNames& mission_names);

  // Returns empty string if mission is not named.
  std::string getNameOfMission(const MissionId& id) const;
  // Returns invalid id if name doesn't exist.
  MissionId getMissionIdForName(const std::string& query_name) const;

  // Picks an arbitrary MissionNames object for now. Creates one if one doesn't
  // exist yet.
  void nameMission(const MissionId& mission, const std::string& name);

  void getUnnamedMissionIds(const VIMap& map, MissionIdList* result);
  // Returns false if there are several unnamed missions.
  bool getUniqueUnnamedMissionId(const VIMap& map, MissionId* id);

 private:
  MissionNames mission_names_;
};

}  // namespace vi_map

#endif  // VI_MAP_SEMANTICS_MANAGER_H_
