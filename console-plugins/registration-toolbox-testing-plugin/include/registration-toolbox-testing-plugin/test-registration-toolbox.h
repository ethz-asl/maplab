#ifndef REGISTRATION_TOOLBOX_TESTING_PLUGIN_TEST_REGISTRATION_TOOLBOX_H_
#define REGISTRATION_TOOLBOX_TESTING_PLUGIN_TEST_REGISTRATION_TOOLBOX_H_

#include <dense-mapping/dense-mapping.h>
#include <vi-map/vi-map.h>

namespace registration_toolbox_testing {

bool testRegistrationToolboxWithMap(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* vi_map_ptr);

}  // namespace registration_toolbox_testing
#endif  //  REGISTRATION_TOOLBOX_TESTING_PLUGIN_TEST_REGISTRATION_TOOLBOX_H_
