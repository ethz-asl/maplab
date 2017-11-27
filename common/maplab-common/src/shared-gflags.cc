// These gflags are shared by multiple packages and generally used in
// combination with the common::Console or one of its child classes
// (e.g. MapLabConsole) and the console plugins.
// Gflags that are used solely in your package or the packages
// depending on it define it there.

#include <string>

#include <gflags/gflags.h>

DEFINE_string(map_key, "", "The map key for a console command.");
DEFINE_string(
    map_folder, "", "The folder on which a console command operates.");
DEFINE_bool(
    overwrite, false,
    "If set to true, existing files on the disks will get overwritten.");
DEFINE_string(map_mission, "", "The mission ID a command should operate on.");
DEFINE_string(
    map_mission_list, "",
    "List of comma separated mission IDs a command should operated on.");
