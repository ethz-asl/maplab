## Using the MapManager

The MapManager is the basic class to interact with and access the map storage.
This tutorial focuses on the usage with respect to VIMaps.

First, make sure that you include the MapManager and VIMap in your code:
```cpp
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>
```

Now, you can instantiate a new MapManager object, which will contain a reference to the map storage object (which is a singleton):
```cpp
vi_map::VIMapManager map_manager;
```

To get a map from the storage, you use
```cpp
vi_map::VIMap& map = map_manager.getMap("key_of_map");
```

Now, you can do your operations on the map.
When you're done, there's no need to commit the map back.

If you want to work with the map in a multithreaded environment, you can use thread safe accesses:
```cpp
{
  vi_map::VIMapManager::MapWriteAccess map = map_manager.getMapWriteAccess("key_of_map");
  // Map is now locked.

  // Work with map.
}
// Map is unlocked when the write access gets out of scope.
```
When the `MapWriteAccess` gets created, the map is automatically locked, blocking all other threads that want to get a thread safe access to this map.
Once the `MapWriteAccess` gets out of scope and is deleted, the map is unlocked and is free to be used by other threads.

If you only need read-access, you can instead use the `MapReadAccess`:
```cpp
vi_map::VIMapManager::MapReadAccess map = map_manager.getReadAccess("key_of_map");
```

A read access only allows `const` access to the map, so no changes can be made.
You can have multiple read accesses at the same time, but only on write access can be active.

To use the thread safe accesses, you can use a pointer-like syntax:
```cpp
vi_map::VIMapManager::MapWriteAccess map = map_manager.getMapWriteAccess("key_of_map");

// Dereference map.
vi_map::VIMap& map_reference = *map;

// Call a function in the map.
map->numMissions();

// Get a raw pointer to the map.
vi_map::VIMap* map_raw_pointer = map.get();
```

Check out the MapManager header to get a list of the possible commands you can do with the MapManger:
https://github.com/ethz-asl/maplab/blob/master/backend/map-manager/include/map-manager/map-manager.h
