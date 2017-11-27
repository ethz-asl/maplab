#ifndef DENSE_RECONSTRUCTION_CONVERSION_TOOLS_H_
#define DENSE_RECONSTRUCTION_CONVERSION_TOOLS_H_

#include <vi-map/vi-map.h>

namespace dense_reconstruction {

bool convertAllDepthMapsToPointClouds(vi_map::VIMap* vi_map);

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_CONVERSION_TOOLS_H_
