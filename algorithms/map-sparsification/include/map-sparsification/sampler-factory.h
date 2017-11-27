#ifndef MAP_SPARSIFICATION_SAMPLER_FACTORY_H_
#define MAP_SPARSIFICATION_SAMPLER_FACTORY_H_

#include "map-sparsification/sampler-base.h"

namespace map_sparsification {

SamplerBase::Ptr createSampler(SamplerBase::Type sampler_type);

}  // namespace map_sparsification
#endif  // MAP_SPARSIFICATION_SAMPLER_FACTORY_H_
