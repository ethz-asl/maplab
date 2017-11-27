#ifndef DESCRIPTOR_PROJECTION_TRAIN_PROJECTION_MATRIX_H_
#define DESCRIPTOR_PROJECTION_TRAIN_PROJECTION_MATRIX_H_
namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace descriptor_projection {
void TrainProjectionMatrix(const vi_map::VIMap& map);
}  // namespace descriptor_projection
#endif  // DESCRIPTOR_PROJECTION_TRAIN_PROJECTION_MATRIX_H_
