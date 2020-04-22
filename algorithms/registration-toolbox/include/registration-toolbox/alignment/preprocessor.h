#ifndef REGISTRATION_TOOLBOX_ALIGNMENT_PREPROCESSOR_H_
#define REGISTRATION_TOOLBOX_ALIGNMENT_PREPROCESSOR_H_

#include "registration-toolbox/alignment/base-preprocessor.h"

namespace regbox {

template <typename T>
class Preprocessor : public BasePreprocessor<T> {
 public:
  void preprocess(T* input) override;
};

}  // namespace regbox

#endif  //  REGISTRATION_TOOLBOX_ALIGNMENT_PREPROCESSOR_H_
