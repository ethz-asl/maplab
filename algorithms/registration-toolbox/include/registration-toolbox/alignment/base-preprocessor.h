#ifndef REGISTRATION_TOOLBOX_ALIGNMENT_BASE_PREPROCESSOR_H_
#define REGISTRATION_TOOLBOX_ALIGNMENT_BASE_PREPROCESSOR_H_

namespace regbox {

template <typename T>
class BasePreprocessor {
 public:
  void preprocess(T* input) = 0;
};

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_ALIGNMENT_BASE_PREPROCESSOR_H_
