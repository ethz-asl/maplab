#include <map>

namespace aslam {

inline std::ostream& operator<<(std::ostream& out, const Distortion::Type& value) {
  static std::map<Distortion::Type, std::string> names;
  if (names.size() == 0) {
    #define INSERT_ELEMENT(type, val) names[type::val] = #val
    INSERT_ELEMENT(Distortion::Type, kNoDistortion);
    INSERT_ELEMENT(Distortion::Type, kEquidistant);
    INSERT_ELEMENT(Distortion::Type, kFisheye);
    INSERT_ELEMENT(Distortion::Type, kRadTan);
    #undef INSERT_ELEMENT
  }
  return out << names[value];
}

}  // namespace aslam
