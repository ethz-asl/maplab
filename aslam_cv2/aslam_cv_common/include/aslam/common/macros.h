#ifndef ASLAM_COMMON_MACROS_H_
#define ASLAM_COMMON_MACROS_H_

#include <memory>

#include <aslam/common/memory.h>

#define ASLAM_DISALLOW_EVIL_CONSTRUCTORS(TypeName)     \
  TypeName(const TypeName&) = delete;                  \
  void operator=(const TypeName&) = delete

#define ASLAM_POINTER_TYPEDEFS(TypeName)               \
  typedef AlignedUniquePtr<TypeName> UniquePtr;        \
  typedef std::shared_ptr<TypeName> Ptr;               \
  typedef std::shared_ptr<const TypeName> ConstPtr

/// Extract the type from an expression which wraps a type inside braces. This
/// is done to protect the commas in some types.
template<typename T> struct ArgumentType;
template<typename T, typename U> struct ArgumentType<T(U)> { typedef U Type; };
#define GET_TYPE(TYPE) ArgumentType<void(TYPE)>::Type

#define ASLAM_FORWARD_MEMBER_CONST_ITERATORS(TypeName, member_variable) \
  typedef TypeName::value_type value_type;                              \
  typedef TypeName::const_iterator const_iterator;                      \
  TypeName::const_iterator begin() const {                              \
    return member_variable.begin();                                     \
  }                                                                     \
  TypeName::const_iterator end() const {                                \
    return member_variable.end();                                       \
  }

namespace aslam {
namespace common {
namespace macros {

constexpr double kEpsilon = 1e-8;

}  // namespace macros
}  // namespace common
}  // namespace aslam

#endif  // ASLAM_COMMON_MACROS_H_
