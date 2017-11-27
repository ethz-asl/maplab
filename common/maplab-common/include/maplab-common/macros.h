#ifndef MAPLAB_COMMON_MACROS_H_
#define MAPLAB_COMMON_MACROS_H_

#include <memory>

#include <aslam/common/memory.h>

#define MAPLAB_POINTER_TYPEDEFS(TypeName)             \
  typedef std::shared_ptr<TypeName> Ptr;              \
  typedef std::shared_ptr<const TypeName> ConstPtr;   \
  typedef AlignedUniquePtr<TypeName> UniquePtr;       \
  typedef std::weak_ptr<TypeName> WeakPtr;            \
  typedef std::weak_ptr<const TypeName> WeakConstPtr; \
  void definePointerTypedefs##__FILE__##__LINE__(void)

#define MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(TypeName) \
  TypeName(const TypeName&) = delete;               \
  void operator=(const TypeName&) = delete

#define MAPLAB_GET_AS_CASTER                                     \
  template <typename T>                                          \
  T& getAs() {                                                   \
    CHECK_NOTNULL(this);                                         \
    T* result = dynamic_cast<T*>(this); /* NOLINT */             \
    CHECK_NOTNULL(result);                                       \
    return *result;                                              \
  }                                                              \
  template <typename T>                                          \
  const T& getAs() const {                                       \
    CHECK_NOTNULL(this);                                         \
    const T* result = dynamic_cast<const T*>(this); /* NOLINT */ \
    CHECK_NOTNULL(result);                                       \
    return *result;                                              \
  }

// Disallow implicit argument type conversion at function call.
#define MAPLAB_DISALLOW_IMPLICIT_ARGUMENT_CAST_1(function_name, return_type) \
  template <typename ValueType1>                                             \
  return_type function_name(ValueType1) = delete
#define MAPLAB_DISALLOW_IMPLICIT_ARGUMENT_CAST_2(function_name, return_type) \
  template <typename ValueType1, typename ValueType2>                        \
  return_type function_name(ValueType1, ValueType2) = delete
#define MAPLAB_DISALLOW_IMPLICIT_ARGUMENT_CAST_3(function_name, return_type) \
  template <typename ValueType1, typename ValueType2, typename ValueType3>   \
  return_type function_name(ValueType1, ValueType2, ValueType3) = delete

#define MAPLAB_DEFINE_ENUM_HASHING(enum_type, underlying_type)   \
  struct enum_type##Hash {                                       \
    underlying_type operator()(const enum_type& hash_id) const { \
      return static_cast<underlying_type>(hash_id);              \
    }                                                            \
  };                                                             \
  extern void defineEnumHash##__FILE__##__LINE__(void)

// Augmented Google logging macros that include the name of the current
// function.
#define LOGF(x) LOG(x) << __func__ << "| "
#define VLOGF(x) VLOG(x) << __func__ << "| "

#define ASSERT_DERIVED(derived, base)        \
  static_assert(                             \
      std::is_base_of<base, derived>::value, \
      "Invalid derived "                     \
      "type.");

#endif  // MAPLAB_COMMON_MACROS_H_
