#ifndef MAPLAB_COMMON_TRAITS_H_
#define MAPLAB_COMMON_TRAITS_H_
#include <memory>

namespace common {

template <typename T>
struct IsPointerType {
  typedef T type;
  typedef const T& const_ref_type;
  enum { value = false };
};
template <typename T>
struct IsPointerType<T&> {
  typedef T type;
  typedef const T& const_ref_type;
  enum { value = false };
};
template <typename T>
struct IsPointerType<const T&> {
  typedef T type;
  typedef const T& const_ref_type;
  enum { value = false };
};
template <typename T>
struct IsPointerType<T*> {
  typedef T type;
  typedef const T& const_ref_type;
  enum { value = true };
};
template <typename T>
struct IsPointerType<const T*> {
  typedef T type;
  typedef const T& const_ref_type;
  enum { value = true };
};
template <typename T>
struct IsPointerType<std::shared_ptr<T> > {
  typedef T type;
  typedef const std::shared_ptr<const T> const_ref_type;
  enum { value = true };
};
template <typename T>
struct IsPointerType<const std::shared_ptr<T> > {
  typedef T type;
  typedef const std::shared_ptr<const T> const_ref_type;
  enum { value = true };
};
template <typename T>
struct IsPointerType<std::shared_ptr<const T> > {
  typedef T type;
  typedef const std::shared_ptr<const T> const_ref_type;
  enum { value = true };
};
template <typename T>
struct IsPointerType<const std::shared_ptr<const T> > {
  typedef T type;
  typedef const std::shared_ptr<const T> const_ref_type;
  enum { value = true };
};

}  // namespace common

#endif  // MAPLAB_COMMON_TRAITS_H_
