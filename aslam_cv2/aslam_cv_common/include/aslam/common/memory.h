#ifndef ASLAM_COMMON_MEMORY_H_
#define ASLAM_COMMON_MEMORY_H_

#include <functional>
#include <map>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "aslam/common/eigen-hash.h"

template <template <typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type>>;

template <typename KeyType, typename ValueType>
using AlignedMap =
    std::map<KeyType, ValueType, std::less<KeyType>,
             Eigen::aligned_allocator<std::pair<const KeyType, ValueType>>>;

template <typename KeyType, typename ValueType>
using AlignedUnorderedMap = std::unordered_map<
    KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
    Eigen::aligned_allocator<std::pair<const KeyType, ValueType>>>;

template <typename KeyType, typename ValueType>
using AlignedUnorderedMultimap = std::unordered_multimap<
    KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
    Eigen::aligned_allocator<std::pair<const KeyType, ValueType>>>;

template <typename Type>
using AlignedUnorderedSet =
    std::unordered_set<Type, std::hash<Type>, std::equal_to<Type>,
                       Eigen::aligned_allocator<Type>>;

/// \brief Aligned allocator to be used like std::make_shared
///
/// To be used like:
///   std::shared_ptr my_ptr = aligned_shared<aslam::RadTanDistortion>(params);
template<typename Type, typename ... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  return std::allocate_shared<Type>(Eigen::aligned_allocator<TypeNonConst>(),
                                    std::forward<Arguments>(arguments)...);
}

namespace internal {
template <typename Type>
struct aligned_delete {
  constexpr aligned_delete() noexcept = default;

  template <typename TypeUp,
            typename = typename std::enable_if<
                std::is_convertible<TypeUp*, Type*>::value>::type>
  aligned_delete(const aligned_delete<TypeUp>&) noexcept {}

  void operator()(Type* ptr) const {
    static_assert(sizeof(Type) > 0, "Can't delete pointer to incomplete type!");
    typedef typename std::remove_const<Type>::type TypeNonConst;
    Eigen::aligned_allocator<TypeNonConst> allocator;
    allocator.destroy(ptr);
    allocator.deallocate(ptr, 1u /*num*/);
  }
};
}  // namespace internal

template <typename Type>
using AlignedUniquePtr = std::unique_ptr<
    Type, internal::aligned_delete<typename std::remove_const<Type>::type>>;

template <typename Type, typename... Arguments>
inline AlignedUniquePtr<Type> aligned_unique(Arguments&&... arguments) {
  typedef typename std::remove_const<Type>::type TypeNonConst;
  Eigen::aligned_allocator<TypeNonConst> allocator;
  TypeNonConst* obj = allocator.allocate(1u);
  allocator.construct(obj, std::forward<Arguments>(arguments)...);
  return std::move(AlignedUniquePtr<Type>(obj));
}

#endif  // ASLAM_COMMON_MEMORY_H_
