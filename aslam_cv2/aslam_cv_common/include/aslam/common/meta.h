#ifndef ASLAM_META_H_
#define ASLAM_META_H_

template<class>
class this_type {
 public:
  using is_valid = void;
};

template<class T, class sfinae_valid_type = void>
class is_cloneable : public std::false_type {};

template<class T>
class is_cloneable<T, typename this_type<decltype(std::declval<T&>().clone())>::is_valid> :
    public std::true_type {};

template<class T> struct is_not_pointer_helper : std::true_type {};
template<class T> struct is_not_pointer_helper<T*> : std::false_type {};
template<class T> struct is_not_pointer_helper<std::shared_ptr<T>> : std::false_type {};
template<class T> struct is_not_pointer_helper<std::unique_ptr<T>> : std::false_type {};
template<class T> struct is_not_pointer_helper<std::vector<std::shared_ptr<T>>> : std::false_type {};
template<class T> struct is_not_pointer : is_not_pointer_helper<typename std::remove_cv<T>::type> {};

#endif  // ASLAM_META_H_
