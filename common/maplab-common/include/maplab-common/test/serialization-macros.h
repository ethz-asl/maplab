#ifndef MAPLAB_COMMON_SERIALIZATION_MACROS_H_
#define MAPLAB_COMMON_SERIALIZATION_MACROS_H_
#include <memory>
#include <sstream>  // NOLINT
#include <string>
#include <type_traits>

#include <glog/logging.h>
#include <maplab-common/traits.h>

namespace cv {
class Mat;
}  // namespace cv

namespace common_serialization {
namespace internal_types {
typedef char yes;
typedef int no;
}  // namespace internal_types
}  // namespace common_serialization

struct AnyT {
  template <class T>
  explicit AnyT(const T&);
};

common_serialization::internal_types::no operator<<(const AnyT&, const AnyT&);

namespace common {
namespace serialization {
namespace internal {

template <class T>
common_serialization::internal_types::yes check(const T&);
common_serialization::internal_types::no check(
    common_serialization::internal_types::no);

struct makeCompilerSilent {  // Rm warning about unused functions.
  void foo() {
    check(common_serialization::internal_types::no());
    AnyT t(5);
    operator<<(t, t);
  }
};

// This template metaprogramming struct can tell us if there is the operator<<
// defined somewhere.
template <typename StreamType, typename T>
class HasOStreamOperator {
  static StreamType& stream;
  static T& x;

 public:
  enum {
    value = sizeof(check(stream << x)) ==
            sizeof(common_serialization::internal_types::yes)
  };
};

template <typename StreamType, typename T>
class HasOStreamOperator<StreamType, std::shared_ptr<T> > {
 public:
  enum { value = HasOStreamOperator<StreamType, T>::value };
};

template <typename StreamType, typename T>
class HasOStreamOperator<StreamType, T*> {
 public:
  enum { value = HasOStreamOperator<StreamType, T>::value };
};

template <typename StreamType, typename T>
class HasOStreamOperator<StreamType, T&> {
 public:
  enum { value = HasOStreamOperator<StreamType, T>::value };
};

// This template metaprogramming struct can tell us whether a class has a member
// function isBinaryEqual.
template <typename T>
class HasIsBinaryEqual {
  // For non const methods (which is actually wrong).
  template <typename U, bool (U::*)(const T&)>
  struct Check;
  template <typename U>
  static char func(Check<U, &U::isBinaryEqual>*);
  template <typename U>
  static int func(...);
  // For const methods.
  template <typename U, bool (U::*)(const T&) const>
  struct CheckConst;
  template <typename U>
  static char funcconst(CheckConst<U, &U::isBinaryEqual>*);
  template <typename U>
  static int funcconst(...);

 public:
  enum {
    value = static_cast<int>(
        (sizeof(func<T>(0)) == sizeof(char)) ||     // NOLINT
        (sizeof(funcconst<T>(0)) == sizeof(char)))  // NOLINT
  };
};
template <typename T>
class HasIsBinaryEqual<std::shared_ptr<T> > {
 public:
  enum { value = HasIsBinaryEqual<T>::value };
};
template <typename T>
class HasIsBinaryEqual<T*> {
 public:
  enum { value = HasIsBinaryEqual<T>::value };
};
template <typename T>
class HasIsBinaryEqual<T&> {
 public:
  enum { value = HasIsBinaryEqual<T>::value };
};

// These structs are used to choose between isBinaryEqual function call and the
// default operator==.
template <bool, typename A>
struct isSame;

template <typename A>
struct isSame<true, A> {
  static bool eval(const A& lhs, const A& rhs) {
    return lhs.isBinaryEqual(rhs);
  }
};

template <typename A>
struct isSame<true, std::shared_ptr<A> > {
  static bool eval(
      const std::shared_ptr<A>& lhs, const std::shared_ptr<A>& rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return lhs->isBinaryEqual(*rhs);
  }
};

template <typename A>
struct isSame<true, A*> {
  static bool eval(const A* const lhs, const A* const rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return lhs->isBinaryEqual(*rhs);
  }
};

template <typename A>
struct isSame<false, A> {
  static bool eval(const A& lhs, const A& rhs) {
    return lhs == rhs;
  }
};

template <typename A>
struct isSame<false, A*> {
  static bool eval(A const* lhs, A const* rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return *lhs == *rhs;
  }
};

template <typename A>
struct isSame<false, std::shared_ptr<A> > {
  static bool eval(
      const std::shared_ptr<A>& lhs, const std::shared_ptr<A>& rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return *lhs == *rhs;
  }
};

// For opencv Mat we have cannot use this method since we otherwise have to
// depend on opencv.
template <typename T, bool B>
struct checkTypeIsNotOpencvMat {
  enum {
    value = true,
  };
};

template <bool B>
struct checkTypeIsNotOpencvMat<cv::Mat, B> {
  enum {
    value = true,  // True is correct here.
  };
  static_assert(
      (B == !B),  // false
      "You cannot use the macro CHECKSAME or "
      "CHECKSAMEMEMBER on opencv mat. Use sm::opencv::isBinaryEqual instead");
};

template <bool B>
struct checkTypeIsNotOpencvMat<std::shared_ptr<cv::Mat>, B> {
  enum {
    value = true,  // Yes true is correct here.
  };
  static_assert(
      (B == !B) /*false*/,
      "You cannot use the macro CHECKSAME or "
      "CHECKSAMEMEMBER on opencv mat. Use sm::opencv::isBinaryEqual instead");
};

template <bool B>
struct checkTypeIsNotOpencvMat<cv::Mat*, B> {
  enum {
    value = true,  // Yes true is correct here.
  };
  static_assert(
      (B == !B) /*false*/,
      "You cannot use the macro CHECKSAME or "
      "CHECKSAMEMEMBER on opencv mat.");
};

// If the object supports it stream to ostream, otherwise put NA.
template <bool, typename A>
struct streamIf;

template <typename A>
struct streamIf<true, A> {
  static std::string eval(const A& rhs) {
    std::stringstream ss;
    ss << rhs;
    return ss.str();
  }
};

template <typename A>
struct streamIf<true, A*> {
  static std::string eval(const A* rhs) {
    if (!rhs) {
      return "NULL";
    }
    std::stringstream ss;
    ss << *rhs;
    return ss.str();
  }
};

template <typename A>
struct streamIf<true, std::shared_ptr<A> > {
  static std::string eval(const std::shared_ptr<A>& rhs) {
    if (!rhs) {
      return "NULL";
    }
    std::stringstream ss;
    ss << *rhs;
    return ss.str();
  }
};

template <typename A>
struct streamIf<true, std::shared_ptr<const A> > {
  static std::string eval(const std::shared_ptr<const A>& rhs) {
    if (!rhs) {
      return "NULL";
    }
    std::stringstream ss;
    ss << *rhs;
    return ss.str();
  }
};

template <typename A>
struct streamIf<false, A> {
  static std::string eval(const A&) {
    return "NA";
  }
};

struct VerboseChecker {
  static bool SetVerbose(bool verbose) {
    Instance().verbose_ = verbose;
    return true;  // Intended.
  }
  static bool Verbose() {
    return Instance().verbose_;
  }

 private:
  static VerboseChecker& Instance() {
    static VerboseChecker instance;
    return instance;
  }
  VerboseChecker() : verbose_(false) {}
  VerboseChecker(const VerboseChecker&);
  VerboseChecker& operator==(const VerboseChecker&);
  bool verbose_;
};
}  // namespace internal
}  // namespace serialization
}  // namespace common

#define IS_CHECKSAME_CURRENTLY_VERBOSE \
  (common::serialization::internal::VerboseChecker::Verbose())

#define SET_CHECKSAME_VERBOSITY(verbose) \
  common::serialization::internal::VerboseChecker::SetVerbose(verbose)

#define SET_CHECKSAME_VERBOSE SET_CHECKSAME_VERBOSITY(true)

#define SET_CHECKSAME_SILENT SET_CHECKSAME_VERBOSITY(false)

#define SERIALIZATION_CHECKSAME_VERBOSE(THIS, OTHER, VERBOSE) \
  SET_CHECKSAME_VERBOSITY(VERBOSE) && SERIALIZATION_CHECKSAME_IMPL(THIS, OTHER)

#define SERIALIZATION_CHECKSAME_IMPL(THIS, OTHER)                              \
  (common::serialization::internal::checkTypeIsNotOpencvMat<                   \
       typename sm::common::StripConstReference<decltype(OTHER)>::result_t,    \
       false>::value &&                                                        \
   /*NOLINT*/ common::serialization::internal::isSame<                         \
       common::serialization::internal::                                       \
           HasIsBinaryEqual</* first run the test of equality: either          \
                               isBinaryEqual or op== */                        \
                            typename sm::common::StripConstReference<decltype( \
                                OTHER)>::result_t>::value,                     \
       typename sm::common::StripConstReference<decltype(                      \
           OTHER)>::result_t>::eval(THIS, OTHER))                              \
      ? true                                                                   \
      : /*return true if good*/                                                \
      (                                                                        \
          IS_CHECKSAME_CURRENTLY_VERBOSE                                       \
              ? (LOG(ERROR)                                                    \
                 << "*** Validation failed on " << #OTHER << ":\n"             \
                 << common::serialization::internal::streamIf<                 \
                        common::serialization::internal::                      \
                            HasOStreamOperator<                                \
                                std::ostream, /* here we check whether         \
                                                 operator<< is available */    \
                                typename sm::common::StripConstReference<      \
                                    decltype(OTHER)>::result_t>::value,        \
                        typename sm::common::StripConstReference<decltype(     \
                            OTHER)>::result_t>::eval(THIS)                     \
                 << "other:\n"                                                 \
                 << common::serialization::internal::streamIf<                 \
                        common::serialization::internal::HasOStreamOperator<   \
                            std::ostream,                                      \
                            typename sm::common::StripConstReference<decltype( \
                                OTHER)>::result_t>::value,                     \
                        typename sm::common::StripConstReference<decltype(     \
                            OTHER)>::result_t>::eval(OTHER)) &&                \
                    false                                                      \
              : false)

#define SERIALIZATION_CHECKMEMBERSSAME_VERBOSE(OTHER, MEMBER, VERBOSE) \
    (SET_CHECKSAME_VERBOSITY(VERBOSE) && \
        SERIALIZATION_CHECKMEMBERSSAME_IMPL(OTHER, MEMBER)

#define SERIALIZATION_CHECKMEMBERSSAME_IMPL(OTHER, MEMBER)                    \
  ((common::serialization::internal::checkTypeIsNotOpencvMat<                 \
       typename sm::common::StripConstReference<decltype(OTHER)>::result_t,   \
       false>::value) &&                                                      \
   /*NOLINT*/ (                                                               \
       common::serialization::internal::isSame<                               \
           common::serialization::internal::HasIsBinaryEqual<                 \
               typename sm::common::StripConstReference<decltype(             \
                   MEMBER)>::result_t>::value,                                \
           typename sm::common::StripConstReference<decltype(                 \
               MEMBER)>::result_t>::eval(this->MEMBER, OTHER.MEMBER)))        \
      ? true                                                                  \
      : (IS_CHECKSAME_CURRENTLY_VERBOSE                                       \
             ? (LOG(ERROR)                                                    \
                << "*** Validation failed on " << #MEMBER << ":\n"            \
                << common::serialization::internal::streamIf<                 \
                       common::serialization::internal::HasOStreamOperator<   \
                           std::ostream,                                      \
                           typename sm::common::StripConstReference<decltype( \
                               MEMBER)>::result_t>::value,                    \
                       typename sm::common::StripConstReference<decltype(     \
                           MEMBER)>::result_t>::eval(this->MEMBER)            \
                << "\nother:\n"                                               \
                << common::serialization::internal::streamIf<                 \
                       common::serialization::internal::HasOStreamOperator<   \
                           std::ostream,                                      \
                           typename sm::common::StripConstReference<decltype( \
                               MEMBER)>::result_t>::value,                    \
                       typename sm::common::StripConstReference<decltype(     \
                           MEMBER)>::result_t>::eval(OTHER.MEMBER)) &&        \
                   false                                                      \
             : false)

// This is an internal default macro parameter deduction.
#define SERIALIZATION_GET_3RD_ARG(arg1, arg2, arg3, ...) arg3
#define SERIALIZATION_GET_4TH_ARG(arg1, arg2, arg3, arg4, ...) arg4

#define SERIALIZATION_MACRO_CHOOSER_MEMBER_SAME(...)       \
  SERIALIZATION_GET_4TH_ARG(                               \
      __VA_ARGS__, SERIALIZATION_CHECKMEMBERSSAME_VERBOSE, \
      SERIALIZATION_CHECKMEMBERSSAME_IMPL)
#define SERIALIZATION_MACRO_CHOOSER_SAME(...)       \
  SERIALIZATION_GET_4TH_ARG(                        \
      __VA_ARGS__, SERIALIZATION_CHECKSAME_VERBOSE, \
      SERIALIZATION_CHECKSAME_IMPL)

// \brief This macro checks this->MEMBER against OTHER.MEMBER  with the
// appropriate IsBinaryEqual or operator==.
// Pointers and std::shared_ptr are handled automatically.
#define CHECKMEMBERSSAME(...) \
  SERIALIZATION_MACRO_CHOOSER_MEMBER_SAME(__VA_ARGS__)(__VA_ARGS__)

// \brief This macro checks THIS against OTHER with the appropriate
// IsBinaryEqual or operator==.
// Pointers and std::shared_ptr are handled automatically.
#define CHECKSAME(...) \
  SERIALIZATION_MACRO_CHOOSER_SAME(__VA_ARGS__)(__VA_ARGS__)

#endif  // MAPLAB_COMMON_SERIALIZATION_MACROS_H_
