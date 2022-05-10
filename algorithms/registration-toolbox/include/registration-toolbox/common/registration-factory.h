#ifndef REGISTRATION_TOOLBOX_COMMON_REGISTRATION_FACTORY_H_
#define REGISTRATION_TOOLBOX_COMMON_REGISTRATION_FACTORY_H_

#include <cxxabi.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include <glog/logging.h>

#include "registration-toolbox/common/common.h"
#include "registration-toolbox/common/supported.h"

namespace regbox {

template <typename Base, typename... Args>
class RegistrationFactory {
 public:
  template <typename... T>
  static std::shared_ptr<Base> make(std::string&& s, T&&... args) {
    const std::string key = regbox::toLower(s);
    auto& controllers = data();
    if (controllers.count(key) == 0)
      return nullptr;
    return controllers.at(key)(std::forward<T>(args)...);
  }
  template <typename... T>
  static std::shared_ptr<Base> make(const std::string& s, T&&... args) {
    const std::string key = regbox::toLower(s);
    auto& controllers = data();
    if (controllers.count(key) == 0)
      return nullptr;
    return controllers.at(key)(std::forward<T>(args)...);
  }
  template <typename... T>
  static std::shared_ptr<Base> make(Aligner aligner, T&&... args) {
    const std::string key = SupportedAligner::getKeyForAligner(aligner);
    return make(key, std::forward<T>(args)...);
  }

  template <typename T>
  struct Registrar : Base {
    friend T;

    static bool registerT() {
      const auto name = regbox::toLower(regbox::nameOf<T>());
      RegistrationFactory::data()[name] =
          [](Args... args) -> std::shared_ptr<Base> {
        return std::make_shared<T>(std::forward<Args>(args)...);
      };
      return true;
    }
    static bool registered;

   private:
    Registrar() : Base(Key{}) {
      (void)registered;
    }
  };

  friend Base;

 protected:
  class Key {
    Key() {}
    template <typename T>
    friend struct Registrar;
  };
  using FuncType = std::shared_ptr<Base> (*)(Args...);
  RegistrationFactory() = default;

  static auto data() -> std::unordered_map<std::string, FuncType>& {

    static std::unordered_map<std::string, FuncType> singleton;
    return singleton;
  }
};

template <typename Base, typename... T_Args>
template <typename T>
bool RegistrationFactory<Base, T_Args...>::Registrar<T>::registered =
    RegistrationFactory<Base, T_Args...>::Registrar<T>::registerT();

}  // namespace regbox

#endif  // REGISTRATION_TOOLBOX_COMMON_REGISTRATION_FACTORY_H_
