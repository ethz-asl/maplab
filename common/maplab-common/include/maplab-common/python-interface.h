#ifndef MAPLAB_COMMON_PYTHON_INTERFACE_H_
#define MAPLAB_COMMON_PYTHON_INTERFACE_H_

#include <memory>
#include <string>
#include <vector>

namespace common {
struct ScopedPyObject;

class PythonInterface {
 public:
  explicit PythonInterface(const std::string& python_script);
  ~PythonInterface();

  bool callFunction(
      const std::string& function_name, const std::vector<double>& input,
      std::vector<double>* output);

 private:
  bool loadModule(const std::string& python_script);
  void unloadModule();
  ScopedPyObject* getFunctionHandle(const std::string& function_name) const;

 private:
  std::unique_ptr<ScopedPyObject> python_module_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_PYTHON_INTERFACE_H_
