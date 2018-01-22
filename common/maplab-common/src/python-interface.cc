#include "maplab-common/python-interface.h"

#include "maplab-common/file-system-tools.h"

#include <vector>
#include <glog/logging.h>
#include <Python.h>

namespace common {

struct ScopedPyObject {
  ScopedPyObject() : obj(nullptr) {}

  explicit ScopedPyObject(PyObject* obj_) : obj(CHECK_NOTNULL(obj_)) {}

  ~ScopedPyObject() {
    free();
  }

  ScopedPyObject(const ScopedPyObject& other) {
    *this = other;
  }

  ScopedPyObject(const ScopedPyObject&& other) {
    obj = other.obj;
  }

  ScopedPyObject& operator=(const ScopedPyObject& other) {
    CHECK(this != &other);
    obj = other.obj;
    Py_INCREF(obj);
  }

  operator bool() const {
    return (obj != nullptr);
  }

  void free() {
    Py_XDECREF(obj);
    obj = nullptr;
  }

  PyObject* obj;
};

PythonInterface::PythonInterface(const std::string& python_script)
    : python_module_(new ScopedPyObject) {
  CHECK(!python_script.empty());

  Py_Initialize();
  CHECK(loadModule(python_script)) << "Could not load python module from: "
                                   << python_script;
}

PythonInterface::~PythonInterface() {
  unloadModule();
  Py_Finalize();
}

bool PythonInterface::loadModule(const std::string& python_script) {
  CHECK(python_module_ && !(*python_module_)) << "Module already loaded.";

  std::string module_path;
  std::string module_filename;
  common::splitPathAndFilename(python_script, &module_path, &module_filename);
  std::string module_name;
  std::string module_extension;
  common::splitFilePathAndExtension(
      module_filename, &module_name, &module_extension);

  if (!module_path.empty()) {
    PyRun_SimpleString("import sys");
    const std::string pycmd("sys.path.append(\"" + module_path + "\")");
    PyRun_SimpleString(pycmd.c_str());
  }
  CHECK(!module_name.empty());

  PyObject* pyname = PyString_FromString(module_name.c_str());
  python_module_->obj = PyImport_Import(pyname);
  Py_DECREF(pyname);
  return *python_module_;
}

void PythonInterface::unloadModule() {
  CHECK(python_module_ && *python_module_) << "Load module first.";
  python_module_->free();
}

ScopedPyObject* PythonInterface::getFunctionHandle(
    const std::string& function_name) const {
  CHECK(python_module_ && *python_module_) << "Module not loaded.";
  ScopedPyObject* function_handle = new ScopedPyObject;
  function_handle->obj =
      PyObject_GetAttrString(python_module_->obj, function_name.c_str());
  CHECK(*function_handle && PyCallable_Check(function_handle->obj))
      << "Cannot find function: " << function_name;
  return function_handle;
}

bool PythonInterface::callFunction(
    const std::string& function_name, const std::vector<double>& input,
    std::vector<double>* output) {
  CHECK_NOTNULL(output)->clear();
  CHECK(!function_name.empty());

  // Get the function handle.
  std::unique_ptr<ScopedPyObject> function_handle(
      getFunctionHandle(function_name));
  CHECK(function_handle && *function_handle);

  // Convert intput arguments.
  ScopedPyObject py_list(PyList_New(input.size()));
  for (size_t i = 0u; i < input.size(); ++i) {
    PyObject* py_value = PyFloat_FromDouble(input[i]);
    CHECK_NOTNULL(py_value);
    // Tuple will steal the py_value pointer, no need to release it.
    PyList_SetItem(py_list.obj, i, py_value);
  }
  ScopedPyObject args(PyTuple_New(1));
  CHECK(args);
  PyTuple_SetItem(args.obj, 0, py_list.obj);

  // Call the function.
  ScopedPyObject retval;
  retval.obj = PyObject_CallObject(function_handle->obj, args.obj);

  if (!retval) {
    PyErr_Print();
    LOG(WARNING) << "Call to function failed: " << function_name;
    return false;
  }

  // Convert the return value back.
  size_t return_size = PyList_Size(retval.obj);
  output->resize(return_size);
  for (size_t i = 0u; i < return_size; ++i) {
    PyObject* value = PyList_GetItem(retval.obj, i);
    CHECK_NOTNULL(value);
    (*output)[i] = PyFloat_AsDouble(value);
  }
  return true;
}

}  // namespace common
