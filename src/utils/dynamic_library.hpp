#pragma once

// Loads shared libraries at run time, such as the CUDA driver, NVRTC and
// compiled CPU kernels (kernel_loader.hpp), so that they are optional
// dependencies.

#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace tds {

class DynamicLibrary {
 public:
  DynamicLibrary() = default;
  ~DynamicLibrary() { close(); }
  DynamicLibrary(const DynamicLibrary&) = delete;
  DynamicLibrary& operator=(const DynamicLibrary&) = delete;

  // Opens the first library of the list that can be loaded. Returns false and
  // sets error() if none can be loaded.
  bool open(const std::vector<std::string>& names) {
    close();
    error_.clear();
    for (const auto& name : names) {
      if (name.empty()) continue;
#ifdef _WIN32
      handle_ = static_cast<void*>(LoadLibraryA(name.c_str()));
      if (!handle_) {
        error_ += name + ": error " + std::to_string(GetLastError()) + "\n";
      }
#else
      handle_ = dlopen(name.c_str(), RTLD_NOW | RTLD_LOCAL);
      if (!handle_) {
        const char* message = dlerror();
        error_ += (message ? std::string(message) : name) + "\n";
      }
#endif
      if (handle_) {
        path_ = name;
        error_.clear();
        return true;
      }
    }
    return false;
  }

  void close() {
    if (!handle_) return;
#ifdef _WIN32
    FreeLibrary(static_cast<HMODULE>(handle_));
#else
    dlclose(handle_);
#endif
    handle_ = nullptr;
  }

  bool is_open() const { return handle_ != nullptr; }
  const std::string& path() const { return path_; }
  const std::string& error() const { return error_; }

  void* symbol(const std::string& name) const {
    if (!handle_) return nullptr;
#ifdef _WIN32
    return reinterpret_cast<void*>(
        GetProcAddress(static_cast<HMODULE>(handle_), name.c_str()));
#else
    return dlsym(handle_, name.c_str());
#endif
  }

  // Looks up a function, returns false if it is missing.
  template <typename F>
  bool load(F& function, const std::string& name) const {
    function = reinterpret_cast<F>(symbol(name));
    return function != nullptr;
  }

 private:
  void* handle_{nullptr};
  std::string path_;
  std::string error_;
};

}  // namespace tds
