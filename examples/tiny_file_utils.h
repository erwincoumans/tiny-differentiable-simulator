#ifndef TINY_FILE_UTILS_H
#define TINY_FILE_UTILS_H

#ifdef __APPLE__
#include <mach-o/dyld.h> /* _NSGetExecutablePath */
#else
#ifdef _WIN32
#include <windows.h>
#else
// not Mac, not Windows, let's cross the fingers it is Linux :-)
#include <unistd.h>
#endif
#endif

#define TINY_MAX_EXE_PATH_LEN 4096

#include <stddef.h>  //ptrdiff_h
#include <stdio.h>
#include <string.h>

struct TinyResourcePath {
  char* m_path;
  TinyResourcePath(int len) {
    m_path = (char*)malloc(len);
    memset(m_path, 0, len);
  }
  virtual ~TinyResourcePath() { free(m_path); }
};

struct TinyFileUtils {
  // find_file searches files in 'data' folder in the current working directory
  // and relative to executable directory and many of their parent directories.
  static bool find_file(const std::string& org_file_name,
                        std::string& relative_file_name) {
    FILE* f = 0;
    f = fopen(org_file_name.c_str(), "rb");
    if (f) {
      relative_file_name = org_file_name;
      fclose(f);
      return true;
    }

    const std::string prefix[] = {
        "./",          "./data/",        "../data/",
        "../../data/", "../../../data/", "../../../../data/"};
    int num_prefixes = sizeof(prefix) / sizeof(std::string);

    f = 0;
    bool file_found = false;
    std::string root = "";

    for (int j = 0; j < 2; j++) {
      for (int i = 0; !f && i < num_prefixes; i++) {
        relative_file_name = root + prefix[i] + org_file_name;
        f = fopen(relative_file_name.c_str(), "rb");
        if (f) {
          file_found = true;
          break;
        }
      }
      char exe_path[TINY_MAX_EXE_PATH_LEN];
      int l = get_exe_path(exe_path, TINY_MAX_EXE_PATH_LEN);
      if (l) {
        char path_to_exe[TINY_MAX_EXE_PATH_LEN];
        int exe_name_pos =
            extract_path(exe_path, path_to_exe, TINY_MAX_EXE_PATH_LEN);
        if (exe_name_pos) {
          root = path_to_exe;
        }
      }
    }
    if (f) {
      fclose(f);
    }

    return file_found;
  }

  static const char* strip2(const char* name, const char* pattern) {
    size_t const patlen = strlen(pattern);
    size_t patcnt = 0;
    const char* oriptr;
    const char* patloc;
    // find how many times the pattern occurs in the original string
    for (oriptr = name; (patloc = strstr(oriptr, pattern));
         oriptr = patloc + patlen) {
      patcnt++;
    }
    return oriptr;
  }

  static int extract_path(const char* file_name, char* path,
                          int max_path_length) {
    const char* stripped = strip2(file_name, "/");
    stripped = strip2(stripped, "\\");

    ptrdiff_t len = stripped - file_name;
    assert((len + 1) < max_path_length);

    if (len && ((len + 1) < max_path_length)) {
      for (int i = 0; i < len; i++) {
        path[i] = file_name[i];
      }
      path[len] = 0;
    } else {
      len = 0;
      assert(max_path_length > 0);
      if (max_path_length > 0) {
        path[len] = 0;
      }
    }
    return len;
  }

  static int get_exe_path(char* path, int max_path_len_in_bytes) {
    int numBytes = 0;

#if __APPLE__
    uint32_t bufsize = uint32_t(max_path_len_in_bytes);

    if (_NSGetExecutablePath(path, &bufsize) != 0) {
      assert("Cannot find executable path\n");
      return false;
    } else {
      numBytes = strlen(path);
    }
#else
#ifdef _WIN32
    // https://msdn.microsoft.com/en-us/library/windows/desktop/ms683197(v=vs.85).aspx

    HMODULE hModule = GetModuleHandle(NULL);
    numBytes = GetModuleFileNameA(hModule, path, max_path_len_in_bytes);

#else
    /// http://stackoverflow.com/questions/933850/how-to-find-the-location-of-the-executable-in-c
    numBytes = (int)readlink("/proc/self/exe", path, max_path_len_in_bytes - 1);
    if (numBytes > 0) {
      path[numBytes] = 0;
    } else {
      assert("Cannot find executable path\n");
    }
#endif  //_WIN32
#endif  //__APPLE__

    return numBytes;
  }
};

#endif  // TINY_FILE_UTILS_H
