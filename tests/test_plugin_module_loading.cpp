#include <cstdlib>
#include <iostream>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace {

void fail(const std::string& message) {
  std::cerr << "FAILED: " << message << std::endl;
  std::exit(1);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    fail("usage: test_plugin_module_loading <library_path> <symbol_name>");
  }

  const std::string library_path = argv[1];
  const std::string symbol_name = argv[2];

#if defined(_WIN32)
  HMODULE handle = LoadLibraryA(library_path.c_str());
  if (!handle) {
    fail("failed to load library: " + library_path);
  }

  const auto symbol = GetProcAddress(handle, symbol_name.c_str());
  if (!symbol) {
    FreeLibrary(handle);
    fail("failed to find symbol: " + symbol_name);
  }

  FreeLibrary(handle);
#else
  void* handle = dlopen(library_path.c_str(), RTLD_LAZY);
  if (!handle) {
    fail(std::string("failed to load library: ") + dlerror());
  }

  dlerror();
  const auto symbol = dlsym(handle, symbol_name.c_str());
  const char* error = dlerror();
  if (error != nullptr || symbol == nullptr) {
    dlclose(handle);
    fail(std::string("failed to find symbol: ") + (error ? error : symbol_name));
  }

  dlclose(handle);
#endif

  std::cout << "loaded " << library_path << " and found " << symbol_name << std::endl;
  return 0;
}
