// Reproduces how ISIS/ASP load the plugin: links csmapi, does NOT link libusgscsm,
// loads it at runtime. Fails if the plugin registers into a private
// csm::Plugin::theList instead of this process's registry, which is what happens
// when a *static* csmapi is linked into the plugin.

#include <csm/Plugin.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <iostream>

// Returns true if the plugin loaded, printing the platform's reason if not.
static bool loadPlugin(const char *path) {
#ifdef _WIN32
  if (LoadLibraryA(path) != nullptr) {
    return true;
  }
  std::cerr << "LoadLibrary failed: " << GetLastError() << "\n";
#else
  if (dlopen(path, RTLD_LAZY) != nullptr) {
    return true;
  }
  std::cerr << "dlopen failed: " << dlerror() << "\n";
#endif
  return false;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0] << " <path to libusgscsm>\n";
    return 2;
  }

  if (!csm::Plugin::getList().empty()) {
    std::cerr << "registry unexpectedly non-empty before load\n";
    return 1;
  }

  if (!loadPlugin(argv[1])) {
    return 1;
  }

  const csm::PluginList &plugins = csm::Plugin::getList();
  for (const csm::Plugin *plugin : plugins) {
    if (plugin->getPluginName() == "UsgsAstroPluginCSM") {
      return 0;
    }
  }

  std::cerr << "UsgsAstroPluginCSM did not register with the host; the host sees "
            << plugins.size() << " plugin(s)\n";
  return 1;
}
