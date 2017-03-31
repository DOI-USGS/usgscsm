#ifndef ORexPlugin_h
#define ORexPlugin_h

#ifdef _WIN32
#  ifdef OREX_LIBRARY
#    define OREX_EXPORT_API __declspec(dllexport)
#  else
#    define OREX_EXPORT_API __declspec(dllimport)
# endif
#else
#  define OREX_EXPORT_API
#endif

#include <string>

#include <csm/Plugin.h>
#include <csm/Version.h>

namespace csm {
  class Isd;
  class Model;
  class Warning;
}

class OREX_EXPORT_API ORexPlugin : public csm::Plugin {

  public:
    ORexPlugin();
    ~ORexPlugin();

    virtual std::string getPluginName() const;
    virtual std::string getManufacturer() const;
    virtual std::string getReleaseDate() const;
    virtual csm::Version getCsmVersion() const;
    virtual size_t getNumModels() const;
    virtual std::string getModelName(size_t modelIndex) const;
    virtual std::string getModelFamily(size_t modelIndex) const;
    virtual csm::Version getModelVersion(const std::string &modelName) const;
    virtual bool canModelBeConstructedFromState(const std::string &modelName,
                                                const std::string &modelState,
                                                csm::WarningList *warnings = NULL) const;
    virtual bool canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings = NULL) const;
    virtual csm::Model *constructModelFromState(const std::string &modelState,
                                                csm::WarningList *warnings = NULL) const;
    virtual csm::Model *constructModelFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings = NULL) const;
    virtual std::string getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings = NULL) const;
    virtual bool canISDBeConvertedToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings = NULL) const;
    virtual std::string convertISDToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings = NULL) const;

  private:
    static const ORexPlugin m_registeredPlugin;

    static const std::string m_pluginName;
    static const std::string m_manufacturerName;
    static const std::string m_releaseDate;
    static const csm::Version m_csmVersion;
    static const int m_numModels;

    // TODO when implementing, add any other necessary members.
};

#endif
