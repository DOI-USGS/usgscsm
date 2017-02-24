#ifndef MdisPlugin_h
#define MdisPlugin_h

#ifdef _WIN32
#  ifdef MDIS_LIBRARY
#    define MDIS_EXPORT_API __declspec(dllexport)
#  else
#    define MDIS_EXPORT_API __declspec(dllimport)
# endif
#else
#  define MDIS_EXPORT_API
#endif 

#include <string>

#include <csm/Plugin.h>
#include <csm/Version.h> 

namespace csm {
  class Isd;
  class Model;
  class Warning;
}

class MDIS_EXPORT_API MdisPlugin : public csm::Plugin {

  public:
    MdisPlugin();
    ~MdisPlugin();

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
    static const MdisPlugin m_registeredPlugin;

    static const std::string m_pluginName;
    static const std::string m_manufacturerName;
    static const std::string m_releaseDate;
    static const csm::Version m_csmVersion;
    static const int m_numModels;

    // TODO when implementing, add any other necessary members.
};

#endif
