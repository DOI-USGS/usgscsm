#ifndef UsgsAstroPlugin_h
#define UsgsAstroPlugin_h


#include <string>

#include <Plugin.h>
#include <Version.h>

#include <json/json.hpp>
using json = nlohmann::json;

class UsgsAstroPlugin : public csm::Plugin {

  public:
    UsgsAstroPlugin();
    ~UsgsAstroPlugin();

    virtual std::string getStateFromISD(csm::Isd imageSupportData) const;
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

    std::string loadImageSupportData(const csm::Isd &imageSupportDataOriginal) const;

    // TODO when implementing, add any other necessary members.

private:
    static const UsgsAstroPlugin m_registeredPlugin;
    static const std::string _PLUGIN_NAME;
    static const std::string _MANUFACTURER_NAME;
    static const std::string _RELEASE_DATE;
    static const int         _N_SENSOR_MODELS;

    typedef csm::Model* (*sensorConstructor)(void);
    static std::map<std::string, sensorConstructor> MODELS;
};

#endif
