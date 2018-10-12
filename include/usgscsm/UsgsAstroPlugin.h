#ifndef USGSAstroPlugin_h
#define USGSAstroPlugin_h


#include <string>

#include <Plugin.h>
#include <Version.h>

#include <json.hpp>
using json = nlohmann::json;

class USGSAstroPlugin : public csm::Plugin {

  public:
    USGSAstroPlugin();
    ~USGSAstroPlugin();

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
<<<<<<< HEAD:include/usgscsm/UsgsAstroPlugin.h
    static const USGSAstroPlugin m_registeredPlugin;
=======
    csm::Isd loadImageSupportData(const csm::Isd &imageSupportData) const; 

    static const UsgsAstroFramePlugin m_registeredPlugin;
>>>>>>> 399c1f28137f2b386fe2b2e454cc72ef104ec835:include/usgscsm/UsgsAstroFramePlugin.h
    static const std::string _PLUGIN_NAME;
    static const std::string _MANUFACTURER_NAME;
    static const std::string _RELEASE_DATE;
    static const int         _N_SENSOR_MODELS;
    static const int         _NUM_ISD_KEYWORDS;
    static const std::string _ISD_KEYWORD[];
    static const int         _NUM_STATE_KEYWORDS;
    static const std::string _STATE_KEYWORD[];
    static const json MODEL_KEYWORDS;

    typedef csm::Model* (*sensorConstructor)(void);
    static std::map<std::string, sensorConstructor> MODELS;
};

#endif
