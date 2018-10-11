#ifndef UsgsAstroFramePlugin_h
#define UsgsAstroFramePlugin_h


#include <string>

#include <Plugin.h>
#include <Version.h>

#ifdef _WIN32
# ifdef USGS_SENSOR_LIBRARY
#  define USGS_SENSOR_EXPORT_API __declspec(dllexport)
# else
#  define USGS_SENSOR_EXPORT_API __declspec(dllimport)
# endif
#elif LINUX_BUILD
# define USGS_SENSOR_EXPORT_API __attribute__ ((visibility("default")))
#else
#  define USGS_SENSOR_EXPORT_API
#endif

class USGS_SENSOR_EXPORT_API UsgsAstroFramePlugin : public csm::Plugin {

  public:
    UsgsAstroFramePlugin();
    ~UsgsAstroFramePlugin();

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

    // TODO when implementing, add any other necessary members.

private:
    csm::Isd loadImageSupportData(const csm::Isd &imageSupportData) const; 

    static const UsgsAstroFramePlugin m_registeredPlugin;
    static const std::string _PLUGIN_NAME;
    static const std::string _MANUFACTURER_NAME;
    static const std::string _RELEASE_DATE;
    static const int         _N_SENSOR_MODELS;
    static const int         _NUM_ISD_KEYWORDS;
    static const std::string _ISD_KEYWORD[];
    static const int         _NUM_STATE_KEYWORDS;
    static const std::string _STATE_KEYWORD[];
};

#endif
