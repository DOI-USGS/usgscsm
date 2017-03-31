#include "ORexPlugin.h"

#include <cstdlib>
#include <string>

#include <csm/csm.h>
#include <csm/Error.h>
#include <csm/Plugin.h>
#include <csm/Warning.h>

#include "ORexSensorModel.h"

// Create static instance of self for plugin registration to work with csm::Plugin
const ORexPlugin ORexPlugin::m_registeredPlugin;

ORexPlugin::ORexPlugin() {
}


ORexPlugin::~ORexPlugin() {
}


std::string ORexPlugin::getPluginName() const {
  return "UsgsAstroFrameORexPluginCSM";
}


std::string ORexPlugin::getManufacturer() const {
  return "UsgsAstrogeology";
}


std::string ORexPlugin::getReleaseDate() const {
  return "TBA";
}


csm::Version ORexPlugin::getCsmVersion() const {
  return csm::Version(3, 1, 0);
}


size_t ORexPlugin::getNumModels() const {
  return 1;
}


std::string ORexPlugin::getModelName(size_t modelIndex) const {

  return ORexSensorModel::_SENSOR_MODEL_NAME;
}


std::string ORexPlugin::getModelFamily(size_t modelIndex) const {
  return "Raster";
}


csm::Version ORexPlugin::getModelVersion(const std::string &modelName) const {

  return csm::Version(1, 0, 0);
}


bool ORexPlugin::canModelBeConstructedFromState(const std::string &modelName,
                                                const std::string &modelState,
                                                csm::WarningList *warnings) const {
  return false;
}


bool ORexPlugin::canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {

  if (modelName != ORexSensorModel::_SENSOR_MODEL_NAME) {
    return false;
  }

  return true;
}


csm::Model *ORexPlugin::constructModelFromState(const std::string&modelState,
                                                csm::WarningList *warnings) const {
  return NULL;
}


csm::Model *ORexPlugin::constructModelFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {

  // Check if the sensor model can be constructed from ISD given the model name
  if (!canModelBeConstructedFromISD(imageSupportData, modelName)) {
    throw csm::Error(csm::Error::ISD_NOT_SUPPORTED,
                     "Sensor model support data provided is not supported by this plugin",
                     "ORexPlugin::constructModelFromISD");
  }

  ORexSensorModel *sensorModel = new ORexSensorModel();

  // Keep track of necessary keywords that are missing from the ISD.
  std::vector<std::string> missingKeywords;

  sensorModel->m_targetName = imageSupportData.param("target_name");

  sensorModel->m_ifov = atof(imageSupportData.param("ifov").c_str());

  sensorModel->m_instrumentID = imageSupportData.param("instrument_id");
  if (imageSupportData.param("instrument_id") == "") {
    missingKeywords.push_back("instrument_id");
  }
  sensorModel->m_focalLength = atof(imageSupportData.param("focal_length").c_str());
  if (imageSupportData.param("focal_length") == "") {
    missingKeywords.push_back("focal_length");
  }
  sensorModel->m_focalLengthEpsilon =
      atof(imageSupportData.param("focal_length_epsilon").c_str());

  sensorModel->m_spacecraftPosition[0] =
      atof(imageSupportData.param("x_sensor_origin").c_str());
  sensorModel->m_spacecraftPosition[1] =
      atof(imageSupportData.param("y_sensor_origin").c_str());
  sensorModel->m_spacecraftPosition[2] =
      atof(imageSupportData.param("z_sensor_origin").c_str());
  if (imageSupportData.param("x_sensor_origin") == "") {
    missingKeywords.push_back("x_sensor_origin");
  }
  if (imageSupportData.param("y_sensor_origin") == "") {
    missingKeywords.push_back("y_sensor_origin");
  }
  if (imageSupportData.param("z_sensor_origin") == "") {
    missingKeywords.push_back("z_sensor_origin");
  }

  sensorModel->m_spacecraftVelocity[0] =
      atof(imageSupportData.param("x_sensor_velocity").c_str());
  sensorModel->m_spacecraftVelocity[1] =
      atof(imageSupportData.param("y_sensor_velocity").c_str());
  sensorModel->m_spacecraftVelocity[2] =
      atof(imageSupportData.param("z_sensor_velocity").c_str());
  // sensor velocity not strictly necessary?

  sensorModel->m_sunPosition[0] =
      atof(imageSupportData.param("x_sun_position").c_str());
  sensorModel->m_sunPosition[1] =
      atof(imageSupportData.param("y_sun_position").c_str());
  sensorModel->m_sunPosition[2] =
      atof(imageSupportData.param("z_sun_position").c_str());
  // sun position is not strictly necessary, but is required for getIlluminationDirection.

  sensorModel->m_omega = atof(imageSupportData.param("omega").c_str());
  sensorModel->m_phi = atof(imageSupportData.param("phi").c_str());
  sensorModel->m_kappa = atof(imageSupportData.param("kappa").c_str());
  if (imageSupportData.param("omega") == "") {
    missingKeywords.push_back("omega");
  }
  if (imageSupportData.param("phi") == "") {
    missingKeywords.push_back("phi");
  }
  if (imageSupportData.param("kappa") == "") {
    missingKeywords.push_back("kappa");
  }

  sensorModel->m_ccdCenter[0] = atof(imageSupportData.param("ccd_center", 0).c_str());
  sensorModel->m_ccdCenter[1] = atof(imageSupportData.param("ccd_center", 1).c_str());

  sensorModel->m_originalHalfLines = atof(imageSupportData.param("original_half_lines").c_str());
  sensorModel->m_spacecraftName = imageSupportData.param("spacecraft_name");

  sensorModel->m_pixelPitch = atof(imageSupportData.param("pixel_pitch").c_str());

  sensorModel->m_ephemerisTime = atof(imageSupportData.param("ephemeris_time").c_str());
  if (imageSupportData.param("ephemeris_time") == "") {
    missingKeywords.push_back("ephemeris_time");
  }

  sensorModel->m_originalHalfSamples =
      atof(imageSupportData.param("original_half_samples").c_str());

  sensorModel->m_boresight[0] = atof(imageSupportData.param("boresight", 0).c_str());
  sensorModel->m_boresight[1] = atof(imageSupportData.param("boresight", 1).c_str());
  sensorModel->m_boresight[2] = atof(imageSupportData.param("boresight", 2).c_str());


  sensorModel->m_nLines = atoi(imageSupportData.param("nlines").c_str());
  sensorModel->m_nSamples = atoi(imageSupportData.param("nsamples").c_str());
  if (imageSupportData.param("nlines") == "") {
    missingKeywords.push_back("nlines");
  }
  if (imageSupportData.param("nsamples") == "") {
    missingKeywords.push_back("nsamples");
  }

  sensorModel->m_transY[0] = atof(imageSupportData.param("transy", 0).c_str());
  sensorModel->m_transY[1] = atof(imageSupportData.param("transy", 1).c_str());
  sensorModel->m_transY[2] = atof(imageSupportData.param("transy", 2).c_str());
  if (imageSupportData.param("transy", 0) == "") {
    missingKeywords.push_back("transy 0");
  }
  else if (imageSupportData.param("transy", 1) == "") {
    missingKeywords.push_back("transy 1");
  }
  else if (imageSupportData.param("transy", 2) == "") {
    missingKeywords.push_back("transy 2");
  }

  sensorModel->m_transX[0] = atof(imageSupportData.param("transx", 0).c_str());
  sensorModel->m_transX[1] = atof(imageSupportData.param("transx", 1).c_str());
  sensorModel->m_transX[2] = atof(imageSupportData.param("transx", 2).c_str());
  if (imageSupportData.param("transx", 0) == "") {
    missingKeywords.push_back("transx 0");
  }
  else if (imageSupportData.param("transx", 1) == "") {
    missingKeywords.push_back("transx 1");
  }
  else if (imageSupportData.param("transx", 2) == "") {
    missingKeywords.push_back("transx");
  }

  sensorModel->m_a_axis = 1000 * atof(imageSupportData.param("semi_a_axis").c_str());
  if (imageSupportData.param("semi_a_axis") == "") {
    missingKeywords.push_back("semi_a_axis");
  }
  sensorModel->m_b_axis = 1000 * atof(imageSupportData.param("semi_b_axis").c_str());
  if (imageSupportData.param("semi_b_axis") == "") {
    missingKeywords.push_back("semi_b_axis");
  }
  sensorModel->m_c_axis = 1000 * atof(imageSupportData.param("semi_c_axis").c_str());
  if (imageSupportData.param("semi_c_axis") == "") {
    missingKeywords.push_back("semi_c_axis");
  }

  // If we are missing necessary keywords from ISD, we cannot create a valid sensor model.
  if (missingKeywords.size() != 0) {

    std::string errorMessage = "ISD is missing the necessary keywords: [";

    for (int i = 0; i < missingKeywords.size(); i++) {
      if (i == missingKeywords.size() - 1) {
        errorMessage += missingKeywords[i] + "]";
      }
      else {
        errorMessage += missingKeywords[i] + ", ";
      }
    }

    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     errorMessage,
                     "ORexPlugin::constructModelFromISD");
  }

  return sensorModel;
}


std::string ORexPlugin::getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings) const {
  return "state";
}


bool ORexPlugin::canISDBeConvertedToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  return false;
}


std::string ORexPlugin::convertISDToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  return "state";
}
