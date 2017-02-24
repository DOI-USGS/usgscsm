#include "MdisPlugin.h"

#include <cstdlib>
#include <string>

#include <csm/csm.h>
#include <csm/Error.h>
#include <csm/Plugin.h>
#include <csm/Warning.h>

#include "MdisNacSensorModel.h"

// Create static instance of self for plugin registration to work with csm::Plugin
const MdisPlugin MdisPlugin::m_registeredPlugin;

MdisPlugin::MdisPlugin() {
}


MdisPlugin::~MdisPlugin() {
}


std::string MdisPlugin::getPluginName() const {
  return "UsgsAstroFrameMdisPluginCSM";
}


std::string MdisPlugin::getManufacturer() const {
  return "UsgsAstrogeology";
}


std::string MdisPlugin::getReleaseDate() const {
  return "TBA";
}


csm::Version MdisPlugin::getCsmVersion() const {
  return csm::Version(3, 1, 0);
}


size_t MdisPlugin::getNumModels() const {
  return 1;
}


std::string MdisPlugin::getModelName(size_t modelIndex) const {

  return MdisNacSensorModel::_SENSOR_MODEL_NAME;
}


std::string MdisPlugin::getModelFamily(size_t modelIndex) const {
  return "Raster";
}


csm::Version MdisPlugin::getModelVersion(const std::string &modelName) const {

  return csm::Version(1, 0, 0);
}


bool MdisPlugin::canModelBeConstructedFromState(const std::string &modelName,
                                                const std::string &modelState,
                                                csm::WarningList *warnings) const {
  return false;
}


bool MdisPlugin::canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {

  if (modelName != MdisNacSensorModel::_SENSOR_MODEL_NAME) {
    return false;
  }

  return true;
}


csm::Model *MdisPlugin::constructModelFromState(const std::string&modelState,
                                                csm::WarningList *warnings) const {
  return NULL;
}


csm::Model *MdisPlugin::constructModelFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {

  // Check if the sensor model can be constructed from ISD given the model name
  if (!canModelBeConstructedFromISD(imageSupportData, modelName)) {
    throw csm::Error(csm::Error::ISD_NOT_SUPPORTED,
                     "Sensor model support data provided is not supported by this plugin",
                     "MdisPlugin::constructModelFromISD");
  }

  MdisNacSensorModel *sensorModel = new MdisNacSensorModel();

  // Keep track of necessary keywords that are missing from the ISD.
  std::vector<std::string> missingKeywords;

  sensorModel->m_startingDetectorSample =
      atof(imageSupportData.param("starting_detector_sample").c_str());
  sensorModel->m_startingDetectorLine =
      atof(imageSupportData.param("starting_detector_line").c_str());

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

  sensorModel->m_odtX[0] = atof(imageSupportData.param("odt_x", 0).c_str());
  sensorModel->m_odtX[1] = atof(imageSupportData.param("odt_x", 1).c_str());
  sensorModel->m_odtX[2] = atof(imageSupportData.param("odt_x", 2).c_str());
  sensorModel->m_odtX[3] = atof(imageSupportData.param("odt_x", 3).c_str());
  sensorModel->m_odtX[4] = atof(imageSupportData.param("odt_x", 4).c_str());
  sensorModel->m_odtX[5] = atof(imageSupportData.param("odt_x", 5).c_str());
  sensorModel->m_odtX[6] = atof(imageSupportData.param("odt_x", 6).c_str());
  sensorModel->m_odtX[7] = atof(imageSupportData.param("odt_x", 7).c_str());
  sensorModel->m_odtX[8] = atof(imageSupportData.param("odt_x", 8).c_str());
  sensorModel->m_odtX[9] = atof(imageSupportData.param("odt_x", 9).c_str());

  sensorModel->m_odtY[0] = atof(imageSupportData.param("odt_y", 0).c_str());
  sensorModel->m_odtY[1] = atof(imageSupportData.param("odt_y", 1).c_str());
  sensorModel->m_odtY[2] = atof(imageSupportData.param("odt_y", 2).c_str());
  sensorModel->m_odtY[3] = atof(imageSupportData.param("odt_y", 3).c_str());
  sensorModel->m_odtY[4] = atof(imageSupportData.param("odt_y", 4).c_str());
  sensorModel->m_odtY[5] = atof(imageSupportData.param("odt_y", 5).c_str());
  sensorModel->m_odtY[6] = atof(imageSupportData.param("odt_y", 6).c_str());
  sensorModel->m_odtY[7] = atof(imageSupportData.param("odt_y", 7).c_str());
  sensorModel->m_odtY[8] = atof(imageSupportData.param("odt_y", 8).c_str());
  sensorModel->m_odtY[9] = atof(imageSupportData.param("odt_y", 9).c_str());


  sensorModel->m_ccdCenter[0] = atof(imageSupportData.param("ccd_center", 0).c_str());
  sensorModel->m_ccdCenter[1] = atof(imageSupportData.param("ccd_center", 1).c_str());

  sensorModel->m_originalHalfLines = atof(imageSupportData.param("original_half_lines").c_str());
  sensorModel->m_spacecraftName = imageSupportData.param("spacecraft_name");

  sensorModel->m_pixelPitch = atof(imageSupportData.param("pixel_pitch").c_str());

  sensorModel->m_iTransS[0] = atof(imageSupportData.param("itrans_sample", 0).c_str());
  sensorModel->m_iTransS[1] = atof(imageSupportData.param("itrans_sample", 1).c_str());
  sensorModel->m_iTransS[2] = atof(imageSupportData.param("itrans_sample", 2).c_str());
  if (imageSupportData.param("itrans_sample", 0) == "") {
    missingKeywords.push_back("itrans_sample needs 3 elements");
  }
  else if (imageSupportData.param("itrans_sample", 1) == "") {
    missingKeywords.push_back("itrans_sample needs 3 elements");
  }
  else if (imageSupportData.param("itrans_sample", 2) == "") {
    missingKeywords.push_back("itrans_sample needs 3 elements");
  }

  sensorModel->m_ephemerisTime = atof(imageSupportData.param("ephemeris_time").c_str());
  if (imageSupportData.param("ephemeris_time") == "") {
    missingKeywords.push_back("ephemeris_time");
  }

  sensorModel->m_originalHalfSamples =
      atof(imageSupportData.param("original_half_samples").c_str());

  sensorModel->m_boresight[0] = atof(imageSupportData.param("boresight", 0).c_str());
  sensorModel->m_boresight[1] = atof(imageSupportData.param("boresight", 1).c_str());
  sensorModel->m_boresight[2] = atof(imageSupportData.param("boresight", 2).c_str());

  sensorModel->m_iTransL[0] = atof(imageSupportData.param("itrans_line", 0).c_str());
  sensorModel->m_iTransL[1] = atof(imageSupportData.param("itrans_line", 1).c_str());
  sensorModel->m_iTransL[2] = atof(imageSupportData.param("itrans_line", 2).c_str());
  if (imageSupportData.param("itrans_line", 0) == "") {
    missingKeywords.push_back("itrans_line needs 3 elements");
  }
  else if (imageSupportData.param("itrans_line", 1) == "") {
    missingKeywords.push_back("itrans_line needs 3 elements");
  }
  else if (imageSupportData.param("itrans_line", 2) == "") {
    missingKeywords.push_back("itrans_line needs 3 elements");
  }

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
    missingKeywords.push_back("transy");
  }
  else if (imageSupportData.param("transy", 1) == "") {
    missingKeywords.push_back("transy");
  }
  else if (imageSupportData.param("transy", 2) == "") {
    missingKeywords.push_back("transy");
  }

  sensorModel->m_transX[0] = atof(imageSupportData.param("transx", 0).c_str());
  sensorModel->m_transX[1] = atof(imageSupportData.param("transx", 1).c_str());
  sensorModel->m_transX[2] = atof(imageSupportData.param("transx", 2).c_str());
  if (imageSupportData.param("transx", 0) == "") {
    missingKeywords.push_back("transx");
  }
  else if (imageSupportData.param("transx", 1) == "") {
    missingKeywords.push_back("transx");
  }
  else if (imageSupportData.param("transx", 2) == "") {
    missingKeywords.push_back("transx");
  }

  sensorModel->m_majorAxis = 1000 * atof(imageSupportData.param("semi_major_axis").c_str());
  if (imageSupportData.param("semi_major_axis") == "") {
    missingKeywords.push_back("semi_major_axis");
  }
  // Do we assume that if we do not have a semi-minor axis, then the body is a sphere?
  if (imageSupportData.param("semi_minor_axis") == "") {
    sensorModel->m_minorAxis = sensorModel->m_majorAxis;
  }
  else {
    sensorModel->m_minorAxis = 1000 * atof(imageSupportData.param("semi_minor_axis").c_str());
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
                     "MdisPlugin::constructModelFromISD");
  }

  return sensorModel;
}


std::string MdisPlugin::getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings) const {
  return "state";
}


bool MdisPlugin::canISDBeConvertedToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  return false;
}


std::string MdisPlugin::convertISDToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  return "state";
}
