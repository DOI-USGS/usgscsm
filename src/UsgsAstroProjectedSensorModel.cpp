/** Copyright  © 2017-2022 BAE Systems Information and Electronic Systems Integration Inc.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. **/

#include "UsgsAstroProjectedSensorModel.h"
#include "UsgsAstroPluginSupport.h"
#include "Utilities.h"

#include <proj.h>

#include <Error.h>
#include <nlohmann/json.hpp>

#include "ale/Util.h"

#define MESSAGE_LOG(...)        \
  if (m_logger) {               \
    m_logger->log(__VA_ARGS__); \
  }

using json = nlohmann::json;

const std::string UsgsAstroProjectedSensorModel::_SENSOR_MODEL_NAME =
    "USGS_ASTRO_PROJECTED_SENSOR_MODEL";

const std::string UsgsAstroProjectedSensorModel::_STATE_KEYWORD[] = {
    "m_modelName",
    "m_imageIdentifier",
    "m_sensorName",
    "m_nLines",
    "m_nSamples",
    "m_platformFlag",
    "m_intTimeLines",
    "m_intTimeStartTimes",
    "m_intTimes",
    "m_startingEphemerisTime",
    "m_centerEphemerisTime",
    "m_detectorSampleSumming",
    "m_detectorSampleSumming",
    "m_startingDetectorSample",
    "m_startingDetectorLine",
    "m_ikCode",
    "m_focalLength",
    "m_zDirection",
    "m_distortionType",
    "m_opticalDistCoeffs",
    "m_iTransS",
    "m_iTransL",
    "m_detectorSampleOrigin",
    "m_detectorLineOrigin",
    "m_majorAxis",
    "m_minorAxis",
    "m_platformIdentifier",
    "m_sensorIdentifier",
    "m_minElevation",
    "m_maxElevation",
    "m_dtEphem",
    "m_t0Ephem",
    "m_dtQuat",
    "m_t0Quat",
    "m_numPositions",
    "m_numQuaternions",
    "m_positions",
    "m_velocities",
    "m_quaternions",
    "m_currentParameterValue",
    "m_parameterType",
    "m_referencePointXyz",
    "m_sunPosition",
    "m_sunVelocity",
    "m_gsd",
    "m_flyingHeight",
    "m_halfSwath",
    "m_halfTime",
    "m_covariance",
    "m_geoTransform",
    "m_projString",
};

//***************************************************************************
// UsgsAstroLineScannerSensorModel::replaceModelState
//***************************************************************************
void UsgsAstroProjectedSensorModel::replaceModelState(const std::string& stateString) {
  reset();

  auto j = stateAsJson(stateString);
  m_majorAxis = j["m_majorAxis"];
  m_minorAxis = j["m_minorAxis"];
  m_geoTransform = j["m_geoTransform"].get<std::vector<double>>();
  m_projString = j["m_projString"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_majorAxis: {} "
      "m_minorAxis: {} "
      "m_geoTransform: {} "
      "m_projString: {} ",
      j["m_majorAxis"].dump(), j["m_minorAxis"].dump(), j["m_geoTransform"].dump(), j["m_projString"].dump());
  m_camera->replaceModelState(stateString);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getModelNameFromModelState
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getModelNameFromModelState(
    const std::string& model_state) {
  // Parse the string to JSON
  auto j = stateAsJson(model_state);
  // If model name cannot be determined, return a blank string
  std::string model_name;

  if (j.find("m_modelName") != j.end()) {
    model_name = j["m_modelName"];
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
    std::string aMessage = "No 'm_modelName' key in the model state object.";
    std::string aFunction = "UsgsAstroProjectedPlugin::getModelNameFromModelState";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  if (model_name != _SENSOR_MODEL_NAME) {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    std::string aMessage = "Sensor model not supported.";
    std::string aFunction = "UsgsAstroProjectedPlugin::getModelNameFromModelState()";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  return model_name;
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getModelState
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getModelState() const {
  auto state = stateAsJson(m_camera->getModelState());
  state["m_modelName"] = _SENSOR_MODEL_NAME;
  state["m_majorAxis"] = m_majorAxis;
  state["m_minorAxis"] = m_minorAxis;
  state["m_geoTransform"] = m_geoTransform;
  state["m_projString"] = m_projString;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_majorAxis: {} "
      "m_minorAxis: {} "
      "m_geoTransform: {}, {}, {}, {}, {}, {} "
      "m_projString: {} ",
      m_majorAxis,
      m_minorAxis,
      m_geoTransform[0], 
      m_geoTransform[1], 
      m_geoTransform[2], 
      m_geoTransform[3], 
      m_geoTransform[4], 
      m_geoTransform[5], 
      m_projString);
  // Use dump(2) to avoid creating the model string as a single long line
  std::string stateString = getModelName() + "\n" + state.dump(2);
  return stateString;
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::reset
//***************************************************************************
void UsgsAstroProjectedSensorModel::reset() {
  MESSAGE_LOG(spdlog::level::debug, "Running reset()");

  m_majorAxis = 3400000.0;
  m_minorAxis = 3350000.0;
  m_geoTransform = std::vector<double>(6, 0.0);
  m_projString = "";
}

//*****************************************************************************
// UsgsAstroProjectedSensorModel Constructor
//*****************************************************************************
UsgsAstroProjectedSensorModel::UsgsAstroProjectedSensorModel() {}

//*****************************************************************************
// UsgsAstroProjectedSensorModel Destructor
//*****************************************************************************
UsgsAstroProjectedSensorModel::~UsgsAstroProjectedSensorModel() {}

//---------------------------------------------------------------------------
// Core Photogrammetry
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedSensorModel::groundToImage
//***************************************************************************
csm::ImageCoord UsgsAstroProjectedSensorModel::groundToImage(
    const csm::EcefCoord &ground_pt, double desired_precision,
    double *achieved_precision, csm::WarningList *warnings) const {
  
  PJ_CONTEXT *C = proj_context_create();

  /* Create a projection. */
  PJ *isdProj = proj_create(C, (m_projString + " +type=crs").c_str());
  if (0 == isdProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create isd transformation object");
    return csm::ImageCoord(0, 0);
  }

  /* Create the geocentric projection for our target */
  std::string radius_a = "+a=" + std::to_string(m_majorAxis);
  std::string radius_b = "+b=" + std::to_string(m_minorAxis);
  std::string projString = "+proj=geocent " + radius_a + " " + radius_b + " +type=crs";
  PJ *ecefProj = proj_create(C, projString.c_str());
  if (0 == ecefProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create geocent transformation object");
    return csm::ImageCoord(0, 0);
  }

  // Compute the transformation from our ISIS projection to ecef
  PJ *isdProj2ecefProj = proj_create_crs_to_crs_from_pj(C, isdProj, ecefProj, 0, 0);
  PJ_COORD c_in;
  c_in.xyz.x = ground_pt.x;
  c_in.xyz.y = ground_pt.y;
  c_in.xyz.z = ground_pt.z;
  MESSAGE_LOG(
      spdlog::level::info,
      "Ground point {}, {}, {}",
      c_in.xyz.x, c_in.xyz.y, c_in.xyz.z);
  PJ_COORD c_out = proj_trans(isdProj2ecefProj, PJ_INV, c_in);
  MESSAGE_LOG(
      spdlog::level::info,
      "Meters {}, {}",
      c_out.xyz.x, c_out.xyz.y);
  std::vector<double> lineSampleCoord = meterToPixel(c_out.xyz.x, c_out.xyz.y, m_geoTransform);
  csm::ImageCoord imagePt(lineSampleCoord[0], lineSampleCoord[1]);
  MESSAGE_LOG(
      spdlog::level::info,
      "groundToImage result of ({}, {})",
      imagePt.line, imagePt.samp);
  return imagePt;
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::groundToImage
//***************************************************************************
csm::ImageCoordCovar UsgsAstroProjectedSensorModel::groundToImage(
    const csm::EcefCoordCovar &groundPt, double desired_precision,
    double *achieved_precision, csm::WarningList *warnings) const {
  csm::ImageCoordCovar imageCoordCovar = m_camera->groundToImage(groundPt, desired_precision, achieved_precision, warnings);
  csm::ImageCoord projImagePt = groundToImage(groundPt);

  imageCoordCovar.line = projImagePt.line;
  imageCoordCovar.samp = projImagePt.samp;
  return imageCoordCovar;
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::imageToGround
//***************************************************************************
csm::EcefCoord UsgsAstroProjectedSensorModel::imageToGround(
    const csm::ImageCoord& image_pt, double height, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing imageToGround for {}, {}, {}, with desired precision {}",
      image_pt.line, image_pt.samp, height, desired_precision);

  double x = 0, y = 0, z = 0;
  double meterLine, meterSamp;
  std::vector<double> meterCoord = pixelToMeter(image_pt.line, image_pt.samp, m_geoTransform);
  meterLine = meterCoord[0];
  meterSamp = meterCoord[1];
  PJ_CONTEXT *C = proj_context_create();

  /* Create a projection. */
  PJ *isdProj = proj_create(C, (m_projString + " +type=crs").c_str());
  if (0 == isdProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create isd transformation object");
    return csm::EcefCoord(x, y, z);
  }

  /* Create the geocentric projection for our target */
  std::string radius_a = "+a=" + std::to_string(m_majorAxis);
  std::string radius_b = "+b=" + std::to_string(m_minorAxis);
  std::string projString = "+proj=geocent " + radius_a + " " + radius_b + " +type=crs";
  PJ *ecefProj = proj_create(C, projString.c_str());
  if (0 == ecefProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create geocent transformation object");
    return csm::EcefCoord(x, y, z);
  }

  // Compute the transformation from our ISIS projection to ecef
  PJ *isdProj2ecefProj = proj_create_crs_to_crs_from_pj(C, isdProj, ecefProj, 0, 0);
  PJ_COORD c_in;
  c_in.xy.x = meterSamp;
  c_in.xy.y = meterLine;
  PJ_COORD c_out = proj_trans(isdProj2ecefProj, PJ_FWD, c_in);
  x = c_out.xyz.x, y = c_out.xyz.y, z = c_out.xyz.z;
  MESSAGE_LOG(
      spdlog::level::info,
      "imageToGround result {} {} {}",
      x, y, z);
  return csm::EcefCoord(x, y, z);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::imageToGround
//***************************************************************************
csm::EcefCoordCovar UsgsAstroProjectedSensorModel::imageToGround(
    const csm::ImageCoordCovar& image_pt, double height, double heightVariance,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  csm::EcefCoord groundCoord = imageToGround(image_pt, height);
  csm::ImageCoord cameraImagePt = m_camera->groundToImage(groundCoord);
  csm::ImageCoordCovar cameraImagePtCovar(cameraImagePt.line, cameraImagePt.samp);
  
  return m_camera->imageToGround(cameraImagePtCovar, height, heightVariance, desired_precision, achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::imageToProximateImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroProjectedSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  csm::EcefCoord projGround = imageToGround(image_pt, 0);
  csm::ImageCoord cameraImagePt = m_camera->groundToImage(projGround);

  return m_camera->imageToProximateImagingLocus(cameraImagePt, ground_pt, desired_precision, achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::imageToRemoteImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroProjectedSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& image_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  // Go from proj x, y to latlon then ground to image
  // Convert imagept to camera imagept
  csm::EcefCoord groundCoord = imageToGround(image_pt, 0);
  csm::ImageCoord cameraImagePt = m_camera->groundToImage(groundCoord);

  return m_camera->imageToRemoteImagingLocus(cameraImagePt, desired_precision, achieved_precision, warnings);
}

//---------------------------------------------------------------------------
// Uncertainty Propagation
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedSensorModel::computeGroundPartials
//***************************************************************************
std::vector<double> UsgsAstroProjectedSensorModel::computeGroundPartials(
    const csm::EcefCoord& ground_pt) const {
  return m_camera->computeGroundPartials(ground_pt);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::computeSensorPartials
//***************************************************************************
csm::RasterGM::SensorPartials UsgsAstroProjectedSensorModel::computeSensorPartials(
    int index, const csm::EcefCoord &groundPt, double desiredPrecision,
    double *achievedPrecision, csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating computeSensorPartials for ground point {}, {}, {} with "
      "desired precision {}",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision)

  csm::ImageCoord imagePt = m_camera->groundToImage(groundPt, desiredPrecision, achievedPrecision);

  return m_camera->computeSensorPartials(index, imagePt, groundPt, desiredPrecision,
                                         achievedPrecision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::computeSensorPartials
//***************************************************************************
csm::RasterGM::SensorPartials UsgsAstroProjectedSensorModel::computeSensorPartials(
    int index, const csm::ImageCoord &imagePt, const csm::EcefCoord &groundPt,
    double desiredPrecision, double *achievedPrecision,
    csm::WarningList *warnings) const {
  return m_camera->computeSensorPartials(index, imagePt, groundPt, desiredPrecision,
                                         achievedPrecision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::computeAllSensorPartials
//***************************************************************************
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroProjectedSensorModel::computeAllSensorPartials(
    const csm::EcefCoord& ground_pt, csm::param::Set pSet,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {

  MESSAGE_LOG(
      spdlog::level::info,
      "Computing computeAllSensorPartials for ground point {}, {}, {} with "
      "desired precision {}",
      ground_pt.x, ground_pt.y, ground_pt.z, desired_precision)
  csm::ImageCoord image_pt =
      m_camera->groundToImage(ground_pt, desired_precision, achieved_precision, warnings);

  return computeAllSensorPartials(image_pt, ground_pt, pSet, desired_precision,
                                  achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::computeAllSensorPartials
//***************************************************************************
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroProjectedSensorModel::computeAllSensorPartials(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    csm::param::Set pSet, double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {

  return m_camera->computeAllSensorPartials(image_pt, ground_pt, pSet, desired_precision,
                                            achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getParameterCovariance
//***************************************************************************
double UsgsAstroProjectedSensorModel::getParameterCovariance(int index1,
                                                      int index2) const {
  return m_camera->getParameterCovariance(index1, index2);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::setParameterCovariance
//***************************************************************************
void UsgsAstroProjectedSensorModel::setParameterCovariance(int index1, int index2,
                                                    double covariance) {
  m_camera->setParameterCovariance(index1, index2, covariance);
}

//---------------------------------------------------------------------------
// Time and Trajectory
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedSensorModel::getTrajectoryIdentifier
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getTrajectoryIdentifier() const {
  return m_camera->getTrajectoryIdentifier();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getReferenceDateAndTime
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getReferenceDateAndTime() const {
  return m_camera->getReferenceDateAndTime();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getImageTime
//***************************************************************************
double UsgsAstroProjectedSensorModel::getImageTime(
    const csm::ImageCoord& image_pt) const {
  return m_camera->getImageTime(image_pt);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getSensorPosition
//***************************************************************************
csm::EcefCoord UsgsAstroProjectedSensorModel::getSensorPosition(
    const csm::ImageCoord& imagePt) const {
  csm::EcefCoord ground = imageToGround(imagePt, 0);
  csm::ImageCoord cameraImagePt = m_camera->groundToImage(ground);

  return getSensorPosition(m_camera->getImageTime(cameraImagePt));
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorPosition
//***************************************************************************
csm::EcefCoord UsgsAstroProjectedSensorModel::getSensorPosition(double time) const {
  return m_camera->getSensorPosition(time);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getSensorVelocity
//***************************************************************************
csm::EcefVector UsgsAstroProjectedSensorModel::getSensorVelocity(
    const csm::ImageCoord& imagePt) const {
  csm::EcefCoord ground = imageToGround(imagePt, 0);
  csm::ImageCoord cameraImagePt = m_camera->groundToImage(ground);

  return getSensorVelocity(m_camera->getImageTime(cameraImagePt));
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorPosition
//***************************************************************************
csm::EcefVector UsgsAstroProjectedSensorModel::getSensorVelocity(double time) const {
  return m_camera->getSensorVelocity(time);
}

//---------------------------------------------------------------------------
// Sensor Model Parameters
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedSensorModel::setParameterValue
//***************************************************************************
void UsgsAstroProjectedSensorModel::setParameterValue(int index, double value) {
  m_camera->setParameterValue(index, value);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getParameterValue
//***************************************************************************
double UsgsAstroProjectedSensorModel::getParameterValue(int index) const {
  return m_camera->getParameterValue(index);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getParameterName
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getParameterName(int index) const {
  return m_camera->getParameterName(index);
}

std::string UsgsAstroProjectedSensorModel::getParameterUnits(int index) const {
  // All parameters are meters or scaled to meters
  return m_camera->getParameterUnits(index);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getNumParameters
//***************************************************************************
int UsgsAstroProjectedSensorModel::getNumParameters() const {
  return m_camera->getNumParameters();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getParameterType
//***************************************************************************
csm::param::Type UsgsAstroProjectedSensorModel::getParameterType(int index) const {
  return m_camera->getParameterType(index);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::setParameterType
//***************************************************************************
void UsgsAstroProjectedSensorModel::setParameterType(int index,
                                              csm::param::Type pType) {
  m_camera->setParameterType(index, pType);
}

//---------------------------------------------------------------------------
// Sensor Model Information
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedSensorModel::getPedigree
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getPedigree() const {
  return "USGS_PROJECTED";
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getImageIdentifier
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getImageIdentifier() const {
  return m_camera->getImageIdentifier();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::setImageIdentifier
//***************************************************************************
void UsgsAstroProjectedSensorModel::setImageIdentifier(const std::string& imageId,
                                                csm::WarningList* warnings) {
  m_camera->setImageIdentifier(imageId, warnings);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getSensorIdentifier
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getSensorIdentifier() const {
  return m_camera->getSensorIdentifier();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getPlatformIdentifier
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getPlatformIdentifier() const {
  return m_camera->getPlatformIdentifier();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::setReferencePoint
//***************************************************************************
void UsgsAstroProjectedSensorModel::setReferencePoint(
    const csm::EcefCoord& ground_pt) {
  m_camera->setReferencePoint(ground_pt);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getReferencePoint
//***************************************************************************
csm::EcefCoord UsgsAstroProjectedSensorModel::getReferencePoint() const {
  // Return ground point at image center
  return m_camera->getReferencePoint();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getSensorModelName
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getModelName() const {
  return UsgsAstroProjectedSensorModel::_SENSOR_MODEL_NAME;
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getImageStart
//***************************************************************************
csm::ImageCoord UsgsAstroProjectedSensorModel::getImageStart() const {
  return m_camera->getImageStart();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getImageSize
//***************************************************************************
csm::ImageVector UsgsAstroProjectedSensorModel::getImageSize() const {
  return m_camera->getImageSize();
}

//---------------------------------------------------------------------------
//  Monoscopic Mensuration
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedSensorModel::getValidHeightRange
//***************************************************************************
std::pair<double, double> UsgsAstroProjectedSensorModel::getValidHeightRange() const {
  return m_camera->getValidHeightRange();
}


//***************************************************************************
// UsgsAstroProjectedSensorModel::getValidImageRange
//***************************************************************************
std::pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroProjectedSensorModel::getValidImageRange() const {
  return m_camera->getValidImageRange();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getIlluminationDirection
//***************************************************************************
csm::EcefVector UsgsAstroProjectedSensorModel::getIlluminationDirection(
    const csm::EcefCoord& groundPt) const {
  return m_camera->getIlluminationDirection(groundPt);
}

//---------------------------------------------------------------------------
//  Error Correction
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedSensorModel::getNumGeometricCorrectionSwitches
//***************************************************************************
int UsgsAstroProjectedSensorModel::getNumGeometricCorrectionSwitches() const {
  return m_camera->getNumGeometricCorrectionSwitches();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getGeometricCorrectionName
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getGeometricCorrectionName(
    int index) const {
  return m_camera->getGeometricCorrectionName(index);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::setGeometricCorrectionSwitch
//***************************************************************************
void UsgsAstroProjectedSensorModel::setGeometricCorrectionSwitch(
    int index, bool value, csm::param::Type pType) {
  m_camera->setGeometricCorrectionSwitch(index, value, pType);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getGeometricCorrectionSwitch
//***************************************************************************
bool UsgsAstroProjectedSensorModel::getGeometricCorrectionSwitch(int index) const {
  return m_camera->getGeometricCorrectionSwitch(index);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getCrossCovarianceMatrix
//***************************************************************************
std::vector<double> UsgsAstroProjectedSensorModel::getCrossCovarianceMatrix(
    const csm::GeometricModel& comparisonModel, csm::param::Set pSet,
    const csm::GeometricModel::GeometricModelList& otherModels) const {
  return m_camera->getCrossCovarianceMatrix(comparisonModel, pSet, otherModels);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getCorrelationModel
//***************************************************************************
const csm::CorrelationModel& UsgsAstroProjectedSensorModel::getCorrelationModel()
    const {
  return m_camera->getCorrelationModel();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getUnmodeledCrossCovariance
//***************************************************************************
std::vector<double> UsgsAstroProjectedSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord& pt1, const csm::ImageCoord& pt2) const {
  return m_camera->getUnmodeledCrossCovariance(pt1, pt2);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getCollectionIdentifier
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getCollectionIdentifier() const {
  return m_camera->getCollectionIdentifier();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::hasShareableParameters
//***************************************************************************
bool UsgsAstroProjectedSensorModel::hasShareableParameters() const {
  return m_camera->hasShareableParameters();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::isParameterShareable
//***************************************************************************
bool UsgsAstroProjectedSensorModel::isParameterShareable(int index) const {
  // Parameter sharing is not supported for this sensor
  return m_camera->isParameterShareable(index);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getParameterSharingCriteria
//***************************************************************************
csm::SharingCriteria UsgsAstroProjectedSensorModel::getParameterSharingCriteria(
    int index) const {
  return m_camera->getParameterSharingCriteria(index);
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getSensorType
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getSensorType() const {
  return m_camera->getSensorType();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getSensorMode
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::getSensorMode() const {
  return m_camera->getSensorMode();
}

//***************************************************************************
// UsgsAstroProjectedSensorModel::getVersion
//***************************************************************************
csm::Version UsgsAstroProjectedSensorModel::getVersion() const {
  return csm::Version(1, 0, 0);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getEllipsoid
//***************************************************************************
csm::Ellipsoid UsgsAstroProjectedSensorModel::getEllipsoid() const {
  return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

void UsgsAstroProjectedSensorModel::setEllipsoid(const csm::Ellipsoid& ellipsoid) {
  m_majorAxis = ellipsoid.getSemiMajorRadius();
  m_minorAxis = ellipsoid.getSemiMinorRadius();
  // Needs to set cameras ellipsoid but RasterGM is not a SettableEllipsoid
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::constructStateFromIsd
//***************************************************************************
std::string UsgsAstroProjectedSensorModel::constructStateFromIsd(
    const std::string imageSupportData, csm::WarningList* warnings) {
  json state = json::parse(imageSupportData);
  std::string modelName = ale::getSensorModelName(state);

  m_camera = getUsgsCsmModel(imageSupportData, modelName, warnings);
  json projState = stateAsJson(m_camera->getModelState());

  // Force update the modelName
  projState["m_modelName"] = _SENSOR_MODEL_NAME;
  projState["m_geoTransform"] = ale::getGeoTransform(state);
  projState["m_projString"] = ale::getProjection(state);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_geoTransform: {} "
      "m_projString: {} ",
      projState["m_geoTransform"].dump(), projState["m_projString"].dump());
  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return projState.dump();
}