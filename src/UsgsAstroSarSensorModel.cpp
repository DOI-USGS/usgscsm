#include "UsgsAstroSarSensorModel.h"
#include "Utilities.h"

#include <functional>
#include <string.h>
#include <cmath>

#include <json/json.hpp>

using json = nlohmann::json;
using namespace std;

#define MESSAGE_LOG(logger, ...) if (logger) { logger->info(__VA_ARGS__); }

const string UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME = "USGS_ASTRO_SAR_SENSOR_MODEL";
const int UsgsAstroSarSensorModel::NUM_PARAMETERS = 6;
const string UsgsAstroSarSensorModel::PARAMETER_NAME[] =
{
   "IT Pos. Bias   ",   // 0
   "CT Pos. Bias   ",   // 1
   "Rad Pos. Bias  ",   // 2
   "X Vel. Bias    ",   // 3
   "Y Vel. Bias    ",   // 4
   "Z Vel. Bias    "    // 5
};

const int UsgsAstroSarSensorModel::NUM_PARAM_TYPES = 4;
const string UsgsAstroSarSensorModel::PARAM_STRING_ALL[] =
{
   "NONE",
   "FICTITIOUS",
   "REAL",
   "FIXED"
};
const csm::param::Type
      UsgsAstroSarSensorModel::PARAM_CHAR_ALL[] =
{
   csm::param::NONE,
   csm::param::FICTITIOUS,
   csm::param::REAL,
   csm::param::FIXED
};

string UsgsAstroSarSensorModel::constructStateFromIsd(
    const string imageSupportData,
    csm::WarningList *warnings
) {
  json isd = json::parse(imageSupportData);
  json state = {};

  csm::WarningList* parsingWarnings = new csm::WarningList;

  int num_params = NUM_PARAMETERS;

  state["m_modelName"] = getSensorModelName(isd, parsingWarnings);
  state["m_imageIdentifier"] = getImageId(isd, parsingWarnings);
  state["m_sensorName"] = getSensorName(isd, parsingWarnings);
  state["m_platformName"] = getPlatformName(isd, parsingWarnings);

  state["m_nLines"] = getTotalLines(isd, parsingWarnings);
  state["m_nSamples"] = getTotalSamples(isd, parsingWarnings);

  // Zero computed state values
  state["m_referencePointXyz"] = std::vector<double>(3, 0.0);

  // sun_position and velocity are required for getIlluminationDirection
  state["m_sunPosition"]= getSunPositions(isd, parsingWarnings);
  state["m_sunVelocity"]= getSunVelocities(isd, parsingWarnings);

  state["m_centerEphemerisTime"] = getCenterTime(isd, parsingWarnings);
  state["m_startingEphemerisTime"] = getStartingTime(isd, parsingWarnings);

  state["m_exposureDuration"] = getExposureDuration(isd, parsingWarnings);

  try {
    state["m_dtEphem"] = isd.at("dt_ephemeris");
  }
  catch(...) {
    parsingWarnings->push_back(
      csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE,
        "dt_ephemeris not in ISD",
        "UsgsAstroSarSensorModel::constructStateFromIsd()"));
  }

  try {
    state["m_t0Ephem"] = isd.at("t0_ephemeris");
  }
  catch(...) {
    parsingWarnings->push_back(
      csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE,
        "t0_ephemeris not in ISD",
        "UsgsAstroSarSensorModel::constructStateFromIsd()"));
  }

  state["m_positions"] = getSensorPositions(isd, parsingWarnings);
  state["m_velocities"] = getSensorVelocities(isd, parsingWarnings);

  state["m_currentParameterValue"] = std::vector<double>(NUM_PARAMETERS, 0.0);

  // get radii
  state["m_minorAxis"] = getSemiMinorRadius(isd, parsingWarnings);
  state["m_majorAxis"] = getSemiMajorRadius(isd, parsingWarnings);

  // set identifiers
  state["m_platformIdentifier"] = getPlatformName(isd, parsingWarnings);
  state["m_sensorIdentifier"] = getSensorName(isd, parsingWarnings);

  // get reference_height
  state["m_minElevation"] = -1000;
  state["m_maxElevation"] = 1000;

  // SAR specific values
  state["m_scaledPixelWidth"] = getScaledPixelWidth(isd, parsingWarnings);
  state["m_scaleConversionCoefficients"] = getScaleConversionCoefficients(isd, parsingWarnings);

  // Default to identity covariance
  state["m_covariance"] =
       std::vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
  for (int i = 0; i < NUM_PARAMETERS; i++) {
   state["m_covariance"][i * NUM_PARAMETERS + i] = 1.0;
  }

  if (!parsingWarnings->empty()) {
    if (warnings) {
      warnings->insert(warnings->end(), parsingWarnings->begin(), parsingWarnings->end());
    }
    delete parsingWarnings;
    parsingWarnings = nullptr;
    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     "ISD is invalid for creating the sensor model.",
                     "UsgsAstroSarSensorModel::constructStateFromIsd");
  }

  delete parsingWarnings;
  parsingWarnings = nullptr;

  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return state.dump();
}

// TEMPORARY hard-coded values. Need to be replaced by accessors.
double START_TIME = 0.0;
double STOP_TIME = 5.0;                         
double EXPOSURE_DURATION = 0.005;
double SCALED_PIXEL_WIDTH = 7.5; 
double WAVELENGTH = 0.125;
double SAMPLES = 1000.0;
double LINES = 1000.0;

csm::ImageCoord UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoord& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const {

  //MESSAGE_LOG(this->m_logger, "Computing groundToImage(ImageCoord) for {}, {}, {}, with desired precision {}",
  //          groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  // Find time of closest approach to groundPt and the corresponding slant range by finding 
  // the root of the doppler shift frequency
  csm::EcefCoord newGroundPt(groundPt);
  double time = dopplerShiftRoot(newGroundPt);
  double slantRange = calcSlantRange(newGroundPt, time);

  // Find the ground range, based on the ground-range-to-slant-range polynomial defined by the 
  // range coefficient set, with a time closest to the calculated time of closest approach
  double groundRange = slantToGroundRange(groundPt, time, slantRange);

  double line = (time - START_TIME) / EXPOSURE_DURATION;
  double sample = groundRange / SCALED_PIXEL_WIDTH;
  return csm::ImageCoord(line, sample);

  throw csm::Error(
    csm::Error::UNKNOWN_ERROR,
    "Could not calculate groundToImage.",
    "groundToImage");
}

// Calculate the root 
double UsgsAstroSarSensorModel::dopplerShiftRoot(
    csm::EcefCoord groundPt) const{

  // Moon body-fixed coordinates of surface point
  csm::EcefCoord surfPt(0.0, 0.0, 0.0); // populate with real value

   std::function<double(double)> dopplerShiftFrequency = [surfPt](double time) { 
     double spacecraftPosition[3] = {3.73740000e+06,  0.00000000e+00, -0.00000000e+00}; //spacecraftPosition(time);
     double spacecraftVelocity[3] = {-0.00000000e+00,  0.00000000e+00, -3.73740000e+0};; //spacecraftPosition(time);
     double lookVector[3];

     lookVector[0] = spacecraftPosition[0] - surfPt.x;
     lookVector[1] = spacecraftPosition[1] - surfPt.y;
     lookVector[2] = spacecraftPosition[2] - surfPt.z;

     double slantRange = sqrt(pow(lookVector[0], 2) +  pow(lookVector[1], 2) + pow(lookVector[2], 2)); 


     double dopplerShift = -2.0 * (lookVector[0]*spacecraftVelocity[0] + lookVector[1]*spacecraftVelocity[1] 
                            + lookVector[2]*spacecraftVelocity[2])/(slantRange * WAVELENGTH);
     return dopplerShift;
   };

  // Lower-bound for doppler-shift
  double startShift = dopplerShiftFrequency(START_TIME);

  // Upper-bound for doppler-shift
  double stopShift = dopplerShiftFrequency(STOP_TIME);

  // Make sure we bound root (dopplerShift = 0.0)
  if (!(((startShift < 0.0) && (stopShift < 0.0)) || ((startShift > 0.0) && (stopShift > 0.0)))) {

    // Do root-finding for "dopplerShift"
    double tolerance = (STOP_TIME - START_TIME) / LINES / 20.0;

    return secantRoot(START_TIME, STOP_TIME, dopplerShiftFrequency, tolerance);
  }
  throw csm::Error(
    csm::Error::UNKNOWN_ERROR,
    "Could not find a root of the doppler shift frequency function",
    "dopplerShiftRoot");
}


double UsgsAstroSarSensorModel::calcSlantRange(csm::EcefCoord surfPt,
    double time) const{
  double spacecraftPosition[3] = {3.73740000e+06,  0.00000000e+00, -0.00000000e+00}; //spacecraftPosition(time);
  double spacecraftVelocity[3] = {-0.00000000e+00,  0.00000000e+00, -3.73740000e+0};; //spacecraftPosition(time);
  double lookVector[3];

  lookVector[0] = spacecraftPosition[0] - surfPt.x;
  lookVector[1] = spacecraftPosition[1] - surfPt.y;
  lookVector[2] = spacecraftPosition[2] - surfPt.z;

  return sqrt(pow(lookVector[0], 2) +  pow(lookVector[1], 2) + pow(lookVector[2], 2)); 
}


double UsgsAstroSarSensorModel::slantToGroundRange(
        const csm::EcefCoord& groundPt,
        double time,
        double slantRange) const{

  // Need to come up with an initial guess when solving for ground 
  // range given slant range. Compute the ground range at the
  // near and far edges of the image by evaluating the sample-to-
  // ground-range equation: groundRange=(sample-1)*scaled_pixel_width
  // at the edges of the image. We also need to add some padding to
  // allow for solving for coordinates that are slightly outside of
  // the actual image area. Use sample=-0.25*image_samples and
  // sample=1.25*image_samples.

  double initialMinGroundRangeGuess = (-0.25 * SAMPLES - 1.0) * SCALED_PIXEL_WIDTH;
  double initialMaxGroundRangeGuess = (1.25 * SAMPLES - 1.0) * SCALED_PIXEL_WIDTH;

  //a1, a2, a3, a4 = getRangeCoefficients(t_min);
  double a1 = 7.99423808710000e+04;
  double a2 = 6.92122900000000e-01;
  double a3 = 3.40193700000000e-06;
  double a4 = -2.39924200000000e-11;

  // Calculates the ground range from the slant range.
  std::function<double(double)> groundRangeFunc = [a1, a2, a3, a4, slantRange](double groundRange){
   return slantRange - (a1 + groundRange * (a2 + groundRange * (a3 + groundRange * a4)));
  };

  // Evaluate the ground range at the 2 extremes of the image
  double minGroundRangeGuess = groundRangeFunc(initialMinGroundRangeGuess);
  double maxGroundRangeGuess = groundRangeFunc(initialMaxGroundRangeGuess);

  // If the ground range guesses at the 2 extremes of the image are equal
  // or they have the same sign, then the ground range cannot be solved for.
  if (!((minGroundRangeGuess == maxGroundRangeGuess) || 
      (minGroundRangeGuess < 0.0 && maxGroundRangeGuess < 0.0) ||
      (minGroundRangeGuess > 0.0 && maxGroundRangeGuess > 0.0))) {

    // Use Brent's algorithm to find a root of the function:
    return brentRoot(minGroundRangeGuess, maxGroundRangeGuess, groundRangeFunc, 0.1);
  }
  throw csm::Error(
    csm::Error::UNKNOWN_ERROR,
    "Could not find a root of the slant range to ground range function",
    "slantToGroundRange");
}


