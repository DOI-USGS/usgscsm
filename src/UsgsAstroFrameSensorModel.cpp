#include "UsgsAstroFrameSensorModel.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include <json.hpp>

#include <Error.h>
#include <Version.h>

#include "Distortion.h"
#include "Utilities.h"

using json = nlohmann::json;
using namespace std;

// Declaration of static variables
const std::string UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME
                                      = "USGS_ASTRO_FRAME_SENSOR_MODEL";
const int UsgsAstroFrameSensorModel::NUM_PARAMETERS = 7;
const std::string UsgsAstroFrameSensorModel::m_parameterName[] = {
  "X Sensor Position (m)",  // 0
  "Y Sensor Position (m)",  // 1
  "Z Sensor Position (m)",  // 2
  "w",                      // 3
  "v1",                     // 4
  "v2",                     // 5
  "v3"                      // 6
};

UsgsAstroFrameSensorModel::UsgsAstroFrameSensorModel() {
    reset();
}

void UsgsAstroFrameSensorModel::reset() {
    m_modelName = _SENSOR_MODEL_NAME;
    m_platformName = "";
    m_sensorName = "";
    m_imageIdentifier = "";
    m_collectionIdentifier = "";
    m_majorAxis = 0.0;
    m_minorAxis = 0.0;
    m_focalLength = 0.0;
    m_startingDetectorSample = 0.0;
    m_startingDetectorLine = 0.0;
    m_targetName = "";
    m_ifov = 0;
    m_instrumentID = "";
    m_focalLengthEpsilon = 0.0;
    m_linePp = 0.0;
    m_samplePp = 0.0;
    m_originalHalfLines = 0.0;
    m_spacecraftName = "";
    m_pixelPitch = 0.0;
    m_ephemerisTime = 0.0;
    m_originalHalfSamples = 0.0;
    m_nLines = 0;
    m_nSamples = 0;

    m_currentParameterValue = std::vector<double>(NUM_PARAMETERS, 0.0);
    m_currentParameterCovariance = std::vector<double>(NUM_PARAMETERS*NUM_PARAMETERS,0.0);
    m_noAdjustments = std::vector<double>(NUM_PARAMETERS,0.0);
    m_ccdCenter = std::vector<double>(2, 0.0);
    m_spacecraftVelocity = std::vector<double>(3, 0.0);
    m_sunPosition = std::vector<double>(3, 0.0);
    m_odtX = std::vector<double>(10, 0.0);
    m_odtY = std::vector<double>(10, 0.0);
    m_transX = std::vector<double>(3, 0.0);
    m_transY = std::vector<double>(3, 0.0);
    m_iTransS = std::vector<double>(3, 0.0);
    m_iTransL = std::vector<double>(3, 0.0);
    m_boresight = std::vector<double>(3, 0.0);
    m_parameterType = std::vector<csm::param::Type>(NUM_PARAMETERS, csm::param::REAL);
    m_referencePointXyz.x = 0;
    m_referencePointXyz.y = 0;
    m_referencePointXyz.z = 0;
}


UsgsAstroFrameSensorModel::~UsgsAstroFrameSensorModel() {}

/**
 * @brief UsgsAstroFrameSensorModel::groundToImage
 * @param groundPt
 * @param desiredPrecision
 * @param achievedPrecision
 * @param warnings
 * @return Returns <line, sample> coordinate in the image corresponding to the ground point
 * without bundle adjustment correction.
 */
csm::ImageCoord UsgsAstroFrameSensorModel::groundToImage(const csm::EcefCoord &groundPt,
                              double desiredPrecision,
                              double *achievedPrecision,
                              csm::WarningList *warnings) const {

  return groundToImage(groundPt, m_noAdjustments, desiredPrecision, achievedPrecision, warnings);
}


/**
 * @brief UsgsAstroFrameSensorModel::groundToImage
 * @param groundPt
 * @param adjustments
 * @param desired_precision
 * @param achieved_precision
 * @param warnings
 * @return Returns <line,sample> coordinate in the image corresponding to the ground point.
 * This function applies bundle adjustments to the final value.
 */
csm::ImageCoord UsgsAstroFrameSensorModel::groundToImage(
    const csm::EcefCoord&      groundPt,
    const std::vector<double>& adjustments,
    double                     desired_precision,
    double*                    achieved_precision,
    csm::WarningList*          warnings ) const {

  double x = groundPt.x;
  double y = groundPt.y;
  double z = groundPt.z;

  double xo = x - getValue(0,adjustments);
  double yo = y - getValue(1,adjustments);
  double zo = z - getValue(2,adjustments);

  double f;
  f = m_focalLength;

  // Camera rotation matrix
  double m[3][3];
  calcRotationMatrix(m, adjustments);

  // Sensor position
  double undistortedx, undistortedy, denom;
  denom = m[0][2] * xo + m[1][2] * yo + m[2][2] * zo;
  undistortedx = (f * (m[0][0] * xo + m[1][0] * yo + m[2][0] * zo)/denom) + m_samplePp;  //m_samplePp like this assumes mm
  undistortedy = (f * (m[0][1] * xo + m[1][1] * yo + m[2][1] * zo)/denom) + m_linePp;

  // Apply the distortion to the line/sample location and then convert back to line/sample
  std::tuple<double, double> distortedPoint;
  distortedPoint = distortionFunction(undistortedx, undistortedy, m_odtX, m_odtY);


  // Convert distorted mm into line/sample
  double sample, line;
  sample = m_iTransS[0] + m_iTransS[1] * std::get<0>(distortedPoint) + m_iTransS[2] * std::get<1>(distortedPoint) + m_ccdCenter[1];
  line =   m_iTransL[0] + m_iTransL[1] * std::get<0>(distortedPoint) + m_iTransL[2] * std::get<1>(distortedPoint) + m_ccdCenter[0];

  return csm::ImageCoord(line, sample);
}


csm::ImageCoordCovar UsgsAstroFrameSensorModel::groundToImage(const csm::EcefCoordCovar &groundPt,
                                   double desiredPrecision,
                                   double *achievedPrecision,
                                   csm::WarningList *warnings) const {

    csm::EcefCoord gp;
    gp.x = groundPt.x;
    gp.y = groundPt.y;
    gp.z = groundPt.z;

    csm::ImageCoord ip = groundToImage(
      gp, desiredPrecision, achievedPrecision, warnings);
   csm::ImageCoordCovar result(ip.line, ip.samp);
   // This is a partial, incorrect implementation to test if SocetGXP needs
   // this method implemented in order to load the sensor.
   return result;
}


csm::EcefCoord UsgsAstroFrameSensorModel::imageToGround(const csm::ImageCoord &imagePt,
                                                 double height,
                                                 double desiredPrecision,
                                                 double *achievedPrecision,
                                                 csm::WarningList *warnings) const {

  double sample = imagePt.samp;
  double line = imagePt.line;

  // Here is where we should be able to apply an adjustment to opk
  double m[3][3];
  calcRotationMatrix(m);

  // Apply the principal point offset, assuming the pp is given in pixels
  double xl, yl, zl, lo, so;
  lo = line - m_linePp;
  so = sample - m_samplePp;

  // Convert from the pixel space into the metric space
  double line_center, sample_center, x_camera, y_camera;
  line_center = m_ccdCenter[0];
  sample_center = m_ccdCenter[1];
  y_camera = m_transY[0] + m_transY[1] * (lo - line_center) + m_transY[2] * (so - sample_center);
  x_camera = m_transX[0] + m_transX[1] * (lo - line_center) + m_transX[2] * (so - sample_center);

  // Apply the distortion model (remove distortion)
  std::tuple<double, double> undistortedPoint;
  undistortedPoint = removeDistortion(x_camera, y_camera, m_odtX, m_odtY);

  // Now back from distorted mm to pixels
  xl = m[0][0] * std::get<0>(undistortedPoint) + m[0][1] * std::get<1>(undistortedPoint) - m[0][2] * - m_focalLength;
  yl = m[1][0] * std::get<0>(undistortedPoint) + m[1][1] * std::get<1>(undistortedPoint) - m[1][2] * - m_focalLength;
  zl = m[2][0] * std::get<0>(undistortedPoint) + m[2][1] * std::get<1>(undistortedPoint) - m[2][2] * - m_focalLength;

  double x, y, z;
  double xc, yc, zc;
  xc = m_currentParameterValue[0];
  yc = m_currentParameterValue[1];
  zc = m_currentParameterValue[2];

  // Intersect with some height about the ellipsoid.

  losEllipsoidIntersect(height, xc, yc, zc, xl, yl, zl, x, y, z);

  return csm::EcefCoord(x, y, z);
}


csm::EcefCoordCovar UsgsAstroFrameSensorModel::imageToGround(const csm::ImageCoordCovar &imagePt, double height,
                                  double heightVariance, double desiredPrecision,
                                  double *achievedPrecision,
                                  csm::WarningList *warnings) const {
    // This is an incomplete implementation to see if SocetGXP needs this method implemented.
    csm::EcefCoordCovar result;
    return result;
}


csm::EcefLocus UsgsAstroFrameSensorModel::imageToProximateImagingLocus(const csm::ImageCoord &imagePt,
                                                                const csm::EcefCoord &groundPt,
                                                                double desiredPrecision,
                                                                double *achievedPrecision,
                                                                csm::WarningList *warnings) const {
  // Ignore the ground point?
  return imageToRemoteImagingLocus(imagePt);
}


csm::EcefLocus UsgsAstroFrameSensorModel::imageToRemoteImagingLocus(const csm::ImageCoord &imagePt,
                                                             double desiredPrecision,
                                                             double *achievedPrecision,
                                                             csm::WarningList *warnings) const {
  // Find the line,sample on the focal plane (mm)
  // CSM center = 0.5, MDIS IK center = 1.0
  double row = imagePt.line - m_ccdCenter[0];
  double col = imagePt.samp - m_ccdCenter[1];
  double focalPlaneX = m_transX[0] + m_transX[1] * row + m_transX[2] * col;
  double focalPlaneY = m_transY[0] + m_transY[1] * row + m_transY[2] * col;

  // Distort
  std::tuple<double, double> undistortedPoint;
  undistortedPoint = removeDistortion(focalPlaneX, focalPlaneY, m_odtX, m_odtY);

  // Get rotation matrix and transform to a body-fixed frame
  double m[3][3];
  calcRotationMatrix(m);
  std::vector<double> lookC { std::get<0>(undistortedPoint), std::get<1>(undistortedPoint), m_focalLength };
  std::vector<double> lookB {
    m[0][0] * lookC[0] + m[0][1] * lookC[1] + m[0][2] * lookC[2],
    m[1][0] * lookC[0] + m[1][1] * lookC[1] + m[1][2] * lookC[2],
    m[2][0] * lookC[0] + m[2][1] * lookC[1] + m[2][2] * lookC[2]
  };

  // Get unit vector
  double mag = sqrt(lookB[0] * lookB[0] + lookB[1] * lookB[1] + lookB[2] * lookB[2]);
  std::vector<double> lookBUnit {
    lookB[0] / mag,
    lookB[1] / mag,
    lookB[2] / mag
  };

  return csm::EcefLocus(m_currentParameterValue[0], m_currentParameterValue[1], m_currentParameterValue[2],
      lookBUnit[0], lookBUnit[1], lookBUnit[2]);
}


csm::ImageCoord UsgsAstroFrameSensorModel::getImageStart() const {

  csm::ImageCoord start;
  start.samp = m_startingDetectorSample;
  start.line = m_startingDetectorLine;
  return start;
}


csm::ImageVector UsgsAstroFrameSensorModel::getImageSize() const {

  csm::ImageVector size;
  size.line = m_nLines;
  size.samp = m_nSamples;
  return size;
}


std::pair<csm::ImageCoord, csm::ImageCoord> UsgsAstroFrameSensorModel::getValidImageRange() const {
    csm::ImageCoord min_pt(m_startingDetectorLine, m_startingDetectorSample);
    csm::ImageCoord max_pt(m_nLines, m_nSamples);
    return std::pair<csm::ImageCoord, csm::ImageCoord>(min_pt, max_pt);
}


std::pair<double, double> UsgsAstroFrameSensorModel::getValidHeightRange() const {
    return std::pair<double, double>(m_minElevation, m_maxElevation);
}


csm::EcefVector UsgsAstroFrameSensorModel::getIlluminationDirection(const csm::EcefCoord &groundPt) const {
  // ground (body-fixed) - sun (body-fixed) gives us the illumination direction.
  return csm::EcefVector {
    groundPt.x - m_sunPosition[0],
    groundPt.y - m_sunPosition[1],
    groundPt.z - m_sunPosition[2]
  };
}


double UsgsAstroFrameSensorModel::getImageTime(const csm::ImageCoord &imagePt) const {

  // check if the image point is in range
  if (imagePt.samp >= m_startingDetectorSample &&
      imagePt.samp <= (m_startingDetectorSample + m_nSamples) &&
      imagePt.line >= m_startingDetectorSample &&
      imagePt.line <= (m_startingDetectorLine + m_nLines)) {
    return m_ephemerisTime;
  }
  else {
    throw csm::Error(csm::Error::BOUNDS,
                     "Image Coordinate out of Bounds",
                     "UsgsAstroFrameSensorModel::getImageTime");
  }
}


csm::EcefCoord UsgsAstroFrameSensorModel::getSensorPosition(const csm::ImageCoord &imagePt) const {

  // check if the image point is in range
  if (imagePt.samp >= m_startingDetectorSample &&
      imagePt.samp <= (m_startingDetectorSample + m_nSamples) &&
      imagePt.line >= m_startingDetectorSample &&
      imagePt.line <= (m_startingDetectorLine + m_nLines)) {
    csm::EcefCoord sensorPosition;
    sensorPosition.x = m_currentParameterValue[0];
    sensorPosition.y = m_currentParameterValue[1];
    sensorPosition.z = m_currentParameterValue[2];

    return sensorPosition;
  }
  else {
    throw csm::Error(csm::Error::BOUNDS,
                     "Image Coordinate out of Bounds",
                     "UsgsAstroFrameSensorModel::getSensorPosition");
  }
}


csm::EcefCoord UsgsAstroFrameSensorModel::getSensorPosition(double time) const {
    if (time == m_ephemerisTime){
        csm::EcefCoord sensorPosition;
        sensorPosition.x = m_currentParameterValue[0];
        sensorPosition.y = m_currentParameterValue[1];
        sensorPosition.z = m_currentParameterValue[2];

        return sensorPosition;
    } else {
        std::string aMessage = "Valid image time is %d", m_ephemerisTime;
        throw csm::Error(csm::Error::BOUNDS,
                         aMessage,
                         "UsgsAstroFrameSensorModel::getSensorPosition");
    }
}


csm::EcefVector UsgsAstroFrameSensorModel::getSensorVelocity(const csm::ImageCoord &imagePt) const {
  // Make sure the passed coordinate is with the image dimensions.
  if (imagePt.samp < 0.0 || imagePt.samp > m_nSamples ||
      imagePt.line < 0.0 || imagePt.line > m_nLines) {
    throw csm::Error(csm::Error::BOUNDS, "Image coordinate out of bounds.",
                     "UsgsAstroFrameSensorModel::getSensorVelocity");
  }

  // Since this is a frame, just return the sensor velocity the ISD gave us.
  return csm::EcefVector {
    m_spacecraftVelocity[0],
    m_spacecraftVelocity[1],
    m_spacecraftVelocity[2]
  };
}


csm::EcefVector UsgsAstroFrameSensorModel::getSensorVelocity(double time) const {
    if (time == m_ephemerisTime){
        return csm::EcefVector {
          m_spacecraftVelocity[0],
          m_spacecraftVelocity[1],
          m_spacecraftVelocity[2]
        };
    } else {
        std::string aMessage = "Valid image time is %d", m_ephemerisTime;
        throw csm::Error(csm::Error::BOUNDS,
                         aMessage,
                         "UsgsAstroFrameSensorModel::getSensorVelocity");
    }
}


csm::RasterGM::SensorPartials UsgsAstroFrameSensorModel::computeSensorPartials(int index,
                                           const csm::EcefCoord &groundPt,
                                           double desiredPrecision,
                                           double *achievedPrecision,
                                           csm::WarningList *warnings) const {

    csm::ImageCoord img_pt = groundToImage(groundPt, desiredPrecision, achievedPrecision);

    return computeSensorPartials(index, img_pt, groundPt, desiredPrecision, achievedPrecision);
}


/**
 * @brief UsgsAstroFrameSensorModel::computeSensorPartials
 * @param index
 * @param imagePt
 * @param groundPt
 * @param desiredPrecision
 * @param achievedPrecision
 * @param warnings
 * @return The partial derivatives in the line,sample directions.
 *
 * Research:  We should investigate using a central difference scheme to approximate
 * the partials.  It is more accurate, but it might be costlier calculation-wise.
 *
 */
csm::RasterGM::SensorPartials UsgsAstroFrameSensorModel::computeSensorPartials(int index,
                                          const csm::ImageCoord &imagePt,
                                          const csm::EcefCoord &groundPt,
                                          double desiredPrecision,
                                          double *achievedPrecision,
                                          csm::WarningList *warnings) const {


  const double delta = 1.0;
  // Update the parameter
  std::vector<double>adj(NUM_PARAMETERS, 0.0);
  adj[index] = delta;

  csm::ImageCoord imagePt1 = groundToImage(groundPt,adj,desiredPrecision,achievedPrecision);

  csm::RasterGM::SensorPartials partials;

  partials.first = (imagePt1.line - imagePt.line)/delta;
  partials.second = (imagePt1.samp - imagePt.samp)/delta;

  return partials;

}

std::vector<csm::RasterGM::SensorPartials> UsgsAstroFrameSensorModel::computeAllSensorPartials(
    const csm::ImageCoord& imagePt,
    const csm::EcefCoord& groundPt,
    csm::param::Set pset,
    double desiredPrecision,
    double *achievedPrecision,
    csm::WarningList *warnings) const {
  std::vector<int> indices = getParameterSetIndices(pset);
  size_t num = indices.size();
  std::vector<csm::RasterGM::SensorPartials> partials;
  for (int index = 0;index < num;index++){
    partials.push_back(computeSensorPartials(
        indices[index],
        imagePt, groundPt,
        desiredPrecision, achievedPrecision, warnings));
  }
  return partials;
}

std::vector<csm::RasterGM::SensorPartials> UsgsAstroFrameSensorModel::computeAllSensorPartials(
    const csm::EcefCoord& groundPt,
    csm::param::Set pset,
    double desiredPrecision,
    double *achievedPrecision,
    csm::WarningList *warnings) const {
  csm::ImageCoord imagePt = groundToImage(groundPt,
                                          desiredPrecision, achievedPrecision, warnings);
  return computeAllSensorPartials(imagePt, groundPt,
                                  pset, desiredPrecision, achievedPrecision, warnings);
    }

std::vector<double> UsgsAstroFrameSensorModel::computeGroundPartials(const csm::EcefCoord
                                                                     &groundPt) const {
  // Partials of line, sample wrt X, Y, Z
  // Uses collinearity equations
  std::vector<double> partials(6, 0.0);

  double m[3][3];
  calcRotationMatrix(m, m_noAdjustments);

  double xo, yo, zo;
  xo = groundPt.x - m_currentParameterValue[0];
  yo = groundPt.y - m_currentParameterValue[1];
  zo = groundPt.z - m_currentParameterValue[2];

  double u, v, w;
  u = m[0][0] * xo + m[0][1] * yo + m[0][2] * zo;
  v = m[1][0] * xo + m[1][1] * yo + m[1][2] * zo;
  w = m[2][0] * xo + m[2][1] * yo + m[2][2] * zo;

  double fdw, udw, vdw;
  fdw = m_focalLength / w;
  udw = u / w;
  vdw = v / w;

  double upx, vpx, wpx;
  upx = m[0][0];
  vpx = m[1][0];
  wpx = m[2][0];
  partials[0] = -fdw * ( upx - udw * wpx );
  partials[3] = -fdw * ( vpx - vdw * wpx );

  double upy, vpy, wpy;
  upy = m[0][1];
  vpy = m[1][1];
  wpy = m[2][1];
  partials[1] = -fdw * ( upy - udw * wpy );
  partials[4] = -fdw * ( vpy - vdw * wpy );

  double upz, vpz, wpz;
  upz = m[0][2];
  vpz = m[1][2];
  wpz = m[2][2];
  partials[2] = -fdw * ( upz - udw * wpz );
  partials[5] = -fdw * ( vpz - vdw * wpz );

  return partials;
}


const csm::CorrelationModel& UsgsAstroFrameSensorModel::getCorrelationModel() const {
  return _no_corr_model;
}


std::vector<double> UsgsAstroFrameSensorModel::getUnmodeledCrossCovariance(const csm::ImageCoord &pt1,
                                                const csm::ImageCoord &pt2) const {

   // No unmodeled error
   return std::vector<double>(4, 0.0);
}


csm::Version UsgsAstroFrameSensorModel::getVersion() const {
  return csm::Version(0,1,0);
}


std::string UsgsAstroFrameSensorModel::getModelName() const {
  return _SENSOR_MODEL_NAME;
}


std::string UsgsAstroFrameSensorModel::getPedigree() const {
  return "USGS_FRAMER";
}


std::string UsgsAstroFrameSensorModel::getImageIdentifier() const {
  return m_imageIdentifier;
}


void UsgsAstroFrameSensorModel::setImageIdentifier(const std::string& imageId,
                                            csm::WarningList* warnings) {
  m_imageIdentifier = imageId;
}


std::string UsgsAstroFrameSensorModel::getSensorIdentifier() const {
  return m_sensorName;
}


std::string UsgsAstroFrameSensorModel::getPlatformIdentifier() const {
  return m_platformName;
}


std::string UsgsAstroFrameSensorModel::getCollectionIdentifier() const {
  return m_collectionIdentifier;
}


std::string UsgsAstroFrameSensorModel::getTrajectoryIdentifier() const {
  return "";
}


std::string UsgsAstroFrameSensorModel::getSensorType() const {
    return CSM_SENSOR_TYPE_EO;
}


std::string UsgsAstroFrameSensorModel::getSensorMode() const {
    return CSM_SENSOR_MODE_FRAME;
}


std::string UsgsAstroFrameSensorModel::getReferenceDateAndTime() const {
  return "";
}


std::string UsgsAstroFrameSensorModel::getModelState() const {
    json state = {
      {"m_modelName", _SENSOR_MODEL_NAME},
      {"m_sensorName", m_sensorName},
      {"m_platformName", m_platformName},
      {"m_focalLength" , m_focalLength},
      {"m_iTransS", {m_iTransS[0], m_iTransS[1], m_iTransS[2]}},
      {"m_iTransL", {m_iTransL[0], m_iTransL[1], m_iTransL[2]}},
      {"m_boresight", {m_boresight[0], m_boresight[1], m_boresight[2]}},
      {"m_transX", {m_transX[0], m_transX[1], m_transX[2]}},
      {"m_transY", {m_transY[0], m_transY[1], m_transY[2]}},
      {"m_iTransS", {m_iTransS[0], m_iTransS[1], m_iTransS[2]}},
      {"m_iTransL", {m_iTransL[0], m_iTransL[1], m_iTransL[2]}},
      {"m_majorAxis", m_majorAxis},
      {"m_minorAxis", m_minorAxis},
      {"m_spacecraftVelocity", {m_spacecraftVelocity[0], m_spacecraftVelocity[1], m_spacecraftVelocity[2]}},
      {"m_sunPosition", {m_sunPosition[0], m_sunPosition[1], m_sunPosition[2]}},
      {"m_startingDetectorSample", m_startingDetectorSample},
      {"m_startingDetectorLine", m_startingDetectorLine},
      {"m_targetName", m_targetName},
      {"m_ifov", m_ifov},
      {"m_instrumentID", m_instrumentID},
      {"m_focalLengthEpsilon", m_focalLengthEpsilon},
      {"m_ccdCenter", {m_ccdCenter[0], m_ccdCenter[1]}},
      {"m_linePp", m_linePp},
      {"m_samplePp", m_samplePp},
      {"m_minElevation", m_minElevation},
      {"m_maxElevation", m_maxElevation},
      {"m_odtX", m_odtX},
      {"m_odtY", m_odtY},
      {"m_originalHalfLines", m_originalHalfLines},
      {"m_originalHalfSamples", m_originalHalfSamples},
      {"m_spacecraftName", m_spacecraftName},
      {"m_pixelPitch", m_pixelPitch},
      {"m_ephemerisTime", m_ephemerisTime},
      {"m_nLines", m_nLines},
      {"m_nSamples", m_nSamples},
      {"m_currentParameterValue", m_currentParameterValue},
      {"m_imageIdentifier", m_imageIdentifier},
      {"m_collectionIdentifier", m_collectionIdentifier},
      {"m_referencePointXyz", {m_referencePointXyz.x,
                               m_referencePointXyz.y,
                               m_referencePointXyz.z}},
      {"m_currentParameterCovariance", m_currentParameterCovariance}
    };
    return state.dump();
}

bool UsgsAstroFrameSensorModel::isValidModelState(const std::string& stringState, csm::WarningList *warnings) {
  std::vector<std::string> requiredKeywords = {
    "m_modelName",
    "m_majorAxis",
    "m_minorAxis",
    "m_focalLength",
    "m_startingDetectorSample",
    "m_startingDetectorLine",
    "m_focalLengthEpsilon",
    "m_nLines",
    "m_nSamples",
    "m_currentParameterValue",
    "m_ccdCenter",
    "m_spacecraftVelocity",
    "m_sunPosition",
    "m_odtX",
    "m_odtY",
    "m_transX",
    "m_transY",
    "m_iTransS",
    "m_iTransL"
  };

  json jsonState = json::parse(stringState);
  std::vector<std::string> missingKeywords;

  for (auto &key : requiredKeywords) {
    if (jsonState.find(key) == jsonState.end()) {
      missingKeywords.push_back(key);
    }
  }

  if (!missingKeywords.empty()) {
    std::ostringstream oss;
    std::copy(missingKeywords.begin(), missingKeywords.end(), std::ostream_iterator<std::string>(oss, " "));
    warnings->push_back(csm::Warning(
      csm::Warning::DATA_NOT_AVAILABLE,
      "State has missing keywrods: " + oss.str(),
      "UsgsAstroFrameSensorModel::isValidModelState"
    ));
  }

  std::string modelName = jsonState.value<std::string>("m_modelName", "");

  if (modelName != _SENSOR_MODEL_NAME) {
    warnings->push_back(csm::Warning(
      csm::Warning::DATA_NOT_AVAILABLE,
      "Incorrect model name in state, expected " + _SENSOR_MODEL_NAME + " but got " + modelName,
      "UsgsAstroFrameSensorModel::isValidModelState"
    ));
  }

  return modelName == _SENSOR_MODEL_NAME && missingKeywords.empty();
}


bool UsgsAstroFrameSensorModel::isValidIsd(const std::string& Isd, csm::WarningList *warnings) {
  // no obvious clean way to truely validate ISD with it's nested structure,
  // or rather, it would be a pain to maintain, so just check if
  // we can get a valid state from ISD. Once ISD schema is 100% clear
  // we can change this.
   try {
     std::string state = constructStateFromIsd(Isd, warnings);
     return isValidModelState(state, warnings);
   }
   catch(...) {
     return false;
   }
}


void UsgsAstroFrameSensorModel::replaceModelState(const std::string& stringState) {

    json state = json::parse(stringState);
    // The json library's .at() will except if key is missing
    try {
        m_modelName = state.at("m_modelName").get<std::string>();
        m_majorAxis = state.at("m_majorAxis").get<double>();
        m_minorAxis = state.at("m_minorAxis").get<double>();
        m_focalLength = state.at("m_focalLength").get<double>();
        m_startingDetectorSample = state.at("m_startingDetectorSample").get<double>();
        m_startingDetectorLine = state.at("m_startingDetectorLine").get<double>();
        m_focalLengthEpsilon = state.at("m_focalLengthEpsilon").get<double>();
        m_nLines = state.at("m_nLines").get<int>();
        m_nSamples = state.at("m_nSamples").get<int>();
        m_currentParameterValue = state.at("m_currentParameterValue").get<std::vector<double>>();
        m_ccdCenter = state.at("m_ccdCenter").get<std::vector<double>>();
        m_spacecraftVelocity = state.at("m_spacecraftVelocity").get<std::vector<double>>();
        m_sunPosition = state.at("m_sunPosition").get<std::vector<double>>();
        m_odtX = state.at("m_odtX").get<std::vector<double>>();
        m_odtY = state.at("m_odtY").get<std::vector<double>>();
        m_transX = state.at("m_transX").get<std::vector<double>>();
        m_transY = state.at("m_transY").get<std::vector<double>>();
        m_iTransS = state.at("m_iTransS").get<std::vector<double>>();
        m_iTransL = state.at("m_iTransL").get<std::vector<double>>();
        m_imageIdentifier = state.at("m_imageIdentifier").get<std::string>();
        m_platformName = state.at("m_platformName").get<std::string>();
        m_sensorName = state.at("m_sensorName").get<std::string>();
        m_collectionIdentifier = state.at("m_collectionIdentifier").get<std::string>();
        std::vector<double> refpt = state.at("m_referencePointXyz").get<std::vector<double>>();
        m_referencePointXyz.x = refpt[0];
        m_referencePointXyz.y = refpt[1];
        m_referencePointXyz.z = refpt[2];
        m_currentParameterCovariance = state.at("m_currentParameterCovariance").get<std::vector<double>>();


        // Leaving unused params commented out
        // m_targetName = state.at("m_targetName").get<std::string>();
        // m_ifov = state.at("m_ifov").get<double>();
        // m_instrumentID = state.at("m_instrumentID").get<std::string>();
        // m_currentParameterCovariance = state.at("m_currentParameterCovariance").get<std::vector<double>>();
        // m_noAdjustments = state.at("m_noAdjustments").get<std::vector<double>>();
        // m_linePp = state.at("m_linePp").get<double>();
        // m_samplePp = state.at("m_samplePp").get<double>();
        // m_originalHalfLines = state.at("m_originalHalfLines").get<double>();
        // m_spacecraftName = state.at("m_spacecraftName").get<std::string>();
        // m_pixelPitch = state.at("m_pixelPitch").get<double>();
        // m_ephemerisTime = state.at("m_ephemerisTime").get<double>();
        // m_originalHalfSamples = state.at("m_originalHalfSamples").get<double>();
        // m_boresight = state.at("m_boresight").get<std::vector<double>>();

        // Cast int vector to csm::param::Type vector by simply copying it
        // std::vector<int> paramType = state.at("m_parameterType").get<std::vector<int>>();
        // m_parameterType = std::vector<csm::param::Type>();
        // for(auto &t : paramType){
           // paramType.push_back(static_cast<csm::param::Type>(t));
        // }
    }
    catch(std::out_of_range& e) {
      throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                       "State keywords required to generate sensor model missing: " + std::string(e.what()) + "\nUsing model string: " + stringState,
                       "UsgsAstroFrameSensorModel::replaceModelState");
    }
}


std::string UsgsAstroFrameSensorModel::constructStateFromIsd(const std::string& jsonIsd, csm::WarningList* warnings) {

    json isd = json::parse(jsonIsd);
    json state = {};

    csm::WarningList* parsingWarnings = new csm::WarningList;

    state["m_modelName"] = getSensorModelName(isd, parsingWarnings);
    state["m_imageIdentifier"] = getImageId(isd, parsingWarnings);
    state["m_sensorName"] = getSensorName(isd, parsingWarnings);
    state["m_platformName"] = getPlatformName(isd, parsingWarnings);

    state["m_startingDetectorSample"] = getDetectorStartingSample(isd, parsingWarnings);
    state["m_startingDetectorLine"] = getDetectorStartingLine(isd, parsingWarnings);


    // get focal length
    state["m_focalLength"] = getFocalLength(isd, parsingWarnings);
    state["m_focalLengthEpsilon"] = getFocalLengthEpsilon(isd, parsingWarnings);


    state["m_currentParameterValue"] = json();

    // get sensor_position
    std::vector<double> position = getSensorPositions(isd, parsingWarnings);
    if (!position.empty() && position.size() != 3) {
      parsingWarnings->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Sensor position does not have 3 values.",
          "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
      state["m_currentParameterValue"][0] = 0;
      state["m_currentParameterValue"][1] = 0;
      state["m_currentParameterValue"][2] = 0;
    }
    else {
      state["m_currentParameterValue"] = position;
    }

    // get sensor_velocity
    std::vector<double> velocity = getSensorVelocities(isd, parsingWarnings);
    if (!velocity.empty() && velocity.size() != 3) {
      parsingWarnings->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Sensor velocity does not have 3 values.",
          "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    }
    else {
      state["m_spacecraftVelocity"] = velocity;
    }


    // get sun_position
    // sun position is not strictly necessary, but is required for getIlluminationDirection.
    state["m_sunPosition"] = getSunPositions(isd);


    // get sensor_orientation quaternion
    std::vector<double> quaternion = getSensorOrientations(isd, parsingWarnings);
    if (!quaternion.empty() && quaternion.size() != 4) {
      parsingWarnings->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Sensor orientation quaternion does not have 4 values.",
          "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    }
    else {
      state["m_currentParameterValue"][3] = quaternion[0];
      state["m_currentParameterValue"][4] = quaternion[1];
      state["m_currentParameterValue"][5] = quaternion[2];
      state["m_currentParameterValue"][6] = quaternion[3];
    }


    // get optical_distortion
    state["m_odtX"] = getTransverseDistortionX(isd, parsingWarnings);
    state["m_odtY"] = getTransverseDistortionY(isd, parsingWarnings);


    // get detector_center
    state["m_ccdCenter"][0] = getDetectorCenterLine(isd, parsingWarnings);
    state["m_ccdCenter"][1] = getDetectorCenterSample(isd, parsingWarnings);


    // get radii
    state["m_minorAxis"] = getSemiMinorRadius(isd, parsingWarnings);
    state["m_majorAxis"] = getSemiMajorRadius(isd, parsingWarnings);


    // get reference_height
    state["m_minElevation"] = getMinHeight(isd, parsingWarnings);
    state["m_maxElevation"] = getMaxHeight(isd, parsingWarnings);


    state["m_ephemerisTime"] = getCenterTime(isd, parsingWarnings);
    state["m_nLines"] = getTotalLines(isd, parsingWarnings);
    state["m_nSamples"] = getTotalSamples(isd, parsingWarnings);

    state["m_iTransL"] = getFocal2PixelLines(isd, parsingWarnings);
    state["m_iTransS"] = getFocal2PixelSamples(isd, parsingWarnings);

    // We don't pass the pixel to focal plane transformation so invert the
    // focal plane to pixel transformation
    try {
      double determinant = state["m_iTransL"][1].get<double>() * state["m_iTransS"][2].get<double>() -
                           state["m_iTransL"][2].get<double>() * state["m_iTransS"][1].get<double>();

      state["m_transX"][1] =  state["m_iTransL"][1].get<double>() / determinant;
      state["m_transX"][2] = -state["m_iTransS"][1].get<double>() / determinant;
      state["m_transX"][0] = -(state["m_transX"][1].get<double>() * state["m_iTransL"][0].get<double>() +
                              state["m_transX"][2].get<double>() * state["m_iTransS"][0].get<double>());

      state["m_transY"][1] = -state["m_iTransL"][2].get<double>() / determinant;
      state["m_transY"][2] =  state["m_iTransS"][2].get<double>() / determinant;
      state["m_transY"][0] = -(state["m_transY"][1].get<double>() * state["m_iTransL"][0].get<double>() +
                               state["m_transY"][2].get<double>() * state["m_iTransS"][0].get<double>());
    }
    catch (...) {
      parsingWarnings->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not compute detector pixel to focal plane coordinate transformation.",
          "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    }


    state["m_referencePointXyz"] = std::vector<double>(3, 0.0);
    state["m_currentParameterCovariance"] = std::vector<double>(NUM_PARAMETERS*NUM_PARAMETERS,0.0);
    state["m_collectionIdentifier"] = "";


    if (!parsingWarnings->empty()) {
      if (warnings) {
        warnings->insert(warnings->end(), parsingWarnings->begin(), parsingWarnings->end());
      }
      delete parsingWarnings;
      parsingWarnings = nullptr;
      throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                       "ISD is invalid for creating the sensor model.",
                       "UsgsAstroFrameSensorModel::constructStateFromIsd");
    }

    delete parsingWarnings;
    parsingWarnings = nullptr;

    return state.dump();

}


csm::EcefCoord UsgsAstroFrameSensorModel::getReferencePoint() const {
  return m_referencePointXyz;
}


void UsgsAstroFrameSensorModel::setReferencePoint(const csm::EcefCoord &groundPt) {
  m_referencePointXyz = groundPt;
}


int UsgsAstroFrameSensorModel::getNumParameters() const {
  return NUM_PARAMETERS;
}


std::string UsgsAstroFrameSensorModel::getParameterName(int index) const {
  return m_parameterName[index];
}


std::string UsgsAstroFrameSensorModel::getParameterUnits(int index) const {

  if (index < 3) {
    return "m";
  }
  else {
    return "radians";
  }
}


bool UsgsAstroFrameSensorModel::hasShareableParameters() const {
  return false;
}


bool UsgsAstroFrameSensorModel::isParameterShareable(int index) const {
  return false;
}


csm::SharingCriteria UsgsAstroFrameSensorModel::getParameterSharingCriteria(int index) const {
   // Parameter sharing is not supported for this sensor,
   // all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index out of range.",
      "UsgsAstroLsSensorModel::getParameterSharingCriteria");
}


double UsgsAstroFrameSensorModel::getParameterValue(int index) const {
  return m_currentParameterValue[index];

}


void UsgsAstroFrameSensorModel::setParameterValue(int index, double value) {
  m_currentParameterValue[index] = value;
}


csm::param::Type UsgsAstroFrameSensorModel::getParameterType(int index) const {
  return m_parameterType[index];
}


void UsgsAstroFrameSensorModel::setParameterType(int index, csm::param::Type pType) {
    m_parameterType[index] = pType;
}


double UsgsAstroFrameSensorModel::getParameterCovariance(int index1, int index2) const {
   int index = UsgsAstroFrameSensorModel::NUM_PARAMETERS * index1 + index2;
   return m_currentParameterCovariance[index];
}


void UsgsAstroFrameSensorModel::setParameterCovariance(int index1, int index2, double covariance) {
   int index = UsgsAstroFrameSensorModel::NUM_PARAMETERS * index1 + index2;
   m_currentParameterCovariance[index] = covariance;
}


int UsgsAstroFrameSensorModel::getNumGeometricCorrectionSwitches() const {
  return 0;
}


std::string UsgsAstroFrameSensorModel::getGeometricCorrectionName(int index) const {
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::getGeometricCorrectionName");
}


void UsgsAstroFrameSensorModel::setGeometricCorrectionSwitch(int index,
                                                      bool value,
                                                      csm::param::Type pType) {
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::setGeometricCorrectionSwitch");
}


bool UsgsAstroFrameSensorModel::getGeometricCorrectionSwitch(int index) const {
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::getGeometricCorrectionSwitch");
}


std::vector<double> UsgsAstroFrameSensorModel::getCrossCovarianceMatrix(
    const GeometricModel &comparisonModel,
    csm::param::Set pSet,
    const GeometricModelList &otherModels) const {

   // No correlation between models.
   const std::vector<int>& indices = getParameterSetIndices(pSet);
   size_t num_rows = indices.size();
   const std::vector<int>& indices2 = comparisonModel.getParameterSetIndices(pSet);
   size_t num_cols = indices.size();

   return std::vector<double>(num_rows * num_cols, 0.0);
}


void UsgsAstroFrameSensorModel::calcRotationMatrix(
    double m[3][3]) const {
  // Trigonometric functions for rotation matrix
  double w = m_currentParameterValue[3];
  double x = m_currentParameterValue[4];
  double y = m_currentParameterValue[5];
  double z = m_currentParameterValue[6];

  m[0][0] = w*w + x*x - y*y - z*z;
  m[0][1] = 2 * (x*y - w*z);
  m[0][2] = 2 * (w*y + x*z);
  m[1][0] = 2 * (x*y + w*z);
  m[1][1] = w*w - x*x + y*y - z*z;
  m[1][2] = 2 * (y*z - w*x);
  m[2][0] = 2 * (x*z - w*y);
  m[2][1] = 2 * (w*x + y*z);
  m[2][2] = w*w - x*x - y*y + z*z;
}


void UsgsAstroFrameSensorModel::calcRotationMatrix(
  double m[3][3], const std::vector<double> &adjustments) const {
  // Trigonometric functions for rotation matrix
  double w = getValue(3, adjustments);
  double x = getValue(4, adjustments);
  double y = getValue(5, adjustments);
  double z = getValue(6, adjustments);

  m[0][0] = w*w + x*x - y*y - z*z;
  m[0][1] = 2 * (x*y - w*z);
  m[0][2] = 2 * (w*y + x*z);
  m[1][0] = 2 * (x*y + w*z);
  m[1][1] = w*w - x*x + y*y - z*z;
  m[1][2] = 2 * (y*z - w*x);
  m[2][0] = 2 * (x*z - w*y);
  m[2][1] = 2 * (w*x + y*z);
  m[2][2] = w*w - x*x - y*y + z*z;
}


void UsgsAstroFrameSensorModel::losEllipsoidIntersect(
      const double height,
      const double xc,
      const double yc,
      const double zc,
      const double xl,
      const double yl,
      const double zl,
      double&       x,
      double&       y,
      double&       z ) const
{
   // Helper function which computes the intersection of the image ray
   // with an expanded ellipsoid.  All vectors are in earth-centered-fixed
   // coordinate system with origin at the center of the earth.

   double ap, bp, k;
   ap = m_majorAxis + height;
   bp = m_minorAxis + height;
   k = ap * ap / (bp * bp);

   // Solve quadratic equation for scale factor
   // applied to image ray to compute ground point

   double at, bt, ct, quadTerm;
   at = xl * xl + yl * yl + k * zl * zl;
   bt = 2.0 * (xl * xc + yl * yc + k * zl * zc);
   ct = xc * xc + yc * yc + k * zc * zc - ap * ap;
   quadTerm = bt * bt - 4.0 * at * ct;

   // If quadTerm is negative, the image ray does not
   // intersect the ellipsoid. Setting the quadTerm to
   // zero means solving for a point on the ray nearest
   // the surface of the ellisoid.

   if ( 0.0 > quadTerm )
   {
      quadTerm = 0.0;
   }
   double scale;
   scale = (-bt - sqrt (quadTerm)) / (2.0 * at);
   // Compute ground point vector

   x = xc + scale * xl;
   y = yc + scale * yl;
   z = zc + scale * zl;
}


/***** Helper Functions *****/

double UsgsAstroFrameSensorModel::getValue(
   int index,
   const std::vector<double> &adjustments) const
{
   return m_currentParameterValue[index] + adjustments[index];
}
