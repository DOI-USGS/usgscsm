#include "UsgsAstroFrameSensorModel.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include <json/json.hpp>

#include <Error.h>
#include <Version.h>

#define MESSAGE_LOG(logger, ...) if (logger) { logger->info(__VA_ARGS__); }

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
    m_detectorSampleSumming = 1.0;
    m_detectorLineSumming = 1.0;
    m_targetName = "";
    m_ifov = 0;
    m_instrumentID = "";
    m_focalLengthEpsilon = 0.0;
    m_originalHalfLines = 0.0;
    m_spacecraftName = "";
    m_pixelPitch = 0.0;
    m_ephemerisTime = 0.0;
    m_originalHalfSamples = 0.0;
    m_nLines = 0;
    m_nSamples = 0;

    m_currentParameterValue = std::vector<double>(NUM_PARAMETERS, 0.0);
    m_currentParameterCovariance = std::vector<double>(NUM_PARAMETERS*NUM_PARAMETERS,0.0);
    for (int i = 0; i < NUM_PARAMETERS*NUM_PARAMETERS; i += NUM_PARAMETERS+1) {
      m_currentParameterCovariance[i] = 1;
    }
    m_noAdjustments = std::vector<double>(NUM_PARAMETERS,0.0);
    m_ccdCenter = std::vector<double>(2, 0.0);
    m_spacecraftVelocity = std::vector<double>(3, 0.0);
    m_sunPosition = std::vector<double>(3, 0.0);
    m_distortionType = DistortionType::TRANSVERSE;
    m_opticalDistCoeffs = std::vector<double>(20, 0.0);
    m_transX = std::vector<double>(3, 0.0);
    m_transY = std::vector<double>(3, 0.0);
    m_iTransS = std::vector<double>(3, 0.0);
    m_iTransL = std::vector<double>(3, 0.0);
    m_boresight = std::vector<double>(3, 0.0);
    m_parameterType = std::vector<csm::param::Type>(NUM_PARAMETERS, csm::param::REAL);
    m_referencePointXyz.x = 0;
    m_referencePointXyz.y = 0;
    m_referencePointXyz.z = 0;
    m_logFile = "";
    m_logger.reset();
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
  MESSAGE_LOG(this->m_logger, "Computing groundToImage(No adjustments) for {}, {}, {}, with desired precision {}",
              groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

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

  MESSAGE_LOG(this->m_logger, "Computing groundToImage for {}, {}, {}, with desired precision {}",
              groundPt.x, groundPt.y, groundPt.z, desired_precision);

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
  calcRotationMatrix(m,adjustments);

  // Sensor position
  double undistortedx, undistortedy, denom;
  denom = m[0][2] * xo + m[1][2] * yo + m[2][2] * zo;
  undistortedx = (f * (m[0][0] * xo + m[1][0] * yo + m[2][0] * zo)/denom);
  undistortedy = (f * (m[0][1] * xo + m[1][1] * yo + m[2][1] * zo)/denom);

  // Apply the distortion to the line/sample location and then convert back to line/sample
  double distortedX, distortedY;
  applyDistortion(undistortedx, undistortedy, distortedX, distortedY,
                  m_opticalDistCoeffs, m_distortionType);


  // Convert distorted mm into line/sample
  double sample, line;
  computePixel(
    distortedX, distortedY,
    m_ccdCenter[1], m_ccdCenter[0],
    m_detectorSampleSumming, m_detectorLineSumming,
    m_startingDetectorSample, m_startingDetectorLine,
    &m_iTransS[0], &m_iTransL[0],
    line, sample);

  return csm::ImageCoord(line, sample);
}


csm::ImageCoordCovar UsgsAstroFrameSensorModel::groundToImage(const csm::EcefCoordCovar &groundPt,
                                   double desiredPrecision,
                                   double *achievedPrecision,
                                   csm::WarningList *warnings) const {
    MESSAGE_LOG(this->m_logger, "Computeing groundToImage(Covar) for {}, {}, {}, with desired precision {}",
                groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

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

  MESSAGE_LOG(this->m_logger, "Computing imageToGround for {}, {}, {}, with desired precision {}",
                imagePt.line, imagePt.samp, height, desiredPrecision);

  double sample = imagePt.samp;
  double line = imagePt.line;

  // Here is where we should be able to apply an adjustment to opk
  double m[3][3];
  calcRotationMatrix(m);
  MESSAGE_LOG(this->m_logger, "Calculated rotation matrix [{}, {}, {}], [{}, {}, {}], [{}, {}, {}]",
                m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]);

  // Apply the principal point offset, assuming the pp is given in pixels
  double xl, yl, zl;

  // Convert from the pixel space into the metric space
  double x_camera, y_camera;
  computeDistortedFocalPlaneCoordinates(
        line, sample,
        m_ccdCenter[1], m_ccdCenter[0],
        m_detectorSampleSumming, m_detectorLineSumming,
        m_startingDetectorSample, m_startingDetectorLine,
        &m_iTransS[0], &m_iTransL[0],
        x_camera, y_camera);

  // Apply the distortion model (remove distortion)
  double undistortedX, undistortedY;
  removeDistortion(x_camera, y_camera, undistortedX, undistortedY,
                   m_opticalDistCoeffs,
                   m_distortionType);
  MESSAGE_LOG(this->m_logger, "Found undistortedX: {}, and undistortedY: {}",
                               undistortedX, undistortedY);

  // Now back from distorted mm to pixels
  xl = m[0][0] * undistortedX + m[0][1] * undistortedY - m[0][2] * - m_focalLength;
  yl = m[1][0] * undistortedX + m[1][1] * undistortedY - m[1][2] * - m_focalLength;
  zl = m[2][0] * undistortedX + m[2][1] * undistortedY - m[2][2] * - m_focalLength;
  MESSAGE_LOG(this->m_logger, "Compute xl, yl, zl as {}, {}, {}", xl, yl, zl);

  double xc, yc, zc;
  xc = m_currentParameterValue[0];
  yc = m_currentParameterValue[1];
  zc = m_currentParameterValue[2];
  MESSAGE_LOG(this->m_logger, "Set xc, yc, zc to {}, {}, {}",
                              m_currentParameterValue[0], m_currentParameterValue[1], m_currentParameterValue[2]);

  // Intersect with some height about the ellipsoid.
  double x, y, z;
  losEllipsoidIntersect(height, xc, yc, zc, xl, yl, zl, x, y, z);

  return csm::EcefCoord(x, y, z);
}


csm::EcefCoordCovar UsgsAstroFrameSensorModel::imageToGround(const csm::ImageCoordCovar &imagePt, double height,
                                  double heightVariance, double desiredPrecision,
                                  double *achievedPrecision,
                                  csm::WarningList *warnings) const {

    MESSAGE_LOG(this->m_logger, "Computeing imageToGround(Covar) for {}, {}, {}, with desired precision {}",
                imagePt.line, imagePt.samp, height, desiredPrecision);
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
  MESSAGE_LOG(this->m_logger, "Computeing imageToProximateImagingLocus(No ground) for point {}, {}, {}, with desired precision {}",
                               imagePt.line, imagePt.samp, desiredPrecision);
  return imageToRemoteImagingLocus(imagePt);
}


csm::EcefLocus UsgsAstroFrameSensorModel::imageToRemoteImagingLocus(const csm::ImageCoord &imagePt,
                                                             double desiredPrecision,
                                                             double *achievedPrecision,
                                                             csm::WarningList *warnings) const {
  MESSAGE_LOG(this->m_logger, "Computeing imageToProximateImagingLocus for {}, {}, {}, with desired precision {}",
                               imagePt.line, imagePt.samp, desiredPrecision);
  // Find the line,sample on the focal plane (mm)
  double focalPlaneX, focalPlaneY;
  computeDistortedFocalPlaneCoordinates(
        imagePt.line, imagePt.samp,
        m_ccdCenter[1], m_ccdCenter[0],
        m_detectorSampleSumming, m_detectorLineSumming,
        m_startingDetectorSample, m_startingDetectorLine,
        &m_iTransS[0], &m_iTransL[0],
        focalPlaneY, focalPlaneY);

  // Distort
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  removeDistortion(focalPlaneX, focalPlaneY,
                   undistortedFocalPlaneX, undistortedFocalPlaneY,
                   m_opticalDistCoeffs,
                   m_distortionType);

  // Get rotation matrix and transform to a body-fixed frame
  double m[3][3];
  calcRotationMatrix(m);
  std::vector<double> lookC { undistortedFocalPlaneX, undistortedFocalPlaneY, m_focalLength };
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

  MESSAGE_LOG(this->m_logger, "Accessing Image Start line: {}, sample: {}",
                              m_startingDetectorLine, m_startingDetectorSample);
  csm::ImageCoord start;
  start.samp = m_startingDetectorSample;
  start.line = m_startingDetectorLine;
  return start;
}


csm::ImageVector UsgsAstroFrameSensorModel::getImageSize() const {

  MESSAGE_LOG(this->m_logger, "Accessing Image Size line: {}, sample: {}",
                              m_nLines, m_nSamples);
  csm::ImageVector size;
  size.line = m_nLines;
  size.samp = m_nSamples;
  return size;
}


std::pair<csm::ImageCoord, csm::ImageCoord> UsgsAstroFrameSensorModel::getValidImageRange() const {
    MESSAGE_LOG(this->m_logger, "Accessing Image Range");
    csm::ImageCoord min_pt(m_startingDetectorLine, m_startingDetectorSample);
    csm::ImageCoord max_pt(m_nLines, m_nSamples);
    return std::pair<csm::ImageCoord, csm::ImageCoord>(min_pt, max_pt);
}


std::pair<double, double> UsgsAstroFrameSensorModel::getValidHeightRange() const {
    MESSAGE_LOG(this->m_logger, "Accessing Image Height min: {}, max: {}",
                                m_minElevation, m_maxElevation);
    return std::pair<double, double>(m_minElevation, m_maxElevation);
}


csm::EcefVector UsgsAstroFrameSensorModel::getIlluminationDirection(const csm::EcefCoord &groundPt) const {
  // ground (body-fixed) - sun (body-fixed) gives us the illumination direction.
  MESSAGE_LOG(this->m_logger, "Accessing illumination direction for ground point {}, {}, {}",
              groundPt.x, groundPt.y, groundPt.z);
  return csm::EcefVector {
    groundPt.x - m_sunPosition[0],
    groundPt.y - m_sunPosition[1],
    groundPt.z - m_sunPosition[2]
  };
}


double UsgsAstroFrameSensorModel::getImageTime(const csm::ImageCoord &imagePt) const {
  MESSAGE_LOG(this->m_logger, "Accessing image time for image point {}, {}",
              imagePt.line, imagePt.samp);
    return m_ephemerisTime;
}


csm::EcefCoord UsgsAstroFrameSensorModel::getSensorPosition(const csm::ImageCoord &imagePt) const {
  MESSAGE_LOG(this->m_logger, "Accessing sensor position for image point {}, {}",
              imagePt.line, imagePt.samp);
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
    MESSAGE_LOG(this->m_logger, "Accessing sensor position for time {}", time);
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
  MESSAGE_LOG(this->m_logger, "Accessing sensor velocity for image point {}, {}",
              imagePt.line, imagePt.samp);
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
    MESSAGE_LOG(this->m_logger, "Accessing sensor position for time {}", time);
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
    MESSAGE_LOG(this->m_logger, "Computing sensor partials image point from ground point {}, {}, {} \
                                 and desiredPrecision: {}", groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

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

  MESSAGE_LOG(this->m_logger, "Computing sensor partials for ground point {}, {}, {}\
                               with point: {}, {}, index: {}, and desiredPrecision: {}",
                               groundPt.x, groundPt.y, groundPt.z, imagePt.line, imagePt.samp,
                               index, desiredPrecision);
  double delta = 1.0;
  // The rotation parameters will usually be small (<1),
  // so a delta of 1.0 is too small.
  // Rotation parameters are indices 3-6
  if (index > 2 && index < 7) {
    delta = 0.01;
  }
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
  MESSAGE_LOG(this->m_logger, "Computing all sensor partials for ground point {}, {}, {}\
                               with point: {}, {}, pset: {}, and desiredPrecision: {}",
                               groundPt.x, groundPt.y, groundPt.z, imagePt.line, imagePt.samp,
                               pset, desiredPrecision);
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
  MESSAGE_LOG(this->m_logger, "Computing all sensor partials image point from ground point {}, {}, {} \
                               and desiredPrecision: {}", groundPt.x, groundPt.y, groundPt.z, desiredPrecision);
  csm::ImageCoord imagePt = groundToImage(groundPt,
                                          desiredPrecision, achievedPrecision, warnings);
  return computeAllSensorPartials(imagePt, groundPt,
                                  pset, desiredPrecision, achievedPrecision, warnings);
    }

std::vector<double> UsgsAstroFrameSensorModel::computeGroundPartials(const csm::EcefCoord
                                                                     &groundPt) const {
  MESSAGE_LOG(this->m_logger, "Computing ground partials for ground point {}, {}, {}",
                               groundPt.x, groundPt.y, groundPt.z);
  // Partial of line, sample wrt X, Y, Z
  double x = groundPt.x;
  double y = groundPt.y;
  double z = groundPt.z;

  csm::ImageCoord ipB = groundToImage(groundPt);
  csm::EcefCoord nextPoint = imageToGround(csm::ImageCoord(ipB.line + 1, ipB.samp + 1));
  double dx = nextPoint.x - x;
  double dy = nextPoint.y - y;
  double dz = nextPoint.z - z;
  double pixelGroundSize = sqrt((dx*dx + dy*dy + dz*dz) / 2.0);

  // If the ground size is too small, try the opposite direction
  if (pixelGroundSize < 1e-10) {
    nextPoint = imageToGround(csm::ImageCoord(ipB.line - 1, ipB.samp - 1));
    dx = nextPoint.x - x;
    dy = nextPoint.y - y;
    dz = nextPoint.z - z;
    pixelGroundSize = sqrt((dx*dx + dy*dy + dz*dz) / 2.0);
  }

  csm::ImageCoord ipX = groundToImage(csm::EcefCoord(x + pixelGroundSize, y, z));
  csm::ImageCoord ipY = groundToImage(csm::EcefCoord(x, y + pixelGroundSize, z));
  csm::ImageCoord ipZ = groundToImage(csm::EcefCoord(x, y, z + pixelGroundSize));

  std::vector<double> partials(6, 0.0);
  partials[0] = (ipX.line - ipB.line) / pixelGroundSize;
  partials[3] = (ipX.samp - ipB.samp) / pixelGroundSize;
  partials[1] = (ipY.line - ipB.line) / pixelGroundSize;
  partials[4] = (ipY.samp - ipB.samp) / pixelGroundSize;
  partials[2] = (ipZ.line - ipB.line) / pixelGroundSize;
  partials[5] = (ipZ.samp - ipB.samp) / pixelGroundSize;

  MESSAGE_LOG(this->m_logger, "Computing ground partials results:\nLine: {}, {}, {}\nSample: {}, {}, {}",
                               partials[0], partials[1], partials[2], partials[3], partials[4], partials[5]);

  return partials;
}


const csm::CorrelationModel& UsgsAstroFrameSensorModel::getCorrelationModel() const {
  MESSAGE_LOG(this->m_logger, "Accessing correlation model");
  return _no_corr_model;
}


std::vector<double> UsgsAstroFrameSensorModel::getUnmodeledCrossCovariance(const csm::ImageCoord &pt1,
                                                const csm::ImageCoord &pt2) const {

   // No unmodeled error
   MESSAGE_LOG(this->m_logger, "Accessing unmodeled cross covar with \
                                point1: {}, {} and point2: {}, {}",
                                pt1.line, pt1.samp, pt2.line, pt2.samp);
   return std::vector<double>(4, 0.0);
}


csm::Version UsgsAstroFrameSensorModel::getVersion() const {
  MESSAGE_LOG(this->m_logger, "Accessing CSM version");
  return csm::Version(0,1,0);
}


std::string UsgsAstroFrameSensorModel::getModelName() const {
  MESSAGE_LOG(this->m_logger, "Accessing CSM name {}", _SENSOR_MODEL_NAME);
  return _SENSOR_MODEL_NAME;
}


std::string UsgsAstroFrameSensorModel::getPedigree() const {
  MESSAGE_LOG(this->m_logger, "Accessing CSM pedigree");
  return "USGS_FRAMER";
}


std::string UsgsAstroFrameSensorModel::getImageIdentifier() const {
  MESSAGE_LOG(this->m_logger, "Accessing image ID {}", m_imageIdentifier);
  return m_imageIdentifier;
}


void UsgsAstroFrameSensorModel::setImageIdentifier(const std::string& imageId,
                                            csm::WarningList* warnings) {
  MESSAGE_LOG(this->m_logger, "Setting image ID to {}", imageId);
  m_imageIdentifier = imageId;
}


std::string UsgsAstroFrameSensorModel::getSensorIdentifier() const {
  MESSAGE_LOG(this->m_logger, "Accessing sensor ID");
  return m_sensorName;
}


std::string UsgsAstroFrameSensorModel::getPlatformIdentifier() const {
  MESSAGE_LOG(this->m_logger, "Accessing platform ID");
  return m_platformName;
}


std::string UsgsAstroFrameSensorModel::getCollectionIdentifier() const {
  MESSAGE_LOG(this->m_logger, "Accessing collection ID");
  return m_collectionIdentifier;
}


std::string UsgsAstroFrameSensorModel::getTrajectoryIdentifier() const {
  MESSAGE_LOG(this->m_logger, "Accessing trajectory ID");
  return "";
}


std::string UsgsAstroFrameSensorModel::getSensorType() const {
    MESSAGE_LOG(this->m_logger, "Accessing sensor type");
    return CSM_SENSOR_TYPE_EO;
}


std::string UsgsAstroFrameSensorModel::getSensorMode() const {
    MESSAGE_LOG(this->m_logger, "Accessing sensor mode");
    return CSM_SENSOR_MODE_FRAME;
}


std::string UsgsAstroFrameSensorModel::getReferenceDateAndTime() const {
  MESSAGE_LOG(this->m_logger, "Accessing reference data and time");
  csm::EcefCoord referencePointGround = UsgsAstroFrameSensorModel::getReferencePoint();
  csm::ImageCoord referencePointImage = UsgsAstroFrameSensorModel::groundToImage(referencePointGround);
  time_t ephemTime = UsgsAstroFrameSensorModel::getImageTime(referencePointImage);
  struct tm t = {0};  // Initalize to all 0's
  t.tm_year = 100;  // This is year-1900, so 100 = 2000
  t.tm_mday = 1;
  time_t timeSinceEpoch = mktime(&t);
  time_t finalTime = ephemTime + timeSinceEpoch;
  char buffer [16];
  strftime(buffer, 16, "%Y%m%dT%H%M%S", localtime(&finalTime));
  buffer[15] = '\0';

  return buffer;
}


std::string UsgsAstroFrameSensorModel::getModelState() const {
    MESSAGE_LOG(this->m_logger, "Dumping model state");
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
      {"m_detectorSampleSumming", m_detectorSampleSumming},
      {"m_detectorLineSumming", m_detectorLineSumming},
      {"m_targetName", m_targetName},
      {"m_ifov", m_ifov},
      {"m_instrumentID", m_instrumentID},
      {"m_focalLengthEpsilon", m_focalLengthEpsilon},
      {"m_ccdCenter", {m_ccdCenter[0], m_ccdCenter[1]}},
      {"m_minElevation", m_minElevation},
      {"m_maxElevation", m_maxElevation},
      {"m_distortionType", m_distortionType},
      {"m_opticalDistCoeffs", m_opticalDistCoeffs},
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
      {"m_currentParameterCovariance", m_currentParameterCovariance},
      {"m_logFile", m_logFile}
    };

    return state.dump();
}

bool UsgsAstroFrameSensorModel::isValidModelState(const std::string& stringState, csm::WarningList *warnings) {
  MESSAGE_LOG(this->m_logger, "Checking if model has valid state");
  std::vector<std::string> requiredKeywords = {
    "m_modelName",
    "m_majorAxis",
    "m_minorAxis",
    "m_focalLength",
    "m_startingDetectorSample",
    "m_startingDetectorLine",
    "m_detectorSampleSumming",
    "m_detectorLineSumming",
    "m_focalLengthEpsilon",
    "m_nLines",
    "m_nSamples",
    "m_currentParameterValue",
    "m_ccdCenter",
    "m_spacecraftVelocity",
    "m_sunPosition",
    "m_distortionType",
    "m_opticalDistCoeffs",
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

  if (!missingKeywords.empty() && warnings) {
    std::ostringstream oss;
    std::copy(missingKeywords.begin(), missingKeywords.end(), std::ostream_iterator<std::string>(oss, " "));
    warnings->push_back(csm::Warning(
      csm::Warning::DATA_NOT_AVAILABLE,
      "State has missing keywrods: " + oss.str(),
      "UsgsAstroFrameSensorModel::isValidModelState"
    ));
  }

  std::string modelName = jsonState.value<std::string>("m_modelName", "");

  if (modelName != _SENSOR_MODEL_NAME && warnings) {
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
  MESSAGE_LOG(this->m_logger, "Building isd to check model state");
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
    MESSAGE_LOG(this->m_logger, "Replaceing model state");
    // The json library's .at() will except if key is missing
    try {
        m_modelName = state.at("m_modelName").get<std::string>();
        m_majorAxis = state.at("m_majorAxis").get<double>();
        m_minorAxis = state.at("m_minorAxis").get<double>();
        m_focalLength = state.at("m_focalLength").get<double>();
        m_startingDetectorSample = state.at("m_startingDetectorSample").get<double>();
        m_startingDetectorLine = state.at("m_startingDetectorLine").get<double>();
        m_detectorSampleSumming = state.at("m_detectorSampleSumming").get<double>();
        m_detectorLineSumming = state.at("m_detectorLineSumming").get<double>();
        m_focalLengthEpsilon = state.at("m_focalLengthEpsilon").get<double>();
        m_nLines = state.at("m_nLines").get<int>();
        m_nSamples = state.at("m_nSamples").get<int>();
        m_ephemerisTime = state.at("m_ephemerisTime").get<int>();
        m_currentParameterValue = state.at("m_currentParameterValue").get<std::vector<double>>();
        m_ccdCenter = state.at("m_ccdCenter").get<std::vector<double>>();
        m_spacecraftVelocity = state.at("m_spacecraftVelocity").get<std::vector<double>>();
        m_sunPosition = state.at("m_sunPosition").get<std::vector<double>>();
        m_distortionType = (DistortionType)state.at("m_distortionType").get<int>();
        m_opticalDistCoeffs = state.at("m_opticalDistCoeffs").get<std::vector<double>>();
        m_transX = state.at("m_transX").get<std::vector<double>>();
        m_transY = state.at("m_transY").get<std::vector<double>>();
        m_iTransS = state.at("m_iTransS").get<std::vector<double>>();
        m_iTransL = state.at("m_iTransL").get<std::vector<double>>();
        m_imageIdentifier = state.at("m_imageIdentifier").get<std::string>();
        m_platformName = state.at("m_platformName").get<std::string>();
        m_sensorName = state.at("m_sensorName").get<std::string>();
        m_collectionIdentifier = state.at("m_collectionIdentifier").get<std::string>();
        // Set reference point to the center of the image
        m_referencePointXyz = imageToGround(csm::ImageCoord(m_nLines/2.0, m_nSamples/2.0));
        m_currentParameterCovariance = state.at("m_currentParameterCovariance").get<std::vector<double>>();
        m_logFile = state.at("m_logFile").get<std::string>();
        if (m_logFile.empty()) {
          m_logger.reset();
        }
        else {
          m_logger = spdlog::get(m_logFile);
          if (!m_logger) {
            m_logger = spdlog::basic_logger_mt(m_logFile, m_logFile);
          }
        }
    }
    catch(std::out_of_range& e) {
      throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                       "State keywords required to generate sensor model missing: " + std::string(e.what()) + "\nUsing model string: " + stringState,
                       "UsgsAstroFrameSensorModel::replaceModelState");
    }
}


std::string UsgsAstroFrameSensorModel::constructStateFromIsd(const std::string& jsonIsd, csm::WarningList* warnings) {
    MESSAGE_LOG(this->m_logger, "Constructing state from isd");
    json isd = json::parse(jsonIsd);
    json state = {};

    csm::WarningList* parsingWarnings = new csm::WarningList;

    state["m_modelName"] = getSensorModelName(isd, parsingWarnings);
    state["m_imageIdentifier"] = getImageId(isd, parsingWarnings);
    state["m_sensorName"] = getSensorName(isd, parsingWarnings);
    state["m_platformName"] = getPlatformName(isd, parsingWarnings);

    state["m_startingDetectorSample"] = getDetectorStartingSample(isd, parsingWarnings);
    state["m_startingDetectorLine"] = getDetectorStartingLine(isd, parsingWarnings);
    state["m_detectorSampleSumming"] = getSampleSumming(isd, parsingWarnings);
    state["m_detectorLineSumming"] = getLineSumming(isd, parsingWarnings);

    // get focal length
    state["m_focalLength"] = getFocalLength(isd, parsingWarnings);
    state["m_focalLengthEpsilon"] = getFocalLengthEpsilon(isd);


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
    if (quaternion.size() != 4) {
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
    state["m_distortionType"] = getDistortionModel(isd, warnings);
    state["m_opticalDistCoeffs"] = getDistortionCoeffs(isd, warnings);

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
    for (int i = 0; i < NUM_PARAMETERS*NUM_PARAMETERS; i += NUM_PARAMETERS+1) {
      state["m_currentParameterCovariance"][i] = 1;
    }
    state["m_collectionIdentifier"] = "";

    // Get the optional logging file
    state["m_logFile"] = getLogFile(isd);


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
  MESSAGE_LOG(this->m_logger, "Accessing reference point x: {}, y: {}, z: {}",
                              m_referencePointXyz.x, m_referencePointXyz.y, m_referencePointXyz.z);
  return m_referencePointXyz;
}


void UsgsAstroFrameSensorModel::setReferencePoint(const csm::EcefCoord &groundPt) {
  MESSAGE_LOG(this->m_logger, "Setting reference point to {}, {}, {}",
                               groundPt.x, groundPt.y, groundPt.z);
  m_referencePointXyz = groundPt;
}


int UsgsAstroFrameSensorModel::getNumParameters() const {
  MESSAGE_LOG(this->m_logger, "Accessing num parameters: {}", NUM_PARAMETERS);
  return NUM_PARAMETERS;
}


std::string UsgsAstroFrameSensorModel::getParameterName(int index) const {
  MESSAGE_LOG(this->m_logger, "Setting parameter name to {}", index);
  return m_parameterName[index];
}


std::string UsgsAstroFrameSensorModel::getParameterUnits(int index) const {
  MESSAGE_LOG(this->m_logger, "Accessing parameter units for {}", index);
  if (index < 3) {
    return "m";
  }
  else {
    return "radians";
  }
}


bool UsgsAstroFrameSensorModel::hasShareableParameters() const {
  MESSAGE_LOG(this->m_logger, "Checking for shareable parameters");
  return false;
}


bool UsgsAstroFrameSensorModel::isParameterShareable(int index) const {
  MESSAGE_LOG(this->m_logger, "Checking is parameter: {} is shareable", index);
  return false;
}


csm::SharingCriteria UsgsAstroFrameSensorModel::getParameterSharingCriteria(int index) const {
   MESSAGE_LOG(this->m_logger, "Checking sharing criteria for parameter {}. "
               "Sharing is not supported, throwing exception", index);
   // Parameter sharing is not supported for this sensor,
   // all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index out of range.",
      "UsgsAstroLsSensorModel::getParameterSharingCriteria");
}


double UsgsAstroFrameSensorModel::getParameterValue(int index) const {
  MESSAGE_LOG(this->m_logger, "Accessing parameter value {} at index: {}", m_currentParameterValue[index], index);
  return m_currentParameterValue[index];

}


void UsgsAstroFrameSensorModel::setParameterValue(int index, double value) {
  MESSAGE_LOG(this->m_logger, "Setting parameter value: {} at index: {}", value, index);
  m_currentParameterValue[index] = value;
}


csm::param::Type UsgsAstroFrameSensorModel::getParameterType(int index) const {
  MESSAGE_LOG(this->m_logger, "Accessing parameter type: {} at index: {}", m_parameterType[index], index);
  return m_parameterType[index];
}


void UsgsAstroFrameSensorModel::setParameterType(int index, csm::param::Type pType) {
    MESSAGE_LOG(this->m_logger, "Setting parameter type: {} at index: {}", pType, index);
    m_parameterType[index] = pType;
}


double UsgsAstroFrameSensorModel::getParameterCovariance(int index1, int index2) const {
   int index = UsgsAstroFrameSensorModel::NUM_PARAMETERS * index1 + index2;
   MESSAGE_LOG(this->m_logger, "Accessing parameter covar: {} between index1: {} and index2: {}", m_currentParameterCovariance[index], index1, index2);
   return m_currentParameterCovariance[index];
}


void UsgsAstroFrameSensorModel::setParameterCovariance(int index1, int index2, double covariance) {
   MESSAGE_LOG(this->m_logger, "Setting parameter covar: {} between index1: {} and index2: {}",
                                covariance, index1, index2);
   int index = UsgsAstroFrameSensorModel::NUM_PARAMETERS * index1 + index2;
   m_currentParameterCovariance[index] = covariance;
}


int UsgsAstroFrameSensorModel::getNumGeometricCorrectionSwitches() const {
  MESSAGE_LOG(this->m_logger, "Accessing num geom correction switches");
  return 0;
}


std::string UsgsAstroFrameSensorModel::getGeometricCorrectionName(int index) const {
   MESSAGE_LOG(this->m_logger, "Accessing name of geometric correction switch {}. "
               "Geometric correction switches are not supported, throwing exception",
               index);
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::getGeometricCorrectionName");
}


void UsgsAstroFrameSensorModel::setGeometricCorrectionSwitch(int index,
                                                      bool value,
                                                      csm::param::Type pType) {
   MESSAGE_LOG(this->m_logger, "Setting geometric correction switch {} to {} "
               "with parameter type {}. "
               "Geometric correction switches are not supported, throwing exception",
               index, value, pType);
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::setGeometricCorrectionSwitch");
}


bool UsgsAstroFrameSensorModel::getGeometricCorrectionSwitch(int index) const {
   MESSAGE_LOG(this->m_logger, "Accessing value of geometric correction switch {}. "
               "Geometric correction switches are not supported, throwing exception",
               index);
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
   MESSAGE_LOG(this->m_logger, "Accessing cross covariance matrix");

   // No correlation between models.
   const std::vector<int>& indices = getParameterSetIndices(pSet);
   size_t num_rows = indices.size();
   const std::vector<int>& indices2 = comparisonModel.getParameterSetIndices(pSet);
   size_t num_cols = indices.size();

   return std::vector<double>(num_rows * num_cols, 0.0);
}


csm::Ellipsoid UsgsAstroFrameSensorModel::getEllipsoid() const {
   MESSAGE_LOG(this->m_logger, "Accessing ellipsoid radii {} {}",
               m_majorAxis, m_minorAxis);
   return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}


void UsgsAstroFrameSensorModel::setEllipsoid(const csm::Ellipsoid &ellipsoid) {
   MESSAGE_LOG(this->m_logger, "Setting ellipsoid radii {} {}",
               ellipsoid.getSemiMajorRadius(), ellipsoid.getSemiMinorRadius());
   m_majorAxis = ellipsoid.getSemiMajorRadius();
   m_minorAxis = ellipsoid.getSemiMinorRadius();
}


void UsgsAstroFrameSensorModel::calcRotationMatrix(
    double m[3][3]) const {
  MESSAGE_LOG(this->m_logger, "Calculating rotation matrix");
  // Trigonometric functions for rotation matrix
  double x = m_currentParameterValue[3];
  double y = m_currentParameterValue[4];
  double z = m_currentParameterValue[5];
  double w = m_currentParameterValue[6];

  double norm = sqrt(x * x + y * y + z * z + w * w);
  x /= norm;
  y /= norm;
  w /= norm;
  z /= norm;

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
  MESSAGE_LOG(this->m_logger, "Calculating rotation matrix with adjustments");
  // Trigonometric functions for rotation matrix
  double x = getValue(3, adjustments);
  double y = getValue(4, adjustments);
  double z = getValue(5, adjustments);
  double w = getValue(6, adjustments);

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
   MESSAGE_LOG(this->m_logger, "Calculating losEllipsoidIntersect with height: {},\n\
                                xc: {}, yc: {}, zc: {}\n\
                                xl: {}, yl: {}, zl: {}", height,
                                xc, yc, zc,
                                xl, yl, zl);
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

   MESSAGE_LOG(this->m_logger, "Calculated losEllipsoidIntersect at: {}, {}, {}",
                                x, y, z);
}


/***** Helper Functions *****/

double UsgsAstroFrameSensorModel::getValue(
   int index,
   const std::vector<double> &adjustments) const
{
   MESSAGE_LOG(this->m_logger, "Accessing value: {} at index: {}, with adjustments", m_currentParameterValue[index] + adjustments[index], index);
   return m_currentParameterValue[index] + adjustments[index];
}

std::shared_ptr<spdlog::logger> UsgsAstroFrameSensorModel::getLogger() {
  return m_logger;
}

void UsgsAstroFrameSensorModel::setLogger(std::shared_ptr<spdlog::logger> logger) {
  m_logger = logger;
}
