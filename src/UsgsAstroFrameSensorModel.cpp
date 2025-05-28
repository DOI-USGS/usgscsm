#include "UsgsAstroFrameSensorModel.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include <nlohmann/json.hpp>

#include <csm/Error.h>
#include <csm/Version.h>

#include "ale/Util.h"

#define MESSAGE_LOG(...)         \
  if (m_logger) {                \
    m_logger->log(__VA_ARGS__); \
  }

using json = nlohmann::json;

// Declaration of static variables
const std::string UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME =
    "USGS_ASTRO_FRAME_SENSOR_MODEL";
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

/**
 * @brief Initializes the sensor model with default values and logs the 
 * creation of the model.
 */
UsgsAstroFrameSensorModel::UsgsAstroFrameSensorModel() {
  MESSAGE_LOG(spdlog::level::debug, "Creating UsgsAstroFrameSensorModel");
  reset();
}

/**
 * @description Sets the model's properties and parameters to default values. 
 * This includes initializing the sensor's position, orientation, and other 
 * imaging properties, as well as setting up parameter covariances and 
 * defaults for imaging geometry.
 */
void UsgsAstroFrameSensorModel::reset() {
  MESSAGE_LOG(spdlog::level::debug, "Resetting UsgsAstroFrameSensorModel");
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
  m_currentParameterCovariance =
      std::vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
  for (int i = 0; i < NUM_PARAMETERS * NUM_PARAMETERS;
       i += NUM_PARAMETERS + 1) {
    m_currentParameterCovariance[i] = 1;
  }
  m_noAdjustments = std::vector<double>(NUM_PARAMETERS, 0.0);
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
  m_lineJitter.clear();
  m_sampleJitter.clear();
  m_lineTimes.clear();
  m_parameterType =
      std::vector<csm::param::Type>(NUM_PARAMETERS, csm::param::REAL);
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
 * @return Returns <line, sample> coordinate in the image corresponding to the
 * ground point without bundle adjustment correction.
 */
csm::ImageCoord UsgsAstroFrameSensorModel::groundToImage(
    const csm::EcefCoord &groundPt, double desiredPrecision,
    double *achievedPrecision, csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing groundToImage(No adjustments) for {}, {}, {}, with desired "
      "precision {}",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  csm::ImageCoord imagePt = groundToImage(
      groundPt, m_noAdjustments, desiredPrecision, achievedPrecision, warnings);
  MESSAGE_LOG(
      spdlog::level::info,
      "groundToImage result of ({}, {})",
      imagePt.line, imagePt.samp);
  return imagePt;
}

/**
 * @brief UsgsAstroFrameSensorModel::groundToImage
 * @param groundPt
 * @param adjustments
 * @param desired_precision
 * @param achieved_precision
 * @param warnings
 * @return Returns <line,sample> coordinate in the image corresponding to the
 * ground point. This function applies bundle adjustments to the final value.
 */
csm::ImageCoord UsgsAstroFrameSensorModel::groundToImage(
    const csm::EcefCoord &groundPt, const std::vector<double> &adjustments,
    double desired_precision, double *achieved_precision,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(
    spdlog::level::debug,
      "Computing groundToImage for {}, {}, {}, with desired precision {}",
      groundPt.x, groundPt.y, groundPt.z, desired_precision);

  double x = groundPt.x;
  double y = groundPt.y;
  double z = groundPt.z;

  double xo = x - getValue(0, adjustments);
  double yo = y - getValue(1, adjustments);
  double zo = z - getValue(2, adjustments);

  double f;
  f = m_focalLength;

  // Camera rotation matrix
  double m[3][3];
  calcRotationMatrix(m, adjustments);
  MESSAGE_LOG(
      spdlog::level::trace,
      "Calculated rotation matrix [{}, {}, {}], [{}, {}, {}], [{}, {}, {}]",
      m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1],
      m[2][2]);

  // Sensor position
  double undistortedx, undistortedy, denom;
  denom = m[0][2] * xo + m[1][2] * yo + m[2][2] * zo;
  undistortedx = (f * (m[0][0] * xo + m[1][0] * yo + m[2][0] * zo) / denom);
  undistortedy = (f * (m[0][1] * xo + m[1][1] * yo + m[2][1] * zo) / denom);

  MESSAGE_LOG(
      spdlog::level::trace,
      "Found undistortedX: {}, and undistortedY: {}",
      undistortedx, undistortedy);
  // Apply the distortion to the line/sample location and then convert back to
  // line/sample
  double distortedX, distortedY;
  applyDistortion(undistortedx, undistortedy, distortedX, distortedY,
                  m_opticalDistCoeffs, m_focalLength, m_distortionType);
  
  MESSAGE_LOG(
      spdlog::level::trace,
      "Found distortedX: {}, and distortedY: {}",
      distortedX, distortedY);

  // Convert distorted mm into line/sample
  double sample, line;
  computePixel(distortedX, distortedY, m_ccdCenter[1], m_ccdCenter[0],
               m_detectorSampleSumming, m_detectorLineSumming,
               m_startingDetectorSample, m_startingDetectorLine, &m_iTransS[0],
               &m_iTransL[0], line, sample);

  // Optionally apply rolling shutter jitter correction
  if (m_lineTimes.size()) {
    double jitteredLine, jitteredSample;
    addJitter(
        line, sample,
        desired_precision, 20,
        m_lineJitter, m_sampleJitter, m_lineTimes,
        jitteredLine, jitteredSample);
    line = jitteredLine;
    sample = jitteredSample;
  }

  MESSAGE_LOG(
      spdlog::level::debug,
      "Computed groundToImage for {}, {}, {} as line, sample: {}, {}",
      groundPt.x, groundPt.y, groundPt.z, line, sample);

  return csm::ImageCoord(line, sample);
}

/**
 * @brief Converts ground coordinates with covariance to image coordinates with 
 * covariance.
 * @description Computes the image coordinates corresponding to a given ground 
 * point (with covariance), attempting to achieve the specified precision for 
 * the conversion. This method is an extension of groundToImage that also 
 * considers the covariance of the input ground point but currently returns 
 * only the image coordinates without covariance due to partial implementation.
 * 
 * @param groundPt The ground point (with covariance) in ECEF coordinates to be 
 * converted to image coordinates.
 * @param desiredPrecision The desired precision for the computation of the 
 * image coordinates.
 * @param achievedPrecision A pointer to a double where the achieved precision 
 * of the computation will be stored. This value indicates how close the 
 * computed image point is to the true image point, given the model's accuracy.
 * @param warnings A pointer to a csm::WarningList where any warnings generated 
 * during the computation will be added. This can include warnings about the 
 * computation process, precision achievements, or any other relevant warnings
 * that do not necessarily indicate a failure.
 * 
 * @return A csm::ImageCoordCovar object representing the computed image 
 * coordinates. Due to the partial implementation, the covariance part of the 
 * returned object is not populated and should not be considered valid.
 */
csm::ImageCoordCovar UsgsAstroFrameSensorModel::groundToImage(
    const csm::EcefCoordCovar &groundPt, double desiredPrecision,
    double *achievedPrecision, csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing groundToImage(Covar) for {}, {}, {}, with desired precision "
      "{}",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  csm::EcefCoord gp;
  gp.x = groundPt.x;
  gp.y = groundPt.y;
  gp.z = groundPt.z;

  csm::ImageCoord ip =
      groundToImage(gp, desiredPrecision, achievedPrecision, warnings);
  csm::ImageCoordCovar result(ip.line, ip.samp);
  // This is a partial, incorrect implementation to test if SocetGXP needs
  // this method implemented in order to load the sensor.
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computed groundToImage(Covar) for {}, {}, {}, as line, sample: {}, {}",
      groundPt.x, groundPt.y, groundPt.z, ip.line, ip.samp);
  return result;
}

/**
 * @brief Transforms image coordinates to ground coordinates.
 * @description Converts a point from image coordinates (line, sample) to 
 * Earth-Centered, Earth-Fixed (ECEF) ground coordinates at a specified height 
 * above the ellipsoid model of the earth. This method accounts for sensor 
 * position, orientation, and distortion to accurately map the image point to 
 * ground coordinates. Additionally, it can apply corrections for rolling 
 * shutter effects.
 * 
 * @param imagePt The image coordinates (line and sample) to be converted to 
 * ground coordinates.
 * @param height The height above the ellipsoid at which to intersect the line 
 * of sight from the image point.
 * @param desiredPrecision The desired precision for the ground coordinate 
 * calculation.
 * @param achievedPrecision A pointer to a double where the achieved precision 
 * of the calculation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings 
 * that occur during the computation.
 * 
 * @return An ECEF coordinate corresponding to the provided image point.
 */
csm::EcefCoord UsgsAstroFrameSensorModel::imageToGround(
    const csm::ImageCoord &imagePt, double height, double desiredPrecision,
    double *achievedPrecision, csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing imageToGround for {}, {}, {}, with desired precision {}",
      imagePt.line, imagePt.samp, height, desiredPrecision);

  double sample = imagePt.samp;
  double line = imagePt.line;

  // Here is where we should be able to apply an adjustment to opk
  double m[3][3];
  calcRotationMatrix(m);
  MESSAGE_LOG(
      spdlog::level::trace,
      "Calculated rotation matrix [{}, {}, {}], [{}, {}, {}], [{}, {}, {}]",
      m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1],
      m[2][2]);

  // Optionally apply rolling shutter jitter correction
  if (m_lineTimes.size()) {
    double dejitteredLine, dejitteredSample;
    removeJitter(line, sample, m_lineJitter, m_sampleJitter, m_lineTimes, dejitteredLine, dejitteredSample);
    line = dejitteredLine;
    sample = dejitteredSample;
  }

  // Convert from the pixel space into the metric space
  double x_camera, y_camera;
  computeDistortedFocalPlaneCoordinates(
      line, sample, m_ccdCenter[1], m_ccdCenter[0], m_detectorSampleSumming,
      m_detectorLineSumming, m_startingDetectorSample, m_startingDetectorLine,
      &m_iTransS[0], &m_iTransL[0], x_camera, y_camera);

  // Apply the distortion model (remove distortion)
  double undistortedX, undistortedY;
  removeDistortion(x_camera, y_camera, undistortedX, undistortedY,
                   m_opticalDistCoeffs, m_focalLength, m_distortionType);
  
  MESSAGE_LOG(
      spdlog::level::trace,
      "Found undistortedX: {}, and undistortedY: {}",
      undistortedX, undistortedY);

  // Now back from distorted mm to pixels
  double xl, yl, zl;
  xl = m[0][0] * undistortedX + m[0][1] * undistortedY -
       m[0][2] * -m_focalLength;
  yl = m[1][0] * undistortedX + m[1][1] * undistortedY -
       m[1][2] * -m_focalLength;
  zl = m[2][0] * undistortedX + m[2][1] * undistortedY -
       m[2][2] * -m_focalLength;
  MESSAGE_LOG(spdlog::level::trace, "Compute xl, yl, zl as {}, {}, {}", xl, yl, zl);

  double xc, yc, zc;
  xc = m_currentParameterValue[0];
  yc = m_currentParameterValue[1];
  zc = m_currentParameterValue[2];
  MESSAGE_LOG(
      spdlog::level::trace,
      "Set xc, yc, zc to {}, {}, {}", m_currentParameterValue[0],
      m_currentParameterValue[1], m_currentParameterValue[2]);

  // Intersect with some height about the ellipsoid.
  double x, y, z;
  losEllipsoidIntersect(height, xc, yc, zc, xl, yl, zl, x, y, z, warnings);

  MESSAGE_LOG(
      spdlog::level::info,
      "Resulting EcefCoordinate: {}, {}, {}",
      x, y, z);

  return csm::EcefCoord(x, y, z);
}

/**
 * @brief Transforms image coordinates with covariance to ground coordinates 
 * with covariance.
 * @description This overload of imageToGround is intended to convert image 
 * coordinates with associated covariance to ground coordinates with 
 * covariance. Currently, this is an incomplete implementation intended to test 
 * software integration capabilities.
 * 
 * @param imagePt The image coordinates with covariance to be converted to 
 * ground coordinates with covariance.
 * @param height The height above the ellipsoid for the ground point.
 * @param heightVariance The variance of the height, contributing to the 
 * covariance of the ground point.
 * @param desiredPrecision The desired precision for the ground coordinate 
 * calculation.
 * @param achievedPrecision A pointer to a double where the achieved 
 * precision of the calculation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings 
 * that occur during the computation.
 * 
 * @return A placeholder ECEF coordinate with covariance indicating an 
 * incomplete implementation.
 */
csm::EcefCoordCovar UsgsAstroFrameSensorModel::imageToGround(
    const csm::ImageCoordCovar &imagePt, double height, double heightVariance,
    double desiredPrecision, double *achievedPrecision,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing imageToGround(Covar) for {}, {}, {}, with desired precision "
      "{}",
      imagePt.line, imagePt.samp, height, desiredPrecision);
  // This is an incomplete implementation to see if SocetGXP needs this method
  // implemented.

  MESSAGE_LOG(
      spdlog::level::trace,
      "This is an incomplete implementation to see if SocetGXP needs this "
      "method implemented");

  csm::EcefCoordCovar result;
  return result;
}

/**
 * @brief Computes the proximate imaging locus for a given image coordinate 
 * relative to a specified ground point.
 * @description This function calculates the locus (a line in 3D space) that 
 * represents the path of the sensor's imaging process for a specific image 
 * point, adjusted to be as close as possible to a given ground point. The 
 * method first computes the remote imaging locus for the image point to 
 * determine the imaging direction. It then adjusts the locus to minimize the 
 * distance to the specified ground point, effectively creating a proximate 
 * imaging locus that represents the imaging geometry relative to that ground 
 * point.
 * 
 * @param imagePt The image coordinate (line and sample) for which to compute 
 * the imaging locus.
 * @param groundPt The ECEF ground coordinate that the computed locus should 
 * be proximate to.
 * @param desiredPrecision The desired precision for the computation of the 
 * proximate imaging locus.
 * @param achievedPrecision A pointer to a double where the achieved precision 
 * of the computation will be stored. This value indicates how close the 
 * computed locus is to the theoretical perfect locus given the sensor model's 
 * accuracy.
 * @param warnings A pointer to a csm::WarningList for logging any warnings 
 * that occur during the computation. This can include warnings about the 
 * computation process, precision achievements, or any other relevant warnings 
 * that do not necessarily indicate a failure.
 * 
 * @return A csm::EcefLocus object representing the computed proximate imaging 
 * locus. The returned locus includes both a point on the locus closest to the 
 * specified ground point and the direction vector of the locus.
 */
csm::EcefLocus UsgsAstroFrameSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord &imagePt, const csm::EcefCoord &groundPt,
    double desiredPrecision, double *achievedPrecision,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing imageToProximateImagingLocus(No ground) for point {}, {}, "
      "with desired precision {}",
      imagePt.line, imagePt.samp, desiredPrecision);

  // The locus is a straight line so we can just use the remote locus to
  // get the direction
  csm::EcefLocus remoteLocus = imageToRemoteImagingLocus(
      imagePt, desiredPrecision, achievedPrecision, warnings);

  // Find the point on the locus closest to the input ground point
  // The direction vector is already normalized so we can skip a division
  csm::EcefVector positionToGround(
      groundPt.x - remoteLocus.point.x,
      groundPt.y - remoteLocus.point.y,
      groundPt.z - remoteLocus.point.z);
  csm::EcefVector positionToClosest =
      dot(remoteLocus.direction, positionToGround) * remoteLocus.direction;

  csm::EcefLocus locus(
      remoteLocus.point.x + positionToClosest.x,
      remoteLocus.point.y + positionToClosest.y,
      remoteLocus.point.z + positionToClosest.z,
      remoteLocus.direction.x,
      remoteLocus.direction.y,
      remoteLocus.direction.z);

    MESSAGE_LOG(
    spdlog::level::info,
    "imageToProximateImagingLocus result of point ({}, {}, {}) "
    "direction ({}, {}, {}).",
    locus.point.x, locus.point.y, locus.point.z,
    locus.direction.x, locus.direction.y, locus.direction.z);

  return locus;
}

/**
 * @brief Computes the imaging locus for a given image point in remote sensing.
 * @description This function calculates the line of sight (locus) from the 
 * sensor model to a point in the image, extending infinitely into space. It 
 * determine the direction vector from the sensor's position through the 
 * specifies image point, effectively modeling how the sensor is imaging that 
 * point from a distance.
 * 
 * @param imagePt The image coordinate (line and sample) for which to compute
 * the imaging locus.
 * @param desiredPrecision The desired precision for the calculation, which
 * influences the accuracy of the computed direction vector.
 * @param achievedPrecision A pointer to a double where the achieved precision
 * of the computation will be stored. This parameter is intended to provide 
 * feedback on the computation's accuracy but is currently set to 0.0 as the 
 * calculation is deterministic.
 * @param warnings A pointer to a csm::WarningList for logging any warnings 
 * that occur during the computation. This might include issues with distortion 
 * correction or other computational warnings.
 * 
 * @return A csm::EcefLocus object representing the computed remote imaging 
 * locus. The locus includes a point (the sensor's position) and a direction 
 * vector indicating the line of sight from the sensor through the image point.
 */
csm::EcefLocus UsgsAstroFrameSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord &imagePt, double desiredPrecision,
    double *achievedPrecision, csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing imageToRemoteImagingLocus for {}, {} with desired "
      "precision {}",
      imagePt.line, imagePt.samp, desiredPrecision);
  // Find the line,sample on the focal plane (mm)
  double focalPlaneX, focalPlaneY;
  computeDistortedFocalPlaneCoordinates(
      imagePt.line, imagePt.samp, m_ccdCenter[1], m_ccdCenter[0],
      m_detectorSampleSumming, m_detectorLineSumming, m_startingDetectorSample,
      m_startingDetectorLine, &m_iTransS[0], &m_iTransL[0], focalPlaneX,
      focalPlaneY);

  MESSAGE_LOG(
      spdlog::level::trace,
      "Found focalPlaneX: {}, and focalPlaneY: {}",
      focalPlaneX, focalPlaneY);
      
  // Distort
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  removeDistortion(focalPlaneX, focalPlaneY, undistortedFocalPlaneX,
                   undistortedFocalPlaneY, m_opticalDistCoeffs,
                   m_focalLength, m_distortionType);

  MESSAGE_LOG(
      spdlog::level::trace,
      "Found undistortedFocalPlaneX: {}, and undistortedFocalPlaneY: {}",
      undistortedFocalPlaneX, undistortedFocalPlaneY);

  // Get rotation matrix and transform to a body-fixed frame
  double m[3][3];
  calcRotationMatrix(m);
  MESSAGE_LOG(
      spdlog::level::trace,
      "Calculated rotation matrix [{}, {}, {}], [{}, {}, {}], [{}, {}, {}]",
      m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1],
      m[2][2]);
  std::vector<double> lookC{undistortedFocalPlaneX, undistortedFocalPlaneY,
                            m_focalLength};
  std::vector<double> lookB{
      m[0][0] * lookC[0] + m[0][1] * lookC[1] + m[0][2] * lookC[2],
      m[1][0] * lookC[0] + m[1][1] * lookC[1] + m[1][2] * lookC[2],
      m[2][0] * lookC[0] + m[2][1] * lookC[1] + m[2][2] * lookC[2]};

  // Get unit vector
  double mag =
      sqrt(lookB[0] * lookB[0] + lookB[1] * lookB[1] + lookB[2] * lookB[2]);
  std::vector<double> lookBUnit{lookB[0] / mag, lookB[1] / mag, lookB[2] / mag};

  if (achievedPrecision) {
    *achievedPrecision = 0.0;
  }

  return csm::EcefLocus(m_currentParameterValue[0], m_currentParameterValue[1],
                        m_currentParameterValue[2], lookBUnit[0], lookBUnit[1],
                        lookBUnit[2]);
}

/**
 * @brief Retrieves the starting coordinates of the image.
 * @description Provides the starting line and sample coordinates of the image
 * as represented by the sensor model. These coordinates indicate the origin
 * (top-left corner) of the image in the sensor's coordinate system, essential
 * for image processing tasks requiring knowledge of the image's initial
 * positioning within the sensor's field of view.
 * 
 * @return csm::ImageCoord containing the starting line and sample of the image.
 * The 'line' attribute represents the first line (vertical coordinate), and
 * the 'samp' attribute represents the first sample (horizontal coordinate).
 */
csm::ImageCoord UsgsAstroFrameSensorModel::getImageStart() const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing Image Start line: {}, sample: {}",
      m_startingDetectorLine, m_startingDetectorSample);
  csm::ImageCoord start;
  start.samp = m_startingDetectorSample;
  start.line = m_startingDetectorLine;
  return start;
}

/**
 * @brief Retrieves the size of the image.
 * @description Returns the total number of lines and samples in the image,
 * providing the image's dimensions. This information is crucial for various
 * image processing tasks such as cropping, scaling, and geometric corrections,
 * and for allocating appropriate memory for image storage and manipulation.
 * 
 * @return csm::ImageVector containing the size of the image. The 'line'
 * attribute represents the total number of lines (vertical extent), and the
 * 'samp' attribute represents the total number of samples (horizontal extent).
 */
csm::ImageVector UsgsAstroFrameSensorModel::getImageSize() const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing Image Size line: {}, sample: {}",
      m_nLines, m_nSamples);
  csm::ImageVector size;
  size.line = m_nLines;
  size.samp = m_nSamples;
  return size;
}

/**
 * @brief Retrieves the valid image coordinate range of the sensor model.
 * @description Determines the minimum and maximum image coordinates that can
 * be processed by the sensor model. This range is essential for understanding
 * the bounds within which image coordinates are considered valid and can be
 * accurately mapped to ground coordinates.
 * 
 * @return A pair of csm::ImageCoord objects. The first element represents the
 * minimum (starting) image coordinates (line and sample), and the second
 * element represents the maximum image coordinates based on the number of
 * lines and samples in the sensor model.
 */
std::pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroFrameSensorModel::getValidImageRange() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing Image Range");
  csm::ImageCoord min_pt(m_startingDetectorLine, m_startingDetectorSample);
  csm::ImageCoord max_pt(m_nLines, m_nSamples);
  MESSAGE_LOG(spdlog::level::debug, "Valid image range: min {}, {} max: {}, {}", min_pt.samp,
              min_pt.line, max_pt.samp, max_pt.line)
  return std::pair<csm::ImageCoord, csm::ImageCoord>(min_pt, max_pt);
}

/**
 * @brief Retrieves the valid height range for the sensor model.
 * @description Provides the minimum and maximum elevation values that can be
 * accurately processed by the sensor model. This range helps in constraining
 * ground point calculations to heights where the model is valid, improving
 * the accuracy of ground-to-image and image-to-ground transformations.
 * 
 * @return A pair of doubles representing the valid height range. The first
 * element is the minimum elevation, and the second element is the maximum
 * elevation, both in meters above the ellipsoid.
 */
std::pair<double, double> UsgsAstroFrameSensorModel::getValidHeightRange()
    const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing Image Height min: {}, max: {}",
      m_minElevation, m_maxElevation);
  return std::pair<double, double>(m_minElevation, m_maxElevation);
}

/**
 * @brief Calculates the illumination direction for a given ground point.
 * @description Computes the vector representing the direction of illumination
 * at a specific ground point, essential for shading, shadowing, and photometric
 * analysis. The illumination direction is calculated as the vector from the sun
 * to the ground point in the ECEF coordinate system.
 * 
 * @param groundPt The ECEF coordinates of the ground point for which to compute
 * the illumination direction.
 * 
 * @return A csm::EcefVector representing the direction of illumination at the
 * given ground point. The vector points from the sun towards the ground point,
 * indicating the direction from which the ground point is illuminated.
 */
csm::EcefVector UsgsAstroFrameSensorModel::getIlluminationDirection(
    const csm::EcefCoord &groundPt) const {
  // ground (body-fixed) - sun (body-fixed) gives us the illumination direction.
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing illumination direction for ground point {}, {}, {}",
      groundPt.x, groundPt.y, groundPt.z);
  return csm::EcefVector{groundPt.x - m_sunPosition[0],
                         groundPt.y - m_sunPosition[1],
                         groundPt.z - m_sunPosition[2]};
}

/**
 * @brief Retrieves the acquisition time of an image point.
 * @description Determines the image acquisition time for a given image
 * coordinate. For this sensor model, the entire image is acquired at once,
 * so the image time for all pixels is the same and set to a constant value.
 * 
 * @param imagePt The image coordinate for which to retrieve the acquisition time.
 * 
 * @return The acquisition time of the image point as a double. This model
 * returns 0.0, indicating a single acquisition time for the entire image.
 */
double UsgsAstroFrameSensorModel::getImageTime(
    const csm::ImageCoord &imagePt) const {
  MESSAGE_LOG(
    spdlog::level::debug,
    "Accessing image time for image point {}, {}",
    imagePt.line, imagePt.samp);
  // The entire image is aquired at once so image time for all pixels is the
  // reference time
  return 0.0;
}

/**
 * @brief Retrieves the sensor's position at the time of imaging for a given image point.
 * @description Calculates the position of the sensor in ECEF coordinates at the
 * time the specified image point was acquired. For this model, since the entire
 * image is acquired simultaneously, the sensor position is constant for all
 * image points and determined by the current parameter values.
 * 
 * @param imagePt The image coordinate for which to determine the sensor's position.
 * 
 * @return The sensor's position in ECEF coordinates as a csm::EcefCoord object.
 * 
 * @throws csm::Error If the given image coordinate is out of the valid range,
 * an error is thrown indicating that the image coordinate is out of bounds.
 */
csm::EcefCoord UsgsAstroFrameSensorModel::getSensorPosition(
    const csm::ImageCoord &imagePt) const {
  MESSAGE_LOG(
    spdlog::level::debug,
    "Accessing sensor position for image point {}, {}",
    imagePt.line, imagePt.samp);
    
  csm::EcefCoord sensorPosition;
  sensorPosition.x = m_currentParameterValue[0];
  sensorPosition.y = m_currentParameterValue[1];
  sensorPosition.z = m_currentParameterValue[2];

  return sensorPosition;
}

/**
 * @brief Retrieves the sensor's position at a specific time.
 * @description Calculates the position of the sensor in ECEF coordinates at the
 * specified time. For this model, the valid image time is a constant (0.0),
 * and the sensor's position is defined by the current parameter values.
 * 
 * @param time The time at which to determine the sensor's position. For this
 * model, the only valid time is 0.0.
 * 
 * @return The sensor's position in ECEF coordinates as a csm::EcefCoord object.
 * 
 * @throws csm::Error If the provided time does not match the valid image time
 * (0.0), an error is thrown indicating that the time is out of bounds.
 */
csm::EcefCoord UsgsAstroFrameSensorModel::getSensorPosition(double time) const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing sensor position for time {}", time);
  if (time == 0.0) {
    csm::EcefCoord sensorPosition;
    sensorPosition.x = m_currentParameterValue[0];
    sensorPosition.y = m_currentParameterValue[1];
    sensorPosition.z = m_currentParameterValue[2];

    return sensorPosition;
  } else {
    MESSAGE_LOG(
        spdlog::level::err,
        "ERROR: UsgsAstroFrameSensorModel::getSensorPosition: Valid image time "
        "is 0.0");
    std::string aMessage = "Valid image time is 0.0";
    throw csm::Error(csm::Error::BOUNDS, aMessage,
                     "UsgsAstroFrameSensorModel::getSensorPosition");
  }
}

/**
 * @brief Retrieves the sensor's velocity for a given image point.
 * @description Calculates the velocity of the sensor in ECEF coordinates at the
 * time the specified image point was acquired. For this model, the velocity is
 * constant for all image points and is directly obtained from the ISD.
 * 
 * @param imagePt The image coordinate for which to determine the sensor's velocity.
 * 
 * @return The sensor's velocity in ECEF coordinates as a csm::EcefVector object.
 * 
 * @throws csm::Error If the given image coordinate is out of the valid range,
 * an error is thrown indicating that the image coordinate is out of bounds.
 */
csm::EcefVector UsgsAstroFrameSensorModel::getSensorVelocity(
    const csm::ImageCoord &imagePt) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing sensor velocity for image point {}, {}",
      imagePt.line, imagePt.samp);
  // Make sure the passed coordinate is with the image dimensions.
  if (imagePt.samp < 0.0 || imagePt.samp > m_nSamples || imagePt.line < 0.0 ||
      imagePt.line > m_nLines) {
    MESSAGE_LOG(spdlog::level::err, "ERROR: Image coordinate out of bounds.")
    throw csm::Error(csm::Error::BOUNDS, "Image coordinate out of bounds.",
                     "UsgsAstroFrameSensorModel::getSensorVelocity");
  }

  // Since this is a frame, just return the sensor velocity the ISD gave us.
  return csm::EcefVector{m_spacecraftVelocity[0], m_spacecraftVelocity[1],
                         m_spacecraftVelocity[2]};
}

/**
 * @brief Retrieves the sensor's velocity at a specific time.
 * @description Calculates the velocity of the sensor in ECEF coordinates at the
 * specified time. For this model, the only valid image time is 0.0, and the
 * sensor's velocity is defined by the spacecraft velocity provided in the ISD.
 * 
 * @param time The time at which to determine the sensor's velocity. For this
 * model, the only valid time is 0.0.
 * 
 * @return The sensor's velocity in ECEF coordinates as a csm::EcefVector object.
 * 
 * @throws csm::Error If the provided time does not match the valid image time
 * (0.0), an error is thrown indicating that the time is out of bounds.
 */
csm::EcefVector UsgsAstroFrameSensorModel::getSensorVelocity(
    double time) const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing sensor position for time {}", time);
  if (time == 0.0) {
    return csm::EcefVector{m_spacecraftVelocity[0], m_spacecraftVelocity[1],
                           m_spacecraftVelocity[2]};
  } else {
    std::string aMessage = "Valid image time is 0.0";
    throw csm::Error(csm::Error::BOUNDS, aMessage,
                     "UsgsAstroFrameSensorModel::getSensorVelocity");
  }
}

/**
 * @brief Computes the sensor model partial derivatives with respect to model parameters.
 * @description Calculates the partial derivatives of the image coordinates with
 * respect to a specific model parameter index, given a ground point. This
 * calculation is essential for optimization and analysis tasks where understanding
 * the sensitivity of image coordinates to model parameters is necessary.
 * 
 * @param index The index of the model parameter for which to compute partials.
 * @param groundPt The ground point in ECEF coordinates for which to compute
 * the sensor model partials.
 * @param desiredPrecision The desired precision of the partial derivatives
 * calculation.
 * @param achievedPrecision A pointer to a double where the achieved precision
 * of the computation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that
 * occur during the computation of sensor partials.
 * 
 * @return A csm::RasterGM::SensorPartials object representing the partial
 * derivatives of the image coordinate with respect to the specified model parameter.
 */
csm::RasterGM::SensorPartials UsgsAstroFrameSensorModel::computeSensorPartials(
    int index, const csm::EcefCoord &groundPt, double desiredPrecision,
    double *achievedPrecision, csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing sensor partials image point from ground point {}, {}, {} \
                                 and desiredPrecision: {}",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  csm::ImageCoord img_pt =
      groundToImage(groundPt, desiredPrecision, achievedPrecision);

  return computeSensorPartials(index, img_pt, groundPt, desiredPrecision,
                               achievedPrecision);
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
 * Research:  We should investigate using a central difference scheme to
 * approximate the partials.  It is more accurate, but it might be costlier
 * calculation-wise.
 *
 */
csm::RasterGM::SensorPartials UsgsAstroFrameSensorModel::computeSensorPartials(
    int index, const csm::ImageCoord &imagePt, const csm::EcefCoord &groundPt,
    double desiredPrecision, double *achievedPrecision,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing sensor partials for ground point {}, {}, {}\
                               with point: {}, {}, index: {}, and desiredPrecision: {}",
      groundPt.x, groundPt.y, groundPt.z, imagePt.line, imagePt.samp, index,
      desiredPrecision);
  double delta = 1.0;
  // The rotation parameters will usually be small (<1),
  // so a delta of 1.0 is too small.
  // Rotation parameters are indices 3-6
  if (index > 2 && index < 7) {
    delta = 0.01;
  }
  // Update the parameter
  std::vector<double> adj(NUM_PARAMETERS, 0.0);
  adj[index] = delta;

  csm::ImageCoord imagePt1 =
      groundToImage(groundPt, adj, desiredPrecision, achievedPrecision);

  csm::RasterGM::SensorPartials partials;

  partials.first = (imagePt1.line - imagePt.line) / delta;
  partials.second = (imagePt1.samp - imagePt.samp) / delta;

  return partials;
}

/**
 * @brief Computes the sensor model partial derivatives with respect to all model parameters.
 * @description Calculates the partial derivatives of the image coordinates with
 * respect to all model parameters for a given image and ground point pair.
 * 
 * @param imagePt The image coordinate for which to compute the sensor partials.
 * @param groundPt The ground point in ECEF coordinates corresponding to the imagePt.
 * @param pset The parameter set indicating which model parameters to consider
 * for the partial derivatives calculation.
 * @param desiredPrecision The desired precision of the partial derivatives calculation.
 * @param achievedPrecision A pointer to a double where the achieved precision of
 * the computation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that
 * occur during the computation of sensor partials.
 * 
 * @return A vector of csm::RasterGM::SensorPartials objects representing the partial
 * derivatives of the image coordinate with respect to each model parameter in the set.
 */
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroFrameSensorModel::computeAllSensorPartials(
    const csm::ImageCoord &imagePt, const csm::EcefCoord &groundPt,
    csm::param::Set pset, double desiredPrecision, double *achievedPrecision,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing all sensor partials for ground point {}, {}, {}\
                               with point: {}, {}, pset: {}, and desiredPrecision: {}",
      groundPt.x, groundPt.y, groundPt.z, imagePt.line, imagePt.samp, pset,
      desiredPrecision);

  return RasterGM::computeAllSensorPartials(imagePt, groundPt, pset, desiredPrecision,
                                            achievedPrecision, warnings);
}

/**
 * @brief Computes the sensor model partial derivatives with respect to all model parameters for a ground point.
 * @description Determines the partial derivatives of the image coordinates
 * obtained by projecting a ground point back to the image, with respect to all
 * model parameters. This method first converts the ground point to an image
 * coordinate and then computes the partials for that image point.
 * 
 * @param groundPt The ground point in ECEF coordinates for which to compute
 * the sensor model partials.
 * @param pset The parameter set indicating which model parameters to consider
 * for the partial derivatives calculation.
 * @param desiredPrecision The desired precision of the partial derivatives calculation.
 * @param achievedPrecision A pointer to a double where the achieved precision of
 * the computation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that
 * occur during the computation of sensor partials.
 * 
 * @return A vector of csm::RasterGM::SensorPartials objects representing the partial
 * derivatives of the computed image coordinate with respect to each model parameter in the set.
 */
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroFrameSensorModel::computeAllSensorPartials(
    const csm::EcefCoord &groundPt, csm::param::Set pset,
    double desiredPrecision, double *achievedPrecision,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing all sensor partials image point from ground point {}, {}, {} \
                               and desiredPrecision: {}",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision);
  csm::ImageCoord imagePt =
      groundToImage(groundPt, desiredPrecision, achievedPrecision, warnings);
  return computeAllSensorPartials(imagePt, groundPt, pset, desiredPrecision,
                                  achievedPrecision, warnings);
}

/**
 * @brief Computes the partial derivatives of image coordinates with respect to ground coordinates.
 * @description Calculates how changes in the ground point's ECEF coordinates (X, Y, Z)
 * affect the corresponding image coordinates (line and sample).
 * 
 * @param groundPt The ground point in ECEF coordinates for which to compute the
 * partial derivatives of the corresponding image coordinates.
 * 
 * @return A vector of six doubles representing the partial derivatives of image
 * coordinates (line and sample) with respect to ground coordinates (X, Y, Z). The
 * order is [dLine/dX, dLine/dY, dLine/dZ, dSample/dX, dSample/dY, dSample/dZ].
 */
std::vector<double> UsgsAstroFrameSensorModel::computeGroundPartials(
    const csm::EcefCoord &groundPt) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing ground partials for ground point {}, {}, {}",
      groundPt.x, groundPt.y, groundPt.z);
  // Partial of line, sample wrt X, Y, Z
  double x = groundPt.x;
  double y = groundPt.y;
  double z = groundPt.z;

  csm::ImageCoord ipB = groundToImage(groundPt);
  csm::EcefCoord nextPoint =
      imageToGround(csm::ImageCoord(ipB.line + 1, ipB.samp + 1));
  double dx = nextPoint.x - x;
  double dy = nextPoint.y - y;
  double dz = nextPoint.z - z;
  double pixelGroundSize = sqrt((dx * dx + dy * dy + dz * dz) / 2.0);

  // If the ground size is too small, try the opposite direction
  if (pixelGroundSize < 1e-10) {
    nextPoint = imageToGround(csm::ImageCoord(ipB.line - 1, ipB.samp - 1));
    dx = nextPoint.x - x;
    dy = nextPoint.y - y;
    dz = nextPoint.z - z;
    pixelGroundSize = sqrt((dx * dx + dy * dy + dz * dz) / 2.0);
  }

  csm::ImageCoord ipX =
      groundToImage(csm::EcefCoord(x + pixelGroundSize, y, z));
  csm::ImageCoord ipY =
      groundToImage(csm::EcefCoord(x, y + pixelGroundSize, z));
  csm::ImageCoord ipZ =
      groundToImage(csm::EcefCoord(x, y, z + pixelGroundSize));

  std::vector<double> partials(6, 0.0);
  partials[0] = (ipX.line - ipB.line) / pixelGroundSize;
  partials[3] = (ipX.samp - ipB.samp) / pixelGroundSize;
  partials[1] = (ipY.line - ipB.line) / pixelGroundSize;
  partials[4] = (ipY.samp - ipB.samp) / pixelGroundSize;
  partials[2] = (ipZ.line - ipB.line) / pixelGroundSize;
  partials[5] = (ipZ.samp - ipB.samp) / pixelGroundSize;

  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing ground partials results:\nLine: {}, {}, {}\nSample: {}, {}, "
      "{}",
      partials[0], partials[1], partials[2], partials[3], partials[4],
      partials[5]);

  return partials;
}

/**
 * @brief Retrieves the correlation model used by the sensor model.
 * @description Returns a reference to the correlation model that describes how
 * image points are correlated with each other in the sensor model. For this
 * implementation, there is no modeled correlation between image points.
 * 
 * @return A constant reference to the sensor model's CorrelationModel object,
 * which in this case indicates no correlation.
 */
const csm::CorrelationModel &UsgsAstroFrameSensorModel::getCorrelationModel()
    const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing correlation model");
  return _no_corr_model;
}

/**
 * @brief Retrieves the unmodeled cross covariance between two image points.
 * @description Computes the cross covariance due to unmodeled errors between
 * two points in the image space. For this implementation, it is assumed that
 * there are no unmodeled errors affecting the covariance between image points.
 * 
 * @param pt1 The first image coordinate.
 * @param pt2 The second image coordinate.
 * 
 * @return A vector of doubles representing the unmodeled cross covariance
 * matrix between the two points, flattened into a vector. Since there are no
 * unmodeled errors, this vector contains only zeros.
 */
std::vector<double> UsgsAstroFrameSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord &pt1, const csm::ImageCoord &pt2) const {
  // No unmodeled error
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing unmodeled cross covar with \
                                point1: {}, {} and point2: {}, {}",
      pt1.line, pt1.samp, pt2.line, pt2.samp);
  return std::vector<double>(4, 0.0);
}

/**
 * @brief Retrieves the version of the sensor model.
 * @description Returns the version of this implementation of the sensor model.
 * 
 * @return A csm::Version object representing the version of the sensor model.
 */
csm::Version UsgsAstroFrameSensorModel::getVersion() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing CSM version");
  return csm::Version(0, 1, 0);
}

/**
 * @brief Retrieves the name of the sensor model.
 * @description Returns the name of the sensor model, which uniquely identifies
 * the model within the CSM framework.
 * 
 * @return A string containing the name of the sensor model.
 */
std::string UsgsAstroFrameSensorModel::getModelName() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing CSM name {}", _SENSOR_MODEL_NAME);
  return _SENSOR_MODEL_NAME;
}

/**
 * @brief Retrieves the pedigree of the sensor model.
 * @description Returns a string that provides information about the source or
 * lineage of the sensor model, which can be useful for tracking model provenance.
 * 
 * @return A string containing the pedigree of the sensor model.
 */
std::string UsgsAstroFrameSensorModel::getPedigree() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing CSM pedigree");
  return "USGS_FRAMER";
}

/**
 * @brief Retrieves the identifier of the image associated with the sensor model.
 * @description Returns the unique identifier of the image that this sensor model
 * represents.
 * 
 * @return A string containing the image identifier.
 */
std::string UsgsAstroFrameSensorModel::getImageIdentifier() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing image ID {}", m_imageIdentifier);
  return m_imageIdentifier;
}

/**
 * @brief Sets the identifier of the image associated with the sensor model.
 * @description Assigns a new image identifier to the sensor model.
 * 
 * @param imageId The new image identifier to be set.
 * @param warnings A pointer to a csm::WarningList for logging any warnings related
 * to setting the image identifier.
 */
void UsgsAstroFrameSensorModel::setImageIdentifier(const std::string &imageId,
                                                   csm::WarningList *warnings) {
  MESSAGE_LOG(spdlog::level::debug, "Setting image ID to {}", imageId);
  m_imageIdentifier = imageId;
}

/**
 * @brief Retrieves the sensor's identifier.
 * @description Returns the identifier of the sensor that acquired the image.
 * 
 * @return A string containing the sensor's identifier.
 */
std::string UsgsAstroFrameSensorModel::getSensorIdentifier() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing sensor ID: {}", m_sensorName);
  return m_sensorName;
}

/**
 * @brief Retrieves the platform's identifier.
 * @description Returns the identifier of the platform or spacecraft that hosts
 * the sensor.
 * 
 * @return A string containing the platform's identifier.
 */
std::string UsgsAstroFrameSensorModel::getPlatformIdentifier() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing platform ID: {}", m_platformName);
  return m_platformName;
}

/**
 * @brief Retrieves the collection's identifier.
 * @description Returns the identifier of the image collection to which the image
 * belongs.
 * 
 * @return A string containing the collection's identifier.
 */
std::string UsgsAstroFrameSensorModel::getCollectionIdentifier() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing collection ID: {}", m_collectionIdentifier);
  return m_collectionIdentifier;
}

/**
 * @brief Retrieves the trajectory's identifier.
 * @description Returns the identifier of the trajectory followed by the platform
 * or sensor. For this model, the trajectory identifier is not used and thus returns
 * an empty string.
 * 
 * @return A string containing the trajectory's identifier, which is empty for this model.
 */
std::string UsgsAstroFrameSensorModel::getTrajectoryIdentifier() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing trajectory ID");
  return "";
}

/**
 * @brief Retrieves the sensor's type.
 * @description Returns the type of sensor, such as electro-optical, radar, etc.,
 * that acquired the image.
 * 
 * @return A string representing the sensor type. For this model, returns CSM_SENSOR_TYPE_EO
 * indicating an electro-optical sensor.
 */
std::string UsgsAstroFrameSensorModel::getSensorType() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing sensor type");
  return CSM_SENSOR_TYPE_EO;
}

/**
 * @brief Retrieves the sensor's mode of operation.
 * @description Returns the mode of operation of the sensor at the time the image
 * was acquired, such as frame, pushbroom, etc.
 * 
 * @return A string representing the sensor mode. For this model, returns CSM_SENSOR_MODE_FRAME,
 * indicating a frame sensor.
 */
std::string UsgsAstroFrameSensorModel::getSensorMode() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing sensor mode");
  return CSM_SENSOR_MODE_FRAME;
}

/**
 * @brief Retrieves the reference date and time for the sensor model.
 * @description Returns the reference date and time that corresponds to the
 * epoch or start time of the image acquisition.
 * 
 * @return A string representing the reference date and time in a human-readable
 * calendar format.
 */
std::string UsgsAstroFrameSensorModel::getReferenceDateAndTime() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing reference data and time");
  time_t ephemTime = m_ephemerisTime;

  return ephemTimeToCalendarTime(ephemTime);
}

/**
 * @brief Extracts the model name from a sensor model's state string.
 * @description Parses a JSON-encoded string representing the state of a sensor model
 * to extract the model's name.
 * 
 * @param model_state A JSON-encoded string representing the state of the sensor model.
 * 
 * @return A string containing the name of the sensor model extracted from the state.
 * 
 * @throws csm::Error If the 'm_modelName' key is missing from the model state, indicating
 * an invalid or incomplete state representation, or if the model name extracted does not
 * match the expected sensor model name, indicating that the sensor model is not supported.
 */
std::string UsgsAstroFrameSensorModel::getModelNameFromModelState(
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
    std::string aFunction = "UsgsAstroFramePlugin::getModelNameFromModelState";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  if (model_name != _SENSOR_MODEL_NAME) {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    std::string aMessage = "Sensor model not supported.";
    std::string aFunction = "UsgsAstroFramePlugin::getModelNameFromModelState()";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  return model_name;
}

/**
 * @brief Serializes the current state of the sensor model to a string.
 * @description Encodes the complete state of the sensor model into a JSON-formatted
 * string.
 * 
 * @return A string containing the JSON-encoded state of the sensor model. The string
 * starts with the model name for identification, followed by the serialized JSON state
 * with a two-space indentation for readability.
 */
std::string UsgsAstroFrameSensorModel::getModelState() const {
  MESSAGE_LOG(spdlog::level::debug, "Dumping model state");
  json state = {
      {"m_modelName", _SENSOR_MODEL_NAME},
      {"m_sensorName", m_sensorName},
      {"m_platformName", m_platformName},
      {"m_focalLength", m_focalLength},
      {"m_iTransS", {m_iTransS[0], m_iTransS[1], m_iTransS[2]}},
      {"m_iTransL", {m_iTransL[0], m_iTransL[1], m_iTransL[2]}},
      {"m_boresight", {m_boresight[0], m_boresight[1], m_boresight[2]}},
      {"m_transX", {m_transX[0], m_transX[1], m_transX[2]}},
      {"m_transY", {m_transY[0], m_transY[1], m_transY[2]}},
      {"m_iTransS", {m_iTransS[0], m_iTransS[1], m_iTransS[2]}},
      {"m_iTransL", {m_iTransL[0], m_iTransL[1], m_iTransL[2]}},
      {"m_lineJitter", m_lineJitter},
      {"m_sampleJitter", m_sampleJitter},
      {"m_lineTimes", m_lineTimes},
      {"m_majorAxis", m_majorAxis},
      {"m_minorAxis", m_minorAxis},
      {"m_spacecraftVelocity",
       {m_spacecraftVelocity[0], m_spacecraftVelocity[1],
        m_spacecraftVelocity[2]}},
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
      {"m_referencePointXyz",
       {m_referencePointXyz.x, m_referencePointXyz.y, m_referencePointXyz.z}},
      {"m_currentParameterCovariance", m_currentParameterCovariance}};

  // Use dump(2) to avoid creating the model string as a single long line
  std::string stateString = getModelName() + "\n" + state.dump(2);
  return stateString;
}

/**
 * @brief Applies a geometric transform to the sensor model's state.
 * @description Updates the sensor model state by applying a rotation and translation
 * transform.
 * 
 * @param r The rotation to be applied to the sensor model state, represented as an
 * ale::Rotation object.
 * @param t The translation to be applied to the sensor model state, represented as an
 * ale::Vec3d object.
 * @param stateString A reference to a string containing the JSON-encoded state of the
 * sensor model. This string will be updated in-place with the transformed state.
 */
void UsgsAstroFrameSensorModel::applyTransformToState(ale::Rotation const& r, ale::Vec3d const& t,
                                                      std::string& stateString) {

  nlohmann::json j = stateAsJson(stateString);

  // The vector having the rotation and translation from camera to
  // ECEF
  std::vector<double> currentParameterValue = j["m_currentParameterValue"];

  // Extract the quaternions from the frame camera, apply the
  // rotation, and put them back.
  std::vector<double> quaternions;
  for (size_t it = 0; it < 4; it++)
    quaternions.push_back(currentParameterValue[it + 3]);
  applyRotationToQuatVec(r, quaternions);
  for (size_t it = 0; it < 4; it++)
    currentParameterValue[it + 3] = quaternions[it];

  // Extract the positions from the frame camera, apply to them the
  // rotation and translation, and put them back.
  std::vector<double> positions;
  for (size_t it = 0; it < 3; it++)
    positions.push_back(currentParameterValue[it]);
  applyRotationTranslationToXyzVec(r, t, positions);
  for (size_t it = 0; it < 3; it++)
    currentParameterValue[it] = positions[it];

  // Put the transformed vector back in the json state.
  j["m_currentParameterValue"] = currentParameterValue;

  // Apply rotation to velocities. The translation does not get
  // applied.
  std::vector<double> velocities = j["m_spacecraftVelocity"];
  ale::Vec3d zero_t(0, 0, 0);
  applyRotationTranslationToXyzVec(r, zero_t, velocities);
  j["m_spacecraftVelocity"] = velocities;

  // Apply the transform to the reference point.
  std::vector<double> refPt = j["m_referencePointXyz"];
  applyRotationTranslationToXyzVec(r, t, refPt);
  j["m_referencePointXyz"] = refPt;

  // We do not change the Sun position or velocity. The idea is that
  // the Sun is so far, that minor camera adjustments won't affect
  // where the Sun is.

  // Update the state string
  stateString = getModelNameFromModelState(stateString) + "\n" + j.dump(2);
}

/**
 * @brief Checks if the provided model state string represents a valid state for the sensor model.
 * @description Validates the JSON-encoded sensor model state string by checking for the presence
 * of all required keywords and comparing the model name against the expected name.
 * 
 * @param stringState A JSON-encoded string representing the state of the sensor model.
 * @param warnings A pointer to a csm::WarningList for logging any issues found during
 * the validation process. This includes missing required keywords and incorrect model names.
 * 
 * @return A boolean value indicating whether the provided model state string is valid. Returns
 * true if all required keywords are present and the model name matches the expected name;
 * otherwise, returns false.
 */
bool UsgsAstroFrameSensorModel::isValidModelState(
    const std::string &stringState, csm::WarningList *warnings) {
  MESSAGE_LOG(spdlog::level::debug, "Checking if model has valid state");
  std::vector<std::string> requiredKeywords = {"m_modelName",
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
                                               "m_iTransL"};

  json jsonState = stateAsJson(stringState);
  std::vector<std::string> missingKeywords;

  for (auto &key : requiredKeywords) {
    if (jsonState.find(key) == jsonState.end()) {
      missingKeywords.push_back(key);
    }
  }

  if (!missingKeywords.empty() && warnings) {
    std::ostringstream oss;
    std::copy(missingKeywords.begin(), missingKeywords.end(),
              std::ostream_iterator<std::string>(oss, " "));

    MESSAGE_LOG(spdlog::level::warn, "State has missing keywords: {} ", oss.str());

    warnings->push_back(
        csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                     "State has missing keywords: " + oss.str(),
                     "UsgsAstroFrameSensorModel::isValidModelState"));
  }

  std::string modelName = jsonState.value<std::string>("m_modelName", "");

  if (modelName != _SENSOR_MODEL_NAME && warnings) {
    MESSAGE_LOG(spdlog::level::warn, "Incorrect model name in state, expected {} but got {}",
                _SENSOR_MODEL_NAME, modelName);

    warnings->push_back(
        csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                     "Incorrect model name in state, expected " +
                         _SENSOR_MODEL_NAME + " but got " + modelName,
                     "UsgsAstroFrameSensorModel::isValidModelState"));
  }

  return modelName == _SENSOR_MODEL_NAME && missingKeywords.empty();
}

/**
 * @brief Validates the Imaging Support Data (ISD) for constructing a sensor model.
 * @description Checks if the provided ISD string can be converted into a valid
 * sensor model state. This serves as a preliminary validation step before attempting
 * to use the ISD for sensor model construction, ensuring that the ISD contains
 * the necessary information.
 * 
 * @param Isd A JSON-encoded string representing the ISD.
 * @param warnings A pointer to a csm::WarningList for logging any issues found during
 * the validation of the ISD.
 * 
 * @return A boolean indicating whether the ISD is valid for model construction.
 * True if a valid model state can be constructed from the ISD, false otherwise.
 */
bool UsgsAstroFrameSensorModel::isValidIsd(const std::string &Isd,
                                           csm::WarningList *warnings) {
  // no obvious clean way to truely validate ISD with it's nested structure,
  // or rather, it would be a pain to maintain, so just check if
  // we can get a valid state from ISD. Once ISD schema is 100% clear
  // we can change this.
  MESSAGE_LOG(spdlog::level::debug, "Building isd to check model state");
  try {
    std::string state = constructStateFromIsd(Isd, warnings);
    return isValidModelState(state, warnings);
  } catch (...) {
    return false;
  }
}

/**
 * @brief Replaces the current model state with a new state.
 * @description Updates the sensor model's state based on a JSON-encoded state
 * string. This allows for dynamic modification of the sensor model's
 * parameters and settings, effectively reconfiguring the model.
 * 
 * @param stringState A JSON-encoded string representing the new state of the sensor model.
 * 
 * @throws csm::Error If any required state keywords are missing or if the state
 * cannot be successfully parsed and applied, indicating that the new state is
 * invalid or incomplete.
 */
void UsgsAstroFrameSensorModel::replaceModelState(
    const std::string &stringState) {

  json state = stateAsJson(stringState);

  MESSAGE_LOG(spdlog::level::debug, "Replacing model state");
  // The json library's .at() will except if key is missing
  try {
    m_modelName = state.at("m_modelName").get<std::string>();
    m_majorAxis = state.at("m_majorAxis").get<double>();
    m_minorAxis = state.at("m_minorAxis").get<double>();
    m_focalLength = state.at("m_focalLength").get<double>();
    m_startingDetectorSample =
        state.at("m_startingDetectorSample").get<double>();
    m_startingDetectorLine = state.at("m_startingDetectorLine").get<double>();
    m_detectorSampleSumming = state.at("m_detectorSampleSumming").get<double>();
    m_detectorLineSumming = state.at("m_detectorLineSumming").get<double>();
    m_focalLengthEpsilon = state.at("m_focalLengthEpsilon").get<double>();
    m_nLines = state.at("m_nLines").get<int>();
    m_nSamples = state.at("m_nSamples").get<int>();
    m_ephemerisTime = state.at("m_ephemerisTime").get<double>();
    m_currentParameterValue =
        state.at("m_currentParameterValue").get<std::vector<double>>();
    m_ccdCenter = state.at("m_ccdCenter").get<std::vector<double>>();
    m_spacecraftVelocity =
        state.at("m_spacecraftVelocity").get<std::vector<double>>();
    m_sunPosition = state.at("m_sunPosition").get<std::vector<double>>();
    m_distortionType = (DistortionType)state.at("m_distortionType").get<int>();
    m_opticalDistCoeffs =
        state.at("m_opticalDistCoeffs").get<std::vector<double>>();
    m_transX = state.at("m_transX").get<std::vector<double>>();
    m_transY = state.at("m_transY").get<std::vector<double>>();
    m_iTransS = state.at("m_iTransS").get<std::vector<double>>();
    m_iTransL = state.at("m_iTransL").get<std::vector<double>>();
    m_imageIdentifier = state.at("m_imageIdentifier").get<std::string>();
    m_platformName = state.at("m_platformName").get<std::string>();
    m_sensorName = state.at("m_sensorName").get<std::string>();
    m_collectionIdentifier =
        state.at("m_collectionIdentifier").get<std::string>();
    // Set reference point to the center of the image
    m_referencePointXyz =
        imageToGround(csm::ImageCoord(m_nLines / 2.0, m_nSamples / 2.0));
    m_currentParameterCovariance =
        state.at("m_currentParameterCovariance").get<std::vector<double>>();

    // These are optional and may not exist in all state files
    if (state.find("m_maxElevation") != state.end())
      m_maxElevation = state.at("m_maxElevation").get<double>();
    if (state.find("m_minElevation") != state.end())
      m_minElevation = state.at("m_minElevation").get<double>();
    if (state.find("m_originalHalfLines") != state.end())
      m_originalHalfLines = state.at("m_originalHalfLines").get<double>();
    if (state.find("m_originalHalfSamples") != state.end())
      m_originalHalfSamples = state.at("m_originalHalfSamples").get<double>();
    if (state.find("m_pixelPitch") != state.end())
      m_pixelPitch = state.at("m_pixelPitch").get<double>();
    if (state.find("m_lineJitter") != state.end())
      m_lineJitter = state.at("m_lineJitter").get<std::vector<double>>();
    if (state.find("m_sampleJitter") != state.end())
      m_sampleJitter = state.at("m_sampleJitter").get<std::vector<double>>();
    if (state.find("m_lineTimes") != state.end())
      m_lineTimes = state.at("m_lineTimes").get<std::vector<double>>();

  } catch (std::out_of_range &e) {
    MESSAGE_LOG(
        spdlog::level::err,
        "State keywords required to generate sensor model missing: " +
        std::string(e.what()) + "\nUsing model string: " + stringState +
        "UsgsAstroFrameSensorModel::replaceModelState");
    throw csm::Error(
        csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
        "State keywords required to generate sensor model missing: " +
            std::string(e.what()) + "\nUsing model string: " + stringState,
        "UsgsAstroFrameSensorModel::replaceModelState");
  }
}

/**
 * @brief Constructs the sensor model state from an Imaging Support Data (ISD) string.
 * @description Parses a JSON-encoded ISD string to extract necessary parameters
 * and constructs the state of the sensor model.
 * 
 * @param jsonIsd A JSON-encoded string representing the ISD from which the state is to be constructed.
 * @param warnings A pointer to a csm::WarningList for logging any issues encountered during
 * the parsing of the ISD or the construction of the state.
 * 
 * @return A JSON-encoded string representing the constructed state of the sensor model.
 * 
 * @throws csm::Error If the ISD does not contain required information or if any errors occur
 * during the parsing process, indicating that the ISD is invalid for creating the sensor model.
 */
std::string UsgsAstroFrameSensorModel::constructStateFromIsd(
    const std::string &jsonIsd, csm::WarningList *warnings) {
  MESSAGE_LOG(spdlog::level::debug, "Constructing state from isd");
  json parsedIsd = json::parse(jsonIsd);
  json state = {};

  std::shared_ptr<csm::WarningList> parsingWarnings(new csm::WarningList);

  state["m_modelName"] = ale::getSensorModelName(parsedIsd);
  state["m_imageIdentifier"] = ale::getImageId(parsedIsd);
  state["m_sensorName"] = ale::getSensorName(parsedIsd);
  state["m_platformName"] = ale::getPlatformName(parsedIsd);

  state["m_startingDetectorSample"] = ale::getDetectorStartingSample(parsedIsd);
  state["m_startingDetectorLine"] = ale::getDetectorStartingLine(parsedIsd);
  state["m_detectorSampleSumming"] = ale::getSampleSumming(parsedIsd);
  state["m_detectorLineSumming"] = ale::getLineSumming(parsedIsd);

  state["m_focalLength"] = ale::getFocalLength(parsedIsd);
  try {
    state["m_focalLengthEpsilon"] = ale::getFocalLengthUncertainty(parsedIsd);
  } catch (std::runtime_error &e) {
    state["m_focalLengthEpsilon"] = 0.0;
  }

  state["m_currentParameterValue"] = json();

  ale::States inst_state = ale::getInstrumentPosition(parsedIsd);
  std::vector<double> ephemTime = inst_state.getTimes();
  std::vector<ale::State> instStates = inst_state.getStates();
  ale::Orientations j2000_to_target = ale::getBodyRotation(parsedIsd);
  ale::State rotatedInstState;
  std::vector<double> positions = {};
  std::vector<double> velocities = {};

  for (int i = 0; i < ephemTime.size(); i++) {
    rotatedInstState =
        j2000_to_target.rotateStateAt(ephemTime[i], instStates[i], ale::SLERP);
    // ALE operates in km and we want m
    positions.push_back(rotatedInstState.position.x * 1000);
    positions.push_back(rotatedInstState.position.y * 1000);
    positions.push_back(rotatedInstState.position.z * 1000);
    velocities.push_back(rotatedInstState.velocity.x * 1000);
    velocities.push_back(rotatedInstState.velocity.y * 1000);
    velocities.push_back(rotatedInstState.velocity.z * 1000);
  }

  if (!positions.empty() && positions.size() != 3) {
    MESSAGE_LOG(
        spdlog::level::warn,
        "Sensor position does not have 3 values, "
        "UsgsAstroFrameSensorModel::constructStateFromIsd()");
    parsingWarnings->push_back(
        csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                     "Sensor position does not have 3 values.",
                     "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    state["m_currentParameterValue"][0] = 0;
    state["m_currentParameterValue"][1] = 0;
    state["m_currentParameterValue"][2] = 0;
  } else {
    state["m_currentParameterValue"] = positions;
  }

  // get sensor_velocity
  if (!velocities.empty() && velocities.size() != 3) {
    MESSAGE_LOG(
        spdlog::level::warn,
        "Sensor velocity does not have 3 values, "
        "UsgsAstroFrameSensorModel::constructStateFromIsd()");
    parsingWarnings->push_back(
        csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                     "Sensor velocity does not have 3 values.",
                     "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
  } else {
    state["m_spacecraftVelocity"] = velocities;
  }

  // get sun_position
  // sun position is not strictly necessary, but is required for
  // getIlluminationDirection.
  ale::States sunState = ale::getSunPosition(parsedIsd);
  std::vector<ale::State> sunStates = sunState.getStates();
  ephemTime = sunState.getTimes();
  ale::State rotatedSunState;
  std::vector<double> sunPositions = {};

  for (int i = 0; i < ephemTime.size(); i++) {
    rotatedSunState = j2000_to_target.rotateStateAt(ephemTime[i], sunStates[i]);
    // ALE operates in km and we want m
    sunPositions.push_back(rotatedSunState.position.x * 1000);
    sunPositions.push_back(rotatedSunState.position.y * 1000);
    sunPositions.push_back(rotatedSunState.position.z * 1000);
  }

  state["m_sunPosition"] = sunPositions;

  // get sensor_orientation quaternion
  ale::Orientations j2000_to_sensor = ale::getInstrumentPointing(parsedIsd);

  // Work-around for issues in ALE <=0.8.5 where Orientations without angular
  // velocities seg fault when you multiply them. This will make the angular
  // velocities in the final Orientations dubious but they are not used
  // in any calculations so this is okay.
  if (j2000_to_sensor.getAngularVelocities().empty()) {
    j2000_to_sensor = ale::Orientations(
        j2000_to_sensor.getRotations(),
        j2000_to_sensor.getTimes(),
        std::vector<ale::Vec3d>(j2000_to_sensor.getTimes().size()),
        j2000_to_sensor.getConstantRotation(),
        j2000_to_sensor.getConstantFrames(),
        j2000_to_sensor.getTimeDependentFrames());
  }
  if (j2000_to_target.getAngularVelocities().empty()) {
    j2000_to_target = ale::Orientations(
        j2000_to_target.getRotations(),
        j2000_to_target.getTimes(),
        std::vector<ale::Vec3d>(j2000_to_target.getTimes().size()),
        j2000_to_target.getConstantRotation(),
        j2000_to_target.getConstantFrames(),
        j2000_to_target.getTimeDependentFrames());
  }

  ale::Orientations sensor_to_j2000 = j2000_to_sensor.inverse();
  ale::Orientations sensor_to_target = j2000_to_target * sensor_to_j2000;
  ephemTime = sensor_to_target.getTimes();
  std::vector<double> quaternion;
  std::vector<double> quaternions;

  for (int i = 0; i < ephemTime.size(); i++) {
    ale::Rotation rotation =
        sensor_to_target.interpolate(ephemTime[i], ale::SLERP);
    quaternion = rotation.toQuaternion();
    quaternions.push_back(quaternion[1]);
    quaternions.push_back(quaternion[2]);
    quaternions.push_back(quaternion[3]);
    quaternions.push_back(quaternion[0]);
  }

  if (quaternions.size() != 4) {
    MESSAGE_LOG(
        spdlog::level::warn,
        "Sensor quaternion does not have 4 values, "
        "UsgsAstroFrameSensorModel::constructStateFromIsd()");
    parsingWarnings->push_back(
        csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                     "Sensor orientation quaternion does not have 4 values.",
                     "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
  } else {
    state["m_currentParameterValue"][3] = quaternions[0];
    state["m_currentParameterValue"][4] = quaternions[1];
    state["m_currentParameterValue"][5] = quaternions[2];
    state["m_currentParameterValue"][6] = quaternions[3];
  }

  // get optical_distortion
  state["m_distortionType"] =
      getDistortionModel(ale::getDistortionModel(parsedIsd));
  state["m_opticalDistCoeffs"] = ale::getDistortionCoeffs(parsedIsd);

  // get detector_center
  state["m_ccdCenter"][0] = ale::getDetectorCenterLine(parsedIsd);
  state["m_ccdCenter"][1] = ale::getDetectorCenterSample(parsedIsd);

  // Get radii
  // ALE operates in km and we want m
  state["m_minorAxis"] = ale::getSemiMinorRadius(parsedIsd) * 1000;
  state["m_majorAxis"] = ale::getSemiMajorRadius(parsedIsd) * 1000;

  // get reference_height
  state["m_minElevation"] = ale::getMinHeight(parsedIsd);
  state["m_maxElevation"] = ale::getMaxHeight(parsedIsd);

  state["m_ephemerisTime"] = ale::getCenterTime(parsedIsd);
  state["m_nLines"] = ale::getTotalLines(parsedIsd);
  state["m_nSamples"] = ale::getTotalSamples(parsedIsd);

  state["m_iTransL"] = ale::getFocal2PixelLines(parsedIsd);
  state["m_iTransS"] = ale::getFocal2PixelSamples(parsedIsd);

  // optional rolling shutter jitter
  if (parsedIsd.find("jitter") != parsedIsd.end()) {
    state["m_lineJitter"] = parsedIsd.at("jitter").at("lineJitterCoefficients");
    state["m_sampleJitter"] = parsedIsd.at("jitter").at("sampleJitterCoefficients");
    state["m_lineTimes"] = parsedIsd.at("jitter").at("lineExposureTimes");
  }

  // We don't pass the pixel to focal plane transformation so invert the
  // focal plane to pixel transformation
  try {
    // "focal2pixel_lines":[0,136.4988453771169,0],"focal2pixel_samples":[136.4988453771169,0,0]
    double determinant = state["m_iTransL"][1].get<double>() *
                             state["m_iTransS"][2].get<double>() -
                         state["m_iTransL"][2].get<double>() *
                             state["m_iTransS"][1].get<double>();

    state["m_transX"][1] = state["m_iTransL"][1].get<double>() / determinant;
    state["m_transX"][2] = -state["m_iTransS"][1].get<double>() / determinant;
    state["m_transX"][0] = -(state["m_transX"][1].get<double>() *
                                 state["m_iTransL"][0].get<double>() +
                             state["m_transX"][2].get<double>() *
                                 state["m_iTransS"][0].get<double>());

    state["m_transY"][1] = -state["m_iTransL"][2].get<double>() / determinant;
    state["m_transY"][2] = state["m_iTransS"][2].get<double>() / determinant;
    state["m_transY"][0] = -(state["m_transY"][1].get<double>() *
                                 state["m_iTransL"][0].get<double>() +
                             state["m_transY"][2].get<double>() *
                                 state["m_iTransS"][0].get<double>());
  } catch (...) {
    MESSAGE_LOG(
        spdlog::level::warn,
        "Could not compute detector pixel to focal plane coordinate "
        "transformation.");
    parsingWarnings->push_back(
        csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                     "Could not compute detector pixel to focal plane "
                     "coordinate transformation.",
                     "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
  }

  state["m_referencePointXyz"] = std::vector<double>(3, 0.0);
  state["m_currentParameterCovariance"] =
      std::vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
  for (int i = 0; i < NUM_PARAMETERS * NUM_PARAMETERS;
       i += NUM_PARAMETERS + 1) {
    state["m_currentParameterCovariance"][i] = 1;
  }
  state["m_collectionIdentifier"] = "";

  if (!parsingWarnings->empty()) {
    if (warnings) {
      warnings->insert(warnings->end(), parsingWarnings->begin(),
                       parsingWarnings->end());
    }
    MESSAGE_LOG(spdlog::level::err, "ISD is invalid for creating the sensor model.");

    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     "ISD is invalid for creating the sensor model.",
                     "UsgsAstroFrameSensorModel::constructStateFromIsd");
  }

  return state.dump();
}

/**
 * @brief Retrieves the reference point of the sensor model in ECEF coordinates.
 * @description Returns the Earth-Centered, Earth-Fixed (ECEF) coordinates of the
 * reference point.
 * 
 * @return A csm::EcefCoord object representing the reference point in ECEF coordinates.
 */
csm::EcefCoord UsgsAstroFrameSensorModel::getReferencePoint() const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing reference point x: {}, y: {}, z: {}",
      m_referencePointXyz.x, m_referencePointXyz.y,
      m_referencePointXyz.z);
  return m_referencePointXyz;
}

/**
 * @brief Sets the reference point of the sensor model.
 * @description Updates the sensor model's reference point to a new location specified
 * by the given ECEF coordinates.
 * 
 * @param groundPt The new reference point in ECEF coordinates.
 */
void UsgsAstroFrameSensorModel::setReferencePoint(
    const csm::EcefCoord &groundPt) {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Setting reference point to {}, {}, {}",
      groundPt.x, groundPt.y, groundPt.z);
  m_referencePointXyz = groundPt;
}

/**
 * @brief Retrieves the number of parameters of the sensor model.
 * @description Returns the total number of adjustable parameters within the sensor model.
 * 
 * @return The number of adjustable parameters in the sensor model.
 */
int UsgsAstroFrameSensorModel::getNumParameters() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing num parameters: {}", NUM_PARAMETERS);
  return NUM_PARAMETERS;
}

/**
 * @brief Retrieves the name of a specific parameter by its index.
 * @description Provides the name of the parameter at the specified index.
 * 
 * @param index The index of the parameter.
 * 
 * @return A string representing the name of the parameter.
 */
std::string UsgsAstroFrameSensorModel::getParameterName(int index) const {
  MESSAGE_LOG(spdlog::level::debug, "Setting parameter name to {}", index);
  return m_parameterName[index];
}

/**
 * @brief Retrieves the units of a specific parameter by its index.
 * @description Provides the units in which a parameter is measured.
 * 
 * @param index The index of the parameter.
 * 
 * @return A string representing the units of the parameter.
 */
std::string UsgsAstroFrameSensorModel::getParameterUnits(int index) const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing parameter units for {}", index);
  if (index < 3) {
    return "m";
  } else {
    return "radians";
  }
}

/**
 * @brief Checks if the sensor model has any parameters that can be shared.
 * @description Determines if any of the sensor model's parameters are designed to be
 * shareable with other models or entities.
 * 
 * @return A boolean indicating if the model has shareable parameters.
 */
bool UsgsAstroFrameSensorModel::hasShareableParameters() const {
  MESSAGE_LOG(spdlog::level::debug, "Checking for shareable parameters");
  return false;
}

/**
 * @brief Checks if a specific parameter is shareable.
 * @description Determines if the parameter at the given index is designed to be
 * shareable with other models or entities.
 * 
 * @param index The index of the parameter.
 * 
 * @return A boolean indicating if the parameter is shareable.
 */
bool UsgsAstroFrameSensorModel::isParameterShareable(int index) const {
  MESSAGE_LOG(spdlog::level::debug, "Checking is parameter: {} is shareable", index);
  return false;
}

/**
 * @brief Retrieves the sharing criteria for a specific parameter.
 * @description Provides the criteria under which a parameter can be shared with
 * other models or entities, if sharing is supported.
 * 
 * @param index The index of the parameter.
 * 
 * @return A csm::SharingCriteria object describing the conditions under which the
 * parameter can be shared.
 */
csm::SharingCriteria UsgsAstroFrameSensorModel::getParameterSharingCriteria(
    int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Checking sharing criteria for parameter {}. "
      "Sharing is not supported.",
      index);
  return csm::SharingCriteria();
}

/**
 * @brief Retrieves the value of a specific parameter by its index.
 * @description Provides the current value of the parameter identified by the given
 * index.
 * 
 * @param index The index of the parameter.
 * 
 * @return The current value of the specified parameter.
 */
double UsgsAstroFrameSensorModel::getParameterValue(int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing parameter value {} at index: {}",
      m_currentParameterValue[index], index);
  return m_currentParameterValue[index];
}

/**
 * @brief Sets the value of a specific parameter by its index.
 * @description Updates the value of the parameter identified by the given index.
 * 
 * @param index The index of the parameter to be updated.
 * @param value The new value for the specified parameter.
 */
void UsgsAstroFrameSensorModel::setParameterValue(int index, double value) {
  MESSAGE_LOG(spdlog::level::debug, "Setting parameter value: {} at index: {}", value, index);
  m_currentParameterValue[index] = value;
}

/**
 * @brief Retrieves the type of a specific parameter by its index.
 * @description Provides the type classification of the parameter, indicating whether
 * it is adjustable, fixed, or another category defined by the csm::param::Type enumeration.
 * 
 * @param index The index of the parameter.
 * 
 * @return The type of the specified parameter as defined by the csm::param::Type enumeration.
 */
csm::param::Type UsgsAstroFrameSensorModel::getParameterType(int index) const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing parameter type: {} at index: {}",
              m_parameterType[index], index);
  return m_parameterType[index];
}

/**
 * @brief Sets the type of a specific parameter by its index.
 * @description Updates the type classification of the parameter, allowing for changes
 * in how the parameter is treated by the model or algorithms that use the model.
 * 
 * @param index The index of the parameter to be updated.
 * @param pType The new type for the specified parameter as defined by the csm::param::Type enumeration.
 */
void UsgsAstroFrameSensorModel::setParameterType(int index,
                                                 csm::param::Type pType) {
  MESSAGE_LOG(spdlog::level::debug, "Setting parameter type: {} at index: {}", pType, index);
  m_parameterType[index] = pType;
}

/**
 * @brief Retrieves the covariance between two parameters.
 * @description Provides the covariance value between the parameters identified by
 * the two indexes.
 * 
 * @param index1 The index of the first parameter.
 * @param index2 The index of the second parameter.
 * 
 * @return The covariance between the specified parameters.
 */
double UsgsAstroFrameSensorModel::getParameterCovariance(int index1,
                                                         int index2) const {
  int index = UsgsAstroFrameSensorModel::NUM_PARAMETERS * index1 + index2;
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing parameter covar: {} between index1: {} and index2: {}",
      m_currentParameterCovariance[index], index1, index2);
  return m_currentParameterCovariance[index];
}

/**
 * @brief Sets the covariance between two parameters.
 * @description Updates the covariance value between the parameters identified by the
 * two indexes.
 * 
 * @param index1 The index of the first parameter.
 * @param index2 The index of the second parameter.
 * @param covariance The new covariance value between the specified parameters.
 */
void UsgsAstroFrameSensorModel::setParameterCovariance(int index1, int index2,
                                                       double covariance) {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Setting parameter covar: {} between index1: {} and index2: {}",
      covariance, index1, index2);
  int index = UsgsAstroFrameSensorModel::NUM_PARAMETERS * index1 + index2;
  m_currentParameterCovariance[index] = covariance;
}

/**
 * @brief Retrieves the number of geometric correction switches.
 * @description Provides the total number of geometric correction switches available
 * in the sensor model. 
 * 
 * @return The number of geometric correction switches in the sensor model.
 */
int UsgsAstroFrameSensorModel::getNumGeometricCorrectionSwitches() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing num geom correction switches");
  return 0;
}

/**
 * @brief Retrieves the name of a specific geometric correction switch by its index.
 * @description Provides the name of the geometric correction switch identified by the
 * given index. This method is not supported in this model, as geometric corrections are
 * not applicable.
 * 
 * @param index The index of the geometric correction switch.
 * 
 * @throws csm::Error If the method is called, indicating that geometric correction switches
 * are not supported by this sensor model.
 */
std::string UsgsAstroFrameSensorModel::getGeometricCorrectionName(
    int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing name of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::getGeometricCorrectionName");
}

/**
 * @brief Sets the state of a specific geometric correction switch by its index.
 * @description Updates the state of the geometric correction switch identified by the
 * given index. This method is not supported in this model, as geometric corrections are
 * not applicable.
 * 
 * @param index The index of the geometric correction switch to be updated.
 * @param value The new state for the specified geometric correction switch.
 * @param pType The parameter type associated with the geometric correction switch.
 * 
 * @throws csm::Error If the method is called, indicating that geometric correction switches
 * are not supported by this sensor model.
 */
void UsgsAstroFrameSensorModel::setGeometricCorrectionSwitch(
    int index, bool value, csm::param::Type pType) {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Setting geometric correction switch {} to {} "
      "with parameter type {}. "
      "Geometric correction switches are not supported, throwing exception",
      index, value, pType);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::setGeometricCorrectionSwitch");
}

/**
 * @brief Retrieves the state of a specific geometric correction switch by its index.
 * @description Provides the state of the geometric correction switch identified by the
 * given index. This method is not supported in this model, as geometric corrections are
 * not applicable.
 * 
 * @param index The index of the geometric correction switch.
 * 
 * @throws csm::Error If the method is called, indicating that geometric correction switches
 * are not supported by this sensor model.
 */
bool UsgsAstroFrameSensorModel::getGeometricCorrectionSwitch(int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing value of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::getGeometricCorrectionSwitch");
}

/**
 * @brief Retrieves the cross-covariance matrix between this model and a comparison model.
 * @description Provides a matrix representing the cross-covariance between the adjustable
 * parameters of this model and those of another geometric model. Since no correlation is
 * assumed between models, a matrix of zeroes is returned.
 * 
 * @param comparisonModel The model with which to compare.
 * @param pSet The set of parameters to consider in the covariance computation.
 * @param otherModels A list of other geometric models in the scene; not used as no cross-model
 * covariance is supported.
 * 
 * @return A vector of doubles representing the flattened cross-covariance matrix, filled with zeroes.
 */
std::vector<double> UsgsAstroFrameSensorModel::getCrossCovarianceMatrix(
    const GeometricModel &comparisonModel, csm::param::Set pSet,
    const GeometricModelList &otherModels) const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing cross covariance matrix");

  // No correlation between models.
  const std::vector<int> &indices = getParameterSetIndices(pSet);
  size_t num_rows = indices.size();
  const std::vector<int> &indices2 =
      comparisonModel.getParameterSetIndices(pSet);
  size_t num_cols = indices.size();

  return std::vector<double>(num_rows * num_cols, 0.0);
}

/**
 * @brief Retrieves the ellipsoid associated with the sensor model.
 * @description Provides the ellipsoid used by the sensor model, defined by its major
 * and minor radii.
 * 
 * @return A csm::Ellipsoid object representing the ellipsoid used by the sensor model.
 */
csm::Ellipsoid UsgsAstroFrameSensorModel::getEllipsoid() const {
  MESSAGE_LOG(spdlog::level::debug, "Accessing ellipsoid radii {} {}", m_majorAxis, m_minorAxis);
  return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

/**
 * @brief Sets the ellipsoid to be used by the sensor model.
 * @description Updates the ellipsoid used by the sensor model to a new one specified by
 * the given csm::Ellipsoid object.
 * 
 * @param ellipsoid The new ellipsoid to be used by the sensor model.
 */
void UsgsAstroFrameSensorModel::setEllipsoid(const csm::Ellipsoid &ellipsoid) {
  MESSAGE_LOG(spdlog::level::debug, "Setting ellipsoid radii {} {}", ellipsoid.getSemiMajorRadius(),
              ellipsoid.getSemiMinorRadius());
  m_majorAxis = ellipsoid.getSemiMajorRadius();
  m_minorAxis = ellipsoid.getSemiMinorRadius();
}

/**
 * @brief Calculates the rotation matrix from the sensor model state or adjusted state.
 * @description Computes the rotation matrix used to rotate vectors from the sensor
 * frame to the earth-centered, earth-fixed (ECEF) frame. This method relies on the
 * quaternion parameters stored in the sensor model's state.
 * 
 * @param m A 3x3 matrix to store the resulting rotation matrix.
 */
void UsgsAstroFrameSensorModel::calcRotationMatrix(double m[3][3]) const {
  MESSAGE_LOG(spdlog::level::trace, "Calculating rotation matrix");
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

  m[0][0] = w * w + x * x - y * y - z * z;
  m[0][1] = 2 * (x * y - w * z);
  m[0][2] = 2 * (w * y + x * z);
  m[1][0] = 2 * (x * y + w * z);
  m[1][1] = w * w - x * x + y * y - z * z;
  m[1][2] = 2 * (y * z - w * x);
  m[2][0] = 2 * (x * z - w * y);
  m[2][1] = 2 * (w * x + y * z);
  m[2][2] = w * w - x * x - y * y + z * z;
}

/**
 * @brief Calculates the rotation matrix given a set of adjustments.
 * @description Computes the rotation matrix using quaternion parameters adjusted
 * by a given set of adjustments.
 * 
 * @param m A 3x3 matrix to store the resulting rotation matrix.
 * @param adjustments A vector of doubles representing the adjustments to the quaternion
 * parameters of the sensor model.
 */
void UsgsAstroFrameSensorModel::calcRotationMatrix(
    double m[3][3], const std::vector<double> &adjustments) const {
  MESSAGE_LOG(spdlog::level::trace, "Calculating rotation matrix with adjustments");
  // Trigonometric functions for rotation matrix
  double x = getValue(3, adjustments);
  double y = getValue(4, adjustments);
  double z = getValue(5, adjustments);
  double w = getValue(6, adjustments);

  m[0][0] = w * w + x * x - y * y - z * z;
  m[0][1] = 2 * (x * y - w * z);
  m[0][2] = 2 * (w * y + x * z);
  m[1][0] = 2 * (x * y + w * z);
  m[1][1] = w * w - x * x + y * y - z * z;
  m[1][2] = 2 * (y * z - w * x);
  m[2][0] = 2 * (x * z - w * y);
  m[2][1] = 2 * (w * x + y * z);
  m[2][2] = w * w - x * x - y * y + z * z;
}

/**
 * @brief Computes the intersection of a line-of-sight vector with an ellipsoid.
 * @description This method calculates the point at which a line-of-sight vector,
 * emanating from a given point in space, intersects an ellipsoid model of a planetary body.
 * The ellipsoid is defined by the sensor model's major and minor axis parameters and can be
 * expanded by a given height to model atmospheric effects or terrain.
 * 
 * @param height The height above the ellipsoid at which to calculate the intersection.
 * @param xc The X-coordinate of the camera position in ECEF space.
 * @param yc The Y-coordinate of the camera position in ECEF space.
 * @param zc The Z-coordinate of the camera position in ECEF space.
 * @param xl The X component of the line-of-sight vector.
 * @param yl The Y component of the line-of-sight vector.
 * @param zl The Z component of the line-of-sight vector.
 * @param x Reference to a double to store the X-coordinate of the intersection point.
 * @param y Reference to a double to store the Y-coordinate of the intersection point.
 * @param z Reference to a double to store the Z-coordinate of the intersection point.
 * @param warnings Optional pointer to a list to store any warnings generated during computation.
 */
void UsgsAstroFrameSensorModel::losEllipsoidIntersect(
    const double height, const double xc, const double yc, const double zc,
    const double xl, const double yl, const double zl, double &x, double &y,
    double &z, csm::WarningList *warnings) const {
  MESSAGE_LOG(
      spdlog::level::trace,
      "Calculating losEllipsoidIntersect with height: {},\n\
                                xc: {}, yc: {}, zc: {}\n\
                                xl: {}, yl: {}, zl: {}",
      height, xc, yc, zc, xl, yl, zl);
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
  // the surface of the ellipsoid.

  if (0.0 > quadTerm) {
    quadTerm = 0.0;
    std::string message = "Image ray does not intersect ellipsoid";
    if (warnings) {
      warnings->push_back(
          csm::Warning(csm::Warning::NO_INTERSECTION, message,
                       "UsgsAstroFrameSensorModel::losEllipsoidIntersect"));
    }
    MESSAGE_LOG(spdlog::level::warn, message);
  }

  double scale;
  scale = (-bt - sqrt(quadTerm)) / (2.0 * at);
  // Compute ground point vector

  x = xc + scale * xl;
  y = yc + scale * yl;
  z = zc + scale * zl;

  MESSAGE_LOG(spdlog::level::trace, "Calculated losEllipsoidIntersect at: {}, {}, {}", x, y, z);
}

/***** Helper Functions *****/

/**
 * @brief Retrieves a parameter value, optionally adjusted by a given set of adjustments.
 * @description Accesses the value of a parameter identified by its index and applies
 * a given adjustment if provided.
 * 
 * @param index The index of the parameter whose value is to be retrieved.
 * @param adjustments A vector of doubles representing adjustments to the sensor model's parameters.
 * @return The adjusted value of the parameter.
 */
double UsgsAstroFrameSensorModel::getValue(
    int index, const std::vector<double> &adjustments) const {
  MESSAGE_LOG(
      spdlog::level::trace,
      "Accessing value: {} at index: {}, with adjustments",
      m_currentParameterValue[index] + adjustments[index], index);
  return m_currentParameterValue[index] + adjustments[index];
}

/**
 * @brief Accessor for the sensor model's logger.
 * @description Provides access to the sensor model's logger for logging messages.
 * 
 * @return A shared pointer to the sensor model's spdlog logger.
 */
std::shared_ptr<spdlog::logger> UsgsAstroFrameSensorModel::getLogger() {
  return m_logger;
}

/**
 * @brief Sets the logger for the sensor model.
 * @description Assigns a logger to the sensor model for logging messages.
 * 
 * @param logName The name of the logger to be used by the sensor model.
 */
void UsgsAstroFrameSensorModel::setLogger(std::string logName) {
  m_logger = spdlog::get(logName);
}
