#include "ORexSensorModel.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include <csm/Error.h>
#include <json/json.hpp>
using json = nlohmann::json;

using namespace std;

const std::string ORexSensorModel::_SENSOR_MODEL_NAME
                                      = "ISIS_ORex_USGSAstro_1_Linux64_csm30.so";



ORexSensorModel::ORexSensorModel() {
  m_transX[0] = 0.0;
  m_transX[1] = 0.0;
  m_transX[2] = 0.0;

  m_transY[0] = 0.0;
  m_transY[1] = 0.0;
  m_transY[2] = 0.0;

  m_iTransS[0] = 0.0;
  m_iTransS[1] = 0.0;
  m_iTransS[2] = 0.0;

  m_iTransL[0] = 0.0;
  m_iTransL[0] = 0.0;
  m_iTransL[0] = 0.0;

  m_a_axis = 0.0;
  m_b_axis = 0.0;
  m_c_axis = 0.0;
  m_omega = 0.0;
  m_phi = 0.0;
  m_kappa = 0.0;
  m_focalLength = 0.0;

  m_spacecraftPosition[0] = 0.0;
  m_spacecraftPosition[1] = 0.0;
  m_spacecraftPosition[2] = 0.0;

  m_spacecraftVelocity[0] = 0.0;
  m_spacecraftVelocity[1] = 0.0;
  m_spacecraftVelocity[2] = 0.0;

  m_sunPosition[0] = 0.0;
  m_sunPosition[1] = 0.0;
  m_sunPosition[2] = 0.0;

  m_startingDetectorSample = 0.0;
  m_startingDetectorLine = 0.0;
  m_targetName = "";
  m_ifov = 0.0;
  m_instrumentID = "";
  m_focalLengthEpsilon = 0.0;

  m_ccdCenter[0] = 0.0;
  m_ccdCenter[1] = 0.0;

  m_line_pp = 0.0;
  m_sample_pp = 0.0;


  m_originalHalfLines = 0.0;
  m_spacecraftName = "";

  m_ephemerisTime = 0.0;
  m_originalHalfSamples = 0.0;
  m_boresight[0] = 0.0;
  m_boresight[1] = 0.0;
  m_boresight[2] = 0.0;

  m_nLines = 0;
  m_nSamples = 0;
}


ORexSensorModel::~ORexSensorModel() {}

csm::ImageCoord ORexSensorModel::groundToImage(const csm::EcefCoord &groundPt,
                              double desiredPrecision,
                              double *achievedPrecision,
                              csm::WarningList *warnings) const {

double xl, yl, zl;
xl = m_spacecraftPosition[0];
yl = m_spacecraftPosition[1];
zl = m_spacecraftPosition[2];

double x, y, z;
x = groundPt.x;
y = groundPt.y;
z = groundPt.z;

double xo, yo, zo;
xo = xl - x;
yo = yl - y;
zo = zl - z;

double f;
f = m_focalLength;

// Camera rotation matrix
double m[3][3];
calcRotationMatrix(m);

// Sensor position
double undistortedx, undistortedy, denom;
denom = m[0][2] * xo + m[1][2] * yo + m[2][2] * zo;
undistortedx = (f * (m[0][0] * xo + m[1][0] * yo + m[2][0] * zo)/denom) + m_sample_pp;  //m_sample_pp like this assumes mm
undistortedy = (f * (m[0][1] * xo + m[1][1] * yo + m[2][1] * zo)/denom) + m_line_pp;

// Apply the distortion to the line/sample location and then convert back to line/sample
//double distortedx, distortedy;
//distortionFunction(undistortedx, undistortedy, distortedx, distortedy);

//Convert distorted mm into line/sample
double sample, line;
sample = m_iTransS[0] + m_iTransS[1] * undistortedx + m_iTransS[2] * undistortedx + m_ccdCenter[0];
line =   m_iTransL[0] + m_iTransL[1] * undistortedy + m_iTransL[2] * undistortedy + m_ccdCenter[0];

return csm::ImageCoord(line, sample);
}


csm::ImageCoordCovar ORexSensorModel::groundToImage(const csm::EcefCoordCovar &groundPt,
                                   double desiredPrecision,
                                   double *achievedPrecision,
                                   csm::WarningList *warnings) const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::groundToImage");
}


csm::EcefCoord ORexSensorModel::imageToGround(const csm::ImageCoord &imagePt,
                                                 double height,
                                                 double desiredPrecision,
                                                 double *achievedPrecision,
                                                 csm::WarningList *warnings) const {

  double sample = imagePt.samp;
  double line = imagePt.line;

  std::cout << "Sample: " << sample << " Line: "<< line << std::endl;

  //Here is where we should be able to apply an adjustment to opk
  double m[3][3];
  calcRotationMatrix(m);

  //Apply the principal point offset, assuming the pp is given in pixels
  double xl, yl, zl, lo, so;
  lo = line - m_line_pp;
  so = sample - m_sample_pp;

  //Convert from the pixel space into the metric space
  double optical_center_x, optical_center_y, x_camera, y_camera;
  optical_center_x = m_ccdCenter[0];
  optical_center_y = m_ccdCenter[1];
  y_camera = m_transY[0] + m_transY[1] * (lo - optical_center_y) + m_transY[2] * (lo - optical_center_y);
  x_camera = m_transX[0] + m_transX[1] * (so - optical_center_x) + m_transX[2] * (so - optical_center_x);

  // Apply the distortion model (remove distortion)
  //double undistorted_cameraX, undistorted_cameraY = 0.0;
  //setFocalPlane(x_camera, y_camera, undistorted_cameraX, undistorted_cameraY);

  //Now back from distorted mm to pixels
  double udx, udy; //distorted line and sample
  //udx = undistorted_cameraX;
  //udy = undistorted_cameraY;

  udx = x_camera;
  udy = y_camera;

  xl = m[0][0] *  + m[0][1] * udy - m[0][2] * -m_focalLength;
  yl = m[1][0] * udx + m[1][1] * udy - m[1][2] * -m_focalLength;
  zl = m[2][0] * udx + m[2][1] * udy - m[2][2] * -m_focalLength;

  double x, y, z;
  double xc, yc, zc;
  xc = m_spacecraftPosition[0];
  yc = m_spacecraftPosition[1];
  zc = m_spacecraftPosition[2];

  // Intersect with some height about the ellipsoid.
  losEllipsoidIntersect(height, xc, yc, zc, xl, yl, zl, x, y, z);

  return csm::EcefCoord(x, y, z);
}

csm::EcefCoordCovar ORexSensorModel::imageToGround(const csm::ImageCoordCovar &imagePt, double height,
                                  double heightVariance, double desiredPrecision,
                                  double *achievedPrecision,
                                  csm::WarningList *warnings) const {
    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::imageToGround");
}

csm::EcefLocus ORexSensorModel::imageToProximateImagingLocus(const csm::ImageCoord &imagePt,
                                                                const csm::EcefCoord &groundPt,
                                                                double desiredPrecision,
                                                                double *achievedPrecision,
                                                                csm::WarningList *warnings) const {
  // Ignore the ground point?
  return imageToRemoteImagingLocus(imagePt);
}


csm::EcefLocus ORexSensorModel::imageToRemoteImagingLocus(const csm::ImageCoord &imagePt,
                                                             double desiredPrecision,
                                                             double *achievedPrecision,
                                                             csm::WarningList *warnings) const {
  // Find the line,sample on the focal plane (mm)
  // CSM center = 0.5, OREX IK center = 0.0
  double col = imagePt.samp - (m_ccdCenter[0] + 0.5);
  double row = imagePt.line - (m_ccdCenter[1] + 0.5);
  double focalPlaneX = m_transX[0] + m_transX[1] * col + m_transX[2] * col;
  double focalPlaneY = m_transY[0] + m_transY[1] * row + m_transY[2] * row;

  //No distortion model, so no need to distort...

  // Get rotation matrix and transform to a body-fixed frame
  double m[3][3];
  calcRotationMatrix(m);
  std::vector<double> lookC { focalPlaneX, focalPlaneY, m_focalLength };
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

  return csm::EcefLocus(m_spacecraftPosition[0], m_spacecraftPosition[1], m_spacecraftPosition[2],
      lookBUnit[0], lookBUnit[1], lookBUnit[2]);
}


csm::ImageCoord ORexSensorModel::getImageStart() const {

  csm::ImageCoord start;
  start.samp = m_startingDetectorSample;
  start.line = m_startingDetectorLine;
  return start;
}

csm::ImageVector ORexSensorModel::getImageSize() const {

  csm::ImageVector size;
  size.line = m_nLines;
  size.samp = m_nSamples;
  return size;
}

std::pair<csm::ImageCoord, csm::ImageCoord> ORexSensorModel::getValidImageRange() const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::getValidImageRange");
}

std::pair<double, double> ORexSensorModel::getValidHeightRange() const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::getValidHeightRange");
}

csm::EcefVector ORexSensorModel::getIlluminationDirection(const csm::EcefCoord &groundPt) const {
  // ground (body-fixed) - sun (body-fixed) gives us the illumination direction.
  return csm::EcefVector {
    groundPt.x - m_sunPosition[0],
    groundPt.y - m_sunPosition[1],
    groundPt.z - m_sunPosition[2]
  };
}

double ORexSensorModel::getImageTime(const csm::ImageCoord &imagePt) const {

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
                     "ORexSensorModel::getImageTime");
  }
}

csm::EcefCoord ORexSensorModel::getSensorPosition(const csm::ImageCoord &imagePt) const {
  // check if the image point is in range
  if (imagePt.samp >= m_startingDetectorSample &&
      imagePt.samp <= (m_startingDetectorSample + m_nSamples) &&
      imagePt.line >= m_startingDetectorSample &&
      imagePt.line <= (m_startingDetectorLine + m_nLines)) {
    csm::EcefCoord sensorPosition;
    sensorPosition.x = m_spacecraftPosition[0];
    sensorPosition.y = m_spacecraftPosition[1];
    sensorPosition.z = m_spacecraftPosition[2];

    return sensorPosition;
  }
  else {
    throw csm::Error(csm::Error::BOUNDS,
                     "Image Coordinate out of Bounds",
                     "ORexSensorModel::getSensorPosition");
  }
}

csm::EcefCoord ORexSensorModel::getSensorPosition(double time) const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::getSensorPosition");
}

csm::EcefVector ORexSensorModel::getSensorVelocity(const csm::ImageCoord &imagePt) const {
  // Make sure the passed coordinate is with the image dimensions.
  if (imagePt.samp < 0.0 || imagePt.samp > m_nSamples ||
      imagePt.line < 0.0 || imagePt.line > m_nLines) {
    std::stringstream ss;
    ss << "Image coordinate (" << imagePt.line << ", " << imagePt.samp << ") out of bounds.";
    throw csm::Error(csm::Error::BOUNDS, ss.str(), "ORexSensorModel::getSensorVelocity");
  }

  // Since this is a frame, just return the sensor velocity the ISD gave us.
  return csm::EcefVector {
    m_spacecraftVelocity[0],
    m_spacecraftVelocity[1],
    m_spacecraftVelocity[2]
  };
}

csm::EcefVector ORexSensorModel::getSensorVelocity(double time) const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::getSensorVelocity");
}

csm::RasterGM::SensorPartials ORexSensorModel::computeSensorPartials(int index, const csm::EcefCoord &groundPt,
                                           double desiredPrecision,
                                           double *achievedPrecision,
                                           csm::WarningList *warnings) const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::computeSensorPartials");
}

csm::RasterGM::SensorPartials ORexSensorModel::computeSensorPartials(int index, const csm::ImageCoord &imagePt,
                                          const csm::EcefCoord &groundPt,
                                          double desiredPrecision,
                                          double *achievedPrecision,
                                          csm::WarningList *warnings) const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::computeSensorPartials");
}

std::vector<double> ORexSensorModel::computeGroundPartials(const csm::EcefCoord &groundPt) const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::computeGroundPartials");
}

const csm::CorrelationModel& ORexSensorModel::getCorrelationModel() const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::getCorrelationModel");
}

std::vector<double> ORexSensorModel::getUnmodeledCrossCovariance(const csm::ImageCoord &pt1,
                                                const csm::ImageCoord &pt2) const {

    throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "ORexSensorModel::getUnmodeledCrossCovariance");
}




csm::Version ORexSensorModel::getVersion() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getVersion");
}


std::string ORexSensorModel::getModelName() const {
  return "UsgsAstroFrameMdisPluginCSM";
}


std::string ORexSensorModel::getPedigree() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getPedigree");
}


std::string ORexSensorModel::getImageIdentifier() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getImageIdentifier");
}


void ORexSensorModel::setImageIdentifier(const std::string& imageId,
                                            csm::WarningList* warnings) {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::setImageIdentifier");
}


std::string ORexSensorModel::getSensorIdentifier() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getSensorIdentifier");
}


std::string ORexSensorModel::getPlatformIdentifier() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getPlatformIdentifier");
}


std::string ORexSensorModel::getCollectionIdentifier() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getCollectionIdentifier");
}


std::string ORexSensorModel::getTrajectoryIdentifier() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getTrajectoryIdentifier");
}


std::string ORexSensorModel::getSensorType() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getSensorType");
}


std::string ORexSensorModel::getSensorMode() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getSensorMode");
}


std::string ORexSensorModel::getReferenceDateAndTime() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getReferenceDateAndTime");
}


std::string ORexSensorModel::getModelState() const {
  // TEMPORARY
  /* commented out for testing the gtest framework
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getModelState");
  */
  json state = {
    {"m_focalLength" , m_focalLength},
    {"m_spacecraftPosition"}, {m_spacecraftPosition[0],
                               m_spacecraftPosition[1],
                               m_spacecraftPosition[2]},
    {"m_iTransS"}, {m_iTransS[0],
                    m_iTransS[1],
                    m_iTransS[2]},
    {"m_iTransL"}, {m_iTransL[0],
                    m_iTransL[1],
                    m_iTransL[2]},
    {"m_boresight"}, {m_boresight[0],
                      m_boresight[1],
                      m_boresight[2]}
  };
  return state.dump();
}


void ORexSensorModel::replaceModelState(const std::string& argState) {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::replaceModelState");
}




csm::EcefCoord ORexSensorModel::getReferencePoint() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getReferencePoint");
}


void ORexSensorModel::setReferencePoint(const csm::EcefCoord &groundPt) {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::setReferencePoint");
}


int ORexSensorModel::getNumParameters() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getNumParameters");
}


std::string ORexSensorModel::getParameterName(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getParameterName");
}


std::string ORexSensorModel::getParameterUnits(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getParameterUnits");
}


bool ORexSensorModel::hasShareableParameters() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::hasShareableParameters");
}


bool ORexSensorModel::isParameterShareable(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::isParameterShareable");
}


csm::SharingCriteria ORexSensorModel::getParameterSharingCriteria(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getParameterSharingCriteria");
}


double ORexSensorModel::getParameterValue(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getParameterValue");
}


void ORexSensorModel::setParameterValue(int index, double value) {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::setParameterValue");
}


csm::param::Type ORexSensorModel::getParameterType(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getParameterType");
}


void ORexSensorModel::setParameterType(int index, csm::param::Type pType) {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::setParameterType");
}


double ORexSensorModel::getParameterCovariance(int index1, int index2) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getParameterCovariance");
}


void ORexSensorModel::setParameterCovariance(int index1, int index2, double covariance) {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::setParameterCovariance");
}


int ORexSensorModel::getNumGeometricCorrectionSwitches() const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getNumGeometricCorrectionSwitches");
}


std::string ORexSensorModel::getGeometricCorrectionName(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getGeometricCorrectionName");
}


void ORexSensorModel::setGeometricCorrectionSwitch(int index,
                                                      bool value,
                                                      csm::param::Type pType) {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::setGeometricCorrectionSwitch");
}


bool ORexSensorModel::getGeometricCorrectionSwitch(int index) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getGeometricCorrectionSwitch");
}


std::vector<double> ORexSensorModel::getCrossCovarianceMatrix(
    const GeometricModel &comparisonModel,
    csm::param::Set pSet,
    const GeometricModelList &otherModels) const {
  throw csm::Error(csm::Error::UNSUPPORTED_FUNCTION,
                   "Unsupported function",
                   "ORexSensorModel::getCrossCovarianceMatrix");
}

void ORexSensorModel::calcRotationMatrix(
  double m[3][3]) const {

  // Trigonometric functions for rotation matrix
  double sinw = std::sin(m_omega);
  double cosw = std::cos(m_omega);
  double sinp = std::sin(m_phi);
  double cosp = std::cos(m_phi);
  double sink = std::sin(m_kappa);
  double cosk = std::cos(m_kappa);

  // Rotation matrix taken from Introduction to Mordern Photogrammetry by
  // Edward M. Mikhail, et al., p. 373

  m[0][0] = cosp * cosk;
  m[0][1] = cosw * sink + sinw * sinp * cosk;
  m[0][2] = sinw * sink - cosw * sinp * cosk;
  m[1][0] = -1 * cosp * sink;
  m[1][1] = cosw * cosk - sinw * sinp * sink;
  m[1][2] = sinw * cosk + cosw * sinp * sink;
  m[2][0] = sinp;
  m[2][1] = -1 * sinw * cosp;
  m[2][2] = cosw * cosp;
}

void ORexSensorModel::losEllipsoidIntersect(
      const double& height,
      const double& xc,
      const double& yc,
      const double& zc,
      const double& xl,
      const double& yl,
      const double& zl,
      double&       x,
      double&       y,
      double&       z ) const
{
   // Helper function which computes the intersection of the image ray
   // with an expanded ellipsoid.  All vectors are in earth-centered-fixed
   // coordinate system with origin at the center of the earth.

   // TODO: This is a triaxial body but the intersection is biaxial as this
   // is just a demo - need to fix.

   double ap, bp, k;
   ap = m_a_axis + height;
   bp = m_b_axis + height;
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
