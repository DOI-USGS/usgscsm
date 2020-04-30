#ifndef Utilities_h
#define Utilities_h

#include "Distortion.h"

#include <vector>
#include <math.h>
#include <tuple>
#include <string>

#include <json/json.hpp>

#include <csm.h>
#include <Warning.h>
// methods pulled out of los2ecf and computeViewingPixel

// for now, put everything in here.
// TODO: later, consider if it makes sense to pull sample/line offsets out
// Compute distorted focalPlane coordinates in mm
void computeDistortedFocalPlaneCoordinates(
  const double& line,
  const double& sample,
  const double& sampleOrigin,
  const double& lineOrigin,
  const double& sampleSumming,
  const double& lineSumming,
  const double& startingSample,
  const double& startingLine,
  const double iTransS[],
  const double iTransL[],
  double &distortedX,
  double &distortedY);

void computePixel(
  const double& distortedX,
  const double& distortedY,
  const double& sampleOrigin,
  const double& lineOrigin,
  const double& sampleSumming,
  const double& lineSumming,
  const double& startingSample,
  const double& startingLine,
  const double iTransS[],
  const double iTransL[],
  double &line,
  double &sample);

void calculateRotationMatrixFromQuaternions(
  double quaternions[4],
  double cameraToBody[9]);

void calculateRotationMatrixFromEuler(
  double euler[],
  double rotationMatrix[]);

void createCameraLookVector(
  const double& undistortedFocalPlaneX,
  const double& undistortedFocalPlaneY,
  const double& zDirection,
  const double& focalLength,
  double cameraLook[]);

void lagrangeInterp (
  const int&     numTime,
  const double*  valueArray,
  const double&  startTime,
  const double&  delTime,
  const double&  time,
  const int&     vectorLength,
  const int&     i_order,
  double*        valueVector);

// Brent's algorithm for finding the roots of a function
// Arguments are two inputs that bracket a root, the function, and a convergence tolerance
double brentRoot(
  double lowerBound,
  double upperBound,
  std::function<double(double)> func,
  double epsilon = 1e-10);

double secantRoot(
    double lowerBound,
    double upperBound,
    std::function<double(double)> func,
    double epsilon = 1e-10,
    int maxIterations = 30);

double computeEllipsoidElevation(
    double x,
    double y,
    double z,
    double semiMajor,
    double semiMinor,
    double desired_precision = 0.001,
    double* achieved_precision = nullptr);

// Vector operations
csm::EcefVector operator*(double scalar, const csm::EcefVector &vec);
csm::EcefVector operator*(const csm::EcefVector &vec, double scalar);
csm::EcefVector operator/(const csm::EcefVector &vec, double scalar);
csm::EcefVector operator+(const csm::EcefVector &vec1, const csm::EcefVector &vec2);
csm::EcefVector operator-(const csm::EcefVector &vec1, const csm::EcefVector &vec2);
double dot(const csm::EcefVector &vec1, const csm::EcefVector &vec2);
csm::EcefVector cross(const csm::EcefVector &vec1, const csm::EcefVector &vec2);
double norm(const csm::EcefVector &vec);
csm::EcefVector normalized(const csm::EcefVector &vec);
// The projection of vec1 onto vec2. The component of vec1 that is parallel to vec2
csm::EcefVector projection(const csm::EcefVector &vec1, const csm::EcefVector &vec2);
// The rejection of vec1 onto vec2. The component of vec1 that is orthogonal to vec2
csm::EcefVector rejection(const csm::EcefVector &vec1, const csm::EcefVector &vec2);

// Methods for checking/accessing the ISD

double metric_conversion(double val, std::string from, std::string to="m");
std::string getSensorModelName(nlohmann::json isd, csm::WarningList *list=nullptr);
std::string getImageId(nlohmann::json isd, csm::WarningList *list=nullptr);
std::string getSensorName(nlohmann::json isd, csm::WarningList *list=nullptr);
std::string getPlatformName(nlohmann::json isd, csm::WarningList *list=nullptr);
std::string getLogFile(nlohmann::json isd, csm::WarningList *list=nullptr);
int getTotalLines(nlohmann::json isd, csm::WarningList *list=nullptr);
int getTotalSamples(nlohmann::json isd, csm::WarningList *list=nullptr);
double getStartingTime(nlohmann::json isd, csm::WarningList *list=nullptr);
double getCenterTime(nlohmann::json isd, csm::WarningList *list=nullptr);
double getEndingTime(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getIntegrationStartLines(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getIntegrationStartTimes(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getIntegrationTimes(nlohmann::json isd, csm::WarningList *list=nullptr);
double getExposureDuration(nlohmann::json isd, csm::WarningList *list=nullptr);
double getScaledPixelWidth(nlohmann::json isd, csm::WarningList *list=nullptr);
std::string getLookDirection(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getScaleConversionCoefficients(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getScaleConversionTimes(nlohmann::json isd, csm::WarningList *list=nullptr);
int getSampleSumming(nlohmann::json isd, csm::WarningList *list=nullptr);
int getLineSumming(nlohmann::json isd, csm::WarningList *list=nullptr);
double getFocalLength(nlohmann::json isd, csm::WarningList *list=nullptr);
double getFocalLengthEpsilon(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getFocal2PixelLines(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getFocal2PixelSamples(nlohmann::json isd, csm::WarningList *list=nullptr);
double getDetectorCenterLine(nlohmann::json isd, csm::WarningList *list=nullptr);
double getDetectorCenterSample(nlohmann::json isd, csm::WarningList *list=nullptr);
double getDetectorStartingLine(nlohmann::json isd, csm::WarningList *list=nullptr);
double getDetectorStartingSample(nlohmann::json isd, csm::WarningList *list=nullptr);
double getMinHeight(nlohmann::json isd, csm::WarningList *list=nullptr);
double getMaxHeight(nlohmann::json isd, csm::WarningList *list=nullptr);
double getSemiMajorRadius(nlohmann::json isd, csm::WarningList *list=nullptr);
double getSemiMinorRadius(nlohmann::json isd, csm::WarningList *list=nullptr);
DistortionType getDistortionModel(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getDistortionCoeffs(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getRadialDistortion(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSunPositions(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSunVelocities(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSensorPositions(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSensorVelocities(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSensorOrientations(nlohmann::json isd, csm::WarningList *list=nullptr);
double getWavelength(nlohmann::json isd, csm::WarningList *list=nullptr);
#endif
