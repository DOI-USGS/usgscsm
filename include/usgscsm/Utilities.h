#ifndef Utilities_h
#define Utilities_h

#include <vector>
#include <math.h>
#include <tuple>
#include <string>

#include <json.hpp>

#include <Warning.h>

// methods pulled out of los2ecf and computeViewingPixel

// for now, put everything in here.
// TODO: later, consider if it makes sense to pull sample/line offsets out
// Compute distorted focalPlane coordinates in mm
std::tuple<double, double> computeDistortedFocalPlaneCoordinates(
  const double& line,
  const double& sample,
  const double& sampleOrigin,
  const double& lineOrigin,
  const double& sampleSumming,
  const double& startingSample,
  const double& lineOffset,
  const double iTransS[],
  const double iTransL[]);

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
  const double& focalLengthBias,
  const double& halfSwath,
  double cameraLook[]);

//void calculateAttitudeCorrection(
//  const double& time,
//
//  double attCorr[9]);
//

// Methods for checking/accessing the ISD

double metric_conversion(double val, std::string from, std::string to="m");
int getTotalLines(nlohmann::json isd, csm::WarningList *list=nullptr);
int getTotalSamples(nlohmann::json isd, csm::WarningList *list=nullptr);
double getStartingTime(nlohmann::json isd, csm::WarningList *list=nullptr);
double getCenterTime(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getIntegrationStartLines(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getIntegrationStartTimes(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getIntegrationTimes(nlohmann::json isd, csm::WarningList *list=nullptr);
int getSampleSumming(nlohmann::json isd, csm::WarningList *list=nullptr);
int getLineSumming(nlohmann::json isd, csm::WarningList *list=nullptr);
double getFocalLength(nlohmann::json isd, csm::WarningList *list=nullptr);
double getFocalLengthEpsilon(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getFocal2PixelLines(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getFocal2PixelSamples(nlohmann::json isd, csm::WarningList *list=nullptr);
csm::ImageCoord getDetectorCenter(nlohmann::json isd, csm::WarningList *list=nullptr);
double getMinHeight(nlohmann::json isd, csm::WarningList *list=nullptr);
double getMaxHeight(nlohmann::json isd, csm::WarningList *list=nullptr);
double getSemiMajorRadius(nlohmann::json isd, csm::WarningList *list=nullptr);
double getSemiMinorRadius(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getTransverseDistortionX(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getTransverseDistortionY(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getRadialDistortion(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSensorPositions(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSensorVelocities(nlohmann::json isd, csm::WarningList *list=nullptr);
std::vector<double> getSensorOrientations(nlohmann::json isd, csm::WarningList *list=nullptr);

#endif
