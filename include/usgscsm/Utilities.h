#ifndef Utilities_h
#define Utilities_h

#include <vector>
#include <math.h>
#include <tuple>

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
  const double& startingSample,
  const double& lineOffset,
  const double iTransS[],
  const double iTransL[],
  std::tuple<double, double>& natFocalPlane);

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

//void calculateAttitudeCorrection(
//  const double& time,
//
//  double attCorr[9]);
//

#endif
