#ifndef INCLUDE_USGSCSM_DISTORTION_H_
#define INCLUDE_USGSCSM_DISTORTION_H_

#include <math.h>
#include <iostream>
#include <tuple>
#include <vector>

// This should be synched with the enum in ale/Distortion.h
enum DistortionType { RADIAL, TRANSVERSE, KAGUYALISM, DAWNFC, LROLROCNAC, CAHVOR, 
                     LUNARORBITER, RADTAN };

// Transverse distortion Jacobian
void transverseDistortionJacobian(double x, double y, double *jacobian,
                                  std::vector<double> const& opticalDistCoeffs);

void computeTransverseDistortion(double ux, double uy, double &dx, double &dy,
                                 std::vector<double> const& opticalDistCoeffs);

void removeDistortion(double dx, double dy, double &ux, double &uy,
                      std::vector<double> const& opticalDistCoeffs,
                      double focalLength,
                      DistortionType distortionType,
                      const double tolerance = 1.0E-6);

void applyDistortion(double ux, double uy, double &dx, double &dy,
                     std::vector<double> const& opticalDistCoeffs,
                     double focalLength,
                     DistortionType distortionType,
                     const double desiredPrecision = 1.0E-6,
                     const double tolerance = 1.0E-6);
#endif  // INCLUDE_USGSCSM_DISTORTION_H_
