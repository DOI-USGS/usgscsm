#ifndef INCLUDE_USGSCSM_DISTORTION_H_
#define INCLUDE_USGSCSM_DISTORTION_H_

#include <math.h>
#include <iostream>
#include <tuple>
#include <vector>

enum DistortionType { RADIAL, TRANSVERSE, KAGUYALISM, DAWNFC, LROLROCNAC, CAHVOR, RADTAN };

// Transverse distortion Jacobian
void transverseDistortionJacobian(double x, double y, double *jacobian,
                                  const std::vector<double> opticalDistCoeffs);

void computeTransverseDistortion(double ux, double uy, double &dx, double &dy,
                                 const std::vector<double> opticalDistCoeffs);

void removeDistortion(double dx, double dy, double &ux, double &uy,
                      const std::vector<double> opticalDistCoeffs,
                      DistortionType distortionType,
                      const double tolerance = 1.0E-6);

void applyDistortion(double ux, double uy, double &dx, double &dy,
                     const std::vector<double> opticalDistCoeffs,
                     DistortionType distortionType,
                     const double desiredPrecision = 1.0E-6,
                     const double tolerance = 1.0E-6);
#endif  // INCLUDE_USGSCSM_DISTORTION_H_
