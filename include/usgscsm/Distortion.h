#ifndef Distortion_h
#define Distortion_h

#include <vector>
#include <math.h>
#include <tuple>
#include <iostream>

enum DistortionType {
  RADIAL,
  TRANSVERSE,
  KAGUYALISM,
  DAWNFC,
  LROLROCNAC
};


// Transverse Distortion
void distortionJacobian(double x, double y, double *jacobian,
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
                     const double desiredPrecision = 1.0E-6, const double tolerance = 1.0E-6);
#endif
