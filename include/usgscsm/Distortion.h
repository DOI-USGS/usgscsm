#ifndef Distortion_h
#define Distortion_h

#include <vector>
#include <math.h>
#include <tuple>
#include <iostream>

enum DistortionType {
  RADIAL,
  TRANSVERSE
};

// Transverse Distortion
void distortionJacobian(double x, double y, double *jacobian,
                        const std::vector<double> opticalDistCoeffs);

void distortionFunction(double ux, double uy, double &dx, double &dy,
                        const std::vector<double> opticalDistCoeffs);

// Radial Distortion
void removeDistortion(double dx, double dy, double &ux, double &uy,
                      const std::vector<double> opticalDistCoeffs,
                      DistortionType distortionType,
                      double desiredPrecision = 0);

void applyDistortion(double dx, double dy, double &ux, double &uy,
                     const std::vector<double> opticalDistCoeffs,
                     DistortionType distortionType,
                     double desiredPrecision = 0);
#endif
