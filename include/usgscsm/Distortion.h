#ifndef Distortion_h
#define Distortion_h

#include <vector>
#include <math.h>
#include <cmath>
#include <tuple>
#include <iostream>

// Transverse Distortion
void removeTransverseDistortion(double dx, double dy, double ux, double uy,
                                const std::vector<double> &odtX, const std::vector<double> &odtY);

void distortionJacobian(double x, double y, double &jacobian,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

void distortionFunction(double ux, double uy, double dx, double dy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

// Radial Distortion
void removeRadialDistortion(double dx, double dy, double ux, double uy,
                            std::vector<double> radialDistortionCoeffs);

void removeDistortion(double inFocalPlaneX, double inFocalPlaneY,
                      double &outFocalPlaneX, double &outFocalPlaneY,
                      const double opticalDistCoef[3], double tolerance = 1.0E-6);

void invertDistortion(double inFocalPlaneX, double inFocalPlaneY,
                      double outFocalPlaneX, double outFocalPlaneY
                      const std::vector<double> opticalDistCoef, double desiredPrecision, double tolerance = 1.0E-6);
#endif
