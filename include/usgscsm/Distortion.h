#ifndef Distortion_h
#define Distortion_h

#include <vector>
#include <math.h>
#include <cmath>
#include <tuple>
#include <iostream>

// Transverse Distortion
std::tuple<double, double> removeDistortion(double dx, double dy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

std::tuple<double, double> removeDistortion(double x, double y,
                                            std::vector<double> radialDistortionCoeffs,
                                            std::vector<std::vector<double>> transverseDistortionCoeffs);

std::vector<std::vector<double>> distortionJacobian(double x, double y,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

std::tuple<double, double> distortionFunction(double ux, double uy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

// Radial Distortion
std::tuple<double, double> removeDistortion(double inFocalPlaneX, double inFocalPlaneY,
                        const double opticalDistCoef[3], double tolerance = 1.0E-6);

std::tuple<double, double> invertDistortion(double inFocalPlaneX, double inFocalPlaneY,
                        const std::vector<double> opticalDistCoef, double desiredPrecision, double tolerance = 1.0E-6);

#endif
