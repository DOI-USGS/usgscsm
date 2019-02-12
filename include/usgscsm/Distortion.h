#ifndef Distortion_h
#define Distortion_h

#include <vector>
#include <math.h>

void distortionJacobian(double x, double y, double &Jxx, double &Jxy,
                        double &Jyx, double &Jyy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

void distortionFunction(double ux, double uy, double &dx, double &dy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

void removeDistortion(double inFocalPlaneX, double inFocalPlaneY,
                      double &outFocalPlaneX, double &outFocalPlaneY, const double opticalDistCoef[3],
                      double tolerance = 1.0E-6);

void invertDistortion(double inFocalPlaneX, double inFocalPlaneY,
                      double &outFocalPlaneX, double &outFocalPlaneY, const double opticalDistCoef[3],
                      double desiredPrecision, double tolerance = 1.0E-6);

#endif
