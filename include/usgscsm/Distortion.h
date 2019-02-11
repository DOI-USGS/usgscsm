#ifndef Distortion_h
#define Distortion_h

#include <vector>

void distortionJacobian(double x, double y, double &Jxx, double &Jxy,
                        double &Jyx, double &Jyy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

void distortionFunction(double ux, double uy, double &dx, double &dy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY);

#endif
