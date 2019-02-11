#include "Distortion.h"

using namespace std;

/**
 * @description Jacobian of the distortion function. The Jacobian was computed
 * algebraically from the function described in the distortionFunction
 * method.
 *
 * @param x
 * @param y
 * @param Jxx  Partial_xx
 * @param Jxy  Partial_xy
 * @param Jyx  Partial_yx
 * @param Jyy  Partial_yy
 * @param odtX opticalDistCoef In X
 * @param odtY opticalDistCoef In Y
 */

void distortionJacobian(double x, double y, double &Jxx, double &Jxy,
                        double &Jyx, double &Jyy,
                        const vector<double> &odtX, const vector<double> &odtY) {

  double d_dx[10];
  d_dx[0] = 0;
  d_dx[1] = 1;
  d_dx[2] = 0;
  d_dx[3] = 2 * x;
  d_dx[4] = y;
  d_dx[5] = 0;
  d_dx[6] = 3 * x * x;
  d_dx[7] = 2 * x * y;
  d_dx[8] = y * y;
  d_dx[9] = 0;
  double d_dy[10];
  d_dy[0] = 0;
  d_dy[1] = 0;
  d_dy[2] = 1;
  d_dy[3] = 0;
  d_dy[4] = x;
  d_dy[5] = 2 * y;
  d_dy[6] = 0;
  d_dy[7] = x * x;
  d_dy[8] = 2 * x * y;
  d_dy[9] = 3 * y * y;

  Jxx = 0.0;
  Jxy = 0.0;
  Jyx = 0.0;
  Jyy = 0.0;

  for (int i = 0; i < 10; i++) {
    Jxx = Jxx + d_dx[i] * odtX[i];
    Jxy = Jxy + d_dy[i] * odtX[i];
    Jyx = Jyx + d_dx[i] * odtY[i];
    Jyy = Jyy + d_dy[i] * odtY[i];
  }
}

/**
 * @description Compute distorted focal plane (dx,dy) coordinate  given an undistorted focal
 * plane (ux,uy) coordinate. This describes the third order Taylor approximation to the
 * distortion model.
 *
 * @param ux Undistored x
 * @param uy Undistored y
 * @param dx Result distorted x
 * @param dy Result distorted y
 * @param odtX opticalDistCoef In X
 * @param odtY opticalDistCoef In Y
 */
void distortionFunction(double ux, double uy, double &dx, double &dy,
                        const vector<double> &odtX, const vector<double> &odtY) {

  double f[10];
  f[0] = 1;
  f[1] = ux;
  f[2] = uy;
  f[3] = ux * ux;
  f[4] = ux * uy;
  f[5] = uy * uy;
  f[6] = ux * ux * ux;
  f[7] = ux * ux * uy;
  f[8] = ux * uy * uy;
  f[9] = uy * uy * uy;

  dx = 0.0;
  dy = 0.0;
  for (int i = 0; i < 10; i++) {
    dx = dx + f[i] * odtX[i];
    dy = dy + f[i] * odtY[i];
  }
}
