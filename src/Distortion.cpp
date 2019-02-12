#include "Distortion.h"

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
                        const std::vector<double> &odtX, const std::vector<double> &odtY) {

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
                        const std::vector<double> &odtX, const std::vector<double> &odtY) {

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

/**
 * @description Compute undistorted focal plane coordinate given a distorted
 * coordinate set and the distortion coefficients
 *
 * @param inFocalPlaneX Distoreted x
 * @param inFocalPlaneY Distoreted y
 * @param outFocalPlaneX Undistoreted x
 * @param outFocalPlaneY Undistoreted y
 * @param opticalDistCoef distortion coefficients
 */
void removeDistortion(double inFocalPlaneX, double inFocalPlaneY,
  double &outFocalPlaneX, double &outFocalPlaneY, const double opticalDistCoef[3],
  double tolerance) {
  double rr = inFocalPlaneX * inFocalPlaneX + inFocalPlaneY * inFocalPlaneY;
  if (rr > tolerance)
  {
    double dr = opticalDistCoef[0] + (rr * (opticalDistCoef[1] + rr * opticalDistCoef[2]));
    outFocalPlaneX = inFocalPlaneX * (1.0 - dr);
    outFocalPlaneY = inFocalPlaneY * (1.0 - dr);
  }
}

/**
 * @description Compute undistorted focal plane coordinate given a distorted
 * focal plane coordinate. This method works by iteratively adding distortion
 * until the new distorted point, r, undistorts to within a tolerance of the
 * original point, rp.
 *
 * @param inFocalPlaneX Distoreted x
 * @param inFocalPlaneY Distoreted y
 * @param outFocalPlaneX Undistoreted x
 * @param outFocalPlaneY Undistoreted y
 * @param opticalDistCoef Distortion coefficients
 * @param desiredPrecision Convergence precision
 * @param tolerance Tolerance of r^2
 */
void invertDistortion(double inFocalPlaneX, double inFocalPlaneY,
  double &outFocalPlaneX, double &outFocalPlaneY, const double opticalDistCoef[3],
  double desiredPrecision, double tolerance) {
  double rp2 = (inFocalPlaneX * inFocalPlaneX) +
               (inFocalPlaneY * inFocalPlaneY);

  if (rp2 > tolerance) {
    double rp = sqrt(rp2);
    // Compute first fractional distortion using rp
    double drOverR = opticalDistCoef[0]
                  + (rp2 * (opticalDistCoef[1] + (rp2 * opticalDistCoef[2])));
    // Compute first distorted point estimate, r
    double r = rp + (drOverR * rp);
    double r_prev, r2_prev;
    int iteration = 0;
    do {
      // Don't get in an end-less loop.  This algorithm should
      // converge quickly.  If not then we are probably way outside
      // of the focal plane.  Just set the distorted position to the
      // undistorted position. Also, make sure the focal plane is less
      // than 1km, it is unreasonable for it to grow larger than that.
      if (iteration >= 15 || r > 1E9) {
        drOverR = 0.0;
        break;
      }

      r_prev = r;
      r2_prev = r * r;

      // Compute new fractional distortion:
      drOverR = opticalDistCoef[0]
             + (r2_prev * (opticalDistCoef[1] + (r2_prev * opticalDistCoef[2])));

      // Compute new estimate of r
      r = rp + (drOverR * r_prev);
      iteration++;
    }
    while (fabs(r * (1 - drOverR) - rp) > desiredPrecision);

    outFocalPlaneX = inFocalPlaneX / (1.0 - drOverR);
    outFocalPlaneY = inFocalPlaneY / (1.0 - drOverR);
  }
}
