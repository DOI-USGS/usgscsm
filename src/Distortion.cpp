#include "Distortion.h"

/**
 * @description Jacobian of the distortion function. The Jacobian was computed
 * algebraically from the function described in the distortionFunction
 * method.
 *
 * @param x
 * @param y
 * @param odtX opticalDistCoef In X
 * @param odtY opticalDistCoef In Y
 *
 * @returns jacobian a jacobian vector of vectors as
                     [0][0]: xx, [0][1]: xy
                     [1][0]: yx, [1][1]: yy
 */

std::vector<std::vector<double>> distortionJacobian(double x, double y,
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

  std::vector<std::vector<double>> jacobian(2, std::vector<double>(2));

  jacobian[0][0] = 0;
  jacobian[0][1] = 0;
  jacobian[1][0] = 0;
  jacobian[1][1] = 0;

  for (int i = 0; i < 10; i++) {
    jacobian[0][0] = jacobian[0][0] + d_dx[i] * odtX[i];
    jacobian[0][1] = jacobian[0][1] + d_dy[i] * odtX[i];
    jacobian[1][0] = jacobian[1][0] + d_dx[i] * odtY[i];
    jacobian[1][1] = jacobian[1][1] + d_dy[i] * odtY[i];
  }

  return jacobian;
}

/**
 * @description Compute distorted focal plane (dx,dy) coordinate  given an undistorted focal
 * plane (ux,uy) coordinate. This uses the third order Taylor approximation to the
 * distortion model.
 *
 * @param ux Undistored x
 * @param uy Undistored y
 * @param odtX opticalDistCoef In X
 * @param odtY opticalDistCoef In Y
 *
 * @returns distortedPoint Newly adjusted focal plane coordinates as an x, y tuple
 */
std::tuple<double, double> distortionFunction(double ux, double uy,
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

  std::tuple<double, double> distortedPoint(0.0, 0.0);
  for (int i = 0; i < 10; i++) {
    distortedPoint = std::make_tuple(std::get<0>(distortedPoint) + f[i] * odtX[i],
                             std::get<1>(distortedPoint) + f[i] * odtY[i]);
  }

  return distortedPoint;
}

/**
 * @description Compute undistorted focal plane coordinate given a distorted
 * coordinate set and the distortion coefficients
 *
 * @param inFocalPlaneX Distorted x
 * @param inFocalPlaneY Distorted y
 * @param opticalDistCoef distortion coefficients
 *
 * @returns undistortedPoint Newly adjusted focal plane coordinates as an x, y tuple
 */
std::tuple<double, double> removeDistortion(double inFocalPlaneX, double inFocalPlaneY,
  const double opticalDistCoef[3], double tolerance) {
  double rr = inFocalPlaneX * inFocalPlaneX + inFocalPlaneY * inFocalPlaneY;
  std::tuple<double, double> undistortedPoint(inFocalPlaneX, inFocalPlaneY);

  if (rr > tolerance)
  {
    double dr = opticalDistCoef[0] + (rr * (opticalDistCoef[1] + rr * opticalDistCoef[2]));
    undistortedPoint = std::make_tuple(inFocalPlaneX * (1.0 - dr), inFocalPlaneY * (1.0 - dr));
  }

  return undistortedPoint;
}

/**
 * @description Compute undistorted focal plane coordinate given a distorted
 * focal plane coordinate. This method works by iteratively adding distortion
 * until the new distorted point, r, undistorts to within a tolerance of the
 * original point, rp.
 *
 * @param inFocalPlaneX Distorted x
 * @param inFocalPlaneY Distorted y
 * @param opticalDistCoef Distortion coefficients
 * @param desiredPrecision Convergence precision
 * @param tolerance Tolerance of r^2
 *
 * @returns undistortedPoint Newly adjusted focal plane coordinates as an x, y tuple
 */
std::tuple<double, double> invertDistortion(double inFocalPlaneX, double inFocalPlaneY,
  const double opticalDistCoef[3], double desiredPrecision, double tolerance) {
  double rp2 = (inFocalPlaneX * inFocalPlaneX) +
               (inFocalPlaneY * inFocalPlaneY);
  std::tuple<double, double> undistortedPoint;

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
    undistortedPoint = std::make_tuple(inFocalPlaneX / (1.0 - drOverR),
                                       inFocalPlaneY / (1.0 - drOverR));
  }
  return undistortedPoint;
}
