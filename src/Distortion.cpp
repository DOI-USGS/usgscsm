#include "Distortion.h"

/**
 * @brief Compute undistorted focal plane x/y.
 *
 * Computes undistorted focal plane (x,y) coordinates given a distorted focal plane (x,y)
 * coordinate. The undistorted coordinates are solved for using the Newton-Raphson
 * method for root-finding if the distortionFunction method is invoked.
 *
 * @param dx distorted focal plane x in millimeters
 * @param dy distorted focal plane y in millimeters
 * @param undistortedX The undistorted x coordinate, in millimeters.
 * @param undistortedY The undistorted y coordinate, in millimeters.
 *
 * @return if the conversion was successful
 * @todo Review the tolerance and maximum iterations of the root-
 *       finding algorithm.
 * @todo Review the handling of non-convergence of the root-finding
 *       algorithm.
 * @todo Add error handling for near-zero determinant.
*/
void removeTransverseDistortion(double dx, double dy, double ux, double uy,
                        const std::vector<double> &odtX, const std::vector<double> &odtY) {
  // Solve the distortion equation using the Newton-Raphson method.
  // Set the error tolerance to about one millionth of a NAC pixel.
  const double tol = 1.4E-5;

  // The maximum number of iterations of the Newton-Raphson method.
  const int maxTries = 60;

  double x;
  double y;
  double fx;
  double fy;

  // Initial guess at the root
  x = dx;
  y = dy;

  distortionFunction(x, y, fx, fy, odtX, odtY);

  for (int count = 1; ((fabs(fx)) +fabs(fy))) > tol) && (count < maxTries); count++) {

    distortedPoint = distortionFunction(x, y, fx, fy, odtX, odtY);

    fx = dx - fx;
    fy = dy - fy;

    double jacobian[4];

    distortionJacobian(x, y, jacobian, odtX, odtY);

    // Jxx * Jyy - Jxy * Jyx
    double determinant = jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2];
    if (fabs(determinant) < 1E-6) {
      ux = x;
      uy = y;
      //
      // Near-zero determinant. Add error handling here.
      //
      //-- Just break out and return with no convergence
    }

    x = x + (jacobian[3] * fx - jacobian[1] * fy) / determinant;
    y = y + (jacobian[0] * fy - jacobian[2] * fx) / determinant;
  }

  if ( (fabs(std::get<0>(distortedPoint)) + fabs(std::get<1>(distortedPoint))) <= tol) {
    // The method converged to a root.
    ux = x;
    uy = y;
  }
  // Otherwise method did not converge to a root within the maximum
  // number of iterations. Return with no distortion.
  ux = dx;
  uy = dy;
}

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

void distortionJacobian(double x, double y, double &jacobian,
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

  jacobian[0] = 0; // 0
  jacobian[1] = 0; // 1
  jacobian[2] = 0; // 2
  jacobian[3] = 0; // 3

  for (int i = 0; i < 10; i++) {
    jacobian[0] = jacobian[0] + d_dx[i] * odtX[i];
    jacobian[1] = jacobian[1] + d_dy[i] * odtX[i];
    jacobian[2] = jacobian[2] + d_dx[i] * odtY[i];
    jacobian[3] = jacobian[3] + d_dy[i] * odtY[i];
  }
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
void distortionFunction(double ux, double uy, double dx, double dy,
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

  for (int i = 0; i < 10; i++) {
    x = x + f[i] * odtX[i];
    y = y + f[i] * odtY[i]);
  }
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
void removeRadialDistortion(double dx, double dy, double ux, double uy,
                            std::vector<double> radialDistortionCoeffs) {
 double rr = dx * dx + dy * dy;

 double radialCoeffSum = 0;
 std::tuple<double, double> undistortedPoint;

 for (int i = 1; i <= radialDistortionCoeffs.size(); i++) {
   radialCoeffSum += radialDistortionCoeffs[i - 1] * (pow(rr, i * 2));
 }

 ux = dx + dx * radialCoeffSum;
 uy = dy + dy * radialCoeffSum;

 return undistortedPoint;
}

void removeDistortion(double inFocalPlaneX, double inFocalPlaneY,
                      double &outFocalPlaneX, double &outFocalPlaneY,
                      const double opticalDistCoef[3], double tolerance) {
  double rr = inFocalPlaneX * inFocalPlaneX + inFocalPlaneY * inFocalPlaneY;
  double dr = 0;

  if (rr > tolerance)
  {
    dr = opticalDistCoef[0] + (rr * (opticalDistCoef[1] + rr * opticalDistCoef[2]));
  }

  outFocalPlaneX = inFocalPlaneX * (1.0 - dr);
  outFocalPlaneY = inFocalPlaneY * (1.0 - dr);
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
void invertDistortion(double inFocalPlaneX, double inFocalPlaneY,
                      double outFocalPlaneX, double outFocalPlaneY
                      const std::vector<double> opticalDistCoef, double desiredPrecision, double tolerance) {
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
    outFocalPlaneY = inFocalPlaneY / (1.0 - drOverR));
  }
  outFocalPlaneX = inFocalPlaneX;
  outFocalPlaneY = outFocalPlaneY;
}
