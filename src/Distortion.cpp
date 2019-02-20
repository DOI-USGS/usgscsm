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

void distortionJacobian(double x, double y, double *jacobian,
                        const std::vector<double> opticalDistCoeffs) {

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

  jacobian[0] = 0; // xx
  jacobian[1] = 0; // xy
  jacobian[2] = 0; // yx
  jacobian[3] = 0; // yy

  int xPointer = 0;
  int yPointer = opticalDistCoeffs.size() / 2;

  for (int i = 0; i < 10; xPointer++, yPointer++, i++) {
    jacobian[0] = jacobian[0] + d_dx[i] * opticalDistCoeffs[xPointer];
    jacobian[1] = jacobian[1] + d_dy[i] * opticalDistCoeffs[xPointer];
    jacobian[2] = jacobian[2] + d_dx[i] * opticalDistCoeffs[yPointer];
    jacobian[3] = jacobian[3] + d_dy[i] * opticalDistCoeffs[yPointer];
  }
}

/**
 * @description Compute distorted focal plane (dx,dy) coordinate  given an undistorted focal
 * plane (ux,uy) coordinate. This uses the third order Taylor approximation to the
 * distortion model.
 *
 * @param ux Undistored x
 * @param uy Undistored y
 * @param opticalDistCoeffs For both X and Y coefficients
 *
 * @returns distortedPoint Newly adjusted focal plane coordinates as an x, y tuple
 */
void distortionFunction(double ux, double uy, double &dx, double &dy,
                        const std::vector<double> opticalDistCoeffs) {

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

  int xPointer = 0;
  int yPointer = opticalDistCoeffs.size() / 2;

  dx = 0.0;
  dy = 0.0;

  for (int i = 0; i < 10; xPointer++, yPointer++, i++) {
    dx = dx + f[i] * opticalDistCoeffs[xPointer];
    dy = dy + f[i] * opticalDistCoeffs[yPointer];
  }
}

void removeDistortion(double dx, double dy, double &ux, double &uy,
                      const std::vector<double> opticalDistCoeffs,
                      DistortionType distortionType,
                      double desiredPrecision) {
  ux = dx;
  uy = dy;

  switch (distortionType) {
    // Compute undistorted focal plane coordinate given a distorted
    // coordinate set and the distortion coefficients
    case RADIAL: {
      double rr = dx * dx + dy * dy;
      double tolerance = 1.0E-6;

      if (rr > tolerance)
      {
        double dr = opticalDistCoeffs[0] + (rr * (opticalDistCoeffs[1] + rr * opticalDistCoeffs[2]));
        ux = dx * (1.0 - dr);
        uy = dy * (1.0 - dr);
      }
    }
    break;
    // Computes undistorted focal plane (x,y) coordinates given a distorted focal plane (x,y)
    // coordinate. The undistorted coordinates are solved for using the Newton-Raphson
    // method for root-finding if the distortionFunction method is invoked.
    case TRANSVERSE: {
      // Solve the distortion equation using the Newton-Raphson method.
      // Set the error tolerance to about one millionth of a NAC pixel.
      const double tol = 1.4E-5;

      // The maximum number of iterations of the Newton-Raphson method.
      const int maxTries = 60;

      double x;
      double y;
      double fx;
      double fy;
      double jacobian[4];

      // Initial guess at the root
      x = dx;
      y = dy;

      distortionFunction(x, y, fx, fy, opticalDistCoeffs);

      for (int count = 1; ((fabs(fx) +fabs(fy)) > tol) && (count < maxTries); count++) {

        distortionFunction(x, y, fx, fy, opticalDistCoeffs);

        fx = dx - fx;
        fy = dy - fy;

        distortionJacobian(x, y, jacobian, opticalDistCoeffs);

        // Jxx * Jyy - Jxy * Jyx
        double determinant = jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2];
        if (fabs(determinant) < 1E-6) {
          ux = x;
          uy = y;
          //
          // Near-zero determinant. Add error handling here.
          //
          //-- Just break out and return with no convergence
          return;
        }

        x = x + (jacobian[3] * fx - jacobian[1] * fy) / determinant;
        y = y + (jacobian[0] * fy - jacobian[2] * fx) / determinant;
      }

      if ((fabs(fx) + fabs(fy)) <= tol) {
        // The method converged to a root.
        ux = x;
        uy = y;

        return;
      }
      // Otherwise method did not converge to a root within the maximum
      // number of iterations
    }
    break;
    // Compute undistorted focal plane coordinate given a distorted
    // focal plane coordinate. This case works by iteratively adding distortion
    // until the new distorted point, r, undistorts to within a tolerance of the
    // original point, rp.
    case INVERSE_RADIAL: {
      const double tol = 1.0E-6;

      double rp2 = (dx * dx) + (dy * dy);

      if (rp2 > tol) {
        double rp = sqrt(rp2);
        // Compute first fractional distortion using rp
        double drOverR = opticalDistCoeffs[0]
                      + (rp2 * (opticalDistCoeffs[1] + (rp2 * opticalDistCoeffs[2])));
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
          drOverR = opticalDistCoeffs[0]
                 + (r2_prev * (opticalDistCoeffs[1] + (r2_prev * opticalDistCoeffs[2])));

          // Compute new estimate of r
          r = rp + (drOverR * r_prev);
          iteration++;
        }
        while (fabs(r * (1 - drOverR) - rp) > desiredPrecision);
        ux = dx / (1.0 - drOverR);
        uy = dy / (1.0 - drOverR);
      }
    }
    break;
  }
}
