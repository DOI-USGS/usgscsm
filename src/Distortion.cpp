#include "Distortion.h"
#include "Utilities.h"

#include <Error.h>
#include <string>

// Jacobian for Transverse distortion
void transverseDistortionJacobian(double x, double y, double *jacobian,
                                  std::vector<double> const& opticalDistCoeffs) {
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

  jacobian[0] = 0;  // xx
  jacobian[1] = 0;  // xy
  jacobian[2] = 0;  // yx
  jacobian[3] = 0;  // yy

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
 * @description Compute distorted focal plane (dx,dy) coordinate  given an
 * undistorted focal plane (ux,uy) coordinate. This uses the third order Taylor
 * approximation to the distortion model.
 *
 * @param ux Undistored x
 * @param uy Undistored y
 * @param opticalDistCoeffs For both X and Y coefficients
 *
 * @returns distortedPoint Newly adjusted focal plane coordinates as an x, y
 * tuple
 */
void computeTransverseDistortion(double ux, double uy, double &dx, double &dy,
                                 std::vector<double> const& opticalDistCoeffs) {
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

// Compute distorted normalized focal plane coordinates given undistorted
// normalized coordinates. Use the radial-tangential distortion model with 5
// coefficients (k1, k2, k3 for radial distortion, and p1, p2 for tangential
// distortion). This was tested to give the same results as the OpenCV
// distortion model, by invoking the function cv::projectPoints() (with zero
// rotation, zero translation, and identity camera matrix). The parameters are
// stored in opticalDistCoeffs in the order: [k1, k2, p1, p2, k3]. 
// Reference: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
void computeRadTanDistortion(double ux, double uy, double &dx, double &dy,
                             std::vector<double> const& opticalDistCoeffs) {

  if (opticalDistCoeffs.size() != 5) {
    csm::Error::ErrorType errorType = csm::Error::INDEX_OUT_OF_RANGE;
    std::string message =
        "Distortion coefficients for the radtan distortion model must be of size 5, "
        "in the order k1, k2, p1, p2, k3. Got: " + std::to_string(opticalDistCoeffs.size());
    std::string function = "computeRadTanDistortion";
    throw csm::Error(errorType, message, function);
  }
  
  // Shorten notation
  double x = ux, y = uy; 
  double k1 = opticalDistCoeffs[0];
  double k2 = opticalDistCoeffs[1];
  double p1 = opticalDistCoeffs[2];
  double p2 = opticalDistCoeffs[3];
  double k3 = opticalDistCoeffs[4];
  
  double r2 = (x * x) + (y * y);
  
  dx = x * (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
     + (2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x));
       
  dy = y * (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
     + (p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y);
}

// Compute the jacobian for radtan distortion at given normalized coordinates
void radTanDistortionJacobian(double x, double y, double *jacobian,
                              std::vector<double> const& opticalDistCoeffs) {

  double k1 = opticalDistCoeffs[0];
  double k2 = opticalDistCoeffs[1];
  double p1 = opticalDistCoeffs[2];
  double p2 = opticalDistCoeffs[3];
  double k3 = opticalDistCoeffs[4];
  
  double r2 = x * x + y * y;
  double dr2dx = 2.0 * x;
  double dr2dy = 2.0 * y;

  // dfx / dx 
  jacobian[0] = (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)    
              + x * (k1 * dr2dx + k2 * dr2dx * 2.0 * r2 + k3 * dr2dx * 3.0 * r2 * r2)
              + 2.0 * p1 * y + p2 * (dr2dx + 4.0 * x);
  
  // dfx / dy
  jacobian[1] = x * (k1 * dr2dy + k2 * dr2dy * 2.0 * r2 + k3 * dr2dy * 3.0 * r2 * r2)
              + 2.0 * p1 * x  + p2 * dr2dy;
              
  // dfy / dx
  jacobian[2] = y * (k1 * dr2dx + k2 * dr2dx * 2.0 * r2 + k3 * dr2dx * 3.0 * r2 * r2)
              + (p1 * dr2dx + 2.0 * p2 * y);
  
  // dfy / dy
  jacobian[3] = (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
              + y * (k1 * dr2dy + k2 * dr2dy * 2.0 * r2 + k3 * dr2dy * 3.0 * r2 * r2)
              + p1 * (dr2dy + 4.0 * y) + 2.0 * p2 * x;
}

void removeDistortion(double dx, double dy, double &ux, double &uy,
                      std::vector<double> const& opticalDistCoeffs,
                      double focalLength,
                      DistortionType distortionType, const double tolerance) {
  ux = dx;
  uy = dy;

  switch (distortionType) {
    // Compute undistorted focal plane coordinate given a distorted
    // coordinate set and the distortion coefficients
    case RADIAL: {
      double rr = dx * dx + dy * dy;

      double dr = opticalDistCoeffs[0] +
                  (rr * (opticalDistCoeffs[1] + rr * opticalDistCoeffs[2]));

      ux = dx * (1.0 - dr);
      uy = dy * (1.0 - dr);
    } break;

    // Computes undistorted focal plane (x,y) coordinates given a distorted
    // focal plane (x,y) coordinate. The undistorted coordinates are solved for
    // using the Newton-Raphson method for root-finding if the
    // distortionFunction method is invoked.
    case TRANSVERSE: {
      // Solve the distortion equation using the Newton-Raphson method.
      // Set the error tolerance to about one millionth of a NAC pixel.
      // The maximum number of iterations of the Newton-Raphson method.
      newtonRaphson(dx, dy, ux, uy, opticalDistCoeffs, distortionType, tolerance, 
                    computeTransverseDistortion, transverseDistortionJacobian);
    } break;

    case KAGUYALISM: {
      // Apply distortion correction
      // see: SEL_TC_V01.TI and SEL_MI_V01.TI
      // r2 = x^2 + y^2
      //   Line-of-sight vector of pixel no. n can be expressed as below.

      //  Distortion coefficients information:
      //   INS<INSTID>_DISTORTION_COEF_X  = ( a0, a1, a2, a3)
      //   INS<INSTID>_DISTORTION_COEF_Y  = ( b0, b1, b2, b3),
      //
      // Distance r from the center:
      //   r = - (n - INS<INSTID>_CENTER) * INS<INSTID>_PIXEL_SIZE.

      // Line-of-sight vector v is calculated as
      //   v[X] = INS<INSTID>BORESIGHT[X]
      //          +a0 +a1*r +a2*r^2 +a3*r^3 ,
      //   v[Y] = INS<INSTID>BORESIGHT[Y]
      //           b0 +b1*r +b2*r^2 +b3*r^3
      //   v[Z] = INS<INSTID>BORESIGHT[Z] .

      // Coeffs should be [boresightX,x0,x1,x2,x3,boresightY,y0,y1,y2,y3]
      if (opticalDistCoeffs.size() != 10) {
        csm::Error::ErrorType errorType = csm::Error::INDEX_OUT_OF_RANGE;
        std::string message =
            "Distortion coefficients for Kaguya LISM must be of size 10, "
            "got: " +
            std::to_string(opticalDistCoeffs.size());
        std::string function = "removeDistortion";
        throw csm::Error(errorType, message, function);
      }

      double boresightX = opticalDistCoeffs[0];
      std::vector<double> odkx(opticalDistCoeffs.begin() + 1,
                               opticalDistCoeffs.begin() + 5);
      double boresightY = opticalDistCoeffs[5];
      std::vector<double> odky(opticalDistCoeffs.begin() + 6,
                               opticalDistCoeffs.begin() + 10);

      double r2 = dx * dx + dy * dy;
      double r = sqrt(r2);
      double r3 = r2 * r;

      double dr_x = odkx[0] + odkx[1] * r + odkx[2] * r2 + odkx[3] * r3;
      double dr_y = odky[0] + odky[1] * r + odky[2] * r2 + odky[3] * r3;

      ux += dr_x + boresightX;
      uy += dr_y + boresightY;
    } break;

    // The dawn distortion model is "reversed" from other distortion models so
    // the remove function iteratively computes undistorted coordinates based on
    // the distorted coordinates, rather than iteratively computing distorted
    // coordinates to undistorted coordinates.
    case DAWNFC: {
      double r2;
      int numAttempts = 1;
      bool done;

      /****************************************************************************
       * Pre-loop initializations
       ****************************************************************************/

      r2 = dy * dy + dx * dx;
      double guess_dx, guess_dy;
      double guess_ux, guess_uy;

      /****************************************************************************
       * Loop ...
       ****************************************************************************/
      do {
        guess_ux = dx / (1.0 + opticalDistCoeffs[0] * r2);
        guess_uy = dy / (1.0 + opticalDistCoeffs[0] * r2);

        r2 = guess_uy * guess_uy + guess_ux * guess_ux;

        guess_dx = guess_ux * (1.0 + opticalDistCoeffs[0] * r2);
        guess_dy = guess_uy * (1.0 + opticalDistCoeffs[0] * r2);

        done = false;

        if (abs(guess_dx - dx) < tolerance && abs(guess_dy - dy) < tolerance) {
          done = true;
        }

        /* Not converging so bomb */
        numAttempts++;
        if (numAttempts > 20) {
          std::cout << "Didn't converge" << std::endl;
          return;
        }
      } while (!done);

      /****************************************************************************
       * Success ...
       ****************************************************************************/

      ux = guess_ux;
      uy = guess_uy;
    } break;

    // LROLROCNAC
    case LROLROCNAC: {
      if (opticalDistCoeffs.size() != 1) {
        csm::Error::ErrorType errorType = csm::Error::INDEX_OUT_OF_RANGE;
        std::string message =
            "Distortion coefficients for LRO LROC NAC must be of size 1, "
            "current size: " +
            std::to_string(opticalDistCoeffs.size());
        std::string function = "removeDistortion";
        throw csm::Error(errorType, message, function);
      }

      double dk1 = opticalDistCoeffs[0];

      double den =
          1 +
          dk1 * dy * dy;  // r = dy*dy = distance from the focal plane center
      if (den == 0.0) {
        csm::Error::ErrorType errorType = csm::Error::ALGORITHM;
        std::string message =
            "Unable to remove distortion for LRO LROC NAC. Focal plane "
            "position " +
            std::to_string(dy);
        std::string function = "removeDistortion";
        throw csm::Error(errorType, message, function);
      }

      ux = dx;
      uy = dy / den;

      return;
    } break;

    // Compute undistorted focal plane coordinate given a distorted
    // coordinate set and the distortion coefficients along with an
    // x, and y offset as the fourth and fifth element
    case CAHVOR:
    {
      double shiftedDx = dx - opticalDistCoeffs[3];
      double shiftedDy = dy - opticalDistCoeffs[4];
      double rr = shiftedDx * shiftedDx + shiftedDy * shiftedDy;

      double dr = opticalDistCoeffs[0] +
                  (rr * (opticalDistCoeffs[1] + rr * opticalDistCoeffs[2]));

      ux = shiftedDx * (1.0 - dr);
      uy = shiftedDy * (1.0 - dr);
      ux += opticalDistCoeffs[3];
      uy += opticalDistCoeffs[4];
    }
    break;
    
    // Compute undistorted focal plane coordinate given distorted coordinates
    // with the radtan model. See computeRadTanDistortion() for more details.
    case RADTAN:
    {
      dx /= focalLength; dy /= focalLength; // Find normalized coordinates
      newtonRaphson(dx, dy, ux, uy, opticalDistCoeffs, distortionType, tolerance, 
                    computeRadTanDistortion, radTanDistortionJacobian);
      ux *= focalLength; uy *= focalLength; // Convert back to pixel coordinates
      
    }
    break;
    
  }
}

void applyDistortion(double ux, double uy, double &dx, double &dy,
                     std::vector<double> const& opticalDistCoeffs,
                     double focalLength,
                     DistortionType distortionType,
                     const double desiredPrecision, const double tolerance) {
  dx = ux;
  dy = uy;

  switch (distortionType) {
    // Compute distorted focal plane coordinate given undistorted
    // focal plane coordinates. This case works by iteratively adding distortion
    // until the new distorted point, r, undistorts to within a tolerance of the
    // original point, rp.
    case RADIAL: {
      double rp2 = (ux * ux) + (uy * uy);

      double rp = sqrt(rp2);
      // Compute first fractional distortion using rp
      double drOverR =
          opticalDistCoeffs[0] +
          (rp2 * (opticalDistCoeffs[1] + (rp2 * opticalDistCoeffs[2])));

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
        if (iteration >= 20 || r > 1E9) {
          drOverR = 0.0;
          break;
        }

        r_prev = r;
        r2_prev = r * r;

        // Compute new fractional distortion:
        drOverR = opticalDistCoeffs[0] +
                  (r2_prev *
                   (opticalDistCoeffs[1] + (r2_prev * opticalDistCoeffs[2])));

        // Compute new estimate of r
        r = rp + (drOverR * r_prev);
        iteration++;
      } while (fabs(r - r_prev) > desiredPrecision);

      dx = ux / (1.0 - drOverR);
      dy = uy / (1.0 - drOverR);
      
    } break;
    case TRANSVERSE: {
      computeTransverseDistortion(ux, uy, dx, dy, opticalDistCoeffs);
    } break;

    case KAGUYALISM: {
      if (opticalDistCoeffs.size() != 10) {
        csm::Error::ErrorType errorType = csm::Error::INDEX_OUT_OF_RANGE;
        std::string message =
            "Distortion coefficients for Kaguya LISM must be of size 10, "
            "got: " +
            std::to_string(opticalDistCoeffs.size());
        std::string function = "applyDistortion";
        throw csm::Error(errorType, message, function);
      }

      double boresightX = opticalDistCoeffs[0];
      std::vector<double> odkx(opticalDistCoeffs.begin() + 1,
                               opticalDistCoeffs.begin() + 5);
      double boresightY = opticalDistCoeffs[5];
      std::vector<double> odky(opticalDistCoeffs.begin() + 6,
                               opticalDistCoeffs.begin() + 10);

      double xt = ux - boresightX;
      double yt = uy - boresightY;

      double xx, yy, r, rr, rrr, dr_x, dr_y;
      double xdistortion, ydistortion;
      double xdistorted, ydistorted;
      double xprevious, yprevious;

      xprevious = 1000000.0;
      yprevious = 1000000.0;

      double tolerance = 0.000001;
      bool bConverged = false;

      // Iterating to introduce distortion...
      // We stop when the difference between distorted coordinates
      // in successive iterations is below the given tolerance
      for (int i = 0; i < 50; i++) {
        xx = xt * xt;
        yy = yt * yt;
        rr = xx + yy;
        r = sqrt(rr);
        rrr = rr * r;

        // Radial distortion
        // dr is the radial distortion contribution
        dr_x = odkx[0] + odkx[1] * r + odkx[2] * rr + odkx[3] * rrr;
        dr_y = odky[0] + odky[1] * r + odky[2] * rr + odky[3] * rrr;

        // Distortion at the current point location
        xdistortion = dr_x;
        ydistortion = dr_y;

        // updated image coordinates
        xt = ux - xdistortion - boresightX;
        yt = uy - ydistortion - boresightY;

        // distorted point corrected for principal point
        xdistorted = xt;
        ydistorted = yt;

        // check for convergence
        if ((fabs(xt - xprevious) < tolerance) &&
            (fabs(yt - yprevious) < tolerance)) {
          bConverged = true;
          break;
        }

        xprevious = xt;
        yprevious = yt;
      }

      if (bConverged) {
        dx = xdistorted;
        dy = ydistorted;
      }
    } break;

    // The dawn distortion model is "reversed" from other distortion models. 
    // The apply function computes distorted coordinates as a
    // function of undistorted coordinates.
    case DAWNFC: {
      double r2;

      r2 = ux * ux + uy * uy;

      dx = ux * (1.0 + opticalDistCoeffs[0] * r2);
      dy = uy * (1.0 + opticalDistCoeffs[0] * r2);
    } break;

    // The LRO LROC NAC distortion model uses an iterative approach to go from
    // undistorted x,y to distorted x,y
    // Algorithm adapted from ISIS3 LRONarrowAngleDistortionMap.cpp
    case LROLROCNAC: {
      double yt = uy;

      double rr, dr;
      double ydistorted;
      double yprevious = 1000000.0;
      double tolerance = 1.0e-10;

      bool bConverged = false;

      if (opticalDistCoeffs.size() != 1) {
        csm::Error::ErrorType errorType = csm::Error::INDEX_OUT_OF_RANGE;
        std::string message =
            "Distortion coefficients for LRO LROC NAC must be of size 1, "
            "current size: " +
            std::to_string(opticalDistCoeffs.size());
        std::string function = "applyDistortion";
        throw csm::Error(errorType, message, function);
      }

      double dk1 = opticalDistCoeffs[0];

      // Owing to the odd distortion model employed in this sensor if |y| is >
      // 116.881145553046 then there is no root to find.  Further, the greatest
      // y that any measure on the sensor will actually distort to is less
      // than 20.  Thus, if any distorted measure is greater that that skip the
      // iterations.  The points isn't in the cube, and exactly how far outside
      // the cube is irrelevant.  Just let the camera model know its not in the
      // cube....
      if (fabs(uy) > 40) {  // if the point is way off the image.....
        dx = ux;
        dy = uy;
        return;
      }

      // iterating to introduce distortion (in sample only)...
      // we stop when the difference between distorted coordinate
      // in successive iterations is at or below the given tolerance
      for (int i = 0; i < 50; i++) {
        rr = yt * yt;

        //  dr is the radial distortion contribution
        dr = 1.0 + dk1 * rr;

        // distortion at the current sample location
        yt = uy * dr;

        // distorted sample
        ydistorted = yt;

        if (yt < -1e121)  // debug
          break;          // debug

        // check for convergence
        if (fabs(yt - yprevious) <= tolerance) {
          bConverged = true;
          break;
        }

        yprevious = yt;
      }

      if (bConverged) {
        dx = ux;
        dy = ydistorted;
      }

      return;
    } break;

    // Compute undistorted focal plane coordinate given a distorted
    // focal plane coordinate. This case works by iteratively adding distortion
    // until the new distorted point, r, undistorts to within a tolerance of the
    // original point, rp. Also applies an initial offset with an
    // x, and y offset as the fourth and fifth element
    // This is untested manually
    case CAHVOR:
    {
      double shiftedUx = ux - opticalDistCoeffs[3];
      double shiftedUy = uy - opticalDistCoeffs[4];
      double rp2 = (ux * ux) + (uy * uy);
      double rp = sqrt(rp2);
      // Compute first fractional distortion using rp
      double drOverR =
          opticalDistCoeffs[0] +
          (rp2 * (opticalDistCoeffs[1] + (rp2 * opticalDistCoeffs[2])));

      // Compute first distorted point estimate, r
      double r = rp + (drOverR * rp);
      double r_prev, r2_prev;
      int iteration = 0;

      do
      {
        // Don't get in an end-less loop.  This algorithm should
        // converge quickly.  If not then we are probably way outside
        // of the focal plane.  Just set the distorted position to the
        // undistorted position. Also, make sure the focal plane is less
        // than 1km, it is unreasonable for it to grow larger than that.
        if (iteration >= 20 || r > 1E9)
        {
          drOverR = 0.0;
          break;
        }

        r_prev = r;
        r2_prev = r * r;

        // Compute new fractional distortion:
        drOverR = opticalDistCoeffs[0] +
                  (r2_prev *
                    (opticalDistCoeffs[1] + (r2_prev * opticalDistCoeffs[2])));

        // Compute new estimate of r
        r = rp + (drOverR * r_prev);
        iteration++;
      } while (fabs(r - r_prev) > desiredPrecision);

      dx = shiftedUx / (1.0 - drOverR);
      dy = shiftedUy / (1.0 - drOverR);
      dx += opticalDistCoeffs[3];
      dy += opticalDistCoeffs[4];
    
    }
    break;
    
    // Compute distorted focal plane coordinate given undistorted coordinates
    // with the RADTAN model. See computeRadTanDistortion() for more details.
    case RADTAN:
    {
      ux /= focalLength; uy /= focalLength; // Find normalized coordinates
      computeRadTanDistortion(ux, uy, dx, dy, opticalDistCoeffs);  
      dx *= focalLength; dy *= focalLength; // Convert back to pixel coordinates
    }  
    break;
    
  }
}
