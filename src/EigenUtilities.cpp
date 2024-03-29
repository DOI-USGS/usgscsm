#include "EigenUtilities.h"

#include <Error.h>
#include <Eigen/Dense>

#include <iostream>

// Keep these utilities separate as using Eigen in an existing source
// file results in a 50% increase in compilation time.

// Compute the best-fitting projective transform that maps a set of 3D points
// to 2D points.
// A best-fit linear transform could be created by eliminating the denominators below.
void usgscsm::computeBestFitProjectiveTransform(std::vector<csm::ImageCoord> const& imagePts,
                                                std::vector<csm::EcefCoord>  const& groundPts,
                                                std::vector<double> & transformCoeffs) {
  
  if (imagePts.size() != groundPts.size()) 
    throw csm::Error(csm::Error::INVALID_USE,
                     "The number of inputs and outputs must agree.",
                     "computeBestFitProjectiveTransform");
  
  int numPts = imagePts.size();
  if (numPts < 8)
    throw csm::Error(csm::Error::INVALID_USE,
                     "At least 8 points are needed to fit a 3D-to-2D projective transform. "
                     "Ideally more are preferred.",
                     "computeBestFitProjectiveTransform");

  int numMatRows = 2 * numPts; // there exist x and y coords for each point
  int numMatCols = 14; // Number of variables in the projective transform
  
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(numMatRows, numMatCols);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(numMatRows);
  
  for (int it = 0; it < numPts; it++) {

    double x = groundPts[it].x, y = groundPts[it].y, z = groundPts[it].z;
    double r = imagePts[it].line, c = imagePts[it].samp;

    // If the solution coefficients are u0, u1, ..., must have:
 
    // (u0 + u1 * x + u2 * y + u3  * z) / (1 + u4  * x + u5  * y + u6  * z) = r
    // (u7 + u8 * x + u9 * y + u10 * z) / (1 + u11 * x + u12 * y + u13 * z) = c

    // Multiply by the denominators. Get a linear regression in the coefficients.
    
    M.row(2 * it + 0) << 1, x, y, z, -x * r, -y * r, -z * r, 0, 0, 0, 0, 0, 0, 0;
    M.row(2 * it + 1) << 0, 0, 0, 0, 0, 0, 0, 1, x, y, z, -x * c, -y * c, -z * c;

    b[2 * it + 0] = r;
    b[2 * it + 1] = c;
  }

  // Solve the over-determined system, per:
  // https://eigen.tuxfamily.org/dox/group__LeastSquares.html
  Eigen::VectorXd u = M.colPivHouseholderQr().solve(b);

  // Copy back the result to a standard vector (to avoid using Eigen too much as
  // that is slow to compile).
  transformCoeffs.resize(numMatCols);
  for (int it = 0; it < numMatCols; it++)
    transformCoeffs[it] = u[it];

  return;
} 
