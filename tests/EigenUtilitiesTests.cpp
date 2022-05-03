#include "EigenUtilities.h"

#include <gtest/gtest.h>
#include <cmath>

// See if fitting a projective transform to points which were created
// with such a transform to start with can recover it. 
TEST(EigenUtilitiesTests, createProjectiveApproximation) {

  // Create a projective transform with coeffs stored in a vector.
  // The exact values are not important.
  std::vector<double> u(14);
  for (size_t it = 0; it < u.size(); it++) 
    u[it] = sin(it); // why not
  
  // Create inputs and outputs
  std::vector<csm::ImageCoord> imagePts;
  std::vector<csm::EcefCoord>  groundPts;
  for (int r = 0; r < 5; r++) {
    for (int c = 0; c < 5; c++) {

      // Create some 3D points. The precise values are not important
      // as long as they are well-distributed
      double x = c, y = r, z = 0.3 * x + 2 * y + 0.04 * x * y;
      
      // Find corresponding 2D points
      double p = (u[0] + u[1] * x + u[2] * y + u[3]  * z) / (1 + u[4]  * x + u[5]  * y + u[6]  * z);
      double q = (u[7] + u[8] * x + u[9] * y + u[10] * z) / (1 + u[11] * x + u[12] * y + u[13] * z);

      imagePts.push_back(csm::ImageCoord(p, q));
      groundPts.push_back(csm::EcefCoord(x, y, z));
    }
  }

  // Find the transform
  std::vector<double> transformCoeffs;
  usgscsm::computeBestFitProjectiveTransform(imagePts, groundPts, transformCoeffs);

  // Verify that we recover the transform
  double err = 0.0;
  for (size_t it = 0; it < u.size(); it++)
    err = std::max(err, std::abs(u[it] - transformCoeffs[it]));
  
  EXPECT_NEAR(0.0, err, 1e-12);
}

