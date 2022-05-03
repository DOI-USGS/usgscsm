#ifndef INCLUDE_USGSCSM_EIGENUTILITIES_H_
#define INCLUDE_USGSCSM_EIGENUTILITIES_H_

// Do not include Eigen header files here as those will slow down the compilation
// whereever this header file is included.

#include <csm.h>

namespace usgscsm {
  
// Compute the best-fitting projective transform that maps ground
// points to image points.
void computeBestFitProjectiveTransform(std::vector<csm::ImageCoord> const& imagePts,
                                       std::vector<csm::EcefCoord>  const& groundPts,
                                       std::vector<double> & transformCoeffs);
}
#endif  // INCLUDE_USGSCSM_EIGENUTILITIES_H_
