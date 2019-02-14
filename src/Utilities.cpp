#include "Utilities.h"

// Calculates a rotation matrix from Euler angles
void calculateRotationMatrixFromEuler(double euler[],
                                      double rotationMatrix[]) {
  double cos_a = cos(euler[0]);
  double sin_a = sin(euler[0]);
  double cos_b = cos(euler[1]);
  double sin_b = sin(euler[1]);
  double cos_c = cos(euler[2]);
  double sin_c = sin(euler[2]);

  rotationMatrix[0] = cos_b * cos_c;
  rotationMatrix[1] = -cos_a * sin_c + sin_a * sin_b * cos_c;
  rotationMatrix[2] = sin_a * sin_c + cos_a * sin_b * cos_c;
  rotationMatrix[3] = cos_b * sin_c;
  rotationMatrix[4] = cos_a * cos_c + sin_a * sin_b * sin_c;
  rotationMatrix[5] = -sin_a * cos_c + cos_a * sin_b * sin_c;
  rotationMatrix[6] = -sin_b;
  rotationMatrix[7] = sin_a * cos_b;
  rotationMatrix[8] = cos_a * cos_b;
}


// uses a quaternion to calclate a rotation matrix.
void calculateRotationMatrixFromQuaternions(double q[4], double rotationMatrix[9]) {
  double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;

  rotationMatrix[0] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  rotationMatrix[1] = 2 * (q[0] * q[1] - q[2] * q[3]);
  rotationMatrix[2] = 2 * (q[0] * q[2] + q[1] * q[3]);
  rotationMatrix[3] = 2 * (q[0] * q[1] + q[2] * q[3]);
  rotationMatrix[4] = -q[0] * q[0] + q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  rotationMatrix[5] = 2 * (q[1] * q[2] - q[0] * q[3]);
  rotationMatrix[6] = 2 * (q[0] * q[2] - q[1] * q[3]);
  rotationMatrix[7] = 2 * (q[1] * q[2] + q[0] * q[3]);
  rotationMatrix[8] = -q[0] * q[0] - q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
}

std::tuple<double, double> computeDistortedFocalPlaneCoordinates(
  const double& line,
  const double& sample,
  const double& sampleOrigin,
  const double& lineOrigin, 
  const double& sampleSumming,
  const double& startingSample,
  const double& lineOffset,
  const double iTransS[],
  const double iTransL[]) {
  double detSample = (sample - 1.0) * sampleSumming + startingSample;
  double m11 = iTransL[1];
  double m12 = iTransL[2];
  double m21 = iTransS[1];
  double m22 = iTransS[2];
  double t1 = line + lineOffset - lineOrigin - iTransL[0];
  double t2 = detSample - sampleOrigin - iTransS[0];
  double determinant = m11 * m22 - m12 * m21;
  double p11 = m11 / determinant;
  double p12 = -m12 / determinant;
  double p21 = -m21 / determinant;
  double p22 = m22 / determinant;

  double distortedLine = p11 * t1 + p12 * t2;
  double distortedSample = p21 * t1 + p22 * t2;
  return std::make_tuple(distortedLine, distortedSample);  
};

// Define imaging ray in image space (In other words, create a look vector in camera space)
void createCameraLookVector(
  const double& undistortedFocalPlaneX,
  const double& undistortedFocalPlaneY,
  const double& zDirection,
  const double& focalLength,
  const double& focalLengthBias,
  const double& halfSwath,
  double cameraLook[]) {
   cameraLook[0] = -undistortedFocalPlaneX * zDirection;
   cameraLook[1] = -undistortedFocalPlaneY * zDirection;
   cameraLook[2] = -focalLength * (1.0 - focalLengthBias / halfSwath);
   double magnitude = sqrt(cameraLook[0] * cameraLook[0]
                  + cameraLook[1] * cameraLook[1]
                  + cameraLook[2] * cameraLook[2]);
   cameraLook[0] /= magnitude;
   cameraLook[1] /= magnitude;
   cameraLook[2] /= magnitude;
}
