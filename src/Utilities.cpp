#include "Utilities.h"

#include <Error.h>
#include <cmath>
#include <stack>
#include <stdexcept>
#include <utility>
#include <ctime>
#include <iostream>
#include <fstream>

#include "ale/Distortion.h"

using json = nlohmann::json;

/**
 * @description Calculates a rotation matrix from Euler angles. This function takes
 * in Euler angles (yaw, pitch, and roll) and calculates the corresponding rotation 
 * matrix. Euler angles are expected in radians and in the order of rotation about 
 * the Z-axis (yaw), Y-axis (pitch), and X-axis (roll). The resulting rotation matrix
 * is in the row-major order and represents the rotation in three-dimensional space 
 * according to the right-hand rule.
 * 
 * @param euler An array of three doubles representing the Euler angles in radians:
 * - euler[0]: Rotation angle about the Z-axis (yaw).
 * - euler[1]: Rotation angle about the Y-axis (pitch).
 * - euler[2]: Rotation angle about the X-axis (roll).
 * @param rotationMatrix An array of nine doubles where the resulting rotation matrix 
 * will be stored. The matrix is stored in row-major order as follows:
 * [ R[0], R[1], R[2],
 *   R[3], R[4], R[5],
 *   R[6], R[7], R[8] ]
 * where R[] represents the elements of the rotation matrix.
 */
void calculateRotationMatrixFromEuler(double euler[], double rotationMatrix[]) {
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

/**
 * @description Calculates a rotation matrix from a quaternion representation. This 
 * function converts a given quaternion into its corresponding rotation matrix. 
 * Quaternions provide a compact and efficient way to encode the orientation of an 
 * object in three-dimensional space. The input quaternion should be normalized or 
 * will be normalized within the function to ensure the rotation matrix is orthogonal 
 * and properly represents a rotation. The resulting rotation matrix is in row-major 
 * order and reflects the rotation in 3D space as described by the quaternion.
 * 
 * @param q An array of four doubles representing the quaternion components in the 
 * order: q[0] (w), q[1] (x), q[2] (y), q[3] (z). The quaternion should represent a 
 * unit quaternion to correctly encode a rotation.
 * @param rotationMatrix An array of nine doubles where the resulting rotation matrix 
 * will be stored. The matrix is stored in row-major order as follows:
 * [ R[0], R[1], R[2],
 *   R[3], R[4], R[5],
 *   R[6], R[7], R[8] ]
 * where R[] represents the elements of the rotation matrix. This matrix represents 
 * the rotation in three-dimensional space according to the quaternion provided.
 */
void calculateRotationMatrixFromQuaternions(double q[4],
                                            double rotationMatrix[9]) {
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

/**
 * @description Computes the distorted focal plane coordinates for a given image pixel. 
 * This function calculates the position of a pixel in the distorted focal plane given its 
 * position in the CCD (Charge-Coupled Device) image sensor coordinate system. The 
 * calculation accounts for the CCD coordinate system's origin, pixel summing, and the 
 * starting position of the CCD readout. It uses transformation matrices from the focal 
 * plane to CCD samples and lines to perform the conversion. The result is the distorted 
 * (x, y) coordinates in the focal plane.
 * 
 * @param line The line (y-coordinate) of the pixel in the CCD coordinate system.
 * @param sample The sample (x-coordinate) of the pixel in the CCD coordinate system.
 * @param sampleOrigin The origin of the CCD coordinate system relative to the top left of 
 * the CCD in the x-direction.
 * @param lineOrigin The origin of the CCD coordinate system relative to the top left of 
 * the CCD in the y-direction.
 * @param sampleSumming The summing mode of samples (x-direction), indicating how many 
 * physical pixels are combined into a single image pixel.
 * @param lineSumming The summing mode of lines (y-direction), indicating how many physical 
 * pixels are combined into a single image pixel.
 * @param startingSample The first CCD sample (x-coordinate) corresponding to the start of 
 * the image.
 * @param startingLine The first CCD line (y-coordinate) corresponding to the start of the 
 * image.
 * @param iTransS An array of three doubles representing the transformation from focal plane 
 * to CCD samples. It includes the offset and scaling factors.
 * @param iTransL An array of three doubles representing the transformation from focal plane 
 * to CCD lines. It includes the offset and scaling factors.
 * @param distortedX Reference to a double where the x-coordinate of the distorted focal 
 * plane position will be stored.
 * @param distortedY Reference to a double where the y-coordinate of the distorted focal 
 * plane position will be stored.
 */
void computeDistortedFocalPlaneCoordinates(
    const double &line, const double &sample, const double &sampleOrigin,
    const double &lineOrigin, const double &sampleSumming,
    const double &lineSumming, const double &startingSample,
    const double &startingLine, const double iTransS[], const double iTransL[],
    double &distortedX, double &distortedY) {
  double detSample = sample * sampleSumming + startingSample;
  double detLine = line * lineSumming + startingLine;
  double m11 = iTransL[1];
  double m12 = iTransL[2];
  double m21 = iTransS[1];
  double m22 = iTransS[2];
  double t1 = detLine - lineOrigin - iTransL[0];
  double t2 = detSample - sampleOrigin - iTransS[0];
  double determinant = m11 * m22 - m12 * m21;
  double p11 = m22 / determinant;
  double p12 = -m12 / determinant;
  double p21 = -m21 / determinant;
  double p22 = m11 / determinant;

  distortedX = p11 * t1 + p12 * t2;
  distortedY = p21 * t1 + p22 * t2;
}

/**
 * @description Computes the de-jittered pixel coordinates given jittered image
 * coordinates, a set of jitter coefficients for both line and sample 
 * dimensions, and line exposure times. This function is designed to correct 
 * for jitter in images, typically caused by motion or instability during image 
 * capture, especially in systems with a rolling shutter. The jitter 
 * coefficients should be provided in descending order of power, with the 
 * highest power coefficient first and no constant term. For instance, 
 * coefficients {1, 2, 3} would correspond to the polynomial 1*t^3 + 2*t^2 + 3*t, 
 * where t is the time variable.
 * 
 * @param line The original line (y-coordinate) of the jittered pixel.
 * @param sample The original sample (x-coordinate) of the jittered pixel.
 * @param lineJitterCoeffs A vector of doubles representing the jitter 
 * coefficients for the line dimension.
 * @param sampleJitterCoeffs A vector of doubles representing the jitter 
 * coefficients for the sample dimension.
 * @param lineTimes A vector of doubles representing the exposure times for each 
 * line, used to calculate the actual time at which each line was exposed. This is 
 * particularly relevant for rolling shutter cameras where each line of the sensor 
 * is exposed at a slightly different time.
 * @param dejitteredLine Reference to a double where the de-jittered line 
 * coordinate will be stored.
 * @param dejitteredSample Reference to a double where the de-jittered sample 
 * coordinate will be stored.
 * 
 * @throws csm::Error If the sizes of the jitter coefficient vectors do not match or 
 * if the lineTimes vector is empty.
 */
void removeJitter(
    const double &line, const double &sample,
    const std::vector<double> lineJitterCoeffs, const std::vector<double> sampleJitterCoeffs,
    const std::vector<double> lineTimes,
    double &dejitteredLine, double &dejitteredSample) {
  // Check input
  if (lineJitterCoeffs.size() != sampleJitterCoeffs.size() ||
      lineTimes.size() == 0) {
    throw csm::Error(
        csm::Error::INDEX_OUT_OF_RANGE,
        "Jitter coefficient vectors must be the same size.",
        "removeJitter");
  }
  if (lineTimes.size() == 0) {
    throw csm::Error(
        csm::Error::INDEX_OUT_OF_RANGE,
        "Line exposure times must be non-empty.",
        "removeJitter");
  }
  double lineJitter = 0;
  double sampleJitter = 0;
  // Bound line index to the vector of line exposure times;
  double time = lineTimes[std::max(std::min((int)std::round(line), (int)lineTimes.size()), 1) - 1];
  for (unsigned int n = 0; n < lineJitterCoeffs.size(); n++) {
    double timeTerm = pow(time, lineJitterCoeffs.size() - n);
    lineJitter += lineJitterCoeffs[n] * timeTerm;
    sampleJitter += sampleJitterCoeffs[n] * timeTerm;
  }
  dejitteredLine = line - lineJitter;
  dejitteredSample = sample - sampleJitter;

  return;
}

/**
 * @description Adds jitter to a given pixel coordinate based on specified 
 * jitter coefficients and line exposure times. This function simulates the 
 * effect of jitter on image coordinates by applying a reverse process of the 
 * jitter removal algorithm. It iteratively applies jitter until the change in 
 * the jittered coordinates falls below a specified tolerance or the maximum 
 * number of iterations is reached. This approach is useful for testing or 
 * simulating the impact of jitter on image data, especially in imaging systems 
 * with a rolling shutter.
 *
 * @param line The original line (y-coordinate) of the pixel before adding 
 * jitter.
 * @param sample The original sample (x-coordinate) of the pixel before adding 
 * jitter.
 * @param tolerance The tolerance within which the difference between the 
 * jittered and original coordinates must fall to consider the jittering 
 * process complete.
 * @param maxIts The maximum number of iterations to attempt adding jitter. 
 * This prevents infinite loops in cases where the specified tolerance cannot 
 * be met.
 * @param lineJitterCoeffs A vector of doubles representing the coefficients 
 * for jitter in the line dimension. These coefficients define the polynomial 
 * used to model jitter as a function of time.
 * @param sampleJitterCoeffs A vector of doubles representing the coefficients 
 * for jitter in the sample dimension. Similar to lineJitterCoeffs, these 
 * define the polynomial for modeling jitter.
 * @param lineTimes A vector of doubles representing the exposure times for 
 * each line, used to calculate the actual time at which each line was exposed 
 * for the purpose of adding jitter based on time.
 * @param jitteredLine Reference to a double where the line coordinate after
 * adding jitter will be stored.
 * @param jitteredSample Reference to a double where the sample coordinate 
 * after adding jitter will be stored.
 */
void addJitter(
    const double &line, const double &sample,
    const double &tolerance, const int &maxIts,
    const std::vector<double> lineJitterCoeffs, const std::vector<double> sampleJitterCoeffs,
    const std::vector<double> lineTimes,
    double &jitteredLine, double &jitteredSample) {
  int iteration = 0;
  double dejitteredLine = line - 1;
  double dejitteredSample = sample - 1;
  double currentLine = line;
  double currentSample =  sample;

  while (iteration < maxIts) {
    removeJitter(
        currentLine, currentSample,
        lineJitterCoeffs, sampleJitterCoeffs, lineTimes,
        dejitteredLine, dejitteredSample);

    if (fabs(dejitteredLine - line) < tolerance &&
        fabs(dejitteredSample - sample) < tolerance) {
      break;
    }

    currentLine = line + currentLine - dejitteredLine;
    currentSample = sample + currentSample - dejitteredSample;
    iteration++;
  }

  jitteredLine = currentLine;
  jitteredSample = currentSample;

  return;
}


/**
 * @description Computes the image pixel coordinates from a distorted focal 
 * plane coordinate. This function converts coordinates from the distorted 
 * focal plane back to the image sensor (CCD) coordinate system, taking into 
 * account the CCD's origin, pixel summing, and starting positions for the 
 * sample and line. It uses transformation matrices to map from the focal 
 * plane to the CCD coordinates, adjusting for distortion effects that may 
 * have been applied previously.
 * 
 * @param distortedX The x-coordinate in the distorted focal plane.
 * @param distortedY The y-coordinate in the distorted focal plane.
 * @param sampleOrigin The x-coordinate of the origin of the CCD coordinate 
 * system relative to the top left of the CCD.
 * @param lineOrigin The y-coordinate of the origin of the CCD coordinate 
 * system relative to the top left of the CCD.
 * @param sampleSumming The number of physical pixels combined into a single 
 * image pixel along the x-direction, affecting resolution and sensitivity.
 * @param lineSumming The number of physical pixels combined into a single 
 * image pixel along the y-direction, affecting resolution and sensitivity.
 * @param startingSample The x-coordinate of the first CCD sample corresponding 
 * to the start of the image area.
 * @param startingLine The y-coordinate of the first CCD line corresponding 
 * to the start of the image area.
 * @param iTransS An array of three doubles representing the transformation 
 * from the focal plane to CCD samples, including translation and scaling factors.
 * @param iTransL An array of three doubles representing the transformation 
 * from the focal plane to CCD lines, including translation and scaling factors.
 * @param line Reference to a double where the computed line (y-coordinate) in 
 * the image pixel coordinate system will be stored.
 * @param sample Reference to a double where the computed sample (x-coordinate) 
 * in the image pixel coordinate system will be stored.
 */
void computePixel(const double &distortedX, const double &distortedY,
                  const double &sampleOrigin, const double &lineOrigin,
                  const double &sampleSumming, const double &lineSumming,
                  const double &startingSample, const double &startingLine,
                  const double iTransS[], const double iTransL[], double &line,
                  double &sample) {
  double centeredSample =
      iTransS[0] + iTransS[1] * distortedX + iTransS[2] * distortedY;
  double centeredLine =
      iTransL[0] + iTransL[1] * distortedX + iTransL[2] * distortedY;
  double detSample = centeredSample + sampleOrigin;
  double detLine = centeredLine + lineOrigin;
  sample = (detSample - startingSample) / sampleSumming;
  line = (detLine - startingLine) / lineSumming;
}

/**
 * @description Creates a normalized look vector (also known as an imaging ray) 
 * in camera space, given coordinates on the undistorted focal plane and the 
 * camera's focal length. This function computes the direction vector from the 
 * camera's optical center through a point on the undistorted focal plane, 
 * effectively defining the direction in which the camera is looking at a 
 * specific point in the scene. The zDirection parameter allows for adjusting 
 * the look vector based on the orientation of the camera's boresight.
 * 
 * @param undistortedFocalPlaneX The x-coordinate on the undistorted focal 
 * plane, typically in units of length (e.g., millimeters), representing the 
 * horizontal displacement from the optical axis.
 * @param undistortedFocalPlaneY The y-coordinate on the undistorted focal 
 * plane, typically in units of length, representing the vertical displacement 
 * from the optical axis.
 * @param zDirection The direction of the camera's boresight along the z-axis. 
 * This should be either 1 or -1, indicating whether the look vector should be 
 * oriented along or against the optical axis.
 * @param focalLength The focal length of the camera, defining the distance 
 * from the optical center to the focal plane. This value is crucial for 
 * determining the depth component of the look vector.
 * @param cameraLook An array of three doubles where the normalized look vector 
 * will be stored. The vector is represented as [x, y, z] components in camera 
 * space, where the z-axis aligns with the optical axis, and the x and y axes 
 * correspond to the horizontal and vertical displacements on the focal plane, 
 * respectively.
 */
void createCameraLookVector(const double &undistortedFocalPlaneX,
                            const double &undistortedFocalPlaneY,
                            const double &zDirection, const double &focalLength,
                            double cameraLook[]) {
  cameraLook[0] = -undistortedFocalPlaneX * zDirection;
  cameraLook[1] = -undistortedFocalPlaneY * zDirection;
  cameraLook[2] = -focalLength;
  double magnitude =
      sqrt(cameraLook[0] * cameraLook[0] + cameraLook[1] * cameraLook[1] +
           cameraLook[2] * cameraLook[2]);
  cameraLook[0] /= magnitude;
  cameraLook[1] /= magnitude;
  cameraLook[2] /= magnitude;
}

/**
 * @description Performs Lagrange interpolation for equally spaced data to 
 * estimate the value at a given time point. This function is designed for 
 * uniform data intervals and supports up to 8th order interpolation. It 
 * gracefully handles points far from the data center to avoid calculation 
 * failures. The interpolation order is dynamically adjusted based on the 
 * proximity of the desired time to the data points, ensuring optimal accuracy 
 * while avoiding overfitting. The function is particularly useful for 
 * estimating values from a discrete set of data points in applications such as 
 * signal processing or numerical analysis.
 *
 * @param numTime The number of time points in the dataset.
 * @param valueArray Pointer to the array of values corresponding to each time 
 * point. The values for each point are assumed to be stored consecutively in 
 * memory.
 * @param startTime The start time of the dataset.
 * @param delTime The uniform time interval between consecutive data points.
 * @param time The time at which the value is to be estimated using 
 * interpolation.
 * @param vectorLength The length of the vectors stored in valueArray. This 
 * parameter allows the function to handle multidimensional data by treating 
 * each set of vectorLength consecutive elements in valueArray as a vector 
 * associated with a single time point.
 * @param i_order The maximum order of interpolation desired. The actual order 
 * used may be less than this value depending on the proximity of the 
 * interpolation point to the data boundaries.
 * @param valueVector Pointer to the array where the interpolated value(s) will 
 * be stored. This array must be pre-allocated with at least vectorLength 
 * elements. The interpolated values are stored as a vector of length 
 * vectorLength.
 *
 * @throws csm::Error If numTime is less than 2, indicating insufficient data 
 * points for interpolation.
 */
void lagrangeInterp(const int &numTime, const double *valueArray,
                    const double &startTime, const double &delTime,
                    const double &time, const int &vectorLength,
                    const int &i_order, double *valueVector) {
  // Lagrange interpolation for uniform post interval.
  // Largest order possible is 8th. Points far away from
  // data center are handled gracefully to avoid failure.

  if (numTime < 2) {
    throw csm::Error(
        csm::Error::INDEX_OUT_OF_RANGE,
        "At least 2 points are required to perform Lagrange interpolation.",
        "lagrangeInterp");
  }

  // Compute index

  double fndex = (time - startTime) / delTime;
  int index = static_cast<int>(fndex);

  if (index < 0) {
    index = 0;
  }
  if (index > numTime - 2) {
    index = numTime - 2;
  }

  // Define order, max is 8

  int order;
  if (index >= 3 && index < numTime - 4) {
    order = 8;
  } else if (index >= 2 && index < numTime - 3) {
    order = 6;
  } else if (index >= 1 && index < numTime - 2) {
    order = 4;
  } else {
    order = 2;
  }
  if (order > i_order) {
    order = i_order;
  }

  // Compute interpolation coefficients
  double tp3, tp2, tp1, tm1, tm2, tm3, tm4, d[8];
  double t0 = fndex - index;
  if (order == 2) {
    tm1 = t0 - 1;
    d[0] = -tm1;
    d[1] = t0;
  } else if (order == 4) {
    tp1 = t0 + 1;
    tm1 = t0 - 1;
    tm2 = t0 - 2;
    d[0] = -t0 * tm1 * tm2 / 6.0;
    d[1] = tp1 * tm1 * tm2 / 2.0;
    d[2] = -tp1 * t0 * tm2 / 2.0;
    d[3] = tp1 * t0 * tm1 / 6.0;
  } else if (order == 6) {
    tp2 = t0 + 2;
    tp1 = t0 + 1;
    tm1 = t0 - 1;
    tm2 = t0 - 2;
    tm3 = t0 - 3;
    d[0] = -tp1 * t0 * tm1 * tm2 * tm3 / 120.0;
    d[1] = tp2 * t0 * tm1 * tm2 * tm3 / 24.0;
    d[2] = -tp2 * tp1 * tm1 * tm2 * tm3 / 12.0;
    d[3] = tp2 * tp1 * t0 * tm2 * tm3 / 12.0;
    d[4] = -tp2 * tp1 * t0 * tm1 * tm3 / 24.0;
    d[5] = tp2 * tp1 * t0 * tm1 * tm2 / 120.0;
  } else if (order == 8) {
    tp3 = t0 + 3;
    tp2 = t0 + 2;
    tp1 = t0 + 1;
    tm1 = t0 - 1;
    tm2 = t0 - 2;
    tm3 = t0 - 3;
    tm4 = t0 - 4;
    // The denominators are hard-coded because the sampling is uniform
    // hence they can be computed explicitly.
    // Note: Trying pre-compute the many repeated multiplications
    // below does not speed things up.
    d[0] = -tp2 * tp1 * t0 * tm1 * tm2 * tm3 * tm4 / 5040.0;
    d[1] = tp3 * tp1 * t0 * tm1 * tm2 * tm3 * tm4  / 720.0;
    d[2] = -tp3 * tp2 * t0 * tm1 * tm2 * tm3 * tm4 / 240.0;
    d[3] = tp3 * tp2 * tp1 * tm1 * tm2 * tm3 * tm4 / 144.0;
    d[4] = -tp3 * tp2 * tp1 * t0 * tm2 * tm3 * tm4 / 144.0;
    d[5] = tp3 * tp2 * tp1 * t0 * tm1 * tm3 * tm4  / 240.0;
    d[6] = -tp3 * tp2 * tp1 * t0 * tm1 * tm2 * tm4 / 720.0;
    d[7] = tp3 * tp2 * tp1 * t0 * tm1 * tm2 * tm3  / 5040.0;
  }

  // Compute interpolated point
  int indx0 = index - order / 2 + 1;
  for (int i = 0; i < vectorLength; i++) {
    valueVector[i] = 0.0;
  }

  for (int i = 0; i < order; i++) {
    int jndex = vectorLength * (indx0 + i);
    for (int j = 0; j < vectorLength; j++) {
      valueVector[j] += d[i] * valueArray[jndex + j];
    }
  }
}

/**
 * @description Finds a root of a function within a specified interval using 
 * Brent's method. Brent's method is an efficient algorithm for finding roots 
 * of a function. It combines the bisection method, the secant method, and 
 * inverse quadratic interpolation. It has the reliability of bisection but 
 * can converge much faster when close to the root. The function to find the 
 * root of is provided as a parameter, allowing for flexible usage with any 
 * continuous function.
 *
 * @param lowerBound The lower bound of the interval in which to search for the 
 * root.
 * @param upperBound The upper bound of the interval in which to search for the 
 * root. It is assumed that the function crosses the x-axis at least once in 
 * the interval [lowerBound, upperBound].
 * @param func The function for which to find the root. This is a std::function 
 * object that takes a double and returns a double. The function should be 
 * continuous, and func(lowerBound) and func(upperBound) should have opposite 
 * signs.
 * @param epsilon The tolerance for convergence. The algorithm stops when the 
 * interval between the current best guess of the root and the previous best 
 * guess is less than epsilon, or when the function value at the current guess 
 * is within epsilon of zero.
 *
 * @return The estimated root of the function within the specified interval and 
 * tolerance.
 * 
 * @throws std::invalid_argument If func(lowerBound) and func(upperBound) have 
 * the same sign, indicating that Brent's method may not be applicable as there 
 * might not be a root within the interval or the function is not continuous.
 */
double brentRoot(double lowerBound, double upperBound,
                 std::function<double(double)> func, double epsilon) {
  double counterPoint = lowerBound;
  double currentPoint = upperBound;
  double counterFunc = func(counterPoint);
  double currentFunc = func(currentPoint);
  if (counterFunc * currentFunc > 0.0) {
    throw std::invalid_argument(
        "Function values at the boundaries have the same sign [brentRoot].");
  }
  if (fabs(counterFunc) < fabs(currentFunc)) {
    std::swap(counterPoint, currentPoint);
    std::swap(counterFunc, currentFunc);
  }

  double previousPoint = counterPoint;
  double previousFunc = counterFunc;
  double evenOlderPoint = previousPoint;
  double nextPoint;
  double nextFunc;
  int iteration = 0;
  bool bisected = true;

  do {
    // Inverse quadratic interpolation
    if (counterFunc != previousFunc && counterFunc != currentFunc &&
        currentFunc != previousFunc) {
      nextPoint = (counterPoint * currentFunc * previousFunc) /
                  ((counterFunc - currentFunc) * (counterFunc - previousFunc));
      nextPoint += (currentPoint * counterFunc * previousFunc) /
                   ((currentFunc - counterFunc) * (currentFunc - previousFunc));
      nextPoint +=
          (previousPoint * currentFunc * counterFunc) /
          ((previousFunc - counterFunc) * (previousFunc - currentFunc));
    } else {
      // Secant method
      nextPoint = currentPoint - currentFunc * (currentPoint - counterPoint) /
                                     (currentFunc - counterFunc);
    }

    // Bisection method
    if (((currentPoint - nextPoint) *
             (nextPoint - (3 * counterPoint + currentPoint) / 4) <
         0) ||
        (bisected && fabs(nextPoint - currentPoint) >=
                         fabs(currentPoint - previousPoint) / 2) ||
        (!bisected && fabs(nextPoint - currentPoint) >=
                          fabs(previousPoint - evenOlderPoint) / 2) ||
        (bisected && fabs(currentPoint - previousPoint) < epsilon) ||
        (!bisected && fabs(previousPoint - evenOlderPoint) < epsilon)) {
      nextPoint = (currentPoint + counterPoint) / 2;
      bisected = true;
    } else {
      bisected = false;
    }

    // Setup for next iteration
    evenOlderPoint = previousPoint;
    previousPoint = currentPoint;
    previousFunc = currentFunc;
    nextFunc = func(nextPoint);

    // This is a bugfix. Without it, the code gets lost and can't find the solution.
    // See also the implementation at https://en.wikipedia.org/wiki/Brent%27s_method
    if (nextFunc == 0)
      return nextPoint;

    if (counterFunc * nextFunc < 0) {
      currentPoint = nextPoint;
      currentFunc = nextFunc;
    } else {
      counterPoint = nextPoint;
      counterFunc = nextFunc;
    }
  } while (++iteration < 30 && fabs(counterPoint - currentPoint) > epsilon);

  return nextPoint;
}

/**
 * @description Applies the Newton-Raphson method to un-distort a pixel 
 * coordinate (dx, dy), producing the undistorted coordinate (ux, uy).
 * 
 * @param dx The distorted x-coordinate of the pixel.
 * @param dy The distorted y-coordinate of the pixel.
 * @param ux Reference to a double where the undistorted x-coordinate will be 
 * stored.
 * @param uy Reference to a double where the undistorted y-coordinate will be 
 * stored.
 * @param opticalDistCoeffs A constant reference to a vector of doubles 
 * containing the optical distortion coefficients. These coefficients are 
 * specific to the distortion model being used.
 * @param distortionType An enumeration of the type of distortion model to be 
 * corrected. This parameter dictates how the distortion and its Jacobian 
 * functions interpret the distortion coefficients.
 * @param tolerance The tolerance level for the convergence of the 
 * Newton-Raphson method. The iterative process will stop when the sum of the 
 * absolute differences between the distorted coordinates and their corrections 
 * falls below this tolerance or when the maximum number of iterations is 
 * reached.
 * @param distortionFunction A std::function object representing the distortion 
 * function. It takes distorted coordinates (x, y), a reference to the 
 * distortion coefficients, and outputs the distortion effects (fx, fy) to be 
 * applied to (x, y).
 * @param distortionJacobian A std::function object representing the Jacobian 
 * of the distortion function. It takes distorted coordinates (x, y) and 
 * outputs the Jacobian matrix as an array of 4 doubles, which is used to 
 * calculate the next iteration's correction.
 */
void newtonRaphson(double dx, double dy, double &ux, double &uy,
                    std::vector<double> const& opticalDistCoeffs,
                    DistortionType distortionType, const double tolerance,
                    std::function<void(double, double, double &, double &,
                                       std::vector<double> const&)> distortionFunction,
                    std::function<void(double, double, double *, 
                                       std::vector<double> const&)> distortionJacobian) {

  const int maxTries = 20;

  double x, y, fx, fy, jacobian[4];

  // Initial guess for the root
  x = dx;
  y = dy;

  distortionFunction(x, y, fx, fy, opticalDistCoeffs);

  for (int count = 1;
        ((fabs(fx) + fabs(fy)) > tolerance) && (count < maxTries); count++) {
    distortionFunction(x, y, fx, fy, opticalDistCoeffs);

    fx = dx - fx;
    fy = dy - fy;

    distortionJacobian(x, y, jacobian, opticalDistCoeffs);

    // Jxx * Jyy - Jxy * Jyx
    double determinant =
        jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2];
    if (fabs(determinant) < 1e-6) {
      ux = x;
      uy = y;
      // Near-zero determinant. Cannot continue. Return most recent result.
      return;
    }

    x = x + (jacobian[3] * fx - jacobian[1] * fy) / determinant;
    y = y + (jacobian[0] * fy - jacobian[2] * fx) / determinant;
  }

  if ((fabs(fx) + fabs(fy)) <= tolerance) {
    // The method converged to a root.
    ux = x;
    uy = y;

    return;
  }
}

double evaluatePolynomial(const std::vector<double> &coeffs, double x) {
  if (coeffs.empty()) {
    throw std::invalid_argument("Polynomial coeffs must be non-empty.");
  }
  auto revIt = coeffs.crbegin();
  double value = *revIt;
  ++revIt;
  for (; revIt != coeffs.crend(); ++revIt) {
    value *= x;
    value += *revIt;
  }
  return value;
}

/**
 * @description Evaluates a polynomial at a given point x. The polynomial is 
 * defined by its coefficients, with the coefficients provided in ascending 
 * order of power. This function calculates the value of the polynomial for a 
 * specific x-value using Horner's method.
 *
 * @param coeffs A constant reference to a vector of doubles representing the 
 * coefficients of the polynomial. The first element represents the 
 * coefficient of the lowest power term, and the last element represents the 
 * coefficient of the highest power term.
 * @param x The point at which to evaluate the polynomial.
 *
 * @return The value of the polynomial at the given point x.
 *
 * @throws std::invalid_argument If the coeffs vector is empty, as a polynomial 
 * requires at least one coefficient.
 */
double evaluatePolynomialDerivative(const std::vector<double> &coeffs,
                                    double x) {
  if (coeffs.empty()) {
    throw std::invalid_argument("Polynomial coeffs must be non-empty.");
  }
  int i = coeffs.size() - 1;
  double value = i * coeffs[i];
  --i;
  for (; i > 0; --i) {
    value *= x;
    value += i * coeffs[i];
  }
  return value;
}

/**
 * @description Finds an approximate root of a polynomial near a given initial 
 * guess using Newton's method. This iterative algorithm refines the guess 
 * based on the polynomial and its derivative, converging to a root when the 
 * value of the polynomial at the current guess is below a specified threshold. 
 * Newton's method is efficient and effective for well-behaved functions and 
 * starting guesses close to the actual root. However, convergence is not 
 * guaranteed if the function's derivative is very small or if the initial 
 * guess is far from any root.
 *
 * @param coeffs A vector of doubles representing the coefficients of the 
 * polynomial to find the root of. The coefficients should be ordered from the 
 * lowest degree term to the highest.
 * @param guess An initial guess for the root of the polynomial.
 * @param threshold The value below which the absolute value of the polynomial 
 * is considered close enough to zero to conclude that the root has been found.
 * @param maxIterations The maximum number of iterations to attempt before 
 * giving up on finding a root.
 *
 * @return The approximate root of the polynomial, found near the initial guess.
 *
 * @throws std::invalid_argument If the derivative of the polynomial at the 
 * current guess is too close to zero, making it impossible to proceed with the 
 * next iteration. This condition indicates either a poor initial guess or that 
 * the guess is near a point where the polynomial's slope is horizontal.
 * @throws std::invalid_argument If the function does not converge to a root 
 * within the specified number of iterations. This may indicate that the 
 * initial guess is too far from any root or that the specified threshold and 
 * maximum number of iterations are too strict.
 */
double polynomialRoot(const std::vector<double> &coeffs, double guess,
                      double threshold, int maxIterations) {
  double root = guess;
  double polyValue = evaluatePolynomial(coeffs, root);
  double polyDeriv = 0.0;
  for (int iteration = 0; iteration < maxIterations + 1; iteration++) {
    if (fabs(polyValue) < threshold) {
      return root;
    }
    polyDeriv = evaluatePolynomialDerivative(coeffs, root);
    if (fabs(polyDeriv) < 1e-15) {
      throw std::invalid_argument("Derivative at guess (" +
                                  std::to_string(guess) +
                                  ") is too close to 0.");
    }
    root -= polyValue / polyDeriv;
    polyValue = evaluatePolynomial(coeffs, root);
  }
  throw std::invalid_argument("Root finder did not converge after " +
                              std::to_string(maxIterations) + " iterations");
}

/**
 * @description Computes the elevation of a point above or below an ellipsoidal 
 * model of the Earth, given the point's Cartesian coordinates. The function 
 * iteratively adjusts the elevation estimate until the change between 
 * iterations is less than the specified desired precision or until a maximum 
 * number of iterations is reached.
 *
 * @param x The x-coordinate of the point in Cartesian coordinate system.
 * @param y The y-coordinate of the point in Cartesian coordinate system.
 * @param z The z-coordinate of the point in Cartesian coordinate system.
 * @param semiMajor The semi-major axis of the ellipsoid, typically 
 * representing the equatorial radius.
 * @param semiMinor The semi-minor axis of the ellipsoid, typically 
 * representing the polar radius.
 * @param desired_precision The desired precision for the elevation 
 * calculation. The iterative process will stop once the change in elevation 
 * estimate between iterations is less than this value.
 * @param achieved_precision Pointer to a double where the achieved precision 
 * of the calculation will be stored. This is the absolute difference in 
 * elevation estimate between the final two iterations. If null, this output 
 * is not provided.
 *
 * @return The elevation of the point above (positive value) or below 
 * (negative value) the ellipsoidal model, in the same units as the semi-major 
 * and semi-minor axes.
 */
double computeEllipsoidElevation(double x, double y, double z, double semiMajor,
                                 double semiMinor, double desired_precision,
                                 double *achieved_precision) {
  // Compute elevation given xyz
  // Requires semi-major-axis and eccentricity-square
  const int MKTR = 10;
  double ecc_sqr = 1.0 - semiMinor * semiMinor / semiMajor / semiMajor;
  double ep2 = 1.0 - ecc_sqr;
  double d2 = x * x + y * y;
  double d = sqrt(d2);
  double h = 0.0;
  int ktr = 0;
  double hPrev, r;

  // Suited for points near equator
  if (d >= z) {
    double tt, zz, n;
    double tanPhi = z / d;
    do {
      hPrev = h;
      tt = tanPhi * tanPhi;
      r = semiMajor / sqrt(1.0 + ep2 * tt);
      zz = z + r * ecc_sqr * tanPhi;
      n = r * sqrt(1.0 + tt);
      h = sqrt(d2 + zz * zz) - n;
      tanPhi = zz / d;
      ktr++;
    } while (MKTR > ktr && fabs(h - hPrev) > desired_precision);
  } else {
    // Suited for points near the poles
    double cc, dd, nn;
    double cotPhi = d / z;
    do {
      hPrev = h;
      cc = cotPhi * cotPhi;
      r = semiMajor / sqrt(ep2 + cc);
      dd = d - r * ecc_sqr * cotPhi;
      nn = r * sqrt(1.0 + cc) * ep2;
      h = sqrt(dd * dd + z * z) - nn;
      cotPhi = dd / z;
      ktr++;
    } while (MKTR > ktr && fabs(h - hPrev) > desired_precision);
  }

  if (achieved_precision) {
    *achieved_precision = fabs(h - hPrev);
  }

  return h;
}

/**
 * @description Multiplies a scalar by an ECEF vector.
 * 
 * @param scalar A double value to scale the ECEF vector.
 * @param vec A constant reference to an ECEF vector to be scaled.
 * @return An ECEF vector where each component is multiplied by the scalar.
 */
csm::EcefVector operator*(double scalar, const csm::EcefVector &vec) {
  return csm::EcefVector(scalar * vec.x, scalar * vec.y, scalar * vec.z);
}

/**
 * @description Multiplies an ECEF vector by a scalar.
 * 
 * @param vec A constant reference to an ECEF vector to be scaled.
 * @param scalar A double value to scale the ECEF vector.
 * @return An ECEF vector where each component is multiplied by the scalar.
 */
csm::EcefVector operator*(const csm::EcefVector &vec, double scalar) {
  return scalar * vec;
}

/**
 * @description Divides an ECEF vector by a scalar.
 * 
 * @param vec A constant reference to an ECEF vector to be divided.
 * @param scalar A double value by which the ECEF vector components are divided.
 * @return An ECEF vector where each component is divided by the scalar.
 */
csm::EcefVector operator/(const csm::EcefVector &vec, double scalar) {
  return 1.0 / scalar * vec;
}

/**
 * @description Adds two ECEF vectors.
 * 
 * @param vec1 A constant reference to the first ECEF vector.
 * @param vec2 A constant reference to the second ECEF vector.
 * @return An ECEF vector resulting from component-wise addition of vec1 and vec2.
 */
csm::EcefVector operator+(const csm::EcefVector &vec1,
                          const csm::EcefVector &vec2) {
  return csm::EcefVector(vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z);
}

/**
 * @description Subtracts one ECEF vector from another.
 * 
 * @param vec1 A reference to the ECEF vector from which vec2 is subtracted.
 * @param vec2 A reference to the ECEF vector to subtract from vec1.
 * @return An ECEF vector resulting from component-wise subtraction of vec2 from vec1.
 */
csm::EcefVector operator-(const csm::EcefVector &vec1,
                          const csm::EcefVector &vec2) {
  return csm::EcefVector(vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z);
}

/**
 * @description Calculates the dot product of two ECEF vectors.
 * 
 * @param vec1 A constant reference to the first ECEF vector.
 * @param vec2 A constant reference to the second ECEF vector.
 * @return A double representing the dot product of vec1 and vec2.
 */
double dot(const csm::EcefVector &vec1, const csm::EcefVector &vec2) {
  return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}

/**
 * @description Calculates the cross product of two ECEF vectors.
 * 
 * @param vec1 A constant reference to the first ECEF vector.
 * @param vec2 A constant reference to the second ECEF vector.
 * @return An ECEF vector representing the cross product of vec1 and vec2. 
 * The resulting vector is orthogonal to both vec1 and vec2.
 */
csm::EcefVector cross(const csm::EcefVector &vec1,
                      const csm::EcefVector &vec2) {
  return csm::EcefVector(vec1.y * vec2.z - vec1.z * vec2.y,
                         vec1.z * vec2.x - vec1.x * vec2.z,
                         vec1.x * vec2.y - vec1.y * vec2.x);
}

/**
 * @description Calculates the norm (magnitude) of an ECEF vector.
 * 
 * @param vec A constant reference to the ECEF vector.
 * @return A double value representing the Euclidean norm of the vector, which 
 * is the square root of the sum of the squares of its components.
 */
double norm(const csm::EcefVector &vec) {
  return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

/**
 * @description Normalizes an ECEF vector, scaling it to a unit vector.
 * 
 * @param vec A constant reference to the ECEF vector to be normalized.
 * @return An ECEF vector that points in the same direction as the original 
 * vector but with a norm (magnitude) of 1. If the input vector is a zero 
 * vector, the behavior is undefined.
 */
csm::EcefVector normalized(const csm::EcefVector &vec) {
  return vec / norm(vec);
}

/**
 * @description Calculates the projection of one ECEF vector onto another.
 * 
 * @param vec1 A constant reference to the ECEF vector being projected.
 * @param vec2 A constant reference to the ECEF vector upon which vec1 is projected.
 * @return An ECEF vector representing the projection of vec1 onto vec2. This 
 * is the component of vec1 that lies in the direction of vec2.
 */
csm::EcefVector projection(const csm::EcefVector &vec1,
                           const csm::EcefVector &vec2) {
  return dot(vec1, vec2) / dot(vec2, vec2) * vec2;
}

/**
 * @description Calculates the rejection of one ECEF vector from another.
 * 
 * @param vec1 A constant reference to the ECEF vector being rejected.
 * @param vec2 A constant reference to the ECEF vector from which vec1 is rejected.
 * @return An ECEF vector representing the rejection of vec1 from vec2. This is 
 * the component of vec1 that is orthogonal to vec2, effectively vec1 minus its 
 * projection on vec2.
 */
csm::EcefVector rejection(const csm::EcefVector &vec1,
                          const csm::EcefVector &vec2) {
  return vec1 - projection(vec1, vec2);
}

/**
 * @description Converts a measurement value from one metric unit to another.
 * 
 * @param val The numerical value to be converted.
 * @param from The unit of measure of the input value (e.g., "m" for meters, 
 * "km" for kilometers).
 * @param to The target unit of measure for the conversion (e.g., "m" for 
 * meters, "km" for kilometers).
 *
 * @return The converted value in the target unit of measure.
 */
double metric_conversion(double val, std::string from, std::string to) {
  json typemap = {{"m", 0}, {"km", 3}};

  // everything to lowercase
  std::transform(from.begin(), from.end(), from.begin(), ::tolower);
  std::transform(to.begin(), to.end(), to.begin(), ::tolower);
  return val * pow(10, typemap[from].get<int>() - typemap[to].get<int>());
}

/**
 * @description Extracts the sensor model name from an ISD JSON object.
 * 
 * @param isd The ISD JSON object containing sensor model information.
 * @param list Pointer to a csm::WarningList where warnings can be added if 
 * the sensor model name cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 * 
 * @return The sensor model name as a string. Returns an empty string if the 
 * name cannot be found or parsed from the ISD.
 */
std::string getSensorModelName(json isd, csm::WarningList *list) {
  std::string name = "";
  try {
    name = isd.at("name_model");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the sensor model name.",
                                   "Utilities::getSensorModelName()"));
    }
  }
  return name;
}

/**
 * @description Extracts the image identifier from an ISD JSON object.
 * 
 * @param isd The ISD JSON object containing image identification information.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * image identifier cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 * 
 * @return The image identifier as a string. Returns an empty string if the 
 * identifier cannot be found or parsed from the ISD.
 */
std::string getImageId(json isd, csm::WarningList *list) {
  std::string id = "";
  try {
    id = isd.at("image_identifier");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the image identifier.",
                                   "Utilities::getImageId()"));
    }
  }
  return id;
}

/**
 * @description Extracts the sensor name from an ISD JSON object.
 * 
 * @param isd The ISD JSON object containing the sensor name information.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * sensor name cannot be parsed from the ISD. This parameter is optional; pass 
 * nullptr if you do not need to capture warnings.
 * 
 * @return The sensor name as a string. Returns an empty string if the name 
 * cannot be found or parsed from the ISD.
 */
std::string getSensorName(json isd, csm::WarningList *list) {
  std::string name = "";
  try {
    name = isd.at("name_sensor");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the sensor name.",
                                   "Utilities::getSensorName()"));
    }
  }
  return name;
}

/**
 * @description Extracts the platform name from an ISD JSON object.
 * 
 * @param isd The ISD JSON object containing the platform name information.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * platform name cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 * 
 * @return The platform name as a string. Returns an empty string if the name 
 * cannot be found or parsed from the ISD.
 *  */
std::string getPlatformName(json isd, csm::WarningList *list) {
  std::string name = "";
  try {
    name = isd.at("name_platform");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the platform name.",
                                   "Utilities::getPlatformName()"));
    }
  }
  return name;
}

/**
 * @description Extracts the log file name from an ISD JSON object.
 * 
 * @param isd The ISD JSON object containing the log file name information.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * log file name cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 * 
 * @return The log file name as a string. Returns an empty string if the log 
 * file name cannot be found or parsed from the ISD.
 */
std::string getLogFile(nlohmann::json isd, csm::WarningList *list) {
  std::string file = "";
  try {
    file = isd.at("log_file");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the log filename.",
                                   "Utilities::getLogFile()"));
    }
  }
  return file;
}

/**
 * @description Retrieves the total number of lines in an image from an ISD 
 * JSON object.
 * 
 * @param isd The ISD JSON object containing metadata about the image.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * total number of lines cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 * 
 * @return The total number of lines in the image as an integer. Returns 0 if 
 * the information cannot be found or parsed from the ISD.
 */
int getTotalLines(json isd, csm::WarningList *list) {
  int lines = 0;
  try {
    lines = isd.at("image_lines");
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the number of lines in the image.",
                       "Utilities::getTotalLines()"));
    }
  }
  return lines;
}

/**
 * Retrieves the total number of samples in an image from an ISD JSON object.
 * 
 * @param isd The ISD JSON object containing metadata about the image.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * total number of samples cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 * 
 * @return The total number of samples in the image as an integer. Returns 0 if 
 * the information cannot be found or parsed from the ISD.
 */
int getTotalSamples(json isd, csm::WarningList *list) {
  int samples = 0;
  try {
    samples = isd.at("image_samples");
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the number of samples in the image.",
                       "Utilities::getTotalSamples()"));
    }
  }
  return samples;
}

/**
 * @description Retrieves the starting ephemeris time of the image capture from 
 * an ISD JSON object.
 * 
 * @param isd The ISD JSON object containing temporal data about the image.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * starting ephemeris time of the image capture cannot be parsed from the ISD. 
 * This parameter is optional; pass nullptr if you do not need to capture 
 * warnings.
 * 
 * @return The starting ephemeris time of the image capture as a double. 
 * Returns 0.0 if the information cannot be found or parsed from the ISD.
 */
double getStartingTime(json isd, csm::WarningList *list) {
  double time = 0.0;
  try {
    time = isd.at("starting_ephemeris_time");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the image start time.",
                                   "Utilities::getStartingTime()"));
    }
  }
  return time;
}

/**
 * @description Retrieves the center ephemeris time of the image capture from 
 * an ISD JSON object.
 *
 * @param isd The ISD JSON object containing temporal data about the image.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * center ephemeris time cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The center ephemeris time of the image capture as a double. Returns 
 * 0.0 if the information cannot be found or parsed from the ISD.
 */
double getCenterTime(json isd, csm::WarningList *list) {
  double time = 0.0;
  try {
    time = isd.at("center_ephemeris_time");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the center image time.",
                                   "Utilities::getCenterTime()"));
    }
  }
  return time;
}

/**
 * @description Retrieves the ending ephemeris time of the image capture from 
 * an ISD JSON object.
 *
 * @param isd The ISD JSON object containing temporal data about the image.
 * @param list Pointer to a csm::WarningList where warnings can be added if the 
 * ending ephemeris time cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The ending ephemeris time of the image capture as a double. Returns 
 * 0.0 if the information cannot be found or parsed from the ISD.
 */
double getEndingTime(json isd, csm::WarningList *list) {
  double time = 0.0;
  try {
    time = isd.at("ending_ephemeris_time");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the ending image time.",
                                   "Utilities::getEndingTime()"));
    }
  }
  return time;
}

/**
 * @description Extracts the integration start lines from a line scan rate 
 * table.
 * 
 * @param lineScanRate A vector of vectors, each inner vector containing three 
 * doubles: the start line, the start time, and the duration of each 
 * integration period.
 * @param list Pointer to a csm::WarningList where warnings are added if any 
 * issues occur during parsing.
 * 
 * @return A vector of doubles, each representing the start line of an 
 * integration period extracted from the line scan rate table.
 */
std::vector<double> getIntegrationStartLines(
    std::vector<std::vector<double>> lineScanRate, csm::WarningList *list) {
  std::vector<double> lines;
  try {
    for (auto &scanRate : lineScanRate) {
      if (scanRate.size() != 3) {
        throw csm::Error(
            csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
            "Unable to parse integration start lines from line "
            "scan rate due to malformed vector. Expected vector size of 3.",
            "Utilities::getIntegrationStartLines()");
      }
      lines.push_back(scanRate[0]);
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the integration start "
                                   "lines in the integration time table.",
                                   "Utilities::getIntegrationStartLines()"));
    }
  }
  return lines;
}

/**
 * @description Extracts the integration start times from a line scan rate 
 * table.
 * 
 * @param lineScanRate A vector of vectors, each inner vector containing three 
 * doubles: the start line, the start time, and the duration of each 
 * integration period.
 * @param list Pointer to a csm::WarningList where warnings are added if any 
 * issues occur during parsing.
 * 
 * @return A vector of doubles, each representing the start time of an 
 * integration period extracted from the line scan rate table.
 */
std::vector<double> getIntegrationStartTimes(
    std::vector<std::vector<double>> lineScanRate, csm::WarningList *list) {
  std::vector<double> times;
  try {
    for (auto &scanRate : lineScanRate) {
      if (scanRate.size() != 3) {
        throw csm::Error(
            csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
            "Unable to parse integration start times from line "
            "scan rate due to malformed vector. Expected vector size of 3.",
            "Utilities::getIntegrationStartTimes()");
      }
      times.push_back(scanRate[1]);
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the integration start "
                                   "times in the integration time table.",
                                   "Utilities::getIntegrationStartTimes()"));
    }
  }
  return times;
}

/**
 * @description Extracts the integration times (durations) from a line scan 
 * rate table.
 * 
 * @param lineScanRate A vector of vectors, each inner vector containing three 
 * doubles: the start line, the start time, and the duration of each 
 * integration period.
 * @param list Pointer to a csm::WarningList where warnings are added if any 
 * issues occur during parsing.
 * 
 * @return A vector of doubles, each representing the duration of an 
 * integration period extracted from the line scan rate table.
 */
std::vector<double> getIntegrationTimes(
    std::vector<std::vector<double>> lineScanRate, csm::WarningList *list) {
  std::vector<double> times;
  try {
    for (auto &scanRate : lineScanRate) {
      if (scanRate.size() != 3) {
        throw csm::Error(
            csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
            "Unable to parse integration times from line "
            "scan rate due to malformed vector. Expected vector size of 3.",
            "Utilities::getIntegrationTimes()");
      }
      times.push_back(scanRate[2]);
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the integration times in "
                                   "the integration time table.",
                                   "Utilities::getIntegrationTimes()"));
    }
  }
  return times;
}

/**
 * @description Retrieves the sample summing value from an ISD JSON object. 
 * This value represents the pixel summing in the sample (horizontal) 
 * direction, which is used to reduce the image resolution for various 
 * operational reasons.
 *
 * @param isd The ISD JSON object containing the imaging data.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * sample summing value cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The sample summing value as an integer. Returns 0 if the information 
 * cannot be found or parsed from the ISD.
 */
int getSampleSumming(json isd, csm::WarningList *list) {
  int summing = 0;
  try {
    summing = isd.at("detector_sample_summing");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sample direction detector pixel summing.",
          "Utilities::getSampleSumming()"));
    }
  }
  return summing;
}

/**
 * @description Retrieves the detector line summing value from an ISD JSON object.
 *
 * @param isd The ISD JSON object containing metadata about the imaging sensor.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * line summing value cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The line summing value as an integer, representing the number of 
 * detector pixels summed in the line (vertical) direction to form the image. 
 * Returns 0 if the information cannot be found or parsed from the ISD.
 */
int getLineSumming(json isd, csm::WarningList *list) {
  int summing = 0;
  try {
    summing = isd.at("detector_line_summing");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the line direction detector pixel summing.",
          "Utilities::getLineSumming()"));
    }
  }
  return summing;
}

/**
 * @description Retrieves the focal length of the imaging system from an ISD 
 * JSON object.
 *
 * @param isd The ISD JSON object containing the focal length model of the 
 * imaging sensor.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * focal length cannot be parsed from the ISD. This parameter is optional; pass 
 * nullptr if you do not need to capture warnings.
 *
 * @return The focal length as a double, usually in millimeters. Returns 0.0 if 
 * the focal length cannot be found or parsed from the ISD.
 */
double getFocalLength(json isd, csm::WarningList *list) {
  double length = 0.0;
  try {
    length = isd.at("focal_length_model").at("focal_length");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the focal length.",
                                   "Utilities::getFocalLength()"));
    }
  }
  return length;
}

/**
 * @description Retrieves the uncertainty of the focal length (focal epsilon) 
 * from an ISD JSON object.
 *
 * @param isd The ISD JSON object containing the focal length model and its 
 * uncertainty.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * focal length uncertainty cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The focal length uncertainty (epsilon) as a double, usually in 
 * millimeters. Returns 0.0 if the uncertainty cannot be found or parsed from 
 * the ISD.
 */
double getFocalLengthEpsilon(json isd, csm::WarningList *list) {
  double epsilon = 0.0;
  try {
    epsilon = isd.at("focal_length_model").at("focal_epsilon");
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the focal length uncertainty.",
                       "Utilities::getFocalLengthEpsilon()"));
    }
  }
  return epsilon;
}

/**
 * @description Retrieves the transformation from focal plane coordinates to 
 * detector lines as a vector of doubles from an ISD JSON object.
 *
 * @param isd The ISD JSON object containing the transformation data.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * transformation cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the transformation from focal plane 
 * coordinates to detector lines. Returns an empty vector if the transformation 
 * cannot be found or parsed from the ISD.
 */
std::vector<double> getFocal2PixelLines(json isd, csm::WarningList *list) {
  std::vector<double> transformation;
  try {
    transformation = isd.at("focal2pixel_lines").get<std::vector<double>>();
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the focal plane coordinate "
                                   "to detector lines transformation.",
                                   "Utilities::getFocal2PixelLines()"));
    }
  }
  return transformation;
}

/**
 * @description Retrieves the transformation from focal plane coordinates to 
 * detector samples as a vector of doubles from an ISD JSON object.
 *
 * @param isd The ISD JSON object containing the transformation data.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * transformation cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the transformation from focal plane 
 * coordinates to detector samples. Returns an empty vector if the transformation 
 * cannot be found or parsed from the ISD.
 */
std::vector<double> getFocal2PixelSamples(json isd, csm::WarningList *list) {
  std::vector<double> transformation;
  try {
    transformation = isd.at("focal2pixel_samples").get<std::vector<double>>();
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the focal plane coordinate "
                                   "to detector samples transformation.",
                                   "Utilities::getFocal2PixelSamples()"));
    }
  }
  return transformation;
}

/**
 * @description Retrieves the detector center line from an ISD JSON object.
 *
 * @param isd The ISD JSON object containing the detector center information.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * detector center line cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The detector center line as a double. Returns 0.0 if the center line 
 * cannot be found or parsed from the ISD.
 */
double getDetectorCenterLine(json isd, csm::WarningList *list) {
  double line;
  try {
    line = isd.at("detector_center").at("line");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the detector center line.",
                                   "Utilities::getDetectorCenterLine()"));
    }
  }
  return line;
}

/**
 * @description Retrieves the detector center sample from an ISD JSON object.
 *
 * @param isd The ISD JSON object containing the detector center information.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * detector center sample cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The detector center sample as a double. Returns 0.0 if the center 
 * sample cannot be found or parsed from the ISD.
 */
double getDetectorCenterSample(json isd, csm::WarningList *list) {
  double sample;
  try {
    sample = isd.at("detector_center").at("sample");
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the detector center sample.",
                       "Utilities::getDetectorCenterSample()"));
    }
  }
  return sample;
}

/**
 * @description Retrieves the starting line of the detector from an ISD JSON 
 * object.
 *
 * @param isd The ISD JSON object containing the starting line information.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * starting line cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return The starting line of the detector as a double. Returns 0.0 if the 
 * starting line cannot be found or parsed from the ISD.
 */
double getDetectorStartingLine(json isd, csm::WarningList *list) {
  double line;
  try {
    line = isd.at("starting_detector_line");
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the detector starting line.",
                       "Utilities::getDetectorStartingLine()"));
    }
  }
  return line;
}

/**
 * @description Retrieves the starting sample of the detector from an ISD JSON 
 * object.
 *
 * @param isd The ISD JSON object containing the starting sample information.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * starting sample cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return The starting sample of the detector as a double. Returns 0.0 if the 
 * starting sample cannot be found or parsed from the ISD.
 */
double getDetectorStartingSample(json isd, csm::WarningList *list) {
  double sample;
  try {
    sample = isd.at("starting_detector_sample");
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the detector starting sample.",
                       "Utilities::getDetectorStartingSample()"));
    }
  }
  return sample;
}

/**
 * @description Retrieves the minimum height above the reference ellipsoid from 
 * an ISD JSON object. The height is converted to meters if necessary.
 *
 * @param isd The ISD JSON object containing the minimum height and its unit.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * minimum height cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return The minimum height above the reference ellipsoid in meters as a 
 * double. Returns 0.0 if the minimum height cannot be found or parsed from the 
 * ISD.
 */
double getMinHeight(json isd, csm::WarningList *list) {
  double height = 0.0;
  try {
    json referenceHeight = isd.at("reference_height");
    json minHeight = referenceHeight.at("minheight");
    json unit = referenceHeight.at("unit");
    height =
        metric_conversion(minHeight.get<double>(), unit.get<std::string>());
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the minimum height above the reference ellipsoid.",
          "Utilities::getMinHeight()"));
    }
  }
  return height;
}

/**
 * @description Retrieves the maximum height above the reference ellipsoid from 
 * an ISD JSON object. The height is converted to meters if necessary.
 *
 * @param isd The ISD JSON object containing the maximum height and its unit.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * maximum height cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return The maximum height above the reference ellipsoid in meters as a 
 * double. Returns 0.0 if the maximum height cannot be found or parsed from the 
 * ISD.
 */
double getMaxHeight(json isd, csm::WarningList *list) {
  double height = 0.0;
  try {
    json referenceHeight = isd.at("reference_height");
    json maxHeight = referenceHeight.at("maxheight");
    json unit = referenceHeight.at("unit");
    height =
        metric_conversion(maxHeight.get<double>(), unit.get<std::string>());
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the maximum height above the reference ellipsoid.",
          "Utilities::getMaxHeight()"));
    }
  }
  return height;
}

/**
 * @description Retrieves the semimajor radius of the reference ellipsoid from 
 * an ISD JSON object. The radius is converted to meters if necessary.
 *
 * @param isd The ISD JSON object containing the semimajor radius and its unit.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * semimajor radius cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return The semimajor radius of the reference ellipsoid in meters as a 
 * double. Returns 0.0 if the semimajor radius cannot be found or parsed from 
 * the ISD.
 */
double getSemiMajorRadius(json isd, csm::WarningList *list) {
  double radius = 0.0;
  try {
    json radii = isd.at("radii");
    json semiMajor = radii.at("semimajor");
    json unit = radii.at("unit");
    radius =
        metric_conversion(semiMajor.get<double>(), unit.get<std::string>());
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the reference ellipsoid semimajor radius.",
          "Utilities::getSemiMajorRadius()"));
    }
  }
  return radius;
}

/**
 * @description Retrieves the semiminor radius of the reference ellipsoid from 
 * an ISD JSON object. The radius is converted to meters if necessary.
 *
 * @param isd The ISD JSON object containing the semiminor radius and its unit.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * semiminor radius cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return The semiminor radius of the reference ellipsoid in meters as a 
 * double. Returns 0.0 if the semiminor radius cannot be found or parsed from 
 * the ISD.
 */
double getSemiMinorRadius(json isd, csm::WarningList *list) {
  double radius = 0.0;
  try {
    json radii = isd.at("radii");
    json semiMinor = radii.at("semiminor");
    json unit = radii.at("unit");
    radius =
        metric_conversion(semiMinor.get<double>(), unit.get<std::string>());
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the reference ellipsoid semiminor radius.",
          "Utilities::getSemiMinorRadius()"));
    }
  }
  return radius;
}

/**
 * @description Converts the distortion model name from the ISD JSON object to 
 * the enumeration type. This function searches the "optical_distortion" field 
 * for a key that matches a known distortion model name and converts it to the 
 * corresponding DistortionType enum. Defaults to DistortionType::TRANSVERSE if 
 * no match is found or the model cannot be parsed.
 *
 * @param isd The ISD JSON object containing the optical distortion information.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * distortion model name cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The DistortionType enum corresponding to the distortion model found 
 * in the ISD. Returns DistortionType::TRANSVERSE by default if the model name 
 * cannot be parsed or is not recognized.
 */
DistortionType getDistortionModel(json isd, csm::WarningList *list) {
  try {
    json distortion_subset = isd.at("optical_distortion");

    json::iterator it = distortion_subset.begin();

    std::string distortion = (std::string)it.key();

    if (distortion.compare("transverse") == 0) {
      return DistortionType::TRANSVERSE;
    } else if (distortion.compare("radial") == 0) {
      return DistortionType::RADIAL;
    } else if (distortion.compare("kaguyalism") == 0) {
      return DistortionType::KAGUYALISM;
    } else if (distortion.compare("dawnfc") == 0) {
      return DistortionType::DAWNFC;
    } else if (distortion.compare("lrolrocnac") == 0) {
      return DistortionType::LROLROCNAC;
    } else if (distortion.compare("cahvor") == 0) {
      return DistortionType::CAHVOR;
    } else if (distortion.compare("lunarorbiter") == 0) {
      return DistortionType::LUNARORBITER;
    } else if (distortion.compare("radtan") == 0) {
      return DistortionType::RADTAN;
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the distortion model.",
                                   "Utilities::getDistortionModel()"));
    }
  }
  return DistortionType::TRANSVERSE;
}

/**
 * @description Converts an integer representing a distortion model from ALE 
 * (Abstraction Library for Ephemerides) to the corresponding DistortionType 
 * enum. This function directly maps integers to the DistortionType enumeration 
 * based on the ALE distortion model definitions. Defaults to 
 * DistortionType::TRANSVERSE if the integer does not match any known model or 
 * the model cannot be parsed.
 *
 * @param aleDistortionModel An integer representing the ALE distortion model.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * distortion model integer cannot be mapped to a DistortionType enum. This 
 * parameter is optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The DistortionType enum corresponding to the ALE distortion model 
 * integer. Returns DistortionType::TRANSVERSE by default if the model integer 
 * cannot be mapped or is not recognized.
 */
DistortionType getDistortionModel(int aleDistortionModel,
                                  csm::WarningList *list) {
  try {
    ale::DistortionType aleDistortionType;
    aleDistortionType = (ale::DistortionType)aleDistortionModel;

    if (aleDistortionType == ale::DistortionType::TRANSVERSE) {
      return DistortionType::TRANSVERSE;
    } else if (aleDistortionType == ale::DistortionType::RADIAL) {
      return DistortionType::RADIAL;
    } else if (aleDistortionType == ale::DistortionType::KAGUYALISM) {
      return DistortionType::KAGUYALISM;
    } else if (aleDistortionType == ale::DistortionType::DAWNFC) {
      return DistortionType::DAWNFC;
    } else if (aleDistortionType == ale::DistortionType::LROLROCNAC) {
      return DistortionType::LROLROCNAC;
    }else if (aleDistortionType == ale::DistortionType::CAHVOR) {
      return DistortionType::CAHVOR;
    }else if (aleDistortionType == ale::DistortionType::LUNARORBITER) {
      return DistortionType::LUNARORBITER;
    }else if (aleDistortionType == ale::DistortionType::RADTAN) {
      return DistortionType::RADTAN;
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the distortion model.",
                                   "Utilities::getDistortionModel()"));
    }
  }
  return DistortionType::TRANSVERSE;
}

/**
 * @description Retrieves the distortion coefficients for the specified 
 * distortion model from an ISD JSON object. This function first determines the 
 * distortion model using getDistortionModel() and then extracts the 
 * coefficients specific to that model. If the coefficients cannot be parsed, a 
 * default set of coefficients is returned based on the model, and a warning is 
 * added to the list.
 *
 * @param isd The ISD JSON object containing the optical distortion coefficients 
 * for the specific distortion model.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * distortion coefficients cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the distortion coefficients. The 
 * content and size of the vector depend on the distortion model. If the 
 * coefficients cannot be parsed, a model-specific default vector is returned, 
 * and a warning is added to the list.
 */
std::vector<double> getDistortionCoeffs(json isd, csm::WarningList *list) {
  std::vector<double> coefficients;

  DistortionType distortion = getDistortionModel(isd);

  switch (distortion) {
    case DistortionType::TRANSVERSE: {
      try {
        std::vector<double> coefficientsX, coefficientsY;

        coefficientsX = isd.at("optical_distortion")
                            .at("transverse")
                            .at("x")
                            .get<std::vector<double>>();
        coefficientsX.resize(10, 0.0);

        coefficientsY = isd.at("optical_distortion")
                            .at("transverse")
                            .at("y")
                            .get<std::vector<double>>();
        coefficientsY.resize(10, 0.0);

        coefficients = coefficientsX;

        coefficients.insert(coefficients.end(), coefficientsY.begin(),
                            coefficientsY.end());
        return coefficients;
      } catch (...) {
        if (list) {
          list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                       "Could not parse a set of transverse "
                                       "distortion model coefficients.",
                                       "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(20, 0.0);
        coefficients[1] = 1.0;
        coefficients[12] = 1.0;
      }
    } break;
    case DistortionType::RADIAL: {
      try {
        coefficients = isd.at("optical_distortion")
                           .at("radial")
                           .at("coefficients")
                           .get<std::vector<double>>();

        return coefficients;
      } catch (...) {
        if (list) {
          list->push_back(csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the radial distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(3, 0.0);
      }
    } break;
    case DistortionType::KAGUYALISM: {
      try {
        std::vector<double> coefficientsX = isd.at("optical_distortion")
                                                .at("kaguyalism")
                                                .at("x")
                                                .get<std::vector<double>>();
        std::vector<double> coefficientsY = isd.at("optical_distortion")
                                                .at("kaguyalism")
                                                .at("y")
                                                .get<std::vector<double>>();
        double boresightX = isd.at("optical_distortion")
                                .at("kaguyalism")
                                .at("boresight_x")
                                .get<double>();
        double boresightY = isd.at("optical_distortion")
                                .at("kaguyalism")
                                .at("boresight_y")
                                .get<double>();

        coefficientsX.resize(4, 0.0);
        coefficientsY.resize(4, 0.0);
        coefficientsX.insert(coefficientsX.begin(), boresightX);
        coefficientsY.insert(coefficientsY.begin(), boresightY);
        coefficientsX.insert(coefficientsX.end(), coefficientsY.begin(),
                             coefficientsY.end());

        return coefficientsX;
      } catch (...) {
        if (list) {
          list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                       "Could not parse a set of Kaguya LISM "
                                       "distortion model coefficients.",
                                       "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(8, 0.0);
      }
    } break;
    case DistortionType::DAWNFC: {
      try {
        coefficients = isd.at("optical_distortion")
                           .at("dawnfc")
                           .at("coefficients")
                           .get<std::vector<double>>();

        return coefficients;
      } catch (...) {
        if (list) {
          list->push_back(csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the dawn radial distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(1, 0.0);
      }
    } break;
    case DistortionType::LROLROCNAC: {
      try {
        coefficients = isd.at("optical_distortion")
                           .at("lrolrocnac")
                           .at("coefficients")
                           .get<std::vector<double>>();
        return coefficients;
      } catch (...) {
        if (list) {
          list->push_back(csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the lrolrocnac distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(1, 0.0);
      }
    } break;
    case DistortionType::CAHVOR: {
      try {
        coefficients = isd.at("optical_distortion")
                           .at("cahvor")
                           .at("coefficients")
                           .get<std::vector<double>>();

        return coefficients;
      } catch (...) {
        if (list) {
          list->push_back(csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the radial distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(6, 0.0);
      }
    } break;
    case DistortionType::RADTAN: {
      try {
        coefficients = isd.at("optical_distortion")
                           .at("radtan")
                           .at("coefficients")
                           .get<std::vector<double>>();

        return coefficients;
      } catch (...) {
        if (list) {
          list->push_back(csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the radtan distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(5, 0.0);
      }
    } break;
  }
  
  if (list) {
    list->push_back(
        csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                     "Could not parse the distortion model coefficients.",
                     "Utilities::getDistortion()"));
  }

  return coefficients;
}

/**
 * @description Retrieves the sun positions from an ISD JSON object, converting 
 * the values to meters if necessary based on the unit specified in the ISD. The 
 * positions are returned as a flat vector of doubles, where each set of three 
 * consecutive values represent the x, y, and z coordinates of the sun's 
 * position at a given time.
 *
 * @param isd The ISD JSON object containing the sun position information and 
 * units.
 * @param list Pointer to a csm::WarningList where warnings are added if the sun 
 * positions cannot be parsed from the ISD. This parameter is optional; pass 
 * nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the sun positions in meters. If the 
 * positions cannot be parsed, an empty vector is returned.
 */
std::vector<double> getSunPositions(json isd, csm::WarningList *list) {
  std::vector<double> positions;
  try {
    json jayson = isd.at("sun_position");
    json unit = jayson.at("unit");
    for (auto &location : jayson.at("positions")) {
      positions.push_back(metric_conversion(location[0].get<double>(),
                                            unit.get<std::string>()));
      positions.push_back(metric_conversion(location[1].get<double>(),
                                            unit.get<std::string>()));
      positions.push_back(metric_conversion(location[2].get<double>(),
                                            unit.get<std::string>()));
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the sun positions.",
                                   "Utilities::getSunPositions()"));
    }
  }
  return positions;
}

/**
 * @description Retrieves the sun velocities from an ISD JSON object, converting 
 * the values to meters per second if necessary based on the unit specified in 
 * the ISD. The velocities are returned as a flat vector of doubles, where each 
 * set of three consecutive values represent the x, y, and z components of the 
 * sun's velocity at a given time.
 *
 * @param isd The ISD JSON object containing the sun velocity information and 
 * units.
 * @param list Pointer to a csm::WarningList where warnings are added if the sun 
 * velocities cannot be parsed from the ISD. This parameter is optional; pass 
 * nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the sun velocities in meters per 
 * second. If the velocities cannot be parsed, an empty vector is returned.
 */
std::vector<double> getSunVelocities(json isd, csm::WarningList *list) {
  std::vector<double> velocities;
  try {
    json jayson = isd.at("sun_position");
    json unit = jayson.at("unit");
    for (auto &location : jayson.at("velocities")) {
      velocities.push_back(metric_conversion(location[0].get<double>(),
                                             unit.get<std::string>()));
      velocities.push_back(metric_conversion(location[1].get<double>(),
                                             unit.get<std::string>()));
      velocities.push_back(metric_conversion(location[2].get<double>(),
                                             unit.get<std::string>()));
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the sun velocities.",
                                   "Utilities::getSunVelocities()"));
    }
  }
  return velocities;
}

/**
 * @description Retrieves the sensor positions from an ISD JSON object, 
 * converting the values to meters if necessary based on the unit specified in 
 * the ISD. The positions are returned as a flat vector of doubles, where each 
 * set of three consecutive values represent the x, y, and z coordinates of the 
 * sensor's position at a given time.
 *
 * @param isd The ISD JSON object containing the sensor position information and 
 * units.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * sensor positions cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the sensor positions in meters. If 
 * the positions cannot be parsed, an empty vector is returned.
 */
std::vector<double> getSensorPositions(json isd, csm::WarningList *list) {
  std::vector<double> positions;
  try {
    json jayson = isd.at("sensor_position");
    json unit = jayson.at("unit");
    for (auto &location : jayson.at("positions")) {
      positions.push_back(metric_conversion(location[0].get<double>(),
                                            unit.get<std::string>()));
      positions.push_back(metric_conversion(location[1].get<double>(),
                                            unit.get<std::string>()));
      positions.push_back(metric_conversion(location[2].get<double>(),
                                            unit.get<std::string>()));
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the sensor positions.",
                                   "Utilities::getSensorPositions()"));
    }
  }
  return positions;
}

/**
 * @description Retrieves the sensor velocities from an ISD JSON object, 
 * converting the values to meters per second if necessary based on the unit 
 * specified in the ISD. The velocities are returned as a flat vector of 
 * doubles, where each set of three consecutive values represent the x, y, and z 
 * components of the sensor's velocity at a given time.
 *
 * @param isd The ISD JSON object containing the sensor velocity information and 
 * units.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * sensor velocities cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the sensor velocities in meters per 
 * second. If the velocities cannot be parsed, an empty vector is returned.
 */
std::vector<double> getSensorVelocities(json isd, csm::WarningList *list) {
  std::vector<double> velocities;
  try {
    json jayson = isd.at("sensor_position");
    json unit = jayson.at("unit");
    for (auto &velocity : jayson.at("velocities")) {
      velocities.push_back(metric_conversion(velocity[0].get<double>(),
                                             unit.get<std::string>()));
      velocities.push_back(metric_conversion(velocity[1].get<double>(),
                                             unit.get<std::string>()));
      velocities.push_back(metric_conversion(velocity[2].get<double>(),
                                             unit.get<std::string>()));
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the sensor velocities.",
                                   "Utilities::getSensorVelocities()"));
    }
  }
  return velocities;
}

/**
 * @description Retrieves the sensor orientations from an ISD JSON object. The 
 * orientations are returned as a flat vector of doubles, where each set of four 
 * consecutive values represent a quaternion (x, y, z, w) describing the 
 * sensor's orientation at a given time.
 *
 * @param isd The ISD JSON object containing the sensor orientation quaternions.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * sensor orientations cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the sensor orientations as 
 * quaternions. If the orientations cannot be parsed, an empty vector is 
 * returned.
 */
std::vector<double> getSensorOrientations(json isd, csm::WarningList *list) {
  std::vector<double> quaternions;
  try {
    for (auto &quaternion : isd.at("sensor_orientation").at("quaternions")) {
      quaternions.push_back(quaternion[0]);
      quaternions.push_back(quaternion[1]);
      quaternions.push_back(quaternion[2]);
      quaternions.push_back(quaternion[3]);
    }
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the sensor orientations.",
                                   "Utilities::getSensorOrientations()"));
    }
  }
  return quaternions;
}

/**
 * @description Retrieves the line exposure duration from an ISD JSON object. 
 * The exposure duration is the time, typically in seconds, for which each line 
 * of the sensor was exposed during image capture.
 *
 * @param isd The ISD JSON object containing the line exposure duration.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * line exposure duration cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The line exposure duration as a double. If the duration cannot be 
 * parsed, 0.0 is returned.
 */
double getExposureDuration(nlohmann::json isd, csm::WarningList *list) {
  double duration;
  try {
    duration = isd.at("line_exposure_duration");
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the line exposure duration.",
                       "Utilities::getExposureDuration()"));
    }
  }
  return duration;
}

/**
 * @description Retrieves the scaled pixel width from an ISD JSON object. The 
 * scaled pixel width is the physical width of a pixel on the sensor, typically 
 * in meters, after applying any necessary scale factors.
 *
 * @param isd The ISD JSON object containing the scaled pixel width.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * scaled pixel width cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return The scaled pixel width as a double. If the width cannot be parsed, 
 * 0.0 is returned.
 */
double getScaledPixelWidth(nlohmann::json isd, csm::WarningList *list) {
  double width;
  try {
    width = isd.at("scaled_pixel_width");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the scaled pixel width.",
                                   "Utilities::getScaledPixelWidth()"));
    }
  }
  return width;
}

/**
 * @description Retrieves the look direction from an ISD JSON object. The look 
 * direction is a string that indicates the general direction in which the 
 * sensor was pointing when the image was captured, such as "NADIR", "FORWARD", 
 * or "BACKWARD".
 *
 * @param isd The ISD JSON object containing the look direction.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * look direction cannot be parsed from the ISD. This parameter is optional; 
 * pass nullptr if you do not need to capture warnings.
 *
 * @return The look direction as a std::string. If the look direction cannot be 
 * parsed, an empty string is returned.
 */
std::string getLookDirection(nlohmann::json isd, csm::WarningList *list) {
  std::string lookDirection = "";
  try {
    lookDirection = isd.at("look_direction");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the scaled pixel width.",
                                   "Utilities::getScaledPixelWidth()"));
    }
  }
  return lookDirection;
}

/**
 * @description Retrieves the range conversion times from an ISD JSON object. 
 * These times are used in the conversion of instrument data to physical units 
 * and are typically in seconds.
 *
 * @param isd The ISD JSON object containing the range conversion times.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * range conversion times cannot be parsed from the ISD. This parameter is 
 * optional; pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the range conversion times. If the 
 * times cannot be parsed, an empty vector is returned.
 */
std::vector<double> getScaleConversionTimes(nlohmann::json isd,
                                            csm::WarningList *list) {
  std::vector<double> time;
  try {
    time = isd.at("range_conversion_times").get<std::vector<double>>();
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the range conversion times.",
                       "Utilities::getScaleConversionTimes()"));
    }
  }
  return time;
}

/**
 * @description Retrieves the range conversion coefficients from an ISD JSON 
 * object. These coefficients are used in the conversion of instrument data to 
 * physical units.
 *
 * @param isd The ISD JSON object containing the range conversion coefficients.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * range conversion coefficients cannot be parsed from the ISD. This parameter 
 * is optional; pass nullptr if you do not need to capture warnings.
 *
 * @return A std::vector<double> containing the range conversion coefficients. 
 * If the coefficients cannot be parsed, an empty vector is returned.
 */
std::vector<double> getScaleConversionCoefficients(nlohmann::json isd,
                                                   csm::WarningList *list) {
  std::vector<double> coefficients;
  try {
    for (auto &location : isd.at("range_conversion_coefficients")) {
      coefficients.push_back(location[0]);
      coefficients.push_back(location[1]);
      coefficients.push_back(location[2]);
      coefficients.push_back(location[3]);
    }
  } catch (...) {
    if (list) {
      list->push_back(
          csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                       "Could not parse the range conversion coefficients.",
                       "Utilities::getScaleConversionCoefficients()"));
    }
  }
  return coefficients;
}

/**
 * @description Retrieves the wavelength from an ISD JSON object. The wavelength 
 * is typically specified in meters and represents the central wavelength of the 
 * sensor's operational band.
 *
 * @param isd The ISD JSON object containing the wavelength.
 * @param list Pointer to a csm::WarningList where warnings are added if the 
 * wavelength cannot be parsed from the ISD. This parameter is optional; pass 
 * nullptr if you do not need to capture warnings.
 *
 * @return The wavelength as a double. If the wavelength cannot be parsed, 0.0 
 * is returned.
 */
double getWavelength(json isd, csm::WarningList *list) {
  double wavelength = 0.0;
  try {
    wavelength = isd.at("wavelength");
  } catch (...) {
    if (list) {
      list->push_back(csm::Warning(csm::Warning::DATA_NOT_AVAILABLE,
                                   "Could not parse the wavelength.",
                                   "Utilities::getWavelength()"));
    }
  }
  return wavelength;
}

/**
 * @description Converts a model state string into a JSON object. This function 
 * first sanitizes the input string by removing non-printable characters and 
 * then parses the string from the first occurrence of "{" to the last 
 * occurrence of "}" into a JSON object.
 *
 * @param modelState The string representation of the model state to be 
 * converted into JSON format.
 *
 * @return A JSON object parsed from the sanitized input string. If the string 
 * does not contain valid JSON format, the function may throw a json::parse_error.
 */
json stateAsJson(std::string modelState) {
  // Remove special characters from string
  sanitize(modelState);

  std::size_t foundFirst = modelState.find_first_of("{");
  std::size_t foundLast = modelState.find_last_of("}");

  if (foundFirst == std::string::npos) {
    foundFirst = 0;
  }
  return json::parse(modelState.begin() + foundFirst, modelState.begin() + foundLast + 1);
}

/**
 * @description Sanitizes a given string by replacing characters that are not 
 * printable with newlines. This function modifies the original string.
 *
 * @param input The string to be sanitized.
 */
void sanitize(std::string &input){
  // Replaces characters from the string that are not printable with newlines
  std::replace_if(input.begin(), input.end(), [](int c){return !::isprint(c);}, '\n');
}

/**
 * @description Reads the entire content of a file into a single string. This 
 * function clears the output string before reading the file content.
 *
 * @param filename The path to the file to be read.
 * @param str The output string where the file content will be stored.
 *
 * @return A boolean value indicating whether the file was successfully opened 
 * and its content read. Returns true if the operation was successful, false 
 * otherwise. If the file cannot be opened, an error message is printed to 
 * std::cout.
 */
bool readFileInString(std::string const& filename, std::string & str) {

  str.clear(); // clear the output

  std::ifstream ifs(filename.c_str());
  if (!ifs.is_open()) {
    std::cout << "Cannot open file: " << filename << std::endl;
    return false;
  }

  ifs.seekg(0, std::ios::end);
  str.reserve(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  str.assign((std::istreambuf_iterator<char>(ifs)),
             std::istreambuf_iterator<char>());
  ifs.close();

  return true;
}

/**
 * @description Applies a rotation to a vector of quaternions. The function 
 * modifies the quaternions in-place, applying the specified rotation to each 
 * quaternion in the vector. Quaternions are expected to be in the format x, y, 
 * z, w, but the rotation operation requires them in the format w, x, y, z.
 *
 * @param r The rotation to be applied, represented as an ale::Rotation object.
 * @param quaternions A reference to a vector of doubles representing the 
 * quaternions to be rotated. The vector is modified in-place.
 */
void applyRotationToQuatVec(ale::Rotation const& r, std::vector<double> & quaternions) {
  int num_quat = quaternions.size();

  for (int it = 0; it < num_quat/4; it++) {

    double * q = &quaternions[4*it];

    // Note that quaternion in q is stored in order x, y, z, w, while
    // the rotation matrix wants it as w, x, y, z.

    std::vector<double> trans_q = (r * ale::Rotation(q[3], q[0], q[1], q[2])).toQuaternion();

    // Modify in-place
    q[0] = trans_q[1];
    q[1] = trans_q[2];
    q[2] = trans_q[3];
    q[3] = trans_q[0];
  }
}

/**
 * @description Applies a rotation and translation to a vector of xyz vectors. 
 * The function modifies the xyz vectors in-place, first applying the specified 
 * rotation and then the translation to each vector in the list.
 *
 * @param r The rotation to be applied, represented as an ale::Rotation object.
 * @param t The translation to be applied, represented as an ale::Vec3d object.
 * @param xyz A reference to a vector of doubles representing the xyz vectors to 
 * be transformed. The vector is modified in-place.
 */
void applyRotationTranslationToXyzVec(ale::Rotation const& r, ale::Vec3d const& t,
                                      std::vector<double> & xyz) {

  int num_xyz = xyz.size();
  for (int it = 0; it < num_xyz/3; it++) {

    double * p = &xyz[3*it];

    ale::Vec3d p_vec(p[0], p[1], p[2]);

    // Apply the rotation
    p_vec = r(p_vec);

    // Apply the translation
    p_vec += t;

    // Modify in-place
    p[0] = p_vec.x;
    p[1] = p_vec.y;
    p[2] = p_vec.z;

  }
}

/**
 * @description Converts ephemeris time, given in seconds since January 1, 2000 
 * 12:00:00 AM GMT, to a calendar time string formatted as 
 * "YYYY-MM-DDTHH:MM:SSZ". The function accounts for the base time and converts 
 * the input ephemeris time to the equivalent calendar time in UTC.
 *
 * @param ephemTime The ephemeris time in seconds since the epoch 
 * (January 1, 2000 12:00:00 AM GMT).
 *
 * @return A string representing the calendar time in UTC format 
 * "YYYY-MM-DDTHH:MM:SSZ".
 */
std::string ephemTimeToCalendarTime(double ephemTime) {

  // Care must be taken to not use mktime() or localtime() as their
  // precise value depends on if a location respects Daylight Savings Time.

  std::time_t y2k = 946684800; // January 1, 2000 12:00:00 AM GMT
  std::time_t finalTime = ephemTime + y2k;
  char buffer[22];
  strftime(buffer, 22, "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&finalTime));
  buffer[21] = '\0';
  return buffer;
}

/**
 * @description Converts pixel coordinates (line, sample) to metric coordinates 
 * (x, y) using a given geotransform vector. The geotransform vector contains 
 * coefficients that map pixel space to world space.
 *
 * @param line The line (y) coordinate in pixel space.
 * @param sample The sample (x) coordinate in pixel space.
 * @param geoTransform A vector of doubles representing the geotransformation 
 * coefficients.
 *
 * @return A vector of doubles containing the metric coordinates (y, x) 
 * corresponding to the input pixel coordinates.
 */
std::vector<double> pixelToMeter(double line, double sample, std::vector<double> geoTransform) {
  double meter_x = (sample * geoTransform[1]) + geoTransform[0];
  double meter_y = (line * geoTransform[5]) + geoTransform[3];

  return {meter_y, meter_x};
}

/**
 * @description Converts metric coordinates (x, y) to pixel coordinates (line, 
 * sample) using a given geotransform vector. The geotransform vector contains 
 * coefficients that map world space to pixel space.
 *
 * @param meter_x The x coordinate in metric space.
 * @param meter_y The y coordinate in metric space.
 * @param geoTransform A vector of doubles representing the geotransformation 
 * coefficients.
 *
 * @return A vector of doubles containing the pixel coordinates (line, sample) 
 * corresponding to the input metric coordinates.
 */
std::vector<double> meterToPixel(double meter_x, double meter_y, std::vector<double> geoTransform) {
  double sample = (meter_x - geoTransform[0]) / geoTransform[1];
  double line = (meter_y - geoTransform[3]) / geoTransform[5];

  return {line, sample};
}