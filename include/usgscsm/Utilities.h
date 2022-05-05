#ifndef INCLUDE_USGSCSM_UTILITIES_H_
#define INCLUDE_USGSCSM_UTILITIES_H_

#include "Distortion.h"

#include <math.h>
#include <string>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>

#include <ale/Rotation.h>
#include <ale/Vectors.h>

#include <Warning.h>
#include <csm.h>

// Compute distorted focalPlane coordinates in mm
void computeDistortedFocalPlaneCoordinates(
    const double &line, const double &sample, const double &sampleOrigin,
    const double &lineOrigin, const double &sampleSumming,
    const double &lineSumming, const double &startingSample,
    const double &startingLine, const double iTransS[], const double iTransL[],
    double &distortedX, double &distortedY);

void computePixel(const double &distortedX, const double &distortedY,
                  const double &sampleOrigin, const double &lineOrigin,
                  const double &sampleSumming, const double &lineSumming,
                  const double &startingSample, const double &startingLine,
                  const double iTransS[], const double iTransL[], double &line,
                  double &sample);

void calculateRotationMatrixFromQuaternions(double quaternions[4],
                                            double cameraToBody[9]);

void calculateRotationMatrixFromEuler(double euler[], double rotationMatrix[]);

void createCameraLookVector(const double &undistortedFocalPlaneX,
                            const double &undistortedFocalPlaneY,
                            const double &zDirection, const double &focalLength,
                            double cameraLook[]);

void lagrangeInterp(const int &numTime, const double *valueArray,
                    const double &startTime, const double &delTime,
                    const double &time, const int &vectorLength,
                    const int &i_order, double *valueVector);

// Brent's algorithm for finding the roots of a function
// Arguments are two inputs that bracket a root, the function, and a convergence
// tolerance
double brentRoot(double lowerBound, double upperBound,
                 std::function<double(double)> func, double epsilon = 1e-10);

// Evaluate a polynomial function.
// Coefficients should be ordered least order to greatest I.E. {1, 2, 3} is 1 +
// 2x + 3x^2
double evaluatePolynomial(const std::vector<double> &coeffs, double x);

// Evaluate the derivative of a polynomial function.
// Coefficients should be ordered least order to greatest I.E. {1, 2, 3} is 1 +
// 2x + 3x^2
double evaluatePolynomialDerivative(const std::vector<double> &coeffs,
                                    double x);

// Find a root of a polynomial using Newton's method.
// Coefficients should be ordered least order to greatest I.E. {1, 2, 3} is 1 +
// 2x + 3x^2
double polynomialRoot(const std::vector<double> &coeffs, double guess,
                      double threshold = 1e-10, int maxIterations = 30);

double computeEllipsoidElevation(double x, double y, double z, double semiMajor,
                                 double semiMinor,
                                 double desired_precision = 0.001,
                                 double *achieved_precision = nullptr);

// Vector operations
csm::EcefVector operator*(double scalar, const csm::EcefVector &vec);
csm::EcefVector operator*(const csm::EcefVector &vec, double scalar);
csm::EcefVector operator/(const csm::EcefVector &vec, double scalar);
csm::EcefVector operator+(const csm::EcefVector &vec1,
                          const csm::EcefVector &vec2);
csm::EcefVector operator-(const csm::EcefVector &vec1,
                          const csm::EcefVector &vec2);
double dot(const csm::EcefVector &vec1, const csm::EcefVector &vec2);
csm::EcefVector cross(const csm::EcefVector &vec1, const csm::EcefVector &vec2);
double norm(const csm::EcefVector &vec);
csm::EcefVector normalized(const csm::EcefVector &vec);
// The projection of vec1 onto vec2. The component of vec1 that is parallel to
// vec2
csm::EcefVector projection(const csm::EcefVector &vec1,
                           const csm::EcefVector &vec2);
// The rejection of vec1 onto vec2. The component of vec1 that is orthogonal to
// vec2
csm::EcefVector rejection(const csm::EcefVector &vec1,
                          const csm::EcefVector &vec2);

// Methods for checking/accessing the ISD

double metric_conversion(double val, std::string from, std::string to = "m");
std::string getSensorModelName(nlohmann::json isd,
                               csm::WarningList *list = nullptr);
std::string getImageId(nlohmann::json isd, csm::WarningList *list = nullptr);
std::string getSensorName(nlohmann::json isd, csm::WarningList *list = nullptr);
std::string getPlatformName(nlohmann::json isd,
                            csm::WarningList *list = nullptr);
std::string getLogFile(nlohmann::json isd, csm::WarningList *list = nullptr);
int getTotalLines(nlohmann::json isd, csm::WarningList *list = nullptr);
int getTotalSamples(nlohmann::json isd, csm::WarningList *list = nullptr);
double getStartingTime(nlohmann::json isd, csm::WarningList *list = nullptr);
double getCenterTime(nlohmann::json isd, csm::WarningList *list = nullptr);
double getEndingTime(nlohmann::json isd, csm::WarningList *list = nullptr);
std::vector<double> getIntegrationStartLines(
    std::vector<std::vector<double>> lineScanRate,
    csm::WarningList *list = nullptr);
std::vector<double> getIntegrationStartTimes(
    std::vector<std::vector<double>> lineScanRate,
    csm::WarningList *list = nullptr);
std::vector<double> getIntegrationTimes(
    std::vector<std::vector<double>> lineScanRate,
    csm::WarningList *list = nullptr);
double getExposureDuration(nlohmann::json isd,
                           csm::WarningList *list = nullptr);
double getScaledPixelWidth(nlohmann::json isd,
                           csm::WarningList *list = nullptr);
std::string getLookDirection(nlohmann::json isd,
                             csm::WarningList *list = nullptr);
std::vector<double> getScaleConversionCoefficients(
    nlohmann::json isd, csm::WarningList *list = nullptr);
std::vector<double> getScaleConversionTimes(nlohmann::json isd,
                                            csm::WarningList *list = nullptr);
int getSampleSumming(nlohmann::json isd, csm::WarningList *list = nullptr);
int getLineSumming(nlohmann::json isd, csm::WarningList *list = nullptr);
double getFocalLength(nlohmann::json isd, csm::WarningList *list = nullptr);
double getFocalLengthEpsilon(nlohmann::json isd,
                             csm::WarningList *list = nullptr);
std::vector<double> getFocal2PixelLines(nlohmann::json isd,
                                        csm::WarningList *list = nullptr);
std::vector<double> getFocal2PixelSamples(nlohmann::json isd,
                                          csm::WarningList *list = nullptr);
double getDetectorCenterLine(nlohmann::json isd,
                             csm::WarningList *list = nullptr);
double getDetectorCenterSample(nlohmann::json isd,
                               csm::WarningList *list = nullptr);
double getDetectorStartingLine(nlohmann::json isd,
                               csm::WarningList *list = nullptr);
double getDetectorStartingSample(nlohmann::json isd,
                                 csm::WarningList *list = nullptr);
double getMinHeight(nlohmann::json isd, csm::WarningList *list = nullptr);
double getMaxHeight(nlohmann::json isd, csm::WarningList *list = nullptr);
double getSemiMajorRadius(nlohmann::json isd, csm::WarningList *list = nullptr);
double getSemiMinorRadius(nlohmann::json isd, csm::WarningList *list = nullptr);
DistortionType getDistortionModel(nlohmann::json isd,
                                  csm::WarningList *list = nullptr);
DistortionType getDistortionModel(int aleDistortionModel,
                                  csm::WarningList *list = nullptr);
std::vector<double> getDistortionCoeffs(nlohmann::json isd,
                                        csm::WarningList *list = nullptr);
std::vector<double> getRadialDistortion(nlohmann::json isd,
                                        csm::WarningList *list = nullptr);
std::vector<double> getSunPositions(nlohmann::json isd,
                                    csm::WarningList *list = nullptr);
std::vector<double> getSunVelocities(nlohmann::json isd,
                                     csm::WarningList *list = nullptr);
std::vector<double> getSensorPositions(nlohmann::json isd,
                                       csm::WarningList *list = nullptr);
std::vector<double> getSensorVelocities(nlohmann::json isd,
                                        csm::WarningList *list = nullptr);
std::vector<double> getSensorOrientations(nlohmann::json isd,
                                          csm::WarningList *list = nullptr);
double getWavelength(nlohmann::json isd, csm::WarningList *list = nullptr);
nlohmann::json stateAsJson(std::string modelState);

// Apply transforms to orientations and vectors
void applyRotationToQuatVec(ale::Rotation const& r, std::vector<double> & quaternions);
void applyRotationTranslationToXyzVec(ale::Rotation const& r, ale::Vec3d const& t,
                                      std::vector<double> & xyz);

// Convert ephemeris time, in seconds since January 1, 2000 12:00:00 AM GMT,
// to a calendar time string, such as 2000-01-01T00:16:40Z.
std::string ephemTimeToCalendarTime(double ephemTime);

#endif  // INCLUDE_USGSCSM_UTILITIES_H_
