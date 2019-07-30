#include "Utilities.h"
#include <Error.h>

using json = nlohmann::json;

// Calculates a rotation matrix from Euler angles
// in - euler[3]
// out - rotationMatrix[9]
void calculateRotationMatrixFromEuler(
    double euler[],
    double rotationMatrix[])
{
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
// in - q[4]
// out - rotationMatrix[9]
void calculateRotationMatrixFromQuaternions(
    double q[4],
    double rotationMatrix[9])
{
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

// Compue the distorted focal plane coordinate for a given image pixel
// in - line
// in - sample
// in - sampleOrigin - the origin of the ccd coordinate system relative to the top left of the ccd
// in - lineOrigin - the origin of the ccd coordinate system relative to the top left of the ccd
// in - sampleSumming
// in - startingSample - first ccd sample for the image
// in - iTransS[3] - the transformation from focal plane to ccd samples
// in - iTransL[3] - the transformation from focal plane to ccd lines
// out - distortedX
// out - distortedY
void computeDistortedFocalPlaneCoordinates(
    const double& line,
    const double& sample,
    const double& sampleOrigin,
    const double& lineOrigin,
    const double& sampleSumming,
    const double& lineSumming,
    const double& startingSample,
    const double& startingLine,
    const double iTransS[],
    const double iTransL[],
    double &distortedX,
    double &distortedY)
{
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
};

// Compue the image pixel for a distorted focal plane coordinate
// in - line
// in - sample
// in - sampleOrigin - the origin of the ccd coordinate system relative to the top left of the ccd
// in - lineOrigin - the origin of the ccd coordinate system relative to the top left of the ccd
// in - sampleSumming
// in - startingSample - first ccd sample for the image
// in - iTransS[3] - the transformation from focal plane to ccd samples
// in - iTransL[3] - the transformation from focal plane to ccd lines
// out - natFocalPlane
void computePixel(
  const double& distortedX,
  const double& distortedY,
  const double& sampleOrigin,
  const double& lineOrigin,
  const double& sampleSumming,
  const double& lineSumming,
  const double& startingSample,
  const double& startingLine,
  const double iTransS[],
  const double iTransL[],
  double &line,
  double &sample)
{
  double centeredSample = iTransS[0] + iTransS[1] * distortedX + iTransS[2] * distortedY;
  double centeredLine =  iTransL[0] + iTransL[1] * distortedX + iTransL[2] * distortedY;
  double detSample = centeredSample + sampleOrigin;
  double detLine = centeredLine + lineOrigin;
  sample = (detSample - startingSample) / sampleSumming;
  line = (detLine - startingLine) / lineSumming;
};

// Define imaging ray in image space (In other words, create a look vector in camera space)
// in - undistortedFocalPlaneX
// in - undistortedFocalPlaneY
// in - zDirection - Either 1 or -1. The direction of the boresight
// in - focalLength
// out - cameraLook[3]
void createCameraLookVector(
    const double& undistortedFocalPlaneX,
    const double& undistortedFocalPlaneY,
    const double& zDirection,
    const double& focalLength,
    double cameraLook[])
{
  cameraLook[0] = -undistortedFocalPlaneX * zDirection;
  cameraLook[1] = -undistortedFocalPlaneY * zDirection;
  cameraLook[2] = -focalLength;
  double magnitude = sqrt(cameraLook[0] * cameraLook[0]
                + cameraLook[1] * cameraLook[1]
                + cameraLook[2] * cameraLook[2]);
  cameraLook[0] /= magnitude;
  cameraLook[1] /= magnitude;
  cameraLook[2] /= magnitude;
}

// Lagrange Interpolation for equally spaced data
void lagrangeInterp(
   const int&     numTime,
   const double*  valueArray,
   const double&  startTime,
   const double&  delTime,
   const double&  time,
   const int&     vectorLength,
   const int&     i_order,
   double*        valueVector) {
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
  int    index = int(fndex);

  if (index < 0)
  {
    index = 0;
  }
  if (index > numTime - 2)
  {
    index = numTime - 2;
  }

  // Define order, max is 8

  int order;
  if (index >= 3 && index < numTime - 4) {
    order = 8;
  }
  else if (index >= 2 && index < numTime - 3) {
    order = 6;
  }
  else if (index >= 1 && index < numTime - 2) {
    order = 4;
  }
  else {
    order = 2;
  }
  if (order > i_order) {
    order = i_order;
  }

  // Compute interpolation coefficients
  double tp3, tp2, tp1, tm1, tm2, tm3, tm4, d[8];
  double tau = fndex - index;
  if (order == 2) {
    tm1 = tau - 1;
    d[0] = -tm1;
    d[1] = tau;
  }
  else if (order == 4) {
    tp1 = tau + 1;
    tm1 = tau - 1;
    tm2 = tau - 2;
    d[0] = -tau * tm1 * tm2 / 6.0;
    d[1] = tp1 *       tm1 * tm2 / 2.0;
    d[2] = -tp1 * tau *       tm2 / 2.0;
    d[3] = tp1 * tau * tm1 / 6.0;
  }
  else if (order == 6) {
    tp2 = tau + 2;
    tp1 = tau + 1;
    tm1 = tau - 1;
    tm2 = tau - 2;
    tm3 = tau - 3;
    d[0] = -tp1 * tau * tm1 * tm2 * tm3 / 120.0;
    d[1] = tp2 *       tau * tm1 * tm2 * tm3 / 24.0;
    d[2] = -tp2 * tp1 *       tm1 * tm2 * tm3 / 12.0;
    d[3] = tp2 * tp1 * tau *       tm2 * tm3 / 12.0;
    d[4] = -tp2 * tp1 * tau * tm1 *       tm3 / 24.0;
    d[5] = tp2 * tp1 * tau * tm1 * tm2 / 120.0;
  }
  else if (order == 8) {
    tp3 = tau + 3;
    tp2 = tau + 2;
    tp1 = tau + 1;
    tm1 = tau - 1;
    tm2 = tau - 2;
    tm3 = tau - 3;
    tm4 = tau - 4;
    // Why are the denominators hard coded, as it should be x[0] - x[i]
    d[0] = -tp2 * tp1 * tau * tm1 * tm2 * tm3 * tm4 / 5040.0;
    d[1] = tp3 *       tp1 * tau * tm1 * tm2 * tm3 * tm4 / 720.0;
    d[2] = -tp3 * tp2 *       tau * tm1 * tm2 * tm3 * tm4 / 240.0;
    d[3] = tp3 * tp2 * tp1 *       tm1 * tm2 * tm3 * tm4 / 144.0;
    d[4] = -tp3 * tp2 * tp1 * tau *       tm2 * tm3 * tm4 / 144.0;
    d[5] = tp3 * tp2 * tp1 * tau * tm1 *       tm3 * tm4 / 240.0;
    d[6] = -tp3 * tp2 * tp1 * tau * tm1 * tm2 *       tm4 / 720.0;
    d[7] = tp3 * tp2 * tp1 * tau * tm1 * tm2 * tm3 / 5040.0;
  }

  // Compute interpolated point
  int    indx0 = index - order / 2 + 1;
  for (int i = 0; i < vectorLength; i++)
  {
    valueVector[i] = 0.0;
  }

  for (int i = 0; i < order; i++)
  {
    int jndex = vectorLength * (indx0 + i);
    for (int j = 0; j < vectorLength; j++)
    {
       valueVector[j] += d[i] * valueArray[jndex + j];
    }
  }
}

// convert a measurement
double metric_conversion(double val, std::string from, std::string to) {
    json typemap = {
      {"m", 0},
      {"km", 3}
    };

    // everything to lowercase
    std::transform(from.begin(), from.end(), from.begin(), ::tolower);
    std::transform(to.begin(), to.end(), to.begin(), ::tolower);
    return val*pow(10, typemap[from].get<int>() - typemap[to].get<int>());
}

std::string getSensorModelName(json isd, csm::WarningList *list) {
  std::string name = "";
  try {
    name = isd.at("name_model");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sensor model name.",
          "Utilities::getSensorModelName()"));
    }
  }
  return name;
}

std::string getImageId(json isd, csm::WarningList *list) {
  std::string id = "";
  try {
    id = isd.at("image_identifier");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the image identifier.",
          "Utilities::getImageId()"));
    }
  }
  return id;
}

std::string getSensorName(json isd, csm::WarningList *list) {
  std::string name = "";
  try {
    name = isd.at("name_sensor");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sensor name.",
          "Utilities::getSensorName()"));
    }
  }
  return name;
}

std::string getPlatformName(json isd, csm::WarningList *list) {
  std::string name = "";
  try {
    name = isd.at("name_platform");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the platform name.",
          "Utilities::getPlatformName()"));
    }
  }
  return name;
}


std::string getLogFile(nlohmann::json isd, csm::WarningList *list) {
  std::string file = "";
  try {
    file = isd.at("log_file");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the log filename.",
          "Utilities::getLogFile()"));
    }
  }
  return file;
}

int getTotalLines(json isd, csm::WarningList *list) {
  int lines = 0;
  try {
    lines = isd.at("image_lines");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the number of lines in the image.",
          "Utilities::getTotalLines()"));
    }
  }
  return lines;
}

int getTotalSamples(json isd, csm::WarningList *list) {
  int samples = 0;
  try {
    samples = isd.at("image_samples");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the number of samples in the image.",
          "Utilities::getTotalSamples()"));
    }
  }
  return samples;
}

double getStartingTime(json isd, csm::WarningList *list) {
    double time = 0.0;
    try {
      time = isd.at("starting_ephemeris_time");
    }
    catch (...) {
      if (list) {
        list->push_back(
          csm::Warning(
            csm::Warning::DATA_NOT_AVAILABLE,
            "Could not parse the image start time.",
            "Utilities::getStartingTime()"));
      }
    }
    return time;
}

double getCenterTime(json isd, csm::WarningList *list) {
  double time = 0.0;
  try {
    time = isd.at("center_ephemeris_time");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the center image time.",
          "Utilities::getCenterTime()"));
    }
  }
  return time;
}

std::vector<double> getIntegrationStartLines(json isd, csm::WarningList *list) {
  std::vector<double> lines;
  try {
    for (auto& scanRate : isd.at("line_scan_rate")) {
      lines.push_back(scanRate[0]);
    }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the integration start lines in the integration time table.",
          "Utilities::getIntegrationStartLines()"));
    }
  }
  return lines;
}

std::vector<double> getIntegrationStartTimes(json isd, csm::WarningList *list) {
  std::vector<double> times;
  try {
    for (auto& scanRate : isd.at("line_scan_rate")) {
      times.push_back(scanRate[1]);
    }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the integration start times in the integration time table.",
          "Utilities::getIntegrationStartTimes()"));
    }
  }
  return times;
}

std::vector<double> getIntegrationTimes(json isd, csm::WarningList *list) {
  std::vector<double> times;
  try {
    for (auto& scanRate : isd.at("line_scan_rate")) {
      times.push_back(scanRate[2]);
    }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the integration times in the integration time table.",
          "Utilities::getIntegrationTimes()"));
    }
  }
  return times;
}

int getSampleSumming(json isd, csm::WarningList *list) {
  int summing = 0;
  try {
    summing = isd.at("detector_sample_summing");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sample direction detector pixel summing.",
          "Utilities::getSampleSumming()"));
    }
  }
  return summing;
}

int getLineSumming(json isd, csm::WarningList *list) {
  int summing = 0;
  try {
    summing = isd.at("detector_line_summing");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the line direction detector pixel summing.",
          "Utilities::getLineSumming()"));
    }
  }
  return summing;
}

double getFocalLength(json isd, csm::WarningList *list) {
  double length = 0.0;
  try {
    length = isd.at("focal_length_model").at("focal_length");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the focal length.",
          "Utilities::getFocalLength()"));
    }
  }
  return length;
}

double getFocalLengthEpsilon(json isd, csm::WarningList *list) {
  double epsilon = 0.0;
  try {
    epsilon = isd.at("focal_length_model").at("focal_epsilon");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the focal length uncertainty.",
          "Utilities::getFocalLengthEpsilon()"));
    }
  }
  return epsilon;
}

std::vector<double> getFocal2PixelLines(json isd, csm::WarningList *list) {
  std::vector<double> transformation;
  try {
    transformation = isd.at("focal2pixel_lines").get<std::vector<double>>();
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the focal plane coordinate to detector lines transformation.",
          "Utilities::getFocal2PixelLines()"));
    }
  }
  return transformation;
}

std::vector<double> getFocal2PixelSamples(json isd, csm::WarningList *list) {
  std::vector<double> transformation;
  try {
    transformation = isd.at("focal2pixel_samples").get<std::vector<double>>();
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the focal plane coordinate to detector samples transformation.",
          "Utilities::getFocal2PixelSamples()"));
    }
  }
  return transformation;
}

double getDetectorCenterLine(json isd, csm::WarningList *list) {
  double line;
  try {
    line = isd.at("detector_center").at("line");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the detector center line.",
          "Utilities::getDetectorCenterLine()"));
    }
  }
  return line;
}

double getDetectorCenterSample(json isd, csm::WarningList *list) {
  double sample;
  try {
    sample = isd.at("detector_center").at("sample");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the detector center sample.",
          "Utilities::getDetectorCenterSample()"));
    }
  }
  return sample;
}


double getDetectorStartingLine(json isd, csm::WarningList *list) {
  double line;
  try {
    line = isd.at("starting_detector_line");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the detector starting line.",
          "Utilities::getDetectorStartingLine()"));
    }
  }
  return line;
}

double getDetectorStartingSample(json isd, csm::WarningList *list) {
  double sample;
  try {
    sample = isd.at("starting_detector_sample");
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the detector starting sample.",
          "Utilities::getDetectorStartingSample()"));
    }
  }
  return sample;
}

double getMinHeight(json isd, csm::WarningList *list) {
  double height = 0.0;
  try {
    json referenceHeight = isd.at("reference_height");
    json minHeight = referenceHeight.at("minheight");
    json unit = referenceHeight.at("unit");
    height = metric_conversion(minHeight.get<double>(), unit.get<std::string>());
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the minimum height above the reference ellipsoid.",
          "Utilities::getMinHeight()"));
    }
  }
  return height;
}

double getMaxHeight(json isd, csm::WarningList *list) {
  double height = 0.0;
  try {
    json referenceHeight = isd.at("reference_height");
    json maxHeight = referenceHeight.at("maxheight");
    json unit = referenceHeight.at("unit");
    height = metric_conversion(maxHeight.get<double>(), unit.get<std::string>());
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the maximum height above the reference ellipsoid.",
          "Utilities::getMaxHeight()"));
    }
  }
  return height;
}

double getSemiMajorRadius(json isd, csm::WarningList *list) {
  double radius = 0.0;
  try {
    json radii = isd.at("radii");
    json semiMajor = radii.at("semimajor");
    json unit = radii.at("unit");
    radius = metric_conversion(semiMajor.get<double>(), unit.get<std::string>());
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the reference ellipsoid semimajor radius.",
          "Utilities::getSemiMajorRadius()"));
    }
  }
  return radius;
}


double getSemiMinorRadius(json isd, csm::WarningList *list) {
  double radius = 0.0;
  try {
    json radii = isd.at("radii");
    json semiMinor = radii.at("semiminor");
    json unit = radii.at("unit");
    radius = metric_conversion(semiMinor.get<double>(), unit.get<std::string>());
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the reference ellipsoid semiminor radius.",
          "Utilities::getSemiMinorRadius()"));
    }
  }
  return radius;
}


// Converts the distortion model name from the ISD (string) to the enumeration
// type. Defaults to transverse
DistortionType getDistortionModel(json isd, csm::WarningList *list) {
  try {
    json distoriton_subset = isd.at("optical_distortion");

    json::iterator it = distoriton_subset.begin();

    std::string distortion = (std::string)it.key();

    if (distortion.compare("transverse") == 0) {
      return DistortionType::TRANSVERSE;
    }
    else if (distortion.compare("radial") == 0) {
      return DistortionType::RADIAL;
    }
    else if (distortion.compare("kaguyatc") == 0) {
      return DistortionType::KAGUYATC;
    }
    else if (distortion.compare("dawnfc") == 0) {
      return DistortionType::DAWNFC;
    }
    else if (distortion.compare("lrolrocnac") == 0) {
      return DistortionType::LROLROCNAC;
    }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the distortion model.",
          "Utilities::getDistortionModel()"));
    }
  }
  return DistortionType::TRANSVERSE;

}

std::vector<double> getDistortionCoeffs(json isd, csm::WarningList *list) {
  std::vector<double> coefficients;

  DistortionType distortion = getDistortionModel(isd);

  switch (distortion) {
    case DistortionType::TRANSVERSE: {
      try {
        std::vector<double> coefficientsX, coefficientsY;

        coefficientsX = isd.at("optical_distortion").at("transverse").at("x").get<std::vector<double>>();
        coefficientsX.resize(10, 0.0);

        coefficientsY = isd.at("optical_distortion").at("transverse").at("y").get<std::vector<double>>();
        coefficientsY.resize(10, 0.0);

        coefficients = coefficientsX;

        coefficients.insert(coefficients.end(), coefficientsY.begin(), coefficientsY.end());
        return coefficients;
      }
      catch (...) {
        if (list) {
          list->push_back(
            csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse a set of transverse distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(20, 0.0);
        coefficients[1] = 1.0;
        coefficients[12] = 1.0;
      }
    }
    break;
    case DistortionType::RADIAL: {
      try {
        coefficients = isd.at("optical_distortion").at("radial").at("coefficients").get<std::vector<double>>();

        return coefficients;
      }
      catch (...) {
        if (list) {
          list->push_back(
            csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the radial distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(3, 0.0);
      }
    }
    break;
    case DistortionType::KAGUYATC: {
      try {
        std::vector<double> coefficientsX(4,0);
        std::vector<double> coefficientsY(4,0);

        coefficientsX = isd.at("optical_distortion").at("kaguyatc").at("x").get<std::vector<double>>();
        coefficientsY = isd.at("optical_distortion").at("kaguyatc").at("y").get<std::vector<double>>();
        coefficientsX.insert(coefficientsX.end(), coefficientsY.begin(), coefficientsY.end());

        return coefficientsX;
      }
      catch (...) {
        if (list) {
          list->push_back(
            csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse a set of transverse distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(8, 0.0);
      }
    }
    break;
    case DistortionType::DAWNFC: {
      try {
        coefficients = isd.at("optical_distortion").at("dawnfc").at("coefficients").get<std::vector<double>>();

        return coefficients;
      }
      catch (...) {
        if (list) {
          list->push_back(
            csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the dawn radial distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(1, 0.0);
      }
    }
    break;
    case DistortionType::LROLROCNAC: {
      try {
        coefficients = isd.at("optical_distortion").at("lrolrocnac").at("coefficients").get<std::vector<double>>();
        return coefficients;
      }
      catch (...) {
        if (list) {
          list->push_back(
            csm::Warning(
              csm::Warning::DATA_NOT_AVAILABLE,
              "Could not parse the lrolrocnac distortion model coefficients.",
              "Utilities::getDistortion()"));
        }
        coefficients = std::vector<double>(1, 0.0);
      }
    }
    break;
  }
  if (list) {
    list->push_back(
      csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE,
        "Could not parse the distortion model coefficients.",
        "Utilities::getDistortion()"));
  }

  return coefficients;
}

std::vector<double> getSunPositions(json isd, csm::WarningList *list) {
  std::vector<double> positions;
  try {
    json jayson = isd.at("sun_position");
    json unit = jayson.at("unit");
    for (auto& location : jayson.at("positions")) {
      positions.push_back(metric_conversion(location[0].get<double>(), unit.get<std::string>()));
      positions.push_back(metric_conversion(location[1].get<double>(), unit.get<std::string>()));
      positions.push_back(metric_conversion(location[2].get<double>(), unit.get<std::string>()));
    }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sun positions.",
          "Utilities::getSunPositions()"));
    }
  }
  return positions;
}

std::vector<double> getSensorPositions(json isd, csm::WarningList *list) {
  std::vector<double> positions;
  try {
    json jayson = isd.at("sensor_position");
    json unit = jayson.at("unit");
    for (auto& location : jayson.at("positions")) {
      positions.push_back(metric_conversion(location[0].get<double>(), unit.get<std::string>()));
      positions.push_back(metric_conversion(location[1].get<double>(), unit.get<std::string>()));
      positions.push_back(metric_conversion(location[2].get<double>(), unit.get<std::string>()));
    }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sensor positions.",
          "Utilities::getSensorPositions()"));
    }
  }
  return positions;
}

std::vector<double> getSensorVelocities(json isd, csm::WarningList *list) {
  std::vector<double> velocities;
  try {
    json jayson = isd.at("sensor_position");
    json unit = jayson.at("unit");
    for (auto& velocity : jayson.at("velocities")) {
      velocities.push_back(metric_conversion(velocity[0].get<double>(), unit.get<std::string>()));
      velocities.push_back(metric_conversion(velocity[1].get<double>(), unit.get<std::string>()));
      velocities.push_back(metric_conversion(velocity[2].get<double>(), unit.get<std::string>()));
    }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sensor velocities.",
          "Utilities::getSensorVelocities()"));
    }
  }
  return velocities;
}

std::vector<double> getSensorOrientations(json isd, csm::WarningList *list) {
  std::vector<double> quaternions;
  try {
    for (auto& quaternion : isd.at("sensor_orientation").at("quaternions")) {
      quaternions.push_back(quaternion[0]);
      quaternions.push_back(quaternion[1]);
      quaternions.push_back(quaternion[2]);
      quaternions.push_back(quaternion[3]);
     }
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the sensor orientations.",
          "Utilities::getSensorOrientations()"));
    }
  }
  return quaternions;
}
