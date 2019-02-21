#include "Utilities.h"

using json = nlohmann::json;

// Calculates a rotation matrix from Euler angles
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

void computeDistortedFocalPlaneCoordinates(
    const double& line,
    const double& sample,
    const double& sampleOrigin,
    const double& lineOrigin,
    const double& sampleSumming,
    const double& startingSample,
    const double& lineOffset,
    const double iTransS[],
    const double iTransL[],
    std::tuple<double, double>& natFocalPlane)
{
  double detSample = sample * sampleSumming + startingSample;
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

  std::get<0>(natFocalPlane) = p11 * t1 + p12 * t2;
  std::get<1>(natFocalPlane) = p21 * t1 + p22 * t2;
};

// Define imaging ray in image space (In other words, create a look vector in camera space)
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

std::vector<double> getTransverseDistortionX(json isd, csm::WarningList *list) {
  std::vector<double> coefficients;
  try {
    coefficients = isd.at("optical_distortion").at("transverse").at("x").get<std::vector<double>>();
    coefficients.resize(10, 0.0);
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the transverse distortion model X coefficients.",
          "Utilities::getTransverseDistortionX()"));
    }
  }
  return coefficients;
}

std::vector<double> getTransverseDistortionY(json isd, csm::WarningList *list) {
  std::vector<double> coefficients;
  try {
    coefficients = isd.at("optical_distortion").at("transverse").at("y").get<std::vector<double>>();
    coefficients.resize(10, 0.0);
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the transverse distortion model Y coefficients.",
          "Utilities::getTransverseDistortionY()"));
    }
  }
  return coefficients;
}

std::vector<double> getRadialDistortion(json isd, csm::WarningList *list) {
  std::vector<double> coefficients;
  try {
    coefficients = isd.at("optical_distortion").at("radial").at("coefficients").get<std::vector<double>>();
  }
  catch (...) {
    if (list) {
      list->push_back(
        csm::Warning(
          csm::Warning::DATA_NOT_AVAILABLE,
          "Could not parse the radial distortion model coefficients.",
          "Utilities::getRadialDistortion()"));
    }
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
