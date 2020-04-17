#include "UsgsAstroSarSensorModel.h"

#define MESSAGE_LOG(logger, ...) if (logger) { logger->info(__VA_ARGS__); }

using namespace std;

//UsgsAstroSarSensorModel();
//~UsgsAstroSarSensorModel();
//
//virtual void replaceModelState(const std::string& argState);
//
//virtual std::string getModelState() const;
//
//std::string constructStateFromIsd(const std::string imageSupportData, csm::WarningList *list) const;
//
//static std::string getModelNameFromModelState(const std::string& model_state);

csm::ImageCoord UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoord& groundPt,
    double desiredPrecision = 0.001,
    double* achievedPrecision = NULL,
    csm::WarningList* warnings = NULL) const {

  MESSAGE_LOG(this->m_logger, "Computing groundToImage(ImageCoord) for {}, {}, {}, with desired precision {}",
              groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  // Find time of closest approach to groundPt and the corresponding slant range by finding 
  // the root of the doppler shift wavelength
  foundTime = dopplerShiftRoot(groundPt, time&, slantRange&);

  if (foundTime) {
    // Find the ground range, based on the ground-range-to-slant-range polynomial defined by the 
    // range coefficient set, with a time closest to the calculated time of closest approach
    groundRange = slantToGroundRange(groundPt, time, slantRange);

    if (groundRange =! -1) {
      // Construct a new point and return with:
      double line = 1 + time + startTime / exposureDuration;
      double sample = 1 + groundRange / scaledPixelWidth;
      return csm::ImageCoord(line, sample);
    }
  }
} 


bool UsgsAstroSarSensorModel::dopplerShiftRoot(){
  // Moon body-fixed coordinates of surface point
  surfPt = fixme; 

  // Lower-bound for doppler-shift
  startTime, range = dopperShift(surfPt, START_TIME);

  // Upper-bound for doppler-shift
  stopTime, range = dopplerShift(surfPt, STOP_TIME);

  // Make sure we bound root (Df= 0.0)
  if ((startTime < 0.0) && (stopTime < 0.0)) return (-1); // FAIL
  if ((startTime > 0.0) && (stopTime > 0.0)) return (-1); // FAIL

  // do root-finding for "dopplerShift"

  return true;//time, slantRange;
}


void UsgsAstroSarSensorModel::dopplerShift(
    something surfPt,
    double time,
    double dopplerShift&,
    double slantRange) {
  slantRange = spacecraftPosition(time); // different function, FIXME
  dopplerShift = -2.0 * spacecraftVelocity(time)/(slantRange * WAVELENGTH);
}


double UsgsAstroSarSensorModel::slantToGroundRange(
        const csm::EcefCoord& groundPt,
        double time,
        double slantRange) {
  // Need to come up with an initial guess when solving for ground 
  // range given slantrange. We will compute the ground range at the
  // near and far edges of the image by evaluating the sample-to-
  // ground-range equation: r_gnd=(S-1)*scaled_pixel_width
  // at the edges of the image. We also need to add some padding to
  // allow for solving for coordinates that are slightly outside of
  // the actual image area. Use S=-0.25*image_samples and
  // S=1.25*image_samples.
  double initialMinGroundRangeGuess = (-0.25 * SAMPLES - 1.0)
                                    * SCALED_PIXEL_WIDTH;
  double initialMaxGroundRangeGuess = (1.25 * SAMPLES - 1.0)
                                    * SCALED_PIXEL_WIDTH;

  a1, a2, a3, a4 = getRangeCoefficients(t_min);
    
  // Evaluate the ground range at the 2 extremes of the image
  double minGroundRangeGuess = r_slant - (a1 + initialMinGroundRangeGuess *
                                          (a2 + initialMinGroundRangeGuess * (a3 +
                             initialMinGroundRangeGuess * a4)));
  double maxGroundRangeGuess = r_slant - (a1 + initialMaxGroundRangeGuess *
                             (a2 + initialMaxGroundRangeGuess * (a3 +
                             initialMaxGroundRangeGuess * a4)));

  // If the ground range guesses at the 2 extremes of the image are equal
  // or they have the same sign, then the ground range cannot be solved for.
  // do we want to do this check or just have Brent's method fail? will it fail? 
  if !((minGroundRangeGuess == maxGroundRangeGuess) || 
      (minGroundRangeGuess < 0.0 && maxGroundRangeGuess < 0.0) ||
      (minGroundRangeGuess > 0.0 && maxGroundRangeGuess > 0.0)) {

    // Use Brent's algorithm to find a root of the function:
    // g(groundRange) = slantRange - (a1 + groundRange * (a2 +
    //                  groundRange * (a3 + groundRange * a4)))

    // do I actually want to use a lambda for this? what is most clear? 
    double groundRange = brentRoot(minGroundRange, maxGroundRange, 
              [](double slantRange, 
                 double groundRange, 
                 double a1, 
                 double a2, 
                 double a3, 
                 double a4){
      return slantRange - (a1 + groundRange * (a2 + groundRange * (a3 + groundRange * a4)));
    }
      , 0.1);
    return groundRange
  }
}

csm::ImageCoordCovar groundToImage(
    const csm::EcefCoordCovar& groundPt,
    double desiredPrecision = 0.001,
    double* achievedPrecision = NULL,
    csm::WarningList* warnings = NULL) const {

    MESSAGE_LOG(this->m_logger, "Computing groundToImage(Covar) for {}, {}, {}, with desired precision {}",
                groundPt.x, groundPt.y, groundPt.z, desiredPrecision);


}

//virtual csm::EcefCoord imageToGround(
//    const csm::ImageCoord& imagePt,
//    double height,
//    double desiredPrecision = 0.001,
//    double* achievedPrecision = NULL,
//    csm::WarningList* warnings = NULL) const;
//
//virtual csm::EcefCoordCovar imageToGround(
//    const csm::ImageCoordCovar& imagePt,
//    double height,
//    double heightVariance,
//    double desiredPrecision = 0.001,
//    double* achievedPrecision = NULL,
//    csm::WarningList* warnings = NULL) const;
//
//virtual csm::EcefLocus imageToProximateImagingLocus(
//    const csm::ImageCoord& imagePt,
//    const csm::EcefCoord& groundPt,
//    double desiredPrecision = 0.001,
//    double* achievedPrecision = NULL,
//    csm::WarningList* warnings = NULL) const;
//
//virtual csm::EcefLocus imageToRemoteImagingLocus(
//    const csm::ImageCoord& imagePt,
//    double desiredPrecision = 0.001,
//    double* achievedPrecision = NULL,
//    csm::WarningList* warnings = NULL) const;
//
//virtual csm::ImageCoord getImageStart() const;
//
//virtual csm::ImageVector getImageSize() const;
//
//virtual std::pair<csm::ImageCoord, csm::ImageCoord> getValidImageRange() const;
//
//virtual std::pair<double, double> getValidHeightRange() const;
//
//virtual csm::EcefVector getIlluminationDirection(const csm::EcefCoord& groundPt) const;
//
//virtual double getImageTime(const csm::ImageCoord& imagePt) const;
//
//virtual csm::EcefCoord getSensorPosition(const csm::ImageCoord& imagePt) const;
//
//virtual csm::EcefCoord getSensorPosition(double time) const;
//
//virtual csm::EcefVector getSensorVelocity(const csm::ImageCoord& imagePt) const;
//
//virtual csm::EcefVector getSensorVelocity(double time) const;
//
//virtual csm::RasterGM::SensorPartials computeSensorPartials(
//    int index,
//    const csm::EcefCoord& groundPt,
//    double desiredPrecision = 0.001,
//    double* achievedPrecision = NULL,
//    csm::WarningList* warnings = NULL) const;
//
//virtual csm::RasterGM::SensorPartials computeSensorPartials(
//    int index,
//    const csm::ImageCoord& imagePt,
//    const csm::EcefCoord& groundPt,
//    double desiredPrecision = 0.001,
//    double* achievedPrecision = NULL,
//    csm::WarningList* warnings = NULL) const;
//
//virtual std::vector<csm::RasterGM::SensorPartials> computeAllSensorPartials(
//    const csm::EcefCoord& groundPt,
//    csm::param::Set pSet = csm::param::VALID,
//    double desiredPrecision = 0.001,
//    double* achievedPrecision = NULL,
//    csm::WarningList* warnings = NULL) const;
//
//virtual std::vector<double> computeGroundPartials(const csm::EcefCoord& groundPt) const;
//
//virtual const csm::CorrelationModel& getCorrelationModel() const;
//
//virtual std::vector<double> getUnmodeledCrossCovariance(
//    const csm::ImageCoord& pt1,
//    const csm::ImageCoord& pt2) const;
//
//virtual csm::EcefCoord getReferencePoint() const;
//
//virtual void setReferencePoint(const csm::EcefCoord& groundPt);
//
//virtual int getNumParameters() const;
//
//virtual std::string getParameterName(int index) const;
//
//virtual std::string getParameterUnits(int index) const;
//
//virtual bool hasShareableParameters() const;
//
//virtual bool isParameterShareable(int index) const;
//
//virtual csm::SharingCriteria getParameterSharingCriteria(int index) const;
//
//virtual double getParameterValue(int index) const;
//
//virtual void setParameterValue(int index, double value);
//
//virtual csm::param::Type getParameterType(int index) const;
//
//virtual void setParameterType(int index, csm::param::Type pType);
//
//virtual double getParameterCovariance(
//    int index1,
//    int index2) const;
//
//virtual void setParameterCovariance(
//    int index1,
//    int index2,
//    double covariance);
//
//virtual int getNumGeometricCorrectionSwitches() const;
//
//virtual std::string getGeometricCorrectionName(int index) const;
//
//virtual void setGeometricCorrectionSwitch(int index,
//    bool value,
//    csm::param::Type pType);
//
//virtual bool getGeometricCorrectionSwitch(int index) const;
//
//virtual std::vector<double> getCrossCovarianceMatrix(
//    const csm::GeometricModel& comparisonModel,
//    csm::param::Set pSet = csm::param::VALID,
//    const csm::GeometricModel::GeometricModelList& otherModels = csm::GeometricModel::GeometricModelList()) const;
//
//virtual csm::Version getVersion() const;
//
//virtual std::string getModelName() const;
//
//virtual std::string getPedigree() const;
//
//virtual std::string getImageIdentifier() const;
//
//virtual void setImageIdentifier(
//    const std::string& imageId,
//    csm::WarningList* warnings = NULL);
//
//virtual std::string getSensorIdentifier() const;
//
//virtual std::string getPlatformIdentifier() const;
//
//virtual std::string getCollectionIdentifier() const;
//
//virtual std::string getTrajectoryIdentifier() const;
//
//virtual std::string getSensorType() const;
//
//virtual std::string getSensorMode() const;
//
//virtual std::string getReferenceDateAndTime() const;
//
//virtual csm::Ellipsoid getEllipsoid() const;
//
//virtual void setEllipsoid(const csm::Ellipsoid &ellipsoid);
//
//////////////////////////////
//// Model static variables //
//////////////////////////////
//
//static const std::string      _SENSOR_MODEL_NAME;
//static const std::string      _STATE_KEYWORD[];
//static const int              NUM_PARAM_TYPES;
//static const std::string      PARAM_STRING_ALL[];
//static const csm::param::Type PARAM_CHAR_ALL[];
//static const int              NUM_PARAMETERS;
//static const std::string      PARAMETER_NAME[];
//csm::NoCorrelationModel       _NO_CORR_MODEL; // A way to report no correlation between images is supported
//std::vector<double>           _NO_ADJUSTMENT;
//
/////////////////////////////
//// Model state variables //
/////////////////////////////
//std::string  m_imageIdentifier;
//std::string  m_sensorName;
//int          m_nLines;
//int          m_nSamples;
//double       m_exposureDuration;
//double       m_scaledPixelWidth;
//double       m_startingEphemerisTime;
//double       m_centerEphemerisTime;
//double       m_majorAxis;
//double       m_minorAxis;
//std::string  m_referenceDateAndTime;
//std::string  m_platformIdentifier;
//std::string  m_sensorIdentifier;
//std::string  m_trajectoryIdentifier;
//std::string  m_collectionIdentifier;
//double       m_refElevation;
//double       m_minElevation;
//double       m_maxElevation;
//double       m_dtEphem;
//double       m_t0Ephem;
//std::vector<double> m_scaleConversionCoefficients;
//std::vector<double> m_positions;
//std::vector<double> m_velocities;
//std::vector<double> m_currentParameterValue;
//std::vector<csm::param::Type> m_parameterType;
//csm::EcefCoord m_referencePointXyz;
//std::vector<double> m_covariance;
//std::vector<double> m_sunPosition;
//std::vector<double> m_sunVelocity;
//
//
