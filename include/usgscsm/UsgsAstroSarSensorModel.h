#ifndef __USGS_ASTRO_SAR_SENSORMODEL_H
#define __USGS_ASTRO_SAR_SENSORMODEL_H

#include <RasterGM.h>
#include <SettableEllipsoid.h>
#include <CorrelationModel.h>

class UsgsAstroSarSensorModel : public csm::RasterGM, virtual public csm::SettableEllipsoid
{

  public:
    enum LookDirection {
      LEFT  = 0,
      RIGHT = 1
    };

    UsgsAstroSarSensorModel();
    ~UsgsAstroSarSensorModel() {}

    void reset();

    virtual void replaceModelState(const std::string& argState);

    virtual std::string getModelState() const;

    static std::string constructStateFromIsd(const std::string imageSupportData, csm::WarningList *list);

    static std::string getModelNameFromModelState(const std::string& model_state);

    virtual csm::ImageCoord groundToImage(
        const csm::EcefCoord& groundPt,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual csm::ImageCoordCovar groundToImage(
        const csm::EcefCoordCovar& groundPt,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual csm::EcefCoord imageToGround(
        const csm::ImageCoord& imagePt,
        double height,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual csm::EcefCoordCovar imageToGround(
        const csm::ImageCoordCovar& imagePt,
        double height,
        double heightVariance,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual csm::EcefLocus imageToProximateImagingLocus(
        const csm::ImageCoord& imagePt,
        const csm::EcefCoord& groundPt,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual csm::EcefLocus imageToRemoteImagingLocus(
        const csm::ImageCoord& imagePt,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual csm::ImageCoord getImageStart() const;

    virtual csm::ImageVector getImageSize() const;

    virtual std::pair<csm::ImageCoord, csm::ImageCoord> getValidImageRange() const;

    virtual std::pair<double, double> getValidHeightRange() const;

    virtual csm::EcefVector getIlluminationDirection(const csm::EcefCoord& groundPt) const;

    virtual double getImageTime(const csm::ImageCoord& imagePt) const;

    virtual csm::EcefCoord getSensorPosition(const csm::ImageCoord& imagePt) const;

    virtual csm::EcefCoord getSensorPosition(double time) const;

    virtual csm::EcefVector getSensorVelocity(const csm::ImageCoord& imagePt) const;

    virtual csm::EcefVector getSensorVelocity(double time) const;

    virtual csm::RasterGM::SensorPartials computeSensorPartials(
        int index,
        const csm::EcefCoord& groundPt,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual csm::RasterGM::SensorPartials computeSensorPartials(
        int index,
        const csm::ImageCoord& imagePt,
        const csm::EcefCoord& groundPt,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual std::vector<csm::RasterGM::SensorPartials> computeAllSensorPartials(
        const csm::EcefCoord& groundPt,
        csm::param::Set pSet = csm::param::VALID,
        double desiredPrecision = 0.001,
        double* achievedPrecision = NULL,
        csm::WarningList* warnings = NULL) const;

    virtual std::vector<double> computeGroundPartials(const csm::EcefCoord& groundPt) const;

    virtual const csm::CorrelationModel& getCorrelationModel() const;

    virtual std::vector<double> getUnmodeledCrossCovariance(
        const csm::ImageCoord& pt1,
        const csm::ImageCoord& pt2) const;

    virtual csm::EcefCoord getReferencePoint() const;

    virtual void setReferencePoint(const csm::EcefCoord& groundPt);

    virtual int getNumParameters() const;

    virtual std::string getParameterName(int index) const;

    virtual std::string getParameterUnits(int index) const;

    virtual bool hasShareableParameters() const;

    virtual bool isParameterShareable(int index) const;

    virtual csm::SharingCriteria getParameterSharingCriteria(int index) const;

    virtual double getParameterValue(int index) const;

    virtual void setParameterValue(int index, double value);

    virtual csm::param::Type getParameterType(int index) const;

    virtual void setParameterType(int index, csm::param::Type pType);

    virtual double getParameterCovariance(
        int index1,
        int index2) const;

    virtual void setParameterCovariance(
        int index1,
        int index2,
        double covariance);

    virtual int getNumGeometricCorrectionSwitches() const;

    virtual std::string getGeometricCorrectionName(int index) const;

    virtual void setGeometricCorrectionSwitch(int index,
        bool value,
        csm::param::Type pType);

    virtual bool getGeometricCorrectionSwitch(int index) const;

    virtual std::vector<double> getCrossCovarianceMatrix(
        const csm::GeometricModel& comparisonModel,
        csm::param::Set pSet = csm::param::VALID,
        const csm::GeometricModel::GeometricModelList& otherModels = csm::GeometricModel::GeometricModelList()) const;

    virtual csm::Version getVersion() const;

    virtual std::string getModelName() const;

    virtual std::string getPedigree() const;

    virtual std::string getImageIdentifier() const;

    virtual void setImageIdentifier(
        const std::string& imageId,
        csm::WarningList* warnings = NULL);

    virtual std::string getSensorIdentifier() const;

    virtual std::string getPlatformIdentifier() const;

    virtual std::string getCollectionIdentifier() const;

    virtual std::string getTrajectoryIdentifier() const;

    virtual std::string getSensorType() const;

    virtual std::string getSensorMode() const;

    virtual std::string getReferenceDateAndTime() const;

    virtual csm::Ellipsoid getEllipsoid() const;

    virtual void setEllipsoid(const csm::Ellipsoid &ellipsoid);

    ////////////////////
    // Helper methods //
    ////////////////////
    void determineSensorCovarianceInImageSpace(
       csm::EcefCoord &gp,
       double          sensor_cov[4]) const;
    double dopplerShift(csm::EcefCoord groundPt, double tolerance) const;

    double slantRange(csm::EcefCoord surfPt, double time) const;

    double slantRangeToGroundRange(const csm::EcefCoord& groundPt, double time, double slantRange, double tolerance) const;

    double groundRangeToSlantRange(double groundRange, const std::vector<double> &coeffs) const;

    csm::EcefVector getSpacecraftPosition(double time) const;

    csm::EcefVector getSunPosition(const double imageTime) const;

    std::vector<double> getRangeCoefficients(double time) const;

    ////////////////////////////
    // Model static variables //
    ////////////////////////////

    static const std::string      _SENSOR_MODEL_NAME;
    static const int              NUM_PARAM_TYPES;
    static const std::string      PARAM_STRING_ALL[];
    static const csm::param::Type PARAM_CHAR_ALL[];
    static const int              NUM_PARAMETERS;
    static const std::string      PARAMETER_NAME[];
    csm::NoCorrelationModel       _NO_CORR_MODEL; // A way to report no correlation between images is supported
    std::vector<double>           _NO_ADJUSTMENT;

    ///////////////////////////
    // Model state variables //
    ///////////////////////////
    std::string  m_imageIdentifier;
    std::string  m_platformName;
    std::string  m_sensorName;
    int          m_nLines;
    int          m_nSamples;
    double       m_exposureDuration;
    double       m_scaledPixelWidth;
    double       m_startingEphemerisTime;
    double       m_centerEphemerisTime;
    double       m_endingEphemerisTime;
    double       m_majorAxis;
    double       m_minorAxis;
    std::string  m_platformIdentifier;
    std::string  m_sensorIdentifier;
    std::string  m_trajectoryIdentifier;
    std::string  m_collectionIdentifier;
    double       m_refElevation;
    double       m_minElevation;
    double       m_maxElevation;
    double       m_dtEphem;
    double       m_t0Ephem;
    std::vector<double> m_scaleConversionCoefficients;
    std::vector<double> m_scaleConversionTimes;
    std::vector<double> m_positions;
    std::vector<double> m_velocities;
    std::vector<double> m_currentParameterValue;
    std::vector<csm::param::Type> m_parameterType;
    csm::EcefCoord m_referencePointXyz;
    std::vector<double> m_covariance;
    std::vector<double> m_sunPosition;
    std::vector<double> m_sunVelocity;
    double m_wavelength;
    LookDirection m_lookDirection;
};

#endif
