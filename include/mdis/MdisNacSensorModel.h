#ifndef MdisNacSensorModel_h
#define MdisNacSensorModel_h

#include <cmath>
#include <iostream>
#include <vector>

#include "csm/RasterGM.h"


class MdisNacSensorModel : public csm::RasterGM {
  // MdisPlugin needs to access private members
  friend class MdisPlugin;

  public:
    MdisNacSensorModel();
    ~MdisNacSensorModel();

    virtual csm::ImageCoord groundToImage(const csm::EcefCoord &groundPt,
                                     double desiredPrecision=0.001,
                                     double *achievedPrecision=NULL,
                                     csm::WarningList *warnings=NULL) const;

    virtual csm::ImageCoordCovar groundToImage(const csm::EcefCoordCovar &groundPt,
                                          double desiredPrecision=0.001,
                                          double *achievedPrecision=NULL,
                                          csm::WarningList *warnings=NULL) const;

    /**
    * This function determines if a sample, line intersects the target body and if so, where
    * this intersection occurs in body-fixed coordinates.
    *
    * @param sample Sample of the input image.
    * @param line Line of the input image.
    * @param height ???
    *
    * @return @b vector<double> Returns the body-fixed X,Y,Z coordinates of the intersection.
    *                           If no intersection, returns a 3-element vector of 0's.
    */
    virtual csm::EcefCoord imageToGround(const csm::ImageCoord &imagePt, double height,
                                    double desiredPrecision=0.001, double *achievedPrecision=NULL,
                                    csm::WarningList *warnings=NULL) const;

    virtual csm::EcefCoordCovar imageToGround(const csm::ImageCoordCovar &imagePt, double height,
                                           double heightVariance, double desiredPrecision=0.001,
                                           double *achievedPrecision=NULL,
                                           csm::WarningList *warnings=NULL) const;

    virtual csm::EcefLocus imageToProximateImagingLocus(const csm::ImageCoord &imagePt,
                                                      const csm::EcefCoord &groundPt,
                                                      double desiredPrecision=0.001,
                                                      double *achievedPrecision=NULL,
                                                      csm::WarningList *warnings=NULL) const;

    virtual csm::EcefLocus imageToRemoteImagingLocus(const csm::ImageCoord &imagePt,
                                                   double desiredPrecision=0.001,
                                                   double *achievedPrecision=NULL,
                                                   csm::WarningList *warnings=NULL) const;

    virtual csm::ImageCoord getImageStart() const;

    virtual csm::ImageVector getImageSize() const;

    virtual std::pair<csm::ImageCoord, csm::ImageCoord> getValidImageRange() const;

    virtual std::pair<double, double> getValidHeightRange() const;

    /**
     * Calculates the illumination vector (body-fixed, meters) from the sun to the given ground
     * point.
     * 
     * @param groundPt The ground point to find the illumination vector to.
     * 
     * @return @b csm::EcefVector Returns the illumination vector from the sun to the ground point.
     */
    virtual csm::EcefVector getIlluminationDirection(const csm::EcefCoord &groundPt) const;

    virtual double getImageTime(const csm::ImageCoord &imagePt) const;
  
    /**
     * Determines the body-fixed sensor position for the given image coordinate.
     * 
     * @param imagePt Image coordinate to find the sensor position for.
     * 
     * @return @b csm::EcefCoord Returns the body-fixed sensor position.
     * 
     * @throw csm::Error::BOUNDS "Image coordinate () out of bounds."
     */
    virtual csm::EcefCoord getSensorPosition(const csm::ImageCoord &imagePt) const;

    virtual csm::EcefCoord getSensorPosition(double time) const;                                                                                      

    /**
     * Determines the velocity of the sensor for the given image coordinate (in body-fixed frame).
     * 
     * @param imagePt Image coordinate to find the sensor position for.
     * 
     * @return @b csm::EcefVector Returns the sensor velocity in body-fixed frame.
     * 
     * @throw csm::Error::BOUNDS "Image coordinate () out of bounds."
     */
                        
    virtual csm::EcefVector getSensorVelocity(const csm::ImageCoord &imagePt) const;

    virtual csm::EcefVector getSensorVelocity(double time) const;

    virtual csm::RasterGM::SensorPartials computeSensorPartials(int index,
                                                                const csm::EcefCoord &groundPt,
                                                                double desiredPrecision=0.001,                                        
                                                                double *achievedPrecision=NULL,
                                                                csm::WarningList *warnings=NULL) const;

    virtual csm::RasterGM::SensorPartials computeSensorPartials(int index,
                                                                const csm::ImageCoord &imagePt,
                                                                const csm::EcefCoord &groundPt,
                                                                double desiredPrecision=0.001,
                                                                double *achievedPrecision=NULL,
                                                                csm::WarningList *warnings=NULL) const;

    virtual std::vector<double> computeGroundPartials(const csm::EcefCoord &groundPt) const;

    virtual const csm::CorrelationModel &getCorrelationModel() const;

    virtual std::vector<double> getUnmodeledCrossCovariance(const csm::ImageCoord &pt1,
                                                            const csm::ImageCoord &pt2) const;

    // IMPLEMENT MODEL PURE VIRTUALS
    //---
    // Basic model information
    //---
    virtual csm::Version getVersion() const;
      //> This method returns the version of the model code.  The Version
      //  object can be compared to other Version objects with its comparison
      //  operators.  Not to be confused with the CSM API version.
      //<

    virtual std::string getModelName() const;
      //> This method returns a string identifying the name of the model.
      //<

    virtual std::string getPedigree() const;
      //> This method returns a string that identifies the sensor,
      //  the model type, its mode of acquisition and processing path.
      //  For example, an optical sensor model or a cubic rational polynomial
      //  model created from the same sensor's support data would produce
      //  different pedigrees for each case.
      //<

    //---
    // Basic collection information
    //---
    virtual std::string getImageIdentifier() const;
      //> This method returns an identifier to uniquely indicate the imaging
      //  operation associated with this model.
      //  This is the primary identifier of the model.
      //
      //  This method may return an empty string if the ID is unknown.
      //<

    virtual void setImageIdentifier(const std::string& imageId,
                                    csm::WarningList* warnings = NULL);
      //> This method sets an identifier to uniquely indicate the imaging
      //  operation associated with this model.  Typically used for models
      //  whose initialization does not produce an adequate identifier.
      //
      //  If a non-NULL warnings argument is received, it will be populated
      //  as applicable.
      //<

    virtual std::string getSensorIdentifier() const;
      //> This method returns an identifier to indicate the specific sensor
      //  that was used to acquire the image.  This ID must be unique among
      //  sensors for a given model name.  It is used to determine parameter
      //  correlation and sharing.  Equivalent to camera or mission ID.
      //
      //  This method may return an empty string if the sensor ID is unknown.
      //<

    virtual std::string getPlatformIdentifier() const;
      //> This method returns an identifier to indicate the specific platform
      //  that was used to acquire the image.  This ID must unique among
      //  platforms for a given model name.  It is used to determine parameter
      //  correlation sharing.  Equivalent to vehicle or aircraft tail number.
      //
      //  This method may return an empty string if the platform ID is unknown.
      //<

    virtual std::string getCollectionIdentifier() const;
      //> This method returns an identifer to indicate a collection activity
      //  common to a set of images.  This ID must be unique among collection
      //  activities for a given model name.  It is used to determine parameter
      //  correlation and sharing.
      //<

    virtual std::string getTrajectoryIdentifier() const;
      //> This method returns an identifier to indicate a trajectory common
      //  to a set of images.  This ID must be unique among trajectories
      //  for a given model name.  It is used to determine parameter
      //  correlation and sharing.
      //<

    virtual std::string getSensorType() const;
      //> This method returns a description of the sensor type (EO, IR, SAR,
      //  etc).  See csm.h for a list of common types.  Should return
      //  CSM_SENSOR_TYPE_UNKNOWN if the sensor type is unknown.
      //<

    virtual std::string getSensorMode() const;
      //> This method returns a description of the sensor mode (FRAME,
      //  PUSHBROOM, SPOT, SCAN, etc).  See csm.h for a list of common modes.
      //  Should return CSM_SENSOR_MODE_UNKNOWN if the sensor mode is unknown.
      //<

    virtual std::string getReferenceDateAndTime() const;
      //> This method returns an approximate date and time at which the
      //  image was taken.  The returned string follows the ISO 8601 standard.
      //
      //-    Precision   Format           Example
      //-    year        yyyy             "1961"
      //-    month       yyyymm           "196104"
      //-    day         yyyymmdd         "19610420"
      //-    hour        yyyymmddThh      "19610420T20"
      //-    minute      yyyymmddThhmm    "19610420T2000"
      //-    second      yyyymmddThhmmss  "19610420T200000"
      //<

    //---
    // Sensor Model State
    //---
    virtual std::string getModelState() const;
      //> This method returns a string containing the data to exactly recreate
      //  the current model.  It can be used to restore this model to a
      //  previous state with the replaceModelState method or create a new
      //  model object that is identical to this model.
      //  The string could potentially be saved to a file for later use.
      //  An empty string is returned if it is not possible to save the
      //  current state.
      //<

    virtual void replaceModelState(const std::string& argState);
      //> This method attempts to initialize the current model with the state
      //  given by argState.  The argState argument can be a string previously
      //  retrieved from the getModelState method.
      //
      //  If argState contains a valid state for the current model,
      //  the internal state of the model is updated.
      //
      //  If the model cannot be updated to the given state, a csm::Error is
      //  thrown and the internal state of the model is undefined.
      //
      //  If the argument state string is empty, the model remains unchanged.
      //<

    // IMPLEMENT GEOMETRICMODEL PURE VIRTUALS
    // See GeometricModel.h for documentation
    virtual csm::EcefCoord getReferencePoint() const;
    virtual void setReferencePoint(const csm::EcefCoord &groundPt);
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
    virtual double getParameterCovariance(int index1, int index2) const;
    virtual void setParameterCovariance(int index1, int index2, double covariance);
    virtual int getNumGeometricCorrectionSwitches() const;
    virtual std::string getGeometricCorrectionName(int index) const;
    virtual void setGeometricCorrectionSwitch(int index, bool value, csm::param::Type pType);
    virtual bool getGeometricCorrectionSwitch(int index) const;
    virtual std::vector<double> getCrossCovarianceMatrix(
        const GeometricModel &comparisonModel,
        csm::param::Set pSet = csm::param::VALID,
        const GeometricModelList &otherModels = GeometricModelList()) const;

    static const std::string _SENSOR_MODEL_NAME;





  protected:

    virtual bool setFocalPlane(double dx,double dy,double &undistortedX,double &undistortedY) const;
    virtual void distortionFunction(double ux, double uy, double &dx, double &dy) const;
    virtual void distortionJacobian(double x, double y, double &Jxx,
                                    double &Jxy, double &Jyx, double &Jyy) const;

  private:

    double m_transX[3];
    double m_transY[3];
    double m_majorAxis;
    double m_minorAxis;
    double m_omega;
    double m_phi;
    double m_kappa;
    double m_focalLength;
    double m_spacecraftPosition[3];
    double m_spacecraftVelocity[3];
    double m_sunPosition[3];
    double m_ccdCenter[2];
    double m_line_pp;
    double m_sample_pp;
    double m_startingDetectorSample;
    double m_startingDetectorLine;
    std::string m_targetName;
    double m_ifov;
    std::string m_instrumentID;
    double m_focalLengthEpsilon;
    double m_odtX[10];
    double m_odtY[10];
    double m_originalHalfLines;
    std::string m_spacecraftName;
    double m_pixelPitch;
    double m_iTransS[3];
    double m_iTransL[3];
    double m_ephemerisTime;
    double m_originalHalfSamples;
    double m_boresight[3];
    int m_nLines;
    int m_nSamples;
  
    void calcRotationMatrix(double m[3][3]) const;
    void losEllipsoidIntersect (const double& height,const double& xc,
                                const double& yc, const double& zc,
                                const double& xl, const double& yl,
                                const double& zl,
                                double& x,double& y, double&  z) const;
};

#endif
