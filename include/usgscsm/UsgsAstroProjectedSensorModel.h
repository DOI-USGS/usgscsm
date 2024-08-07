/** Copyright  © 2017-2022 BAE Systems Information and Electronic Systems Integration Inc.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. **/

#ifndef INCLUDE_USGSCSM_USGSASTROPROJECTEDSENSORMODEL_H_
#define INCLUDE_USGSCSM_USGSASTROPROJECTEDSENSORMODEL_H_

#include <RasterGM.h>
#include <SettableEllipsoid.h>

#include<utility>
#include<memory>
#include<string>
#include<vector>

#include "ale/Orientations.h"
#include "ale/States.h"

#include "spdlog/spdlog.h"

#include <proj.h>

#include "UsgsAstroLsSensorModel.h"

class UsgsAstroProjectedSensorModel : public csm::RasterGM,
                                      virtual public csm::SettableEllipsoid {

public:
    // Initializes the class from state data as formatted
    // in a string by the toString() method
    void setState(const std::string &state);

    virtual void replaceModelState(const std::string &stateString);
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

    // This method checks to see if the model name is recognized
    // in the input state string.
    static std::string getModelNameFromModelState(const std::string &model_state);

    std::string constructStateFromIsd(const std::string imageSupportData, 
                                      const std::string modelName,
                                      csm::WarningList *list);

    // State data elements;
    double m_majorAxis;
    double m_minorAxis;
    PJ *m_isdProj = NULL;
    PJ *m_ecefProj = NULL;
    PJ *m_isdProj2ecefProj = NULL;
    std::vector<double> m_geoTransform;
    std::string m_projString;
    std::string m_subModelName;

    // Define logging pointer and file content
    std::shared_ptr<spdlog::logger> m_logger = spdlog::get("usgscsm_logger");

    // Hardcoded
    static const std::string _SENSOR_MODEL_NAME; // state date element 0

    static const std::string _STATE_KEYWORD[];

    // Set to default values
    void reset();

    //--------------------------------------------------------------
    // Constructors/Destructor
    //--------------------------------------------------------------

    UsgsAstroProjectedSensorModel();
    ~UsgsAstroProjectedSensorModel();

    virtual std::string getModelState() const;

    // Set the sensor model based on the input state data
    void set(const std::string &state_data);

    //----------------------------------------------------------------
    // The following public methods are implementations of
    // the methods inherited from RasterGM and SettableEllipsoid.
    // These are defined in the CSM API.
    //----------------------------------------------------------------

    //---
    // Core Photogrammetry
    //---
    virtual csm::ImageCoord groundToImage(
        const csm::EcefCoord &groundPt, double desiredPrecision = 0.001,
        double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;

    //> This method converts the given groundPt (x,y,z in ECEF meters) to a
    //  returned image coordinate (line, sample in full image space pixels).
    //
    //  Iterative algorithms will use desiredPrecision, in meters, as the
    //  convergence criterion, otherwise it will be ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the actual precision, in meters, achieved by iterative
    //  algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //<

    virtual csm::ImageCoordCovar groundToImage(
        const csm::EcefCoordCovar &groundPt, double desiredPrecision = 0.001,
        double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This method converts the given groundPt (x,y,z in ECEF meters and
    //  corresponding 3x3 covariance in ECEF meters squared) to a returned
    //  image coordinate with covariance (line, sample in full image space
    //  pixels and corresponding 2x2 covariance in pixels squared).
    //
    //  Iterative algorithms will use desiredPrecision, in meters, as the
    //  convergence criterion, otherwise it will be ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the actual precision, in meters, achieved by iterative
    //  algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //<

    virtual csm::EcefCoord imageToGround(const csm::ImageCoord &imagePt,
                                         double height,
                                         double desiredPrecision = 0.001,
                                         double *achievedPrecision = NULL,
                                         csm::WarningList *warnings = NULL) const;
    //> This method converts the given imagePt (line,sample in full image
    //  space pixels) and given height (in meters relative to the WGS-84
    //  ellipsoid) to a returned ground coordinate (x,y,z in ECEF meters).
    //
    //  Iterative algorithms will use desiredPrecision, in meters, as the
    //  convergence criterion, otherwise it will be ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the actual precision, in meters, achieved by iterative
    //  algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //<

    virtual csm::EcefCoordCovar imageToGround(
        const csm::ImageCoordCovar &imagePt, double height, double heightVariance,
        double desiredPrecision = 0.001, double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This method converts the given imagePt (line, sample in full image
    //  space pixels and corresponding 2x2 covariance in pixels squared)
    //  and given height (in meters relative to the WGS-84 ellipsoid) and
    //  corresponding heightVariance (in meters) to a returned ground
    //  coordinate with covariance (x,y,z in ECEF meters and corresponding
    //  3x3 covariance in ECEF meters squared).
    //
    //  Iterative algorithms will use desiredPrecision, in meters, as the
    //  convergence criterion, otherwise it will be ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the actual precision, in meters, achieved by iterative
    //  algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //<

    virtual csm::EcefLocus imageToProximateImagingLocus(
        const csm::ImageCoord &imagePt, const csm::EcefCoord &groundPt,
        double desiredPrecision = 0.001, double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This method, for the given imagePt (line, sample in full image space
    //  pixels), returns the position and direction of the imaging locus
    //  nearest the given groundPt (x,y,z in ECEF meters).
    //
    //  Note that there are two opposite directions possible.  Both are
    //  valid, so either can be returned; the calling application can convert
    //  to the other as necessary.
    //
    //  Iterative algorithms will use desiredPrecision, in meters, as the
    //  convergence criterion for the locus position, otherwise it will be
    //  ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the actual precision, in meters, achieved by iterative
    //  algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //<

    virtual csm::EcefLocus imageToRemoteImagingLocus(
        const csm::ImageCoord &imagePt, double desiredPrecision = 0.001,
        double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This method, for the given imagePt (line, sample in full image space
    //  pixels), returns the position and direction of the imaging locus
    //  at the sensor.
    //
    //  Note that there are two opposite directions possible.  Both are
    //  valid, so either can be returned; the calling application can convert
    //  to the other as necessary.
    //
    //  Iterative algorithms will use desiredPrecision, in meters, as the
    //  convergence criterion for the locus position, otherwise it will be
    //  ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the actual precision, in meters, achieved by iterative
    //  algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //
    //  Notes:
    //
    //  The remote imaging locus is only well-defined for optical sensors.
    //  It is undefined for SAR sensors and might not be available for
    //  polynomial and other non-physical models.  The
    //  imageToProximateImagingLocus method should be used instead where
    //  possible.
    //<

    //---
    // Monoscopic Mensuration
    //---
    virtual csm::ImageCoord getImageStart() const;
    //> This method returns the starting coordinate (line, sample in full
    //  image space pixels) for the imaging operation.  Typically (0,0).
    //<

    virtual csm::ImageVector getImageSize() const;
    //> This method returns the number of lines and samples in full image
    //  space pixels for the imaging operation.
    //
    //  Note that the model might not be valid over the entire imaging
    //  operation.  Use getValidImageRange() to get the valid range of image
    //  coordinates.
    //<

    virtual std::pair<csm::ImageCoord, csm::ImageCoord> getValidImageRange()
        const;
    //> This method returns the minimum and maximum image coordinates
    //  (line, sample in full image space pixels), respectively, over which
    //  the current model is valid.  The image coordinates define opposite
    //  corners of a rectangle whose sides are parallel to the line and
    //  sample axes.
    //
    //  The valid image range does not always match the full image
    //  coverage as returned by the getImageStart and getImageSize methods.
    //
    //  Used in conjunction with the getValidHeightRange method, it is
    //  possible to determine the full range of ground coordinates over which
    //  the model is valid.
    //<

    virtual std::pair<double, double> getValidHeightRange() const;
    //> This method returns the minimum and maximum heights (in meters
    //  relative to WGS-84 ellipsoid), respectively, over which the model is
    //  valid.  For example, a model for an airborne platform might not be
    //  designed to return valid coordinates for heights above the aircraft.
    //
    //  If there are no limits defined for the model, (-99999.0,99999.0)
    //  will be returned.
    //<

    virtual csm::EcefVector getIlluminationDirection(
        const csm::EcefCoord &groundPt) const;
    //> This method returns a vector defining the direction of
    //  illumination at the given groundPt (x,y,z in ECEF meters).
    //  Note that there are two opposite directions possible.  Both are
    //  valid, so either can be returned; the calling application can convert
    //  to the other as necessary.
    //<

    //---
    // Time and Trajectory
    //---
    virtual double getImageTime(const csm::ImageCoord &imagePt) const;
    //> This method returns the time in seconds at which the pixel at the
    //  given imagePt (line, sample in full image space pixels) was captured
    //
    //  The time provided is relative to the reference date and time given
    //  by the Model::getReferenceDateAndTime method.
    //<

    virtual csm::EcefCoord getSensorPosition(
        const csm::ImageCoord &imagePt) const;
    //> This method returns the position of the physical sensor
    // (x,y,z in ECEF meters) when the pixel at the given imagePt
    // (line, sample in full image space pixels) was captured.
    //
    // A csm::Error will be thrown if the sensor position is not available.
    //<

    virtual csm::EcefCoord getSensorPosition(double time) const;
    //> This method returns the position of the physical sensor
    //  (x,y,z meters ECEF) at the given time relative to the reference date
    //  and time given by the Model::getReferenceDateAndTime method.
    //<

    virtual csm::EcefVector getSensorVelocity(
        const csm::ImageCoord &imagePt) const;
    //> This method returns the velocity of the physical sensor
    // (x,y,z in ECEF meters per second) when the pixel at the given imagePt
    // (line, sample in full image space pixels) was captured.
    //<

    virtual csm::EcefVector getSensorVelocity(double time) const;
    //> This method returns the velocity of the physical sensor
    //  (x,y,z in ECEF meters per second ) at the given time relative to the
    //  reference date and time given by the Model::getReferenceDateAndTime
    //  method.
    //<

    virtual csm::RasterGM::SensorPartials computeSensorPartials(
        int index, const csm::EcefCoord &groundPt,
        double desiredPrecision = 0.001, double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This is one of two overloaded methods.  This method takes only
    //  the necessary inputs.  Some effieciency can be obtained by using the
    //  other method.  Even more efficiency can be obtained by using the
    //  computeAllSensorPartials method.
    //
    //  This method returns the partial derivatives of line and sample
    //  (in pixels per the applicable model parameter units), respectively,
    //  with respect to the model parameter given by index at the given
    //  groundPt (x,y,z in ECEF meters).
    //
    //  Derived model implementations may wish to implement this method by
    //  calling the groundToImage method and passing the resulting image
    //  coordinate to the other computeSensorPartials method.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the highest actual precision, in meters, achieved by
    //  iterative algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the actual precision, in meters, achieved by iterative
    //  algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //<

    virtual csm::RasterGM::SensorPartials computeSensorPartials(
        int index, const csm::ImageCoord &imagePt, const csm::EcefCoord &groundPt,
        double desiredPrecision = 0.001, double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This is one of two overloaded methods.  This method takes
    //  an input image coordinate for efficiency.  Even more efficiency can
    //  be obtained by using the computeAllSensorPartials method.
    //
    //  This method returns the partial derivatives of line and sample
    //  (in pixels per the applicable model parameter units), respectively,
    //  with respect to the model parameter given by index at the given
    //  groundPt (x,y,z in ECEF meters).
    //
    //  The imagePt, corresponding to the groundPt, is given so that it does
    //  not need to be computed by the method.  Results are unpredictable if
    //  the imagePt provided does not correspond to the result of calling the
    //  groundToImage method with the given groundPt.
    //
    //  Implementations with iterative algorithms (typically ground-to-image
    //  calls) will use desiredPrecision, in meters, as the convergence
    //  criterion, otherwise it will be ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the highest actual precision, in meters, achieved by
    //  iterative algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //<

    virtual std::vector<csm::RasterGM::SensorPartials> computeAllSensorPartials(
        const csm::EcefCoord &groundPt, csm::param::Set pSet = csm::param::VALID,
        double desiredPrecision = 0.001, double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This is one of two overloaded methods.  This method takes only
    //  the necessary inputs.  Some effieciency can be obtained by using the
    //  other method.
    //
    //  This method returns the partial derivatives of line and sample
    //  (in pixels per the applicable model parameter units), respectively,
    //  with respect to to each of the desired model parameters at the given
    //  groundPt (x,y,z in ECEF meters).  Desired model parameters are
    //  indicated by the given pSet.
    //
    //  Implementations with iterative algorithms (typically ground-to-image
    //  calls) will use desiredPrecision, in meters, as the convergence
    //  criterion, otherwise it will be ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the highest actual precision, in meters, achieved by
    //  iterative algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //
    //  The value returned is a vector of pairs with line and sample partials
    //  for one model parameter in each pair.  The indices of the
    //  corresponding model parameters can be found by calling the
    //  getParameterSetIndices method for the given pSet.
    //
    //  Derived models may wish to implement this directly for efficiency,
    //  but an implementation is provided here that calls the
    //  computeSensorPartials method for each desired parameter index.
    //<

    virtual std::vector<csm::RasterGM::SensorPartials> computeAllSensorPartials(
        const csm::ImageCoord &imagePt, const csm::EcefCoord &groundPt,
        csm::param::Set pSet = csm::param::VALID, double desiredPrecision = 0.001,
        double *achievedPrecision = NULL,
        csm::WarningList *warnings = NULL) const;
    //> This is one of two overloaded methods.  This method takes
    //  an input image coordinate for efficiency.
    //
    //  This method returns the partial derivatives of line and sample
    //  (in pixels per the applicable model parameter units), respectively,
    //  with respect to to each of the desired model parameters at the given
    //  groundPt (x,y,z in ECEF meters).  Desired model parameters are
    //  indicated by the given pSet.
    //
    //  The imagePt, corresponding to the groundPt, is given so that it does
    //  not need to be computed by the method.  Results are unpredictable if
    //  the imagePt provided does not correspond to the result of calling the
    //  groundToImage method with the given groundPt.
    //
    //  Implementations with iterative algorithms (typically ground-to-image
    //  calls) will use desiredPrecision, in meters, as the convergence
    //  criterion, otherwise it will be ignored.
    //
    //  If a non-NULL achievedPrecision argument is received, it will be
    //  populated with the highest actual precision, in meters, achieved by
    //  iterative algorithms and 0.0 for deterministic algorithms.
    //
    //  If a non-NULL warnings argument is received, it will be populated
    //  as applicable.
    //
    //  The value returned is a vector of pairs with line and sample partials
    //  for one model parameter in each pair.  The indices of the
    //  corresponding model parameters can be found by calling the
    //  getParameterSetIndices method for the given pSet.
    //
    //  Derived models may wish to implement this directly for efficiency,
    //  but an implementation is provided here that calls the
    //  computeSensorPartials method for each desired parameter index.
    //<

    virtual std::vector<double> computeGroundPartials(
        const csm::EcefCoord &groundPt) const;
    //> This method returns the partial derivatives of line and sample
    //  (in pixels per meter) with respect to the given groundPt
    //  (x,y,z in ECEF meters).
    //
    //  The value returned is a vector with six elements as follows:
    //
    //-  [0] = line wrt x
    //-  [1] = line wrt y
    //-  [2] = line wrt z
    //-  [3] = sample wrt x
    //-  [4] = sample wrt y
    //-  [5] = sample wrt z
    //<

    virtual const csm::CorrelationModel &getCorrelationModel() const;
    //> This method returns a reference to a CorrelationModel.
    //  The CorrelationModel is used to determine the correlation between
    //  the model parameters of different models of the same type.
    //  These correlations are used to establish the "a priori" cross-covariance
    //  between images. While some applications (such as generation of a
    //  replacement sensor model) may wish to call this method directly,
    //  it is reccommended that the inherited method
    //  GeometricModel::getCrossCovarianceMatrix() be called instead.
    //<

    virtual std::vector<double> getUnmodeledCrossCovariance(
        const csm::ImageCoord &pt1, const csm::ImageCoord &pt2) const;
    //> This method returns the 2x2 line and sample cross covariance
    //  (in pixels squared) between the given imagePt1 and imagePt2 for any
    //  model error not accounted for by the model parameters.  The error is
    //  reported as the four terms of a 2x2 matrix, returned as a 4 element
    //  vector.
    //<

    virtual csm::EcefCoord getReferencePoint() const;
    //> This method returns the ground point indicating the general
    //  location of the image.
    //<

    virtual void setReferencePoint(const csm::EcefCoord &groundPt);
    //> This method sets the ground point indicating the general location
    //  of the image.
    //<

    //---
    // Sensor Model Parameters
    //---
    virtual int getNumParameters() const;
    //> This method returns the number of adjustable parameters.
    //<

    virtual std::string getParameterName(int index) const;
    //> This method returns the name for the adjustable parameter
    //  indicated by the given index.
    //
    //  If the index is out of range, a csm::Error may be thrown.
    //<

    virtual std::string getParameterUnits(int index) const;
    //> This method returns the units for the adjustable parameter
    //  indicated by the given index.  This string is intended for human
    //  consumption, not automated analysis.  Preferred unit names are:
    //
    //-    meters                "m"
    //-    centimeters           "cm"
    //-    millimeters           "mm"
    //-    micrometers           "um"
    //-    nanometers            "nm"
    //-    kilometers            "km"
    //-    inches-US             "inch"
    //-    feet-US               "ft"
    //-    statute miles         "mi"
    //-    nautical miles        "nmi"
    //-
    //-    radians               "rad"
    //-    microradians          "urad"
    //-    decimal degrees       "deg"
    //-    arc seconds           "arcsec"
    //-    arc minutes           "arcmin"
    //-
    //-    seconds               "sec"
    //-    minutes               "min"
    //-    hours                 "hr"
    //-
    //-    steradian             "sterad"
    //-
    //-    none                  "unitless"
    //-
    //-    lines per second      "lines/sec"
    //-    samples per second    "samples/sec"
    //-    frames per second     "frames/sec"
    //-
    //-    watts                 "watt"
    //-
    //-    degrees Kelvin        "K"
    //-
    //-    gram                  "g"
    //-    kilogram              "kg"
    //-    pound - US            "lb"
    //-
    //-    hertz                 "hz"
    //-    megahertz             "mhz"
    //-    gigahertz             "ghz"
    //
    //  Units may be combined with "/" or "." to indicate division or
    //  multiplication.  The caret symbol "^" can be used to indicate
    //  exponentiation.  Thus "m.m" and "m^2" are the same and indicate
    //  square meters.  The return "m/sec^2" indicates an acceleration in
    //  meters per second per second.
    //
    //  Derived classes may choose to return additional unit names, as
    //  required.
    //<

    virtual bool hasShareableParameters() const;
    //> This method returns true if there exists at least one adjustable
    //  parameter on the model that is shareable.  See the
    //  isParameterShareable() method.  This method should return false if
    //  all calls to isParameterShareable() return false.
    //<

    virtual bool isParameterShareable(int index) const;
    //> This method returns a flag to indicate whether or not the adjustable
    //  parameter referenced by index is shareable across models.
    //<

    virtual csm::SharingCriteria getParameterSharingCriteria(int index) const;
    //> This method returns characteristics to indicate how the adjustable
    //  parameter referenced by index is shareable across models.
    //<

    virtual double getParameterValue(int index) const;
    //> This method returns the value of the adjustable parameter
    //  referenced by the given index.
    //<

    virtual void setParameterValue(int index, double value);
    //> This method sets the value for the adjustable parameter referenced by
    //  the given index.
    //<

    virtual csm::param::Type getParameterType(int index) const;
    //> This method returns the type of the adjustable parameter
    //  referenced by the given index.
    //<

    virtual void setParameterType(int index, csm::param::Type pType);
    //> This method sets the type of the adjustable parameter
    //  reference by the given index.
    //<

    //---
    // Uncertainty Propagation
    //---
    virtual double getParameterCovariance(int index1, int index2) const;
    //> This method returns the covariance between the parameters
    //  referenced by index1 and index2.  Variance of a single parameter
    //  is indicated by specifying the samve value for index1 and index2.
    //<

    virtual void setParameterCovariance(int index1, int index2,
                                        double covariance);
    //> This method is used to set the covariance between the parameters
    //  referenced by index1 and index2.  Variance of a single parameter
    //  is indicated by specifying the samve value for index1 and index2.
    //<

    //---
    // Error Correction
    //---
    virtual int getNumGeometricCorrectionSwitches() const;
    //> This method returns the number of geometric correction switches
    //  implemented for the current model.
    //<

    virtual std::string getGeometricCorrectionName(int index) const;
    //> This method returns the name for the geometric correction switch
    //  referenced by the given index.
    //<

    virtual void setGeometricCorrectionSwitch(int index, bool value,
                                              csm::param::Type pType);
    //> This method is used to enable/disable the geometric correction switch
    //  referenced by the given index.
    //<

    virtual bool getGeometricCorrectionSwitch(int index) const;
    //> This method returns the value of the geometric correction switch
    //  referenced by the given index.
    //<

    virtual std::vector<double> getCrossCovarianceMatrix(
        const csm::GeometricModel &comparisonModel,
        csm::param::Set pSet = csm::param::VALID,
        const csm::GeometricModel::GeometricModelList &otherModels =
            csm::GeometricModel::GeometricModelList()) const;
    //> This method returns a matrix containing the elements of the error
    //  cross covariance between this model and a given second model
    //  (comparisonModel).  The set of cross covariance elements returned is
    //  indicated by pSet, which, by default, is all VALID parameters.
    //
    //  If comparisonModel is the same as this model, the covariance for
    //  this model will be returned.  It is equivalent to calling
    //  getParameterCovariance() for the same set of elements.  Note that
    //  even if the cross covariance for a particular model type is always
    //  zero, the covariance for this model must still be supported.
    //
    //  The otherModels list contains all of the models in the current
    //  photogrammetric process; some cross-covariance implementations are
    //  influenced by other models.  It can be omitted if it is not needed
    //  by any models being used.
    //
    //  The returned vector will logically be a two-dimensional matrix of
    //  covariances, though for simplicity it is stored in a one-dimensional
    //  vector (STL has no two-dimensional structure).  The height (number of
    //  rows) of this matrix is the number of parameters on the current model,
    //  and the width (number of columns) is the number of parameters on
    //  the comparison model.  Thus, the covariance between p1 on this model
    //  and p2 on the comparison model is found in index (N*p1 + p2)
    //  in the returned vector.  N is the size of the vector returned by
    //  getParameterSetIndices() on the comparison model for the given pSet).
    //
    //  Note that cross covariance is often zero.  Non-zero cross covariance
    //  can occur for models created from the same sensor (or different
    //  sensors on the same platform).  While cross covariances can result
    //  from a bundle adjustment involving multiple models, no mechanism
    //  currently exists within csm to "set" the cross covariance between
    //  models.  It should thus be assumed that the returned cross covariance
    //  reflects the "un-adjusted" state of the models.
    //<

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

    virtual void setImageIdentifier(const std::string &imageId,
                                    csm::WarningList *warnings = NULL);
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
    // virtual std::string setModelState(std::string stateString) const;
    //> This method returns a string containing the data to exactly recreate
    //  the current model.  It can be used to restore this model to a
    //  previous state with the replaceModelState method or create a new
    //  model object that is identical to this model.
    //  The string could potentially be saved to a file for later use.
    //  An empty string is returned if it is not possible to save the
    //  current state.
    //<

    virtual csm::Ellipsoid getEllipsoid() const;
    //> This method returns the planetary ellipsoid.
    //<

    virtual void setEllipsoid(const csm::Ellipsoid &ellipsoid);
    //> This method sets the planetary ellipsoid.
    //<

 protected:
  csm::RasterGM *m_camera = NULL;
};

#endif  // INCLUDE_USGSCSM_USGSASTROPROJECTEDSENSORMODEL_H_
