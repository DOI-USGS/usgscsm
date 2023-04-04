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

#ifndef INCLUDE_USGSCSM_USGSASTROPROJECTEDLSSENSORMODEL_H_
#define INCLUDE_USGSCSM_USGSASTROPROJECTEDLSSENSORMODEL_H_

#include <RasterGM.h>
#include <SettableEllipsoid.h>

#include<utility>
#include<memory>
#include<string>
#include<vector>

#include "ale/Orientations.h"
#include "ale/States.h"

#include "spdlog/spdlog.h"

#include "UsgsAstroLsSensorModel.h"

class UsgsAstroProjectedLsSensorModel : public UsgsAstroLsSensorModel {
 public:
  // Initializes the class from state data as formatted
  // in a string by the toString() method
  void setState(const std::string& state);

  virtual void replaceModelState(const std::string& stateString);
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
  static std::string getModelNameFromModelState(const std::string& model_state);

  std::string constructStateFromIsd(const std::string imageSupportData,
                                    csm::WarningList* list);

  // State data elements;
  std::vector<double> m_geoTransform;
  std::string m_projString;

  // Define logging pointer and file content
  std::shared_ptr<spdlog::logger> m_logger = spdlog::get("usgscsm_logger");

  // Hardcoded
  static const std::string _SENSOR_MODEL_NAME;  // state date element 0

  static const std::string _STATE_KEYWORD[];

  // Set to default values
  void reset();

  //--------------------------------------------------------------
  // Constructors/Destructor
  //--------------------------------------------------------------

  UsgsAstroProjectedLsSensorModel();
  ~UsgsAstroProjectedLsSensorModel();

  virtual std::string getModelState() const;

  // Set the sensor model based on the input state data
  void set(const std::string& state_data);

  //----------------------------------------------------------------
  // The following public methods are implementations of
  // the methods inherited from RasterGM and SettableEllipsoid.
  // These are defined in the CSM API.
  //----------------------------------------------------------------

  //---
  // Core Photogrammetry
  //---
  virtual csm::ImageCoord groundToImage(
      const csm::EcefCoord& groundPt, double desiredPrecision = 0.001,
      double* achievedPrecision = NULL,
      csm::WarningList* warnings = NULL) const;

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
      const csm::EcefCoordCovar& groundPt, double desiredPrecision = 0.001,
      double* achievedPrecision = NULL,
      csm::WarningList* warnings = NULL) const;
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

  virtual csm::EcefCoord imageToGround(const csm::ImageCoord& imagePt,
                                       double height,
                                       double desiredPrecision = 0.001,
                                       double* achievedPrecision = NULL,
                                       csm::WarningList* warnings = NULL) const;
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
      const csm::ImageCoordCovar& imagePt, double height, double heightVariance,
      double desiredPrecision = 0.001, double* achievedPrecision = NULL,
      csm::WarningList* warnings = NULL) const;
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
      const csm::ImageCoord& imagePt, const csm::EcefCoord& groundPt,
      double desiredPrecision = 0.001, double* achievedPrecision = NULL,
      csm::WarningList* warnings = NULL) const;
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
      const csm::ImageCoord& imagePt, double desiredPrecision = 0.001,
      double* achievedPrecision = NULL,
      csm::WarningList* warnings = NULL) const;
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
  // Error Correction
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
};

#endif  // INCLUDE_USGSCSM_USGSASTROPROJECTEDLSSENSORMODEL_H_
