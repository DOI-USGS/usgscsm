//----------------------------------------------------------------------------
//
//                                UNCLASSIFIED
//
// Copyright © 1989-2017 BAE Systems Information and Electronic Systems Integration Inc.
//                            ALL RIGHTS RESERVED
// Use of this software product is governed by the terms of a license
// agreement. The license agreement is found in the installation directory.
//
//             For support, please visit http://www.baesystems.com/gxp
//
//  Revision History:
//  Date        Name         Description
//  ----------- ------------ -----------------------------------------------
//  13-Nov-2015 BAE Systems  Initial Implementation
//  24-Apr-2017 BAE Systems  Update for CSM 3.0.2
//  24-OCT-2017 BAE Systems  Update for CSM 3.0.3
//-----------------------------------------------------------------------------
#define USGS_SENSOR_LIBRARY

#include "UsgsAstroLsSensorModel.h"
#include "Distortion.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <math.h>

#define USGSASTROLINESCANNER_LIBRARY

#include <sstream>
#include <Error.h>
#include <json.hpp>
using json = nlohmann::json;

const std::string UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME
         = "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL";
const int     UsgsAstroLsSensorModel::NUM_PARAMETERS = 16;
const std::string  UsgsAstroLsSensorModel::PARAMETER_NAME[] =
{
   "IT Pos. Bias   ",   // 0
   "CT Pos. Bias   ",   // 1
   "Rad Pos. Bias  ",   // 2
   "IT Vel. Bias   ",   // 3
   "CT Vel. Bias   ",   // 4
   "Rad Vel. Bias  ",   // 5
   "Omega Bias     ",   // 6
   "Phi Bias       ",   // 7
   "Kappa Bias     ",   // 8
   "Omega Rate     ",   // 9
   "Phi Rate       ",   // 10
   "Kappa Rate     ",   // 11
   "Omega Accl     ",   // 12
   "Phi Accl       ",   // 13
   "Kappa Accl     ",   // 14
   "Focal Bias     "    // 15
};


const std::string  UsgsAstroLsSensorModel::_STATE_KEYWORD[] =
{
   "m_sensorModelName",
   "m_imageIdentifier",
   "m_sensorType",
   "m_totalLines",
   "m_totalSamples",
   "m_offsetLines",
   "m_offsetSamples",
   "m_platformFlag",
   "m_aberrFlag",
   "m_atmrefFlag",
   "m_intTimeLines",
   "m_intTimeStartTimes",
   "m_intTimes",
   "m_startingEphemerisTime",
   "m_centerEphemerisTime",
   "m_detectorSampleSumming",
   "m_startingSample",
   "m_ikCode",
   "m_focal",
   "m_zDirection",
   "m_opticalDistCoef",
   "m_iTransS",
   "m_iTransL",
   "m_detectorSampleOrigin",
   "m_detectorLineOrigin",
   "m_detectorLineOffset",
   "m_semiMajorAxis",
   "m_semiMinorAxis",
   "m_referenceDateAndTime",
   "m_platformIdentifier",
   "m_sensorIdentifier",
   "m_trajectoryIdentifier",
   "m_collectionIdentifier",
   "m_refElevation",
   "m_minElevation",
   "m_maxElevation",
   "m_dtEphem",
   "m_t0Ephem",
   "m_dtQuat",
   "m_t0Quat",
   "m_numEphem",
   "m_numQuaternions",
   "m_ephemPts",
   "m_ephemRates",
   "m_quaternions",
   "m_parameterVals",
   "m_parameterType",
   "m_referencePointXyz",
   "m_gsd",
   "m_flyingHeight",
   "m_halfSwath",
   "m_halfTime",
   "m_covariance",
   "m_imageFlipFlag"
};

const int UsgsAstroLsSensorModel::NUM_PARAM_TYPES = 4;
const std::string UsgsAstroLsSensorModel::PARAM_STRING_ALL[] =
{
   "NONE",
   "FICTITIOUS",
   "REAL",
   "FIXED"
};
const csm::param::Type
      UsgsAstroLsSensorModel::PARAM_CHAR_ALL[] =
{
   csm::param::NONE,
   csm::param::FICTITIOUS,
   csm::param::REAL,
   csm::param::FIXED
};


void UsgsAstroLsSensorModel::replaceModelState(const std::string &stateString )
{
   reset();
   auto j = json::parse(stateString);
   int num_params    = NUM_PARAMETERS;
   int num_paramsSq = num_params * num_params;

   m_imageIdentifier = j["m_imageIdentifier"].get<std::string>();
   m_sensorType = j["m_sensorType"];
   m_totalLines = j["m_totalLines"];
   m_totalSamples = j["m_totalSamples"];
   m_offsetLines = j["m_offsetLines"];
   m_offsetSamples = j["m_offsetSamples"];
   m_platformFlag = j["m_platformFlag"];
   m_aberrFlag = j["m_aberrFlag"];
   m_atmRefFlag = j["m_atmRefFlag"];
   m_intTimeLines = j["m_intTimeLines"].get<std::vector<double>>();
   m_intTimeStartTimes = j["m_intTimeStartTimes"].get<std::vector<double>>();
   m_intTimes = j["m_intTimes"].get<std::vector<double>>();
   m_startingEphemerisTime = j["m_startingEphemerisTime"];
   m_centerEphemerisTime = j["m_centerEphemerisTime"];
   m_detectorSampleSumming = j["m_detectorSampleSumming"];
   m_startingSample = j["m_startingSample"];
   m_ikCode = j["m_ikCode"];
   m_focal = j["m_focal"];
   m_zDirection = j["m_zDirection"];
   for (int i = 0; i < 3; i++) {
     m_opticalDistCoef[i] = j["m_opticalDistCoef"][i];
     m_iTransS[i] = j["m_iTransS"][i];
     m_iTransL[i] = j["m_iTransL"][i];
   }
   m_detectorSampleOrigin = j["m_detectorSampleOrigin"];
   m_detectorLineOrigin = j["m_detectorLineOrigin"];
   m_detectorLineOffset = j["m_detectorLineOffset"];
   m_semiMajorAxis = j["m_semiMajorAxis"];
   m_semiMinorAxis = j["m_semiMinorAxis"];
   m_referenceDateAndTime = j["m_referenceDateAndTime"];
   m_platformIdentifier = j["m_platformIdentifier"];
   m_sensorIdentifier = j["m_sensorIdentifier"];
   m_trajectoryIdentifier = j["m_trajectoryIdentifier"];
   m_collectionIdentifier = j["m_collectionIdentifier"];
   m_refElevation = j["m_refElevation"];
   m_minElevation = j["m_minElevation"];
   m_maxElevation = j["m_maxElevation"];
   m_dtEphem = j["m_dtEphem"];
   m_t0Ephem = j["m_t0Ephem"];
   m_dtQuat = j["m_dtQuat"];
   m_t0Quat = j["m_t0Quat"];
   m_numEphem = j["m_numEphem"];
   m_numQuaternions = j["m_numQuaternions"];
   m_referencePointXyz.x = j["m_referencePointXyz"][0];
   m_referencePointXyz.y = j["m_referencePointXyz"][1];
   m_referencePointXyz.z = j["m_referencePointXyz"][2];
   m_gsd = j["m_gsd"];
   m_flyingHeight = j["m_flyingHeight"];
   m_halfSwath = j["m_halfSwath"];
   m_halfTime = j["m_halfTime"];
   m_imageFlipFlag = j["m_imageFlipFlag"];
   // Vector = is overloaded so explicit get with type required.
   m_ephemPts = j["m_ephemPts"].get<std::vector<double>>();
   m_ephemRates = j["m_ephemRates"].get<std::vector<double>>();
   m_quaternions = j["m_quaternions"].get<std::vector<double>>();
   m_parameterVals = j["m_parameterVals"].get<std::vector<double>>();
   m_covariance = j["m_covariance"].get<std::vector<double>>();
   for (int i = 0; i < num_params; i++) {
     for (int k = 0; k < NUM_PARAM_TYPES; k++) {
       if (j["m_parameterType"][i] == PARAM_STRING_ALL[k]) {
         m_parameterType[i] = PARAM_CHAR_ALL[k];
         break;
     }
    }
   }

   // If computed state values are still default, then compute them
   if (m_gsd == 1.0  && m_flyingHeight == 1000.0)
   {
     updateState();
   }

   try
   {
     setLinearApproximation();
   }
   catch (...)
   {
     _linear = false;
   }
}

std::string UsgsAstroLsSensorModel::getModelNameFromModelState(
   const std::string& model_state)
{
   // Parse the string to JSON
   auto j = json::parse(model_state);
   // If model name cannot be determined, return a blank string
   std::string model_name;

   if (j.find("m_sensorModelName") != j.end()) {
       model_name = j["m_sensorModelName"];
   } else {
       csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
       std::string aMessage = "No 'm_sensorModelName' key in the model state object.";
       std::string aFunction = "UsgsAstroLsPlugin::getModelNameFromModelState";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   if (model_name != _SENSOR_MODEL_NAME){
       csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
       std::string aMessage = "Sensor model not supported.";
       std::string aFunction = "UsgsAstroLsPlugin::getModelNameFromModelState()";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   return model_name;
}

std::string UsgsAstroLsSensorModel::getModelState() const {
      json state;
      state["m_imageIdentifier"] = m_imageIdentifier;
      state["m_sensorType"] = m_sensorType;
      state["m_totalLines"] = m_totalLines;
      state["m_totalSamples"] = m_totalSamples;
      state["m_offsetLines"] = m_offsetLines;
      state["m_offsetSamples"] = m_offsetSamples;
      state["m_platformFlag"] = m_platformFlag;
      state["m_aberrFlag"] = m_aberrFlag;
      state["m_atmRefFlag"] = m_atmRefFlag;
      state["m_intTimeLines"] = m_intTimeLines;
      state["m_intTimeStartTimes"] = m_intTimeStartTimes;
      state["m_intTimes"] = m_intTimes;
      state["m_startingEphemerisTime"] = m_startingEphemerisTime;
      state["m_centerEphemerisTime"] = m_centerEphemerisTime;
      state["m_detectorSampleSumming"] = m_detectorSampleSumming;
      state["m_startingSample"] = m_startingSample;
      state["m_ikCode"] = m_ikCode;
      state["m_focal"] = m_focal;
      state["m_zDirection"] = m_zDirection;
      state["m_opticalDistCoef"] = std::vector<double>(m_opticalDistCoef, m_opticalDistCoef+3);
      state["m_iTransS"] = std::vector<double>(m_iTransS, m_iTransS+3);
      state["m_iTransL"] = std::vector<double>(m_iTransL, m_iTransL+3);
      state["m_detectorSampleOrigin"] = m_detectorSampleOrigin;
      state["m_detectorLineOrigin"] = m_detectorLineOrigin;
      state["m_detectorLineOffset"] = m_detectorLineOffset;
      state["m_semiMajorAxis"] = m_semiMajorAxis;
      state["m_semiMinorAxis"] = m_semiMinorAxis;
      state["m_referenceDateAndTime"] = m_referenceDateAndTime;
      state["m_platformIdentifier"] = m_platformIdentifier;
      state["m_sensorIdentifier"] = m_sensorIdentifier;
      state["m_trajectoryIdentifier"] = m_trajectoryIdentifier;
      state["m_collectionIdentifier"] = m_collectionIdentifier;
      state["m_refElevation"] = m_refElevation;
      state["m_minElevation"] = m_minElevation;
      state["m_maxElevation"] = m_maxElevation;
      state["m_dtEphem"] = m_dtEphem;
      state["m_t0Ephem"] = m_t0Ephem;
      state["m_dtQuat"] = m_dtQuat;
      state["m_t0Quat"] = m_t0Quat;
      state["m_numEphem"] = m_numEphem;
      state["m_numQuaternions"] = m_numQuaternions;
      state["m_ephemPts"] = m_ephemPts;
      state["m_ephemRates"] = m_ephemRates;
      state["m_quaternions"] = m_quaternions;
      state["m_parameterVals"] = m_parameterVals;
      state["m_parameterType"] = m_parameterType;
      state["m_gsd"] = m_gsd;
      state["m_flyingHeight"] = m_flyingHeight;
      state["m_halfSwath"] = m_halfSwath;
      state["m_halfTime"] = m_halfTime;
      state["m_covariance"] = m_covariance;
      state["m_imageFlipFlag"] = m_imageFlipFlag;

      state["m_referencePointXyz"] = json();
      state["m_referencePointXyz"]["x"] = m_referencePointXyz.x;
      state["m_referencePointXyz"]["y"] = m_referencePointXyz.y;
      state["m_referencePointXyz"]["z"] = m_referencePointXyz.z;

      return state.dump();
 }

void UsgsAstroLsSensorModel::reset()
{
  _linear = false; // default until a linear model is made
  _u0    = 0.0;
  _du_dx = 0.0;
  _du_dy = 0.0;
  _du_dz = 0.0;
  _v0    = 0.0;
  _dv_dx = 0.0;
  _dv_dy = 0.0;
  _dv_dz = 0.0;

  _no_adjustment.assign(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);

  m_imageIdentifier = "";                    // 1
  m_sensorType    = "USGSAstroLineScanner";  // 2
  m_totalLines    = 0;                       // 3
  m_totalSamples  = 0;                       // 4
  m_offsetLines   = 0.0;                     // 7
  m_offsetSamples = 0.0;                     // 8
  m_platformFlag  = 1;                       // 9
  m_aberrFlag     = 0;                       // 10
  m_atmRefFlag    = 0;                       // 11
  m_intTimeLines.clear();
  m_intTimeStartTimes.clear();
  m_intTimes.clear();
  m_startingEphemerisTime = 0.0;             // 13
  m_centerEphemerisTime = 0.0;               // 14
  m_detectorSampleSumming = 1.0;             // 15
  m_startingSample = 1.0;                    // 16
  m_ikCode = -85600;                         // 17
  m_focal = 350.0;                           // 18
  m_zDirection = 1.0;                        // 19
  m_opticalDistCoef[0] = 0.0;                // 20
  m_opticalDistCoef[1] = 0.0;                // 20
  m_opticalDistCoef[2] = 0.0;                // 20
  m_iTransS[0] = 0.0;                        // 21
  m_iTransS[1] = 0.0;                        // 21
  m_iTransS[2] = 150.0;                      // 21
  m_iTransL[0] = 0.0;                        // 22
  m_iTransL[1] = 150.0;                      // 22
  m_iTransL[2] = 0.0;                        // 22
  m_detectorSampleOrigin = 2500.0;           // 23
  m_detectorLineOrigin = 0.0;                // 24
  m_detectorLineOffset = 0.0;                // 25
  m_semiMajorAxis = 3400000.0;               // 27
  m_semiMinorAxis = 3350000.0;               // 28
  m_referenceDateAndTime = "";               // 30
  m_platformIdentifier = "";                 // 31
  m_sensorIdentifier = "";                   // 32
  m_trajectoryIdentifier = "";               // 33
  m_collectionIdentifier = "";               // 33
  m_refElevation = 30;                       // 34
  m_minElevation = -8000.0;                  // 34
  m_maxElevation =  8000.0;                  // 35
  m_dtEphem = 2.0;                           // 36
  m_t0Ephem = -70.0;                         // 37
  m_dtQuat =  0.1;                           // 38
  m_t0Quat = -40.0;                          // 39
  m_numEphem = 0;                            // 40
  m_numQuaternions = 0;                      // 41
  m_ephemPts.clear();                        // 42
  m_ephemRates.clear();                      // 43
  m_quaternions.clear();                     // 44

  m_parameterVals.assign(NUM_PARAMETERS,0.0);
  m_parameterType.assign(NUM_PARAMETERS,csm::param::REAL);

  m_referencePointXyz.x = 0.0;             // 47
  m_referencePointXyz.y = 0.0;             // 47
  m_referencePointXyz.z = 0.0;             // 47
  m_gsd = 1.0;                             // 48
  m_flyingHeight = 1000.0;                 // 49
  m_halfSwath = 1000.0;                    // 50
  m_halfTime = 10.0;                       // 51

  m_covariance = std::vector<double>(NUM_PARAMETERS * NUM_PARAMETERS,0.0); // 52
  m_imageFlipFlag = 0;                     // 53
}


//*****************************************************************************
// UsgsAstroLsSensorModel Constructor
//*****************************************************************************
UsgsAstroLsSensorModel::UsgsAstroLsSensorModel()
{
   _no_adjustment.assign(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);
}


//*****************************************************************************
// UsgsAstroLsSensorModel Destructor
//*****************************************************************************
UsgsAstroLsSensorModel::~UsgsAstroLsSensorModel()
{
}

//*****************************************************************************
// UsgsAstroLsSensorModel updateState
//*****************************************************************************
void UsgsAstroLsSensorModel::updateState()
{
   // If sensor model is being created for the first time
   // This routine will set some parameters not found in the ISD.

   // Reference point (image center)
   double lineCtr = m_totalLines / 2.0;
   double sampCtr = m_totalSamples / 2.0;
   csm::ImageCoord ip(lineCtr, sampCtr);
   double refHeight = m_refElevation;
   m_referencePointXyz = imageToGround(ip, refHeight);
   // Compute ground sample distance
   ip.line += 1;
   ip.samp += 1;
   csm::EcefCoord delta = imageToGround(ip, refHeight);
   double dx = delta.x - m_referencePointXyz.x;
   double dy = delta.y - m_referencePointXyz.y;
   double dz = delta.z - m_referencePointXyz.z;
   m_gsd = sqrt((dx*dx + dy*dy + dz*dz) / 2.0);

   // Compute flying height
   csm::EcefCoord sensorPos = getSensorPosition(0.0);
   dx = sensorPos.x - m_referencePointXyz.x;
   dy = sensorPos.y - m_referencePointXyz.y;
   dz = sensorPos.z - m_referencePointXyz.z;
   m_flyingHeight = sqrt(dx*dx + dy*dy + dz*dz);

   // Compute half swath
   m_halfSwath = m_gsd * m_totalSamples / 2.0;

   // Compute half time duration
   double fullImageTime = m_intTimeStartTimes.back() - m_intTimeStartTimes.front()
                          + m_intTimes.back() * (m_totalLines - m_intTimeLines.back());
   m_halfTime = fullImageTime / 2.0;

   // Parameter covariance, hardcoded accuracy values
   int num_params = NUM_PARAMETERS;
   int num_paramsSquare = num_params * num_params;
   double variance = m_gsd * m_gsd;
   for (int i = 0; i < num_paramsSquare; i++)
   {
      m_covariance[i] = 0.0;
   }
   for (int i = 0; i < num_params; i++)
   {
      m_covariance[i * num_params + i] = variance;
   }
}


//---------------------------------------------------------------------------
// Core Photogrammetry
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage
//***************************************************************************
csm::ImageCoord UsgsAstroLsSensorModel::groundToImage(
   const csm::EcefCoord& ground_pt,
   double                desired_precision,
   double*               achieved_precision,
   csm::WarningList*     warnings) const
{
   // The public interface invokes the private interface with no adjustments.
   return groundToImage(
      ground_pt, _no_adjustment,
      desired_precision, achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage (internal version)
//***************************************************************************
csm::ImageCoord UsgsAstroLsSensorModel::groundToImage(
   const csm::EcefCoord& ground_pt,
   const std::vector<double>& adj,
   double                desired_precision,
   double*               achieved_precision,
   csm::WarningList*     warnings) const
{
   // Search for the line, sample coordinate that viewed a given ground point.
   // This method uses an iterative secant method to search for the image
   // line.

   // Convert the ground precision to pixel precision so we can
   // check for convergence without re-intersecting
   csm::ImageCoord approxPoint;
   computeLinearApproximation(ground_pt, approxPoint);
   csm::ImageCoord approxNextPoint = approxPoint;
   if (approxNextPoint.line + 1 < m_totalLines) {
      ++approxNextPoint.line;
   }
   else {
      --approxNextPoint.line;
   }
   csm::EcefCoord approxIntersect = imageToGround(approxPoint, m_refElevation);
   csm::EcefCoord approxNextIntersect = imageToGround(approxNextPoint, m_refElevation);
   double lineDX = approxNextIntersect.x - approxIntersect.x;
   double lineDY = approxNextIntersect.y - approxIntersect.y;
   double lineDZ = approxNextIntersect.z - approxIntersect.z;
   double approxLineRes = sqrt(lineDX * lineDX + lineDY * lineDY + lineDZ * lineDZ);
   // Increase the precision by a small amount to ensure the desired precision is met
   double pixelPrec = desired_precision / approxLineRes * 0.9;

   // Start secant method search on the image lines
   double sampCtr = m_totalSamples / 2.0;
   double firstTime = getImageTime(csm::ImageCoord(0.0, sampCtr));
   double lastTime = getImageTime(csm::ImageCoord(m_totalLines, sampCtr));
   double firstOffset = computeViewingPixel(firstTime, ground_pt, adj, pixelPrec/2).line - 0.5;
   double lastOffset = computeViewingPixel(lastTime, ground_pt, adj, pixelPrec/2).line - 0.5;

   // Check if both offsets have the same sign.
   // This means there is not guaranteed to be a zero.
   if ((firstOffset > 0) != (lastOffset < 0)) {
        throw csm::Warning(
           csm::Warning::IMAGE_COORD_OUT_OF_BOUNDS,
           "The image coordinate is out of bounds of the image size.",
           "UsgsAstroLsSensorModel::groundToImage");
   }

   // Start secant method search
   for (int it = 0; it < 30; it++) {
      double nextTime = ((firstTime * lastOffset) - (lastTime * firstOffset))
                      / (lastOffset - firstOffset);
      // Because time across the image is not continuous, find the exposure closest
      // to the computed nextTime and use that.

      // I.E. if the computed nextTime is 0.3, and the middle exposure times for
      // lines are 0.07, 0.17, 0.27, 0.37, and 0.47; then use 0.27 because it is
      // the closest middle exposure time.
      auto referenceTimeIt = std::upper_bound(m_intTimeStartTimes.begin(),
                                              m_intTimeStartTimes.end(),
                                              nextTime);
      if (referenceTimeIt != m_intTimeStartTimes.begin()) {
         --referenceTimeIt;
      }
      size_t referenceIndex = std::distance(m_intTimeStartTimes.begin(), referenceTimeIt);
      double computedLine = (nextTime - m_intTimeStartTimes[referenceIndex]) / m_intTimes[referenceIndex]
                          + m_intTimeLines[referenceIndex] - 0.5; // subtract 0.5 for ISIS -> CSM pixel conversion
      double closestLine = floor(computedLine + 0.5);
      nextTime = getImageTime(csm::ImageCoord(closestLine, sampCtr));

      double nextOffset = computeViewingPixel(nextTime, ground_pt, adj, pixelPrec/2).line - 0.5;

      // remove the farthest away node
      if (fabs(firstTime - nextTime) > fabs(lastTime - nextTime)) {
        firstTime = nextTime;
        firstOffset = nextOffset;
      }
      else {
        lastTime = nextTime;
        lastOffset = nextOffset;
      }
      if (fabs(lastOffset - firstOffset) < pixelPrec) {
         break;
      }
   }

   // Avoid division by 0 if the first and last nodes are the same
   double computedTime = firstTime;
   if (fabs(lastOffset - firstOffset) > 10e-15) {
     computedTime = ((firstTime * lastOffset) - (lastTime * firstOffset))
                         / (lastOffset - firstOffset);
   }

   auto referenceTimeIt = std::upper_bound(m_intTimeStartTimes.begin(),
                                           m_intTimeStartTimes.end(),
                                           computedTime);
   if (referenceTimeIt != m_intTimeStartTimes.begin()) {
      --referenceTimeIt;
   }
   size_t referenceIndex = std::distance(m_intTimeStartTimes.begin(), referenceTimeIt);
   double computedLine = (computedTime - m_intTimeStartTimes[referenceIndex]) / m_intTimes[referenceIndex]
                       + m_intTimeLines[referenceIndex] - 0.5; // subtract 0.5 for ISIS -> CSM pixel conversion
   double closestLine = floor(computedLine + 0.5); // This assumes pixels are the interval [n, n+1)
   computedTime = getImageTime(csm::ImageCoord(closestLine, sampCtr));
   csm::ImageCoord calculatedPixel = computeViewingPixel(computedTime, ground_pt, adj, pixelPrec/2);
   // The computed pioxel is the detector pixel, so we need to convert that to image lines
   calculatedPixel.line += closestLine;

   // Reintersect to ensure the image point actually views the ground point.
   csm::EcefCoord calculatedPoint = imageToGround(calculatedPixel, m_refElevation);
   double dx = ground_pt.x - calculatedPoint.x;
   double dy = ground_pt.y - calculatedPoint.y;
   double dz = ground_pt.z - calculatedPoint.z;
   double len = dx * dx + dy * dy + dz * dz;

   // If the final correction is greater than 10 meters,
   // the solution is not valid enough to report even with a warning
   if (len > 100.0) {
      std::ostringstream msg;
      msg << "Did not converge. Ground point: (" << ground_pt.x << ", "
          << ground_pt.y << ", " << ground_pt.z << ") Computed image point: ("
          << calculatedPixel.line << ", " << calculatedPixel.samp
          << ") Computed ground point: (" << calculatedPoint.x << ", "
          << calculatedPoint.y << ", " << calculatedPoint.z << ")";
      throw csm::Error(
         csm::Error::ALGORITHM,
         msg.str(),
         "UsgsAstroLsSensorModel::groundToImage");
   }

   if (achieved_precision) {
      *achieved_precision = sqrt(len);
   }

   double preSquare = desired_precision * desired_precision;
   if (warnings && (desired_precision > 0.0) && (preSquare < len)) {
      std::stringstream msg;
      msg << "Desired precision not achieved. ";
      msg << len << "  " << preSquare << "\n";
      warnings->push_back(
         csm::Warning(csm::Warning::PRECISION_NOT_MET,
         msg.str().c_str(),
         "UsgsAstroLsSensorModel::groundToImage()"));
   }

   return calculatedPixel;
}

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage
//***************************************************************************
csm::ImageCoordCovar UsgsAstroLsSensorModel::groundToImage(
   const csm::EcefCoordCovar& groundPt,
   double  desired_precision,
   double* achieved_precision,
   csm::WarningList* warnings) const
{
   // Ground to image with error propagation
   // Compute corresponding image point
   csm::EcefCoord gp;
   gp.x = groundPt.x;
   gp.y = groundPt.y;
   gp.z = groundPt.z;

   csm::ImageCoord ip = groundToImage(
      gp, desired_precision, achieved_precision, warnings);
   csm::ImageCoordCovar result(ip.line, ip.samp);

   // Compute partials ls wrt XYZ
   std::vector<double> prt(6, 0.0);
   prt = computeGroundPartials(groundPt);

   // Error propagation
   double ltx, lty, ltz;
   double stx, sty, stz;
   ltx =
      prt[0] * groundPt.covariance[0] +
      prt[1] * groundPt.covariance[3] +
      prt[2] * groundPt.covariance[6];
   lty =
      prt[0] * groundPt.covariance[1] +
      prt[1] * groundPt.covariance[4] +
      prt[2] * groundPt.covariance[7];
   ltz =
      prt[0] * groundPt.covariance[2] +
      prt[1] * groundPt.covariance[5] +
      prt[2] * groundPt.covariance[8];
   stx =
      prt[3] * groundPt.covariance[0] +
      prt[4] * groundPt.covariance[3] +
      prt[5] * groundPt.covariance[6];
   sty =
      prt[3] * groundPt.covariance[1] +
      prt[4] * groundPt.covariance[4] +
      prt[5] * groundPt.covariance[7];
   stz =
      prt[3] * groundPt.covariance[2] +
      prt[4] * groundPt.covariance[5] +
      prt[5] * groundPt.covariance[8];

   double gp_cov[4]; // Input gp cov in image space
   gp_cov[0] = ltx * prt[0] + lty * prt[1] + ltz * prt[2];
   gp_cov[1] = ltx * prt[3] + lty * prt[4] + ltz * prt[5];
   gp_cov[2] = stx * prt[0] + sty * prt[1] + stz * prt[2];
   gp_cov[3] = stx * prt[3] + sty * prt[4] + stz * prt[5];

   std::vector<double> unmodeled_cov = getUnmodeledError(ip);
   double sensor_cov[4]; // sensor cov in image space
   determineSensorCovarianceInImageSpace(gp, sensor_cov);

   result.covariance[0] = gp_cov[0] + unmodeled_cov[0] + sensor_cov[0];
   result.covariance[1] = gp_cov[1] + unmodeled_cov[1] + sensor_cov[1];
   result.covariance[2] = gp_cov[2] + unmodeled_cov[2] + sensor_cov[2];
   result.covariance[3] = gp_cov[3] + unmodeled_cov[3] + sensor_cov[3];

   return result;
}

//***************************************************************************
// UsgsAstroLsSensorModel::imageToGround
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::imageToGround(
   const csm::ImageCoord& image_pt,
   double height,
   double desired_precision,
   double* achieved_precision,
   csm::WarningList* warnings) const
{
   double xc, yc, zc;
   double vx, vy, vz;
   double xl, yl, zl;
   double dxl, dyl, dzl;
   losToEcf(
      image_pt.line, image_pt.samp, _no_adjustment,
      xc, yc, zc, vx, vy, vz, xl, yl, zl);
   if (m_aberrFlag == 1)
   {
      lightAberrationCorr(vx, vy, vz, xl, yl, zl, dxl, dyl, dzl);
      xl += dxl;
      yl += dyl;
      zl += dzl;
   }

   double aPrec;
   double x, y, z;
   losEllipsoidIntersect(
      height, xc, yc, zc, xl, yl, zl, x, y, z, aPrec, desired_precision);

   if (achieved_precision)
      *achieved_precision = aPrec;

   if (warnings && (desired_precision > 0.0) && (aPrec > desired_precision))
   {
      warnings->push_back(
         csm::Warning(
            csm::Warning::PRECISION_NOT_MET,
            "Desired precision not achieved.",
            "UsgsAstroLsSensorModel::imageToGround()"));
   }

   return csm::EcefCoord(x, y, z);
}


void UsgsAstroLsSensorModel::determineSensorCovarianceInImageSpace(
   csm::EcefCoord &gp,
   double         sensor_cov[4] ) const
{
   int i, j, totalAdjParams;
   totalAdjParams = getNumParameters();

   std::vector<csm::RasterGM::SensorPartials> sensor_partials = computeAllSensorPartials(gp);

   sensor_cov[0] = 0.0;
   sensor_cov[1] = 0.0;
   sensor_cov[2] = 0.0;
   sensor_cov[3] = 0.0;

   for (i = 0; i < totalAdjParams; i++)
   {
      for (j = 0; j < totalAdjParams; j++)
      {
         sensor_cov[0] += sensor_partials[i].first  * getParameterCovariance(i, j) * sensor_partials[j].first;
         sensor_cov[1] += sensor_partials[i].second * getParameterCovariance(i, j) * sensor_partials[j].first;
         sensor_cov[2] += sensor_partials[i].first  * getParameterCovariance(i, j) * sensor_partials[j].second;
         sensor_cov[3] += sensor_partials[i].second * getParameterCovariance(i, j) * sensor_partials[j].second;
      }
   }
}


//***************************************************************************
// UsgsAstroLsSensorModel::imageToGround
//***************************************************************************
csm::EcefCoordCovar UsgsAstroLsSensorModel::imageToGround(
   const csm::ImageCoordCovar& image_pt,
   double height,
   double heightVariance,
   double desired_precision,
   double* achieved_precision,
   csm::WarningList* warnings) const
{
   // Image to ground with error propagation
   // Use numerical partials

   const double DELTA_IMAGE = 1.0;
   const double DELTA_GROUND = m_gsd;
   csm::ImageCoord ip(image_pt.line, image_pt.samp);

   csm::EcefCoord gp = imageToGround(
      ip, height, desired_precision, achieved_precision, warnings);

   // Compute numerical partials xyz wrt to lsh
   ip.line = image_pt.line + DELTA_IMAGE;
   ip.samp = image_pt.samp;
   csm::EcefCoord gpl = imageToGround(ip, height, desired_precision);
   double xpl = (gpl.x - gp.x) / DELTA_IMAGE;
   double ypl = (gpl.y - gp.y) / DELTA_IMAGE;
   double zpl = (gpl.z - gp.z) / DELTA_IMAGE;

   ip.line = image_pt.line;
   ip.samp = image_pt.samp + DELTA_IMAGE;
   csm::EcefCoord gps = imageToGround(ip, height, desired_precision);
   double xps = (gps.x - gp.x) / DELTA_IMAGE;
   double yps = (gps.y - gp.y) / DELTA_IMAGE;
   double zps = (gps.z - gp.z) / DELTA_IMAGE;

   ip.line = image_pt.line;
   ip.samp = image_pt.samp; // +DELTA_IMAGE;
   csm::EcefCoord gph = imageToGround(ip, height + DELTA_GROUND, desired_precision);
   double xph = (gph.x - gp.x) / DELTA_GROUND;
   double yph = (gph.y - gp.y) / DELTA_GROUND;
   double zph = (gph.z - gp.z) / DELTA_GROUND;

   // Convert sensor covariance to image space
   double sCov[4];
   determineSensorCovarianceInImageSpace(gp, sCov);

   std::vector<double> unmod = getUnmodeledError(image_pt);

   double iCov[4];
   iCov[0] = image_pt.covariance[0] + sCov[0] + unmod[0];
   iCov[1] = image_pt.covariance[1] + sCov[1] + unmod[1];
   iCov[2] = image_pt.covariance[2] + sCov[2] + unmod[2];
   iCov[3] = image_pt.covariance[3] + sCov[3] + unmod[3];

   // Temporary matrix product
   double t00, t01, t02, t10, t11, t12, t20, t21, t22;
   t00 = xpl * iCov[0] + xps * iCov[2];
   t01 = xpl * iCov[1] + xps * iCov[3];
   t02 = xph * heightVariance;
   t10 = ypl * iCov[0] + yps * iCov[2];
   t11 = ypl * iCov[1] + yps * iCov[3];
   t12 = yph * heightVariance;
   t20 = zpl * iCov[0] + zps * iCov[2];
   t21 = zpl * iCov[1] + zps * iCov[3];
   t22 = zph * heightVariance;

   // Ground covariance
   csm::EcefCoordCovar result;
   result.x = gp.x;
   result.y = gp.y;
   result.z = gp.z;

   result.covariance[0] = t00 * xpl + t01 * xps + t02 * xph;
   result.covariance[1] = t00 * ypl + t01 * yps + t02 * yph;
   result.covariance[2] = t00 * zpl + t01 * zps + t02 * zph;
   result.covariance[3] = t10 * xpl + t11 * xps + t12 * xph;
   result.covariance[4] = t10 * ypl + t11 * yps + t12 * yph;
   result.covariance[5] = t10 * zpl + t11 * zps + t12 * zph;
   result.covariance[6] = t20 * xpl + t21 * xps + t22 * xph;
   result.covariance[7] = t20 * ypl + t21 * yps + t22 * yph;
   result.covariance[8] = t20 * zpl + t21 * zps + t22 * zph;

   return result;
}

//***************************************************************************
// UsgsAstroLsSensorModel::imageToProximateImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroLsSensorModel::imageToProximateImagingLocus(
   const csm::ImageCoord& image_pt,
   const csm::EcefCoord& ground_pt,
   double desired_precision,
   double* achieved_precision,
   csm::WarningList* warnings) const
{
   // Object ray unit direction near given ground location
   const double DELTA_GROUND = m_gsd;

   double x = ground_pt.x;
   double y = ground_pt.y;
   double z = ground_pt.z;

   // Elevation at input ground point
   double height, aPrec;
   computeElevation(x, y, z, height, aPrec, desired_precision);

   // Ground point on object ray with the same elevation
   csm::EcefCoord gp1 = imageToGround(
      image_pt, height, desired_precision, achieved_precision);

   // Vector between 2 ground points above
   double dx1 = x - gp1.x;
   double dy1 = y - gp1.y;
   double dz1 = z - gp1.z;

   // Unit vector along object ray
   csm::EcefCoord gp2 = imageToGround(
      image_pt, height - DELTA_GROUND, desired_precision, achieved_precision);
   double dx2 = gp2.x - gp1.x;
   double dy2 = gp2.y - gp1.y;
   double dz2 = gp2.z - gp1.z;
   double mag2 = sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
   dx2 /= mag2;
   dy2 /= mag2;
   dz2 /= mag2;

   // Point on object ray perpendicular to input point

   // Locus
   csm::EcefLocus locus;
   double scale = dx1 * dx2 + dy1 * dy2 + dz1 * dz2;
   gp2.x = gp1.x + scale * dx2;
   gp2.y = gp1.y + scale * dy2;
   gp2.z = gp1.z + scale * dz2;

   double hLocus;
   computeElevation(gp2.x, gp2.y, gp2.z, hLocus, aPrec, desired_precision);
   locus.point = imageToGround(
      image_pt, hLocus, desired_precision, achieved_precision, warnings);

   locus.direction.x = dx2;
   locus.direction.y = dy2;
   locus.direction.z = dz2;

   return locus;
}

//***************************************************************************
// UsgsAstroLsSensorModel::imageToRemoteImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroLsSensorModel::imageToRemoteImagingLocus(
   const csm::ImageCoord& image_pt,
   double desired_precision,
   double* achieved_precision,
   csm::WarningList* warnings) const
{
   // Object ray unit direction at exposure station
   // Exposure station arbitrary for orthos, so set elevation to zero

   // Set esposure station elevation to zero
   double height = 0.0;
   double GND_DELTA = m_gsd;
   // Point on object ray at zero elevation
   csm::EcefCoord gp1 = imageToGround(
      image_pt, height, desired_precision, achieved_precision, warnings);

   // Unit vector along object ray
   csm::EcefCoord gp2 = imageToGround(
      image_pt, height - GND_DELTA, desired_precision, achieved_precision, warnings);

   double dx2, dy2, dz2;
   dx2 = gp2.x - gp1.x;
   dy2 = gp2.y - gp1.y;
   dz2 = gp2.z - gp1.z;
   double mag2 = sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
   dx2 /= mag2;
   dy2 /= mag2;
   dz2 /= mag2;

   // Locus
   csm::EcefLocus locus;
   locus.point = gp1;
   locus.direction.x = dx2;
   locus.direction.y = dy2;
   locus.direction.z = dz2;

   return locus;
}

//---------------------------------------------------------------------------
// Uncertainty Propagation
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::computeGroundPartials
//***************************************************************************
std::vector<double> UsgsAstroLsSensorModel::computeGroundPartials(
   const csm::EcefCoord& ground_pt) const
{
   double GND_DELTA = m_gsd;
   // Partial of line, sample wrt X, Y, Z
   double x = ground_pt.x;
   double y = ground_pt.y;
   double z = ground_pt.z;

   csm::ImageCoord ipB = groundToImage(ground_pt);
   csm::ImageCoord ipX = groundToImage(csm::EcefCoord(x + GND_DELTA, y, z));
   csm::ImageCoord ipY = groundToImage(csm::EcefCoord(x, y + GND_DELTA, z));
   csm::ImageCoord ipZ = groundToImage(csm::EcefCoord(x, y, z + GND_DELTA));

   std::vector<double> partials(6, 0.0);
   partials[0] = (ipX.line - ipB.line) / GND_DELTA;
   partials[3] = (ipX.samp - ipB.samp) / GND_DELTA;
   partials[1] = (ipY.line - ipB.line) / GND_DELTA;
   partials[4] = (ipY.samp - ipB.samp) / GND_DELTA;
   partials[2] = (ipZ.line - ipB.line) / GND_DELTA;
   partials[5] = (ipZ.samp - ipB.samp) / GND_DELTA;

   return partials;
}


//***************************************************************************
// UsgsAstroLsSensorModel::computeSensorPartials
//***************************************************************************
csm::RasterGM::SensorPartials UsgsAstroLsSensorModel::computeSensorPartials(
   int index,
   const csm::EcefCoord& ground_pt,
   double  desired_precision,
   double* achieved_precision,
   csm::WarningList* warnings) const
{
   // Compute image coordinate first
   csm::ImageCoord img_pt = groundToImage(
      ground_pt, desired_precision, achieved_precision);

   // Call overloaded function
   return computeSensorPartials(
      index, img_pt, ground_pt, desired_precision, achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroLsSensorModel::computeSensorPartials
//***************************************************************************
csm::RasterGM::SensorPartials UsgsAstroLsSensorModel::computeSensorPartials(
   int                    index,
   const csm::ImageCoord& image_pt,
   const csm::EcefCoord&  ground_pt,
   double                 desired_precision,
   double*                achieved_precision,
   csm::WarningList*      warnings) const
{
   // Compute numerical partials ls wrt specific parameter

   const double DELTA = m_gsd;
   std::vector<double> adj(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);
   adj[index] = DELTA;

   csm::ImageCoord img1 = groundToImage(
      ground_pt, adj, desired_precision, achieved_precision, warnings);

   double line_partial = (img1.line - image_pt.line) / DELTA;
   double sample_partial = (img1.samp - image_pt.samp) / DELTA;

   return csm::RasterGM::SensorPartials(line_partial, sample_partial);
}

//***************************************************************************
// UsgsAstroLsSensorModel::computeAllSensorPartials
//***************************************************************************
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroLsSensorModel::computeAllSensorPartials(
   const csm::EcefCoord& ground_pt,
   csm::param::Set       pSet,
   double                desired_precision,
   double*               achieved_precision,
   csm::WarningList*     warnings) const
{
   csm::ImageCoord image_pt = groundToImage(
      ground_pt, desired_precision, achieved_precision, warnings);

   return computeAllSensorPartials(
      image_pt, ground_pt, pSet, desired_precision, achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroLsSensorModel::computeAllSensorPartials
//***************************************************************************
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroLsSensorModel::computeAllSensorPartials(
   const csm::ImageCoord& image_pt,
   const csm::EcefCoord&  ground_pt,
   csm::param::Set        pSet,
   double                 desired_precision,
   double*                achieved_precision,
   csm::WarningList*      warnings) const
{
   std::vector<int> indices = getParameterSetIndices(pSet);
   size_t num = indices.size();
   std::vector<csm::RasterGM::SensorPartials> partials;
   for (int index = 0; index < num; index++)
   {
      partials.push_back(
         computeSensorPartials(
            indices[index], image_pt, ground_pt,
            desired_precision, achieved_precision, warnings));
   }
   return partials;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterCovariance
//***************************************************************************
double UsgsAstroLsSensorModel::getParameterCovariance(
   int index1,
   int index2) const
{
   int index = UsgsAstroLsSensorModel::NUM_PARAMETERS * index1 + index2;
   return m_covariance[index];
}

//***************************************************************************
// UsgsAstroLsSensorModel::setParameterCovariance
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterCovariance(
   int index1,
   int index2,
   double covariance)
{
   int index = UsgsAstroLsSensorModel::NUM_PARAMETERS * index1 + index2;
   m_covariance[index] = covariance;
}

//---------------------------------------------------------------------------
// Time and Trajectory
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getTrajectoryIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getTrajectoryIdentifier() const
{
   return m_trajectoryIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getReferenceDateAndTime
//***************************************************************************
std::string UsgsAstroLsSensorModel::getReferenceDateAndTime() const
{
   if (m_referenceDateAndTime == "UNKNOWN")
   {
      throw csm::Error(
         csm::Error::UNSUPPORTED_FUNCTION,
         "Unsupported function",
         "UsgsAstroLsSensorModel::getReferenceDateAndTime");
   }
   return m_referenceDateAndTime;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getImageTime
//***************************************************************************
double UsgsAstroLsSensorModel::getImageTime(
   const csm::ImageCoord& image_pt) const
{
   // Flip image taken backwards
   double line1 = image_pt.line;

   // CSM image convention: UL pixel center == (0.5, 0.5)
   // USGS image convention: UL pixel center == (1.0, 1.0)

   double lineCSMFull = line1 + m_offsetLines;
   double lineUSGSFull = floor(lineCSMFull) + 0.5;

   // These calculation assumes that the values in the integration time
   // vectors are in terms of ISIS' pixels
   auto referenceLineIt = std::upper_bound(m_intTimeLines.begin(),
                                           m_intTimeLines.end(),
                                           lineUSGSFull);
   if (referenceLineIt != m_intTimeLines.begin()) {
      --referenceLineIt;
   }
   size_t referenceIndex = std::distance(m_intTimeLines.begin(), referenceLineIt);

   double time = m_intTimeStartTimes[referenceIndex]
      + m_intTimes[referenceIndex] * (lineUSGSFull - m_intTimeLines[referenceIndex]);

   return time;

}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorPosition
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::getSensorPosition(
   const csm::ImageCoord& imagePt) const
{
   return getSensorPosition(getImageTime(imagePt));
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorPosition
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::getSensorPosition(double time) const

{
   double x, y, z, vx, vy, vz;
   getAdjSensorPosVel(time, _no_adjustment, x, y, z, vx, vy, vz);

   return csm::EcefCoord(x, y, z);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorVelocity
//***************************************************************************
csm::EcefVector UsgsAstroLsSensorModel::getSensorVelocity(
   const csm::ImageCoord& imagePt) const
{
   return getSensorVelocity(getImageTime(imagePt));
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorVelocity
//***************************************************************************
csm::EcefVector UsgsAstroLsSensorModel::getSensorVelocity(double time) const
{
   double x, y, z, vx, vy, vz;
   getAdjSensorPosVel(time, _no_adjustment, x, y, z, vx, vy, vz);

   return csm::EcefVector(vx, vy, vz);
}

//---------------------------------------------------------------------------
// Sensor Model Parameters
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::setParameterValue
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterValue(int index, double value)
{
   m_parameterVals[index] = value;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterValue
//***************************************************************************
double UsgsAstroLsSensorModel::getParameterValue(int index) const
{
   return m_parameterVals[index];
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterName
//***************************************************************************
std::string UsgsAstroLsSensorModel::getParameterName(int index) const
{
   return PARAMETER_NAME[index];
}

std::string UsgsAstroLsSensorModel::getParameterUnits(int index) const
{
   // All parameters are meters or scaled to meters
   return "m";
}


//***************************************************************************
// UsgsAstroLsSensorModel::getNumParameters
//***************************************************************************
int UsgsAstroLsSensorModel::getNumParameters() const
{
   return UsgsAstroLsSensorModel::NUM_PARAMETERS;
}


//***************************************************************************
// UsgsAstroLsSensorModel::getParameterType
//***************************************************************************
csm::param::Type UsgsAstroLsSensorModel::getParameterType(int index) const
{
   return m_parameterType[index];
}


//***************************************************************************
// UsgsAstroLsSensorModel::setParameterType
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterType(
   int index, csm::param::Type pType)
{
   m_parameterType[index] = pType;
}

//---------------------------------------------------------------------------
// Sensor Model Information
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getPedigree
//***************************************************************************
std::string UsgsAstroLsSensorModel::getPedigree() const
{
   return "USGS_LINE_SCANNER";
}

//***************************************************************************
// UsgsAstroLsSensorModel::getImageIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getImageIdentifier() const
{
   return m_imageIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::setImageIdentifier
//***************************************************************************
void UsgsAstroLsSensorModel::setImageIdentifier(
   const std::string& imageId,
   csm::WarningList* warnings)
{
   // Image id should include the suffix without the path name
   m_imageIdentifier = imageId;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getSensorIdentifier() const
{
   return m_sensorIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getPlatformIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getPlatformIdentifier() const
{
   return m_platformIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::setReferencePoint
//***************************************************************************
void UsgsAstroLsSensorModel::setReferencePoint(const csm::EcefCoord& ground_pt)
{
   m_referencePointXyz = ground_pt;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getReferencePoint
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::getReferencePoint() const
{
   // Return ground point at image center
   return m_referencePointXyz;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorModelName
//***************************************************************************
std::string UsgsAstroLsSensorModel::getModelName() const
{
   return UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getImageStart
//***************************************************************************
csm::ImageCoord UsgsAstroLsSensorModel::getImageStart() const
{
   return csm::ImageCoord(0.0, 0.0);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getImageSize
//***************************************************************************
csm::ImageVector UsgsAstroLsSensorModel::getImageSize() const
{
   return csm::ImageVector(m_totalLines, m_totalSamples);
}

//---------------------------------------------------------------------------
// Sensor Model State
//---------------------------------------------------------------------------
//
// //***************************************************************************
// // UsgsAstroLsSensorModel::getSensorModelState
// //***************************************************************************
// std::string UsgsAstroLsSensorModel::setModelState(std::string stateString) const
// {
//   auto j = json::parse(stateString);
//   int num_params    = NUM_PARAMETERS;
//   int num_paramsSq = num_params * num_params;
//
//   m_imageIdentifier = j["m_imageIdentifier"];
//   m_sensorType = j["m_sensorType"];
//   m_totalLines = j["m_totalLines"];
//   m_totalSamples = j["m_totalSamples"];
//   m_offsetLines = j["m_offsetLines"];
//   m_offsetSamples = j["m_offsetSamples"];
//   m_platformFlag = j["m_platformFlag"];
//   m_aberrFlag = j["m_aberrFlag"];
//   m_atmRefFlag = j["m_atmrefFlag"];
//   m_intTimeLines = j["m_intTimeLines"].get<std::vector<double>>();
//   m_intTimeStartTimes = j["m_intTimeStartTimes"].get<std::vector<double>>();
//   m_intTimes = j["m_intTimes"].get<std::vector<double>>();
//   m_startingEphemerisTime = j["m_startingEphemerisTime"];
//   m_centerEphemerisTime = j["m_centerEphemerisTime"];
//   m_detectorSampleSumming = j["m_detectorSampleSumming"];
//   m_startingSample = j["m_startingSample"];
//   m_ikCode = j["m_ikCode"];
//   m_focal = j["m_focal"];
//   m_isisZDirection = j["m_isisZDirection"];
//   for (int i = 0; i < 3; i++) {
//     m_opticalDistCoef[i] = j["m_opticalDistCoef"][i];
//     m_iTransS[i] = j["m_iTransS"][i];
//     m_iTransL[i] = j["m_iTransL"][i];
//   }
//   m_detectorSampleOrigin = j["m_detectorSampleOrigin"];
//   m_detectorLineOrigin = j["m_detectorLineOrigin"];
//   m_detectorLineOffset = j["m_detectorLineOffset"];
//   for (int i = 0; i < 9; i++) {
//       m_mountingMatrix[i] = j["m_mountingMatrix"][i];
//   }
//   m_semiMajorAxis = j["m_semiMajorAxis"];
//   m_semiMinorAxis = j["m_semiMinorAxis"];
//   m_referenceDateAndTime = j["m_referenceDateAndTime"];
//   m_platformIdentifier = j["m_platformIdentifier"];
//   m_sensorIdentifier = j["m_sensorIdentifier"];
//   m_trajectoryIdentifier = j["m_trajectoryIdentifier"];
//   m_collectionIdentifier = j["m_collectionIdentifier"];
//   m_refElevation = j["m_refElevation"];
//   m_minElevation = j["m_minElevation"];
//   m_maxElevation = j["m_maxElevation"];
//   m_dtEphem = j["m_dtEphem"];
//   m_t0Ephem = j["m_t0Ephem"];
//   m_dtQuat = j["m_dtQuat"];
//   m_t0Quat = j["m_t0Quat"];
//   m_numEphem = j["m_numEphem"];
//   m_numQuaternions = j["m_numQuaternions"];
//   m_referencePointXyz.x = j["m_referencePointXyz"][0];
//   m_referencePointXyz.y = j["m_referencePointXyz"][1];
//   m_referencePointXyz.z = j["m_referencePointXyz"][2];
//   m_gsd = j["m_gsd"];
//   m_flyingHeight = j["m_flyingHeight"];
//   m_halfSwath = j["m_halfSwath"];
//   m_halfTime = j["m_halfTime"];
//   m_imageFlipFlag = j["m_imageFlipFlag"];
//   // Vector = is overloaded so explicit get with type required.
//   m_ephemPts = j["m_ephemPts"].get<std::vector<double>>();
//   m_ephemRates = j["m_ephemRates"].get<std::vector<double>>();
//   m_quaternions = j["m_quaternions"].get<std::vector<double>>();
//   m_parameterVals = j["m_parameterVals"].get<std::vector<double>>();
//   m_covariance = j["m_covariance"].get<std::vector<double>>();
//
//   for (int i = 0; i < num_params; i++) {
//     for (int k = 0; k < NUM_PARAM_TYPES; k++) {
//       if (j["m_parameterType"][i] == PARAM_STRING_ALL[k]) {
//         m_parameterType[i] = PARAM_CHAR_ALL[k];
//         break;
//     }
//    }
//   }
//
// }

//---------------------------------------------------------------------------
//  Monoscopic Mensuration
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getValidHeightRange
//***************************************************************************
std::pair<double, double> UsgsAstroLsSensorModel::getValidHeightRange() const
{
   return std::pair<double, double>(m_minElevation, m_maxElevation);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getValidImageRange
//***************************************************************************
std::pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroLsSensorModel::getValidImageRange() const
{
   return std::pair<csm::ImageCoord, csm::ImageCoord>(
      csm::ImageCoord(0.0, 0.0),
      csm::ImageCoord(m_totalLines, m_totalSamples));
}

//***************************************************************************
// UsgsAstroLsSensorModel::getIlluminationDirection
//***************************************************************************
csm::EcefVector UsgsAstroLsSensorModel::getIlluminationDirection(
   const csm::EcefCoord& groundPt) const
{
   throw csm::Error(
      csm::Error::UNSUPPORTED_FUNCTION,
      "Unsupported function",
      "UsgsAstroLsSensorModel::getIlluminationDirection");
}

//---------------------------------------------------------------------------
//  Error Correction
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getNumGeometricCorrectionSwitches
//***************************************************************************
int UsgsAstroLsSensorModel::getNumGeometricCorrectionSwitches() const
{
   return 0;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getGeometricCorrectionName
//***************************************************************************

std::string UsgsAstroLsSensorModel::getGeometricCorrectionName(int index) const
{
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::getGeometricCorrectionName");
}

//***************************************************************************
// UsgsAstroLsSensorModel::setGeometricCorrectionSwitch
//***************************************************************************
void UsgsAstroLsSensorModel::setGeometricCorrectionSwitch(
   int  index,
   bool value,
   csm::param::Type pType)

{
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::setGeometricCorrectionSwitch");
}

//***************************************************************************
// UsgsAstroLsSensorModel::getGeometricCorrectionSwitch
//***************************************************************************
bool UsgsAstroLsSensorModel::getGeometricCorrectionSwitch(int index) const
{
   // Since there are no geometric corrections, all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index is out of range.",
      "UsgsAstroLsSensorModel::getGeometricCorrectionSwitch");
}

//***************************************************************************
// UsgsAstroLsSensorModel::getCrossCovarianceMatrix
//***************************************************************************
std::vector<double> UsgsAstroLsSensorModel::getCrossCovarianceMatrix(
   const csm::GeometricModel& comparisonModel,
   csm::param::Set            pSet,
   const csm::GeometricModel::GeometricModelList& otherModels) const
{
   // No correlation between models.
   const std::vector<int>& indices = getParameterSetIndices(pSet);
   size_t num_rows = indices.size();
   const std::vector<int>& indices2 = comparisonModel.getParameterSetIndices(pSet);
   size_t num_cols = indices.size();

   return std::vector<double>(num_rows * num_cols, 0.0);
}

const csm::CorrelationModel&
UsgsAstroLsSensorModel::getCorrelationModel() const
{
   // All Line Scanner images are assumed uncorrelated
   return _no_corr_model;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getUnmodeledCrossCovariance
//***************************************************************************
std::vector<double> UsgsAstroLsSensorModel::getUnmodeledCrossCovariance(
   const csm::ImageCoord& pt1,
   const csm::ImageCoord& pt2) const
{
   // No unmodeled error
   return std::vector<double>(4, 0.0);
}


//***************************************************************************
// UsgsAstroLsSensorModel::getCollectionIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getCollectionIdentifier() const
{
   return m_collectionIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::hasShareableParameters
//***************************************************************************
bool UsgsAstroLsSensorModel::hasShareableParameters() const
{
   // Parameter sharing is not supported for this sensor
   return false;
}

//***************************************************************************
// UsgsAstroLsSensorModel::isParameterShareable
//***************************************************************************
bool UsgsAstroLsSensorModel::isParameterShareable(int index) const
{
   // Parameter sharing is not supported for this sensor
   return false;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterSharingCriteria
//***************************************************************************
csm::SharingCriteria UsgsAstroLsSensorModel::getParameterSharingCriteria(
   int index) const
{
   // Parameter sharing is not supported for this sensor,
   // all indices are out of range
   throw csm::Error(
      csm::Error::INDEX_OUT_OF_RANGE,
      "Index out of range.",
      "UsgsAstroLsSensorModel::getParameterSharingCriteria");
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorType
//***************************************************************************
std::string UsgsAstroLsSensorModel::getSensorType() const
{
   return CSM_SENSOR_TYPE_EO;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorMode
//***************************************************************************
std::string UsgsAstroLsSensorModel::getSensorMode() const
{
   return CSM_SENSOR_MODE_PB;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getVersion
//***************************************************************************
csm::Version UsgsAstroLsSensorModel::getVersion() const
{
   return csm::Version(1, 0, 0);
}


csm::Ellipsoid UsgsAstroLsSensorModel::getEllipsoid() const
{
   return csm::Ellipsoid(m_semiMajorAxis, m_semiMinorAxis);
}

void UsgsAstroLsSensorModel::setEllipsoid(
   const csm::Ellipsoid &ellipsoid)
{
   m_semiMajorAxis = ellipsoid.getSemiMajorRadius();
   m_semiMinorAxis = ellipsoid.getSemiMinorRadius();
}



double UsgsAstroLsSensorModel::getValue(
   int   index,
   const std::vector<double> &adjustments) const
{
   return m_parameterVals[index] + adjustments[index];
}


//***************************************************************************
// Functions pulled out of losToEcf and computeViewingPixel
// **************************************************************************

// Compute distorted focalPlane coordinates in mm
void UsgsAstroLsSensorModel::computeDistortedFocalPlaneCoordinates(const double& line, const double& sample, double& distortedLine, double& distortedSample) const{
  double detSample = (sample - 1.0)
      * m_detectorSampleSumming + m_startingSample;
  double m11 = m_iTransL[1];
  double m12 = m_iTransL[2];
  double m21 = m_iTransS[1];
  double m22 = m_iTransS[2];
  double t1 = line + m_detectorLineOffset
               - m_detectorLineOrigin - m_iTransL[0];
  double t2 = detSample - m_detectorSampleOrigin - m_iTransS[0];
  double determinant = m11 * m22 - m12 * m21;
  double p11 = m11 / determinant;
  double p12 = -m12 / determinant;
  double p21 = -m21 / determinant;
  double p22 = m22 / determinant;
  distortedLine = p11 * t1 + p12 * t2;
  distortedSample = p21 * t1 + p22 * t2;
}

// Compute un-distorted image coordinates in mm / apply lens distortion correction
void UsgsAstroLsSensorModel::computeUndistortedFocalPlaneCoordinates(const double &distortedFocalPlaneX, const double& distortedFocalPlaneY, double& undistortedFocalPlaneX, double& undistortedFocalPlaneY) const{
  undistortedFocalPlaneX = distortedFocalPlaneX;
  undistortedFocalPlaneY = distortedFocalPlaneY;

  std::tuple<double, double> dpoint;

  dpoint = removeDistortion(distortedFocalPlaneX, distortedFocalPlaneY, m_opticalDistCoef);
  undistortedFocalPlaneX = std::get<0>(dpoint);
  undistortedFocalPlaneY = std::get<1>(dpoint);
};


// Define imaging ray in image space (In other words, create a look vector in camera space)
void UsgsAstroLsSensorModel::createCameraLookVector(const double& undistortedFocalPlaneX, const double& undistortedFocalPlaneY, const std::vector<double>& adj, double cameraLook[]) const{
   cameraLook[0] = -undistortedFocalPlaneX * m_zDirection;
   cameraLook[1] = -undistortedFocalPlaneY * m_zDirection;
   cameraLook[2] = -m_focal * (1.0 - getValue(15, adj) / m_halfSwath);
   double magnitude = sqrt(cameraLook[0] * cameraLook[0]
                  + cameraLook[1] * cameraLook[1]
                  + cameraLook[2] * cameraLook[2]);
   cameraLook[0] /= magnitude;
   cameraLook[1] /= magnitude;
   cameraLook[2] /= magnitude;
};


// Given a time and a flag to indicate whether the a->b or b->a rotation should be calculated
// uses the quaternions in the m_quaternions member to calclate a rotation matrix.
void UsgsAstroLsSensorModel::calculateRotationMatrixFromQuaternions(const double& time, double rotationMatrix[9]) const {
  int nOrder = 8;
  if (m_platformFlag == 0)
     nOrder = 4;
  int nOrderQuat = nOrder;
  if (m_numQuaternions < 6 && nOrder == 8)
     nOrderQuat = 4;
  double q[4];
  lagrangeInterp(
     m_numQuaternions, &m_quaternions[0], m_t0Quat, m_dtQuat,
     time, 4, nOrderQuat, q);
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
};

// Calculates a rotation matrix from Euler angles
void UsgsAstroLsSensorModel::calculateRotationMatrixFromEuler(double euler[],
                                                              double rotationMatrix[]) const {
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

void UsgsAstroLsSensorModel::calculateAttitudeCorrection(const double& time, const std::vector<double>& adj, double attCorr[9]) const {
  double aTime = time - m_t0Quat;
  double euler[3];
  double nTime = aTime / m_halfTime;
  double nTime2 = nTime * nTime;
  euler[0] =
    (getValue(6, adj) + getValue(9, adj)* nTime + getValue(12, adj)* nTime2) / m_flyingHeight;
  euler[1] =
    (getValue(7, adj) + getValue(10, adj)* nTime + getValue(13, adj)* nTime2) / m_flyingHeight;
  euler[2] =
    (getValue(8, adj) + getValue(11, adj)* nTime + getValue(14, adj)* nTime2) / m_halfSwath;

  calculateRotationMatrixFromEuler(euler, attCorr);
}


//***************************************************************************
// UsgsAstroLsSensorModel::losToEcf
//***************************************************************************
void UsgsAstroLsSensorModel::losToEcf(
   const double& line,       // CSM image convention
   const double& sample,     //    UL pixel center == (0.5, 0.5)
   const std::vector<double>& adj, // Parameter Adjustments for partials
   double&       xc,         // output sensor x coordinate
   double&       yc,         // output sensor y coordinate
   double&       zc,         // output sensor z coordinate
   double&       vx,         // output sensor x velocity
   double&       vy,         // output sensor y velocity
   double&       vz,         // output sensor z velocity
   double&       bodyLookX,         // output line-of-sight x coordinate
   double&       bodyLookY,         // output line-of-sight y coordinate
   double&       bodyLookZ) const  // output line-of-sight z coordinate
{
   //# private_func_description
   //  Computes image ray (look vector) in ecf coordinate system.
   // Compute adjusted sensor position and velocity

   double time = getImageTime(csm::ImageCoord(line, sample));
   getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz);
   // CSM image image convention: UL pixel center == (0.5, 0.5)
   // USGS image convention: UL pixel center == (1.0, 1.0)
   double sampleCSMFull = sample + m_offsetSamples;
   double sampleUSGSFull = sampleCSMFull;
   double fractionalLine = line - floor(line);

   // Compute distorted image coordinates in mm (sample, line on image (pixels) -> focal plane
   double natFocalPlaneX, natFocalPlaneY;
   computeDistortedFocalPlaneCoordinates(fractionalLine, sampleUSGSFull, natFocalPlaneX, natFocalPlaneY);

   // Remove lens distortion
   double focalPlaneX, focalPlaneY;
   computeUndistortedFocalPlaneCoordinates(natFocalPlaneX, natFocalPlaneY, focalPlaneX, focalPlaneY);

  // Define imaging ray (look vector) in camera space
   double cameraLook[3];
   createCameraLookVector(focalPlaneX, focalPlaneY, adj, cameraLook);

   // Apply attitude correction
   double attCorr[9];
   calculateAttitudeCorrection(time, adj, attCorr);

   double correctedCameraLook[3];
   correctedCameraLook[0] = attCorr[0] * cameraLook[0]
                          + attCorr[1] * cameraLook[1]
                          + attCorr[2] * cameraLook[2];
   correctedCameraLook[1] = attCorr[3] * cameraLook[0]
                          + attCorr[4] * cameraLook[1]
                          + attCorr[5] * cameraLook[2];
   correctedCameraLook[2] = attCorr[6] * cameraLook[0]
                          + attCorr[7] * cameraLook[1]
                          + attCorr[8] * cameraLook[2];

// Rotate the look vector into the body fixed frame from the camera reference frame by applying the rotation matrix from the sensor quaternions
   double cameraToBody[9];
   calculateRotationMatrixFromQuaternions(time, cameraToBody);

   bodyLookX = cameraToBody[0] * correctedCameraLook[0]
             + cameraToBody[1] * correctedCameraLook[1]
             + cameraToBody[2] * correctedCameraLook[2];
   bodyLookY = cameraToBody[3] * correctedCameraLook[0]
             + cameraToBody[4] * correctedCameraLook[1]
             + cameraToBody[5] * correctedCameraLook[2];
   bodyLookZ = cameraToBody[6] * correctedCameraLook[0]
             + cameraToBody[7] * correctedCameraLook[1]
             + cameraToBody[8] * correctedCameraLook[2];
}


//***************************************************************************
// UsgsAstroLsSensorModel::lightAberrationCorr
//**************************************************************************
void UsgsAstroLsSensorModel::lightAberrationCorr(
   const double& vx,
   const double& vy,
   const double& vz,
   const double& xl,
   const double& yl,
   const double& zl,
   double&       dxl,
   double&       dyl,
   double&       dzl) const
{
   //# func_description
   //  Computes light aberration correction vector

   // Compute angle between the image ray and the velocity vector

   double dotP = xl * vx + yl * vy + zl * vz;
   double losMag = sqrt(xl * xl + yl * yl + zl * zl);
   double velocityMag = sqrt(vx *  vx + vy * vy + vz * vz);
   double cosThetap = dotP / (losMag * velocityMag);
   double sinThetap = sqrt(1.0 - cosThetap * cosThetap);

   // Image ray is parallel to the velocity vector

   if (1.0 == fabs(cosThetap))
   {
      dxl = 0.0;
      dyl = 0.0;
      dzl = 0.0;
   }

   // Compute the angle between the corrected image ray and spacecraft
   // velocity.  This key equation is derived using Lorentz transform.

   double speedOfLight = 299792458.0;   // meters per second
   double beta = velocityMag / speedOfLight;
   double cosTheta = (beta - cosThetap) / (beta * cosThetap - 1.0);
   double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

   // Compute line-of-sight correction

   double cfac = ((cosTheta * sinThetap
      - sinTheta * cosThetap) * losMag)
      / (sinTheta * velocityMag);
   dxl = cfac * vx;
   dyl = cfac * vy;
   dzl = cfac * vz;
}

//***************************************************************************
// UsgsAstroLsSensorModel::lagrangeInterp
//***************************************************************************
void UsgsAstroLsSensorModel::lagrangeInterp(
   const int&     numTime,
   const double*  valueArray,
   const double&  startTime,
   const double&  delTime,
   const double&  time,
   const int&     vectorLength,
   const int&     i_order,
   double*        valueVector) const
{
   // Lagrange interpolation for uniform post interval.
   // Largest order possible is 8th. Points far away from
   // data center are handled gracefully to avoid failure.

   // Compute index

   double fndex = (time - startTime) / delTime;
   int    index = int(fndex);

   //Time outside range
   //printf("%f | %i\n", fndex, index);
   //if (index < 0 || index >= numTime - 1) {
   //    printf("TIME ISSUE\n");
   // double d1 = fndex / (numTime - 1);
   // double d0 = 1.0 - d1;
   // int indx0 = vectorLength * (numTime - 1);
   // for (int i = 0; i < vectorLength; i++)
   // {
   // valueVector[i] = d0 * valueArray[i] + d1 * valueArray[indx0 + i];
   // }
   // return;
   //}

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
   else if (index == 2 || index == numTime - 4) {
      order = 6;
   }
   else if (index == 1 || index == numTime - 3) {
      order = 4;
   }
   else if (index == 0 || index == numTime - 2) {
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

//***************************************************************************
// UsgsAstroLsSensorModel::computeElevation
//***************************************************************************
void UsgsAstroLsSensorModel::computeElevation(
   const double& x,
   const double& y,
   const double& z,
   double&       height,
   double&       achieved_precision,
   const double& desired_precision) const
{
   // Compute elevation given xyz
   // Requires semi-major-axis and eccentricity-square
   const int MKTR = 10;
   double ecc_sqr = 1.0 - m_semiMinorAxis * m_semiMinorAxis / m_semiMajorAxis / m_semiMajorAxis;
   double ep2 = 1.0 - ecc_sqr;
   double d2 = x * x + y * y;
   double d = sqrt(d2);
   double h = 0.0;
   int ktr = 0;
   double hPrev, r;

   // Suited for points near equator
   if (d >= z)
   {
      double tt, zz, n;
      double tanPhi = z / d;
      do
      {
         hPrev = h;
         tt = tanPhi * tanPhi;
         r = m_semiMajorAxis / sqrt(1.0 + ep2 * tt);
         zz = z + r * ecc_sqr * tanPhi;
         n = r * sqrt(1.0 + tt);
         h = sqrt(d2 + zz * zz) - n;
         tanPhi = zz / d;
         ktr++;
      } while (MKTR > ktr && fabs(h - hPrev) > desired_precision);
   }

   // Suited for points near the poles
   else
   {
      double cc, dd, nn;
      double cotPhi = d / z;
      do
      {
         hPrev = h;
         cc = cotPhi * cotPhi;
         r = m_semiMajorAxis / sqrt(ep2 + cc);
         dd = d - r * ecc_sqr * cotPhi;
         nn = r * sqrt(1.0 + cc) * ep2;
         h = sqrt(dd * dd + z * z) - nn;
         cotPhi = dd / z;
         ktr++;
      } while (MKTR > ktr && fabs(h - hPrev) > desired_precision);
   }

   height = h;
   achieved_precision = fabs(h - hPrev);
}

//***************************************************************************
// UsgsAstroLsSensorModel::losEllipsoidIntersect
//**************************************************************************
void UsgsAstroLsSensorModel::losEllipsoidIntersect(
   const double& height,
   const double& xc,
   const double& yc,
   const double& zc,
   const double& xl,
   const double& yl,
   const double& zl,
   double&       x,
   double&       y,
   double&       z,
   double&       achieved_precision,
   const double& desired_precision) const
{
   // Helper function which computes the intersection of the image ray
   // with the ellipsoid.  All vectors are in earth-centered-fixed
   // coordinate system with origin at the center of the earth.

   const int MKTR = 10;

   double ap, bp, k;
   ap = m_semiMajorAxis + height;
   bp = m_semiMinorAxis + height;
   k = ap * ap / (bp * bp);

   // Solve quadratic equation for scale factor
   // applied to image ray to compute ground point

   double at, bt, ct, quadTerm;
   at = xl * xl + yl * yl + k * zl * zl;
   bt = 2.0 * (xl * xc + yl * yc + k * zl * zc);
   ct = xc * xc + yc * yc + k * zc * zc - ap * ap;
   quadTerm = bt * bt - 4.0 * at * ct;

   // If quadTerm is negative, the image ray does not
   // intersect the ellipsoid. Setting the quadTerm to
   // zero means solving for a point on the ray nearest
   // the surface of the ellisoid.

   if (0.0 > quadTerm)
   {
      quadTerm = 0.0;
   }
   double scale, scale1, h, slope;
   double sprev, hprev;
   double aPrec, sTerm;
   int ktr = 0;

   // Compute ground point vector

   sTerm = sqrt(quadTerm);
   scale = (-bt - sTerm);
   scale1 = (-bt + sTerm);
   if (fabs(scale1) < fabs(scale))
      scale = scale1;
   scale /= (2.0 * at);
   x = xc + scale * xl;
   y = yc + scale * yl;
   z = zc + scale * zl;
   computeElevation(x, y, z, h, aPrec, desired_precision);
   slope = -1;

   while (MKTR > ktr && fabs(height - h) > desired_precision)
   {
      sprev = scale;
      scale += slope * (height - h);
      x = xc + scale * xl;
      y = yc + scale * yl;
      z = zc + scale * zl;
      hprev = h;
      computeElevation(x, y, z, h, aPrec, desired_precision);
      slope = (sprev - scale) / (hprev - h);
      ktr++;
   }

   achieved_precision = fabs(height - h);
}

//***************************************************************************
// UsgsAstroLsSensorModel::losPlaneIntersect
//**************************************************************************
void UsgsAstroLsSensorModel::losPlaneIntersect(
   const double& xc,          // input: camera x coordinate
   const double& yc,          // input: camera y coordinate
   const double& zc,          // input: camera z coordinate
   const double& xl,          // input: component x image ray
   const double& yl,          // input: component y image ray
   const double& zl,          // input: component z image ray
   double&       x,           // input/output: ground x coordinate
   double&       y,           // input/output: ground y coordinate
   double&       z,           // input/output: ground z coordinate
   int&          mode) const // input: -1 fixed component to be computed
                          //         0(X), 1(Y), or 2(Z) fixed
                          // output: 0(X), 1(Y), or 2(Z) fixed
{
   //# func_description
   //  Computes 2 of the 3 coordinates of a ground point given the 3rd
   //  coordinate.  The 3rd coordinate that is held fixed corresponds
   //  to the largest absolute component of the image ray.

   // Define fixed or largest component

   if (-1 == mode)
   {
      if (fabs(xl) > fabs(yl) && fabs(xl) > fabs(zl))
      {
         mode = 0;
      }
      else if (fabs(yl) > fabs(xl) && fabs(yl) > fabs(zl))
      {
         mode = 1;
      }
      else
      {
         mode = 2;
      }
   }

   // X is the fixed or largest component

   if (0 == mode)
   {
      y = yc + (x - xc) * yl / xl;
      z = zc + (x - xc) * zl / xl;
   }

   // Y is the fixed or largest component

   else if (1 == mode)
   {
      x = xc + (y - yc) * xl / yl;
      z = zc + (y - yc) * zl / yl;
   }

   // Z is the fixed or largest component

   else
   {
      x = xc + (z - zc) * xl / zl;
      y = yc + (z - zc) * yl / zl;
   }
}

//***************************************************************************
// UsgsAstroLsSensorModel::imageToPlane
//***************************************************************************
void UsgsAstroLsSensorModel::imageToPlane(
   const double& line,      // CSM Origin UL corner of UL pixel
   const double& sample,    // CSM Origin UL corner of UL pixel
   const double& height,
   const std::vector<double> &adj,
   double&       x,
   double&       y,
   double&       z,
   int&          mode) const
{
   //# func_description
   //  Computes ground coordinates by intersecting image ray with
   //  a plane perpendicular to the coordinate axis with the largest
   //  image ray component.  This routine is primarily called by
   //  groundToImage().

   // *** Computes camera position and image ray in ecf cs.

   double xc, yc, zc;
   double vx, vy, vz;
   double xl, yl, zl;
   double dxl, dyl, dzl;

   losToEcf(line, sample, adj, xc, yc, zc, vx, vy, vz, xl, yl, zl);

   if (m_aberrFlag == 1)
   {
      lightAberrationCorr(vx, vy, vz, xl, yl, zl, dxl, dyl, dzl);
      xl += dxl;
      yl += dyl;
      zl += dzl;
   }

   losPlaneIntersect(xc, yc, zc, xl, yl, zl, x, y, z, mode);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getAdjSensorPosVel
//***************************************************************************
void UsgsAstroLsSensorModel::getAdjSensorPosVel(
   const double& time,
   const std::vector<double> &adj,
   double&       xc,
   double&       yc,
   double&       zc,
   double&       vx,
   double&       vy,
   double&       vz) const
{
   // Sensor position and velocity (4th or 8th order Lagrange).
   int nOrder = 8;
   if (m_platformFlag == 0)
      nOrder = 4;
   double sensPosNom[3];
   lagrangeInterp(m_numEphem, &m_ephemPts[0], m_t0Ephem, m_dtEphem,
      time, 3, nOrder, sensPosNom);
   double sensVelNom[3];
   lagrangeInterp(m_numEphem, &m_ephemRates[0], m_t0Ephem, m_dtEphem,
      time, 3, nOrder, sensVelNom);
   // Compute rotation matrix from ICR to ECF

   double radialUnitVec[3];
   double radMag = sqrt(sensPosNom[0] * sensPosNom[0] +
      sensPosNom[1] * sensPosNom[1] +
      sensPosNom[2] * sensPosNom[2]);
   for (int i = 0; i < 3; i++)
      radialUnitVec[i] = sensPosNom[i] / radMag;
   double crossTrackUnitVec[3];
   crossTrackUnitVec[0] = sensPosNom[1] * sensVelNom[2]
      - sensPosNom[2] * sensVelNom[1];
   crossTrackUnitVec[1] = sensPosNom[2] * sensVelNom[0]
      - sensPosNom[0] * sensVelNom[2];
   crossTrackUnitVec[2] = sensPosNom[0] * sensVelNom[1]
      - sensPosNom[1] * sensVelNom[0];
   double crossMag = sqrt(crossTrackUnitVec[0] * crossTrackUnitVec[0] +
      crossTrackUnitVec[1] * crossTrackUnitVec[1] +
      crossTrackUnitVec[2] * crossTrackUnitVec[2]);
   for (int i = 0; i < 3; i++)
      crossTrackUnitVec[i] /= crossMag;
   double inTrackUnitVec[3];
   inTrackUnitVec[0] = crossTrackUnitVec[1] * radialUnitVec[2]
      - crossTrackUnitVec[2] * radialUnitVec[1];
   inTrackUnitVec[1] = crossTrackUnitVec[2] * radialUnitVec[0]
      - crossTrackUnitVec[0] * radialUnitVec[2];
   inTrackUnitVec[2] = crossTrackUnitVec[0] * radialUnitVec[1]
      - crossTrackUnitVec[1] * radialUnitVec[0];
   double ecfFromIcr[9];
   ecfFromIcr[0] = inTrackUnitVec[0];
   ecfFromIcr[1] = crossTrackUnitVec[0];
   ecfFromIcr[2] = radialUnitVec[0];
   ecfFromIcr[3] = inTrackUnitVec[1];
   ecfFromIcr[4] = crossTrackUnitVec[1];
   ecfFromIcr[5] = radialUnitVec[1];
   ecfFromIcr[6] = inTrackUnitVec[2];
   ecfFromIcr[7] = crossTrackUnitVec[2];
   ecfFromIcr[8] = radialUnitVec[2];
   // Apply position and velocity corrections

   double aTime = time - m_t0Ephem;
   double dvi = getValue(3, adj) / m_halfTime;
   double dvc = getValue(4, adj) / m_halfTime;
   double dvr = getValue(5, adj) / m_halfTime;
   vx = sensVelNom[0]
      + ecfFromIcr[0] * dvi + ecfFromIcr[1] * dvc + ecfFromIcr[2] * dvr;
   vy = sensVelNom[1]
      + ecfFromIcr[3] * dvi + ecfFromIcr[4] * dvc + ecfFromIcr[5] * dvr;
   vz = sensVelNom[2]
      + ecfFromIcr[6] * dvi + ecfFromIcr[7] * dvc + ecfFromIcr[8] * dvr;
   double di = getValue(0, adj) + dvi * aTime;
   double dc = getValue(1, adj) + dvc * aTime;
   double dr = getValue(2, adj) + dvr * aTime;
   xc = sensPosNom[0]
      + ecfFromIcr[0] * di + ecfFromIcr[1] * dc + ecfFromIcr[2] * dr;
   yc = sensPosNom[1]
      + ecfFromIcr[3] * di + ecfFromIcr[4] * dc + ecfFromIcr[5] * dr;
   zc = sensPosNom[2]
      + ecfFromIcr[6] * di + ecfFromIcr[7] * dc + ecfFromIcr[8] * dr;
}


//***************************************************************************
// UsgsAstroLineScannerSensorModel::computeViewingPixel
//***************************************************************************
csm::ImageCoord UsgsAstroLsSensorModel::computeViewingPixel(
   const double& time,
   const csm::EcefCoord& groundPoint,
   const std::vector<double>& adj,
   const double& desiredPrecision) const
{
  // Helper function to compute the CCD pixel that views a ground point based
  // on the exterior orientation at a given time.

   // Get the exterior orientation
   double xc, yc, zc, vx, vy, vz;
   getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz);

   // Compute the look vector
   double bodyLookX = groundPoint.x - xc;
   double bodyLookY = groundPoint.y - yc;
   double bodyLookZ = groundPoint.z - zc;

   // Rotate the look vector into the camera reference frame
   double bodyToCamera[9];
   calculateRotationMatrixFromQuaternions(time, bodyToCamera);

   // Apply transpose of matrix to rotate body->camera
   double cameraLookX = bodyToCamera[0] * bodyLookX
                      + bodyToCamera[3] * bodyLookY
                      + bodyToCamera[6] * bodyLookZ;
   double cameraLookY = bodyToCamera[1] * bodyLookX
                      + bodyToCamera[4] * bodyLookY
                      + bodyToCamera[7] * bodyLookZ;
   double cameraLookZ = bodyToCamera[2] * bodyLookX
                      + bodyToCamera[5] * bodyLookY
                      + bodyToCamera[8] * bodyLookZ;

   // Invert the attitude correction
   double attCorr[9];
   calculateAttitudeCorrection(time, adj, attCorr);

   // Apply transpose of matrix to invert the attidue correction
   double adjustedLookX = attCorr[0] * cameraLookX
                        + attCorr[3] * cameraLookY
                        + attCorr[6] * cameraLookZ;
   double adjustedLookY = attCorr[1] * cameraLookX
                        + attCorr[4] * cameraLookY
                        + attCorr[7] * cameraLookZ;
   double adjustedLookZ = attCorr[2] * cameraLookX
                        + attCorr[5] * cameraLookY
                        + attCorr[8] * cameraLookZ;

   // Convert to focal plane coordinate
   double lookScale = m_focal / adjustedLookZ;
   double focalX = adjustedLookX * lookScale;
   double focalY = adjustedLookY * lookScale;
   std::tuple<double, double> dpoint;

   // Invert distortion
   dpoint = invertDistortion(focalX, focalY, m_opticalDistCoef, desiredPrecision);

   // Convert to detector line and sample
   double detectorLine = m_iTransL[0]
                       + m_iTransL[1] * std::get<0>(dpoint)
                       + m_iTransL[2] * std::get<1>(dpoint);
   double detectorSample = m_iTransS[0]
                         + m_iTransS[1] * std::get<0>(dpoint)
                         + m_iTransS[2] * std::get<1>(dpoint);

   // Convert to image sample line
   double line = detectorLine + m_detectorLineOrigin - m_detectorLineOffset
               - m_offsetLines;
   double sample = (detectorSample + m_detectorSampleOrigin - m_startingSample + 1.0)
                 / m_detectorSampleSumming - m_offsetSamples;

   return csm::ImageCoord(line, sample);
}


//***************************************************************************
// UsgsAstroLineScannerSensorModel::computeLinearApproximation
//***************************************************************************
void UsgsAstroLsSensorModel::computeLinearApproximation(
   const csm::EcefCoord &gp,
   csm::ImageCoord      &ip) const
{
   if (_linear)
   {
      ip.line = _u0 + _du_dx * gp.x + _du_dy * gp.y + _du_dz * gp.z;
      ip.samp = _v0 + _dv_dx * gp.x + _dv_dy * gp.y + _dv_dz * gp.z;

      // Since this is valid only over image,
      // don't let result go beyond the image border.
      double numRows = m_totalLines;
      double numCols = m_totalSamples;
      if (ip.line < 0.0)     ip.line = 0.0;
      if (ip.line > numRows) ip.line = numRows;

      if (ip.samp < 0.0)     ip.samp = 0.0;
      if (ip.samp > numCols) ip.samp = numCols;
   }
   else
   {
      ip.line = m_totalLines / 2.0;
      ip.samp = m_totalSamples / 2.0;
   }
}


//***************************************************************************
// UsgsAstroLineScannerSensorModel::setLinearApproximation
//***************************************************************************
void UsgsAstroLsSensorModel::setLinearApproximation()
{
   double u_factors[4] = { 0.0, 0.0, 1.0, 1.0 };
   double v_factors[4] = { 0.0, 1.0, 0.0, 1.0 };

   csm::EcefCoord refPt = getReferencePoint();

   double height, aPrec;
   double desired_precision = 0.01;
   computeElevation(refPt.x, refPt.y, refPt.z, height, aPrec, desired_precision);
   if (isnan(height))
   {
      _linear = false;
      return;
   }


   double numRows = m_totalLines;
   double numCols = m_totalSamples;

   csm::ImageCoord imagePt;
   csm::EcefCoord  gp[8];

   int i;
   for (i = 0; i < 4; i++)
   {
      imagePt.line = u_factors[i] * numRows;
      imagePt.samp = v_factors[i] * numCols;
      gp[i] = imageToGround(imagePt, height);
   }

   double delz = 100.0;
   height += delz;
   for (i = 0; i < 4; i++)
   {
      imagePt.line = u_factors[i] * numRows;
      imagePt.samp = v_factors[i] * numCols;
      gp[i + 4] = imageToGround(imagePt, height);
   }

   csm::EcefCoord d_du;
   d_du.x = (
      (gp[2].x + gp[3].x + gp[6].x + gp[7].x) -
      (gp[0].x + gp[1].x + gp[4].x + gp[5].x)) / numRows / 4.0;
   d_du.y = (
      (gp[2].y + gp[3].y + gp[6].y + gp[7].y) -
      (gp[0].y + gp[1].y + gp[4].y + gp[5].y)) / numRows / 4.0;
   d_du.z = (
      (gp[2].z + gp[3].z + gp[6].z + gp[7].z) -
      (gp[0].z + gp[1].z + gp[4].z + gp[5].z)) / numRows / 4.0;

   csm::EcefCoord d_dv;
   d_dv.x = (
      (gp[1].x + gp[3].x + gp[5].x + gp[7].x) -
      (gp[0].x + gp[2].x + gp[4].x + gp[6].x)) / numCols / 4.0;
   d_dv.y = (
      (gp[1].y + gp[3].y + gp[5].y + gp[7].y) -
      (gp[0].y + gp[2].y + gp[4].y + gp[6].y)) / numCols / 4.0;
   d_dv.z = (
      (gp[1].z + gp[3].z + gp[5].z + gp[7].z) -
      (gp[0].z + gp[2].z + gp[4].z + gp[6].z)) / numCols / 4.0;

   double mat3x3[9];

   mat3x3[0] = d_du.x;
   mat3x3[1] = d_dv.x;
   mat3x3[2] = 1.0;
   mat3x3[3] = d_du.y;
   mat3x3[4] = d_dv.y;
   mat3x3[5] = 1.0;
   mat3x3[6] = d_du.z;
   mat3x3[7] = d_dv.z;
   mat3x3[8] = 1.0;

   double denom = determinant3x3(mat3x3);

   if (fabs(denom) < 1.0e-8) // can not get derivatives this way
   {
      _linear = false;
      return;
   }

   mat3x3[0] = 1.0;
   mat3x3[3] = 0.0;
   mat3x3[6] = 0.0;
   _du_dx = determinant3x3(mat3x3) / denom;

   mat3x3[0] = 0.0;
   mat3x3[3] = 1.0;
   mat3x3[6] = 0.0;
   _du_dy = determinant3x3(mat3x3) / denom;

   mat3x3[0] = 0.0;
   mat3x3[3] = 0.0;
   mat3x3[6] = 1.0;
   _du_dz = determinant3x3(mat3x3) / denom;

   mat3x3[0] = d_du.x;
   mat3x3[3] = d_du.y;
   mat3x3[6] = d_du.z;

   mat3x3[1] = 1.0;
   mat3x3[4] = 0.0;
   mat3x3[7] = 0.0;
   _dv_dx = determinant3x3(mat3x3) / denom;

   mat3x3[1] = 0.0;
   mat3x3[4] = 1.0;
   mat3x3[7] = 0.0;
   _dv_dy = determinant3x3(mat3x3) / denom;

   mat3x3[1] = 0.0;
   mat3x3[4] = 0.0;
   mat3x3[7] = 1.0;
   _dv_dz = determinant3x3(mat3x3) / denom;

   _u0 = -gp[0].x * _du_dx - gp[0].y * _du_dy - gp[0].z * _du_dz;
   _v0 = -gp[0].x * _dv_dx - gp[0].y * _dv_dy - gp[0].z * _dv_dz;

   _linear = true;
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::determinant3x3
//***************************************************************************
double UsgsAstroLsSensorModel::determinant3x3(double mat[9]) const
{
   return
      mat[0] * (mat[4] * mat[8] - mat[7] * mat[5]) -
      mat[1] * (mat[3] * mat[8] - mat[6] * mat[5]) +
      mat[2] * (mat[3] * mat[7] - mat[6] * mat[4]);
}




std::string UsgsAstroLsSensorModel::constructStateFromIsd(const std::string imageSupportData, csm::WarningList *warnings) const
{
   // Instantiate UsgsAstroLineScanner sensor model
   json isd = json::parse(imageSupportData);
   json state;

   int num_params = NUM_PARAMETERS;

   state["m_imageIdentifier"] = isd.at("image_identifier");
   state["m_sensorType"] = "LINE_SCAN";
   state["m_totalLines"] = isd.at("image_lines");
   state["m_totalSamples"] = isd.at("image_samples");
   state["m_offsetLines"] = 0.0;
   state["m_offsetSamples"] = 0.0;
   state["m_platformFlag"] = 1;
   state["m_aberrFlag"] = 0;
   state["m_atmRefFlag"] = 0;
   state["m_centerEphemerisTime"] = isd.at("center_ephemeris_time");
   state["m_startingEphemerisTime"] = isd.at("starting_ephemeris_time");

   for (auto& scanRate : isd.at("line_scan_rate")) {
     state["m_intTimeLines"].push_back(scanRate[0]);
     state["m_intTimeStartTimes"].push_back(scanRate[1]);
     state["m_intTimes"].push_back(scanRate[2]);
   }

   state["m_detectorSampleSumming"] = isd.at("detector_sample_summing");
   state["m_startingSample"] = isd.at("detector_line_summing");
   state["m_ikCode"] = 0;
   state["m_focal"] = isd.at("focal_length_model").at("focal_length");
   state["m_zDirection"] = 1;
   state["m_opticalDistCoef"] = isd.at("optical_distortion").at("radial").at("coefficients");
   state["m_iTransS"] = isd.at("focal2pixel_samples");
   state["m_iTransL"] = isd.at("focal2pixel_lines");

   state["m_detectorSampleOrigin"] = isd.at("detector_center").at("sample");
   state["m_detectorLineOrigin"] = isd.at("detector_center").at("line");
   state["m_detectorLineOffset"] = 0;

   state["m_dtEphem"] = isd.at("dt_ephemeris");
   state["m_t0Ephem"] = isd.at("t0_ephemeris");
   state["m_dtQuat"] =  isd.at("dt_quaternion");
   state["m_t0Quat"] =  isd.at("t0_quaternion");

   state["m_numEphem"] = isd.at("sensor_position").at("positions").size();
   state["m_numQuaternions"] = isd.at("sensor_orientation").at("quaternions").size();

   for (auto& location : isd.at("sensor_position").at("positions")) {
     state["m_ephemPts"].push_back(location[0]);
     state["m_ephemPts"].push_back(location[1]);
     state["m_ephemPts"].push_back(location[2]);
   }

   for (auto& velocity : isd.at("sensor_position").at("velocities")) {
     state["m_ephemRates"].push_back(velocity[0]);
     state["m_ephemRates"].push_back(velocity[1]);
     state["m_ephemRates"].push_back(velocity[2]);
   }

   for (auto& quaternion : isd.at("sensor_orientation").at("quaternions")) {
     state["m_quaternions"].push_back(quaternion[0]);
     state["m_quaternions"].push_back(quaternion[1]);
     state["m_quaternions"].push_back(quaternion[2]);
     state["m_quaternions"].push_back(quaternion[3]);
    }


   state["m_parameterVals"] = std::vector<double>(NUM_PARAMETERS, 0.0);
   // state["m_parameterVals"][15] = state["m_focal"].get<double>();

   // Set the ellipsoid
   state["m_semiMajorAxis"] = isd.at("radii").at("semimajor");
   state["m_semiMinorAxis"] = isd.at("radii").at("semiminor");

   // Now finish setting the state data from the ISD read in

   // set identifiers
   state["m_referenceDateAndTime"] = "UNKNOWN";
   state["m_platformIdentifier"]   = isd.at("name_platform");
   state["m_sensorIdentifier"]     = isd.at("name_sensor");
   state["m_trajectoryIdentifier"] = "UNKNOWN";
   state["m_collectionIdentifier"] = "UNKNOWN";

   // Ground elevations
   state["m_refElevation"] = 0.0;
   state["m_minElevation"] = isd.at("reference_height").at("minheight");
   state["m_maxElevation"] = isd.at("reference_height").at("maxheight");

   // Zero ateter values
   for (int i = 0; i < NUM_PARAMETERS; i++) {
      state["m_ateterType"][i] = csm::param::REAL;
   }

   // Default to identity covariance
   state["m_covariance"] =
         std::vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
   for (int i = 0; i < NUM_PARAMETERS; i++) {
     state["m_covariance"][i * NUM_PARAMETERS + i] = 1.0;
   }

   // Zero computed state values
   state["m_referencePointXyz"] = std::vector<double>(3, 0.0);
   state["m_gsd"] = 1.0;
   state["m_flyingHeight"] = 1000.0;
   state["m_halfSwath"] = 1000.0;
   state["m_halfTime"] = 10.0;
   state["m_imageFlipFlag"] = 0;
   // The state data will still be updated when a sensor model is created since
   // some state data is notin the ISD and requires a SM to compute them.
   return state.dump();
}
