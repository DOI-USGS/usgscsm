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
//  DESCRIPTION:
//
//    Holds State data that runs the Astro Line Scanner Sensor Model.
//    The format of the state data is determined by the sensor model
//    developer. The CSM plugin converts ISD to state data and the
//    CSM sensor model is instantiated based on the state data.
//
//    By CSM convention, the sensor model name is the first element in
//    the state data. The rest of the state data is order independent
//    with the exception that the length of vectors must come before
//    the vector data.
//
//    For the Astro Line Scanner sensor model, the state data closely
//    follows the support data. This is not always the case. For this
//    model, the state data is a list name/value pairs.
//
//
//    SOFTWARE HISTORY:
//
//    Date          Author       Comment
//    -----------   ----------   -------
//    13-OCT-2017   BAE Systems  Initial Release
//
//#############################################################################
#ifndef __USGS_ASTRO_LINE_SCANNER_STATE_DATA_H
#define __USGS_ASTRO_LINE_SCANNER_STATE_DATA_H

#include <vector>
#include <string>

#include <csm/csm.h>
#include <csm/SettableEllipsoid.h>

class UsgsAstroLsStateData
{
   public:

   UsgsAstroLsStateData()
   {
      reset();
   }

   UsgsAstroLsStateData(const std::string &state)
   {
      reset();
      setState(state);
   }

   ~UsgsAstroLsStateData() {}

   // Formats the state data as a string.
   // This is the format that is used to instantiate a sensor model.
   std::string toString() const;

   // Formats the sate data as a JSON string.
   std::string toJson() const;

   // Initializes the class from state data as formatted
   // in a string by the toString() method
   void setState(const std::string &state);

   // This method checks to see if the model name is recognized
   // in the input state string.
   static std::string getModelNameFromModelState(
      const std::string& model_state);

   // State data elements;
   std::string  m_ImageIdentifier;                // 1
   std::string  m_SensorType;                     // 2
   int          m_TotalLines;                     // 3
   int          m_TotalSamples;                   // 4
   double       m_OffsetLines;                    // 5
   double       m_OffsetSamples;                  // 6
   int          m_PlatformFlag;                   // 7
   int          m_AberrFlag;                      // 8
   int          m_AtmRefFlag;                     // 9
   double       m_IntTime;                        // 10
   double       m_StartingEphemerisTime;          // 11
   double       m_CenterEphemerisTime;            // 12
   double       m_DetectorSampleSumming;          // 13
   double       m_StartingSample;                 // 14
   int          m_IkCode;                         // 15
   double       m_Focal;                          // 16
   double       m_IsisZDirection;                 // 17
   double       m_OpticalDistCoef[3];             // 18
   double       m_ITransS[3];                     // 19
   double       m_ITransL[3];                     // 20
   double       m_DetectorSampleOrigin;           // 21
   double       m_DetectorLineOrigin;             // 22
   double       m_DetectorLineOffset;             // 23
   double       m_MountingMatrix[9];              // 24
   double       m_SemiMajorAxis;                  // 25
   double       m_SemiMinorAxis;                  // 26
   std::string  m_ReferenceDateAndTime;           // 27
   std::string  m_PlatformIdentifier;             // 28
   std::string  m_SensorIdentifier;               // 29
   std::string  m_TrajectoryIdentifier;           // 30
   std::string  m_CollectionIdentifier;           // 31
   double       m_RefElevation;                   // 32
   double       m_MinElevation;                   // 33
   double       m_MaxElevation;                   // 34
   double       m_DtEphem;                        // 35
   double       m_T0Ephem;                        // 36
   double       m_DtQuat;                         // 37
   double       m_T0Quat;                         // 38
   int          m_NumEphem;                       // 39
   int          m_NumQuaternions;                 // 40
   std::vector<double> m_EphemPts;                // 41
   std::vector<double> m_EphemRates;              // 42
   std::vector<double> m_Quaternions;             // 43
   std::vector<double> m_ParameterVals;           // 44
   std::vector<csm::param::Type> m_ParameterType; // 45
   csm::EcefCoord m_ReferencePointXyz;            // 46
   double       m_Gsd;                            // 47
   double       m_FlyingHeight;                   // 48
   double       m_HalfSwath;                      // 49
   double       m_HalfTime;                       // 50
   std::vector<double> m_Covariance;              // 51
   int          m_ImageFlipFlag;                  // 52

   // Hardcoded
   static const std::string      SENSOR_MODEL_NAME; // state date element 0

   static const std::string      STATE_KEYWORD[];
   static const int              NUM_PARAM_TYPES;
   static const std::string      PARAM_STRING_ALL[];
   static const csm::param::Type PARAM_CHAR_ALL[];
   static const int              NUM_PARAMETERS;
   static const std::string      PARAMETER_NAME[];

   enum
   {
       STA_SENSOR_MODEL_NAME,
       STA_IMAGE_IDENTIFIER,
       STA_SENSOR_TYPE,
       STA_TOTAL_LINES,
       STA_TOTAL_SAMPLES,
       STA_OFFSET_LINES,
       STA_OFFSET_SAMPLES,
       STA_PLATFORM_FLAG,
       STA_ABERR_FLAG,
       STA_ATMREF_FLAG,
       STA_INT_TIME,
       STA_STARTING_EPHEMERIS_TIME,
       STA_CENTER_EPHEMERIS_TIME,
       STA_DETECTOR_SAMPLE_SUMMING,
       STA_STARTING_SAMPLE,
       STA_IK_CODE,
       STA_FOCAL,
       STA_ISIS_Z_DIRECTION,
       STA_OPTICAL_DIST_COEF,
       STA_I_TRANS_S,
       STA_I_TRANS_L,
       STA_DETECTOR_SAMPLE_ORIGIN,
       STA_DETECTOR_LINE_ORIGIN,
       STA_DETECTOR_LINE_OFFSET,
       STA_MOUNTING_MATRIX,
       STA_SEMI_MAJOR_AXIS,
       STA_SEMI_MINOR_AXIS,
       STA_REFERENCE_DATE_AND_TIME,
       STA_PLATFORM_IDENTIFIER,
       STA_SENSOR_IDENTIFIER,
       STA_TRAJECTORY_IDENTIFIER,
       STA_COLLECTION_IDENTIFIER,
       STA_REF_ELEVATION,
       STA_MIN_ELEVATION,
       STA_MAX_ELEVATION,
       STA_DT_EPHEM,
       STA_T0_EPHEM,
       STA_DT_QUAT,
       STA_T0_QUAT,
       STA_NUM_EPHEM,
       STA_NUM_QUATERNIONS,
       STA_EPHEM_PTS,
       STA_EPHEM_RATES,
       STA_QUATERNIONS,
       STA_PARAMETER_VALS,
       STA_PARAMETER_TYPE,
       STA_REFERENCE_POINT_XYZ,
       STA_GSD,
       STA_FLYING_HEIGHT,
       STA_HALF_SWATH,
       STA_HALF_TIME,
       STA_COVARIANCE,
       STA_IMAGE_FLIP_FLAG,
       _NUM_STATE_KEYWORDS
   };

   // Set to default values
   void reset();
};

#endif
