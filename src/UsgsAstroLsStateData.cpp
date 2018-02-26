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
//    FILENAME:          UsgsAstroLsStateData.h
//
//    DESCRIPTION:
//
//    Holds state data used to build the Astro Line Scanner sensor model.
//
//    LIMITATIONS:       None
//
//    SOFTWARE HISTORY:
//
//    Date          Author       Comment
//    -----------   ----------   -------
//    13-OCT-2017   BAE Systems  Initial Release
//
//
//#############################################################################

#define USGSASTROLINESCANNER_LIBRARY


#include <genericls/UsgsAstroLsStateData.h>
#include <genericls/UsgsAstroLsPlugin.h>
#include <sstream>
#include <csm/Error.h>
#include <json/json.hpp>
using json = nlohmann::json;

const std::string  UsgsAstroLsStateData::SENSOR_MODEL_NAME
         = "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL";
const int     UsgsAstroLsStateData::NUM_PARAMETERS = 16;
const std::string  UsgsAstroLsStateData::PARAMETER_NAME[] =
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


const std::string  UsgsAstroLsStateData::STATE_KEYWORD[] =
{
   "STA_SENSOR_MODEL_NAME",
   "STA_IMAGE_IDENTIFIER",
   "STA_SENSOR_TYPE",
   "STA_TOTAL_LINES",
   "STA_TOTAL_SAMPLES",
   "STA_OFFSET_LINES",
   "STA_OFFSET_SAMPLES",
   "STA_PLATFORM_FLAG",
   "STA_ABERR_FLAG",
   "STA_ATMREF_FLAG",
   "STA_INT_TIME_LINES",
   "STA_INT_TIME_START_TIMES",
   "STA_INT_TIMES",
   "STA_STARTING_EPHEMERIS_TIME",
   "STA_CENTER_EPHEMERIS_TIME",
   "STA_DETECTOR_SAMPLE_SUMMING",
   "STA_STARTING_SAMPLE",
   "STA_IK_CODE",
   "STA_FOCAL",
   "STA_ISIS_Z_DIRECTION",
   "STA_OPTICAL_DIST_COEF",
   "STA_I_TRANS_S",
   "STA_I_TRANS_L",
   "STA_DETECTOR_SAMPLE_ORIGIN",
   "STA_DETECTOR_LINE_ORIGIN",
   "STA_DETECTOR_LINE_OFFSET",
   "STA_MOUNTING_MATRIX",
   "STA_SEMI_MAJOR_AXIS",
   "STA_SEMI_MINOR_AXIS",
   "STA_REFERENCE_DATE_AND_TIME",
   "STA_PLATFORM_IDENTIFIER",
   "STA_SENSOR_IDENTIFIER",
   "STA_TRAJECTORY_IDENTIFIER",
   "STA_COLLECTION_IDENTIFIER",
   "STA_REF_ELEVATION",
   "STA_MIN_ELEVATION",
   "STA_MAX_ELEVATION",
   "STA_DT_EPHEM",
   "STA_T0_EPHEM",
   "STA_DT_QUAT",
   "STA_T0_QUAT",
   "STA_NUM_EPHEM",
   "STA_NUM_QUATERNIONS",
   "STA_EPHEM_PTS",
   "STA_EPHEM_RATES",
   "STA_QUATERNIONS",
   "STA_PARAMETER_VALS",
   "STA_PARAMETER_TYPE",
   "STA_REFERENCE_POINT_XYZ",
   "STA_GSD",
   "STA_FLYING_HEIGHT",
   "STA_HALF_SWATH",
   "STA_HALF_TIME",
   "STA_COVARIANCE",
   "STA_IMAGE_FLIP_FLAG"
};

const int UsgsAstroLsStateData::NUM_PARAM_TYPES = 4;
const std::string UsgsAstroLsStateData::PARAM_STRING_ALL[] =
{
   "NONE",
   "FICTITIOUS",
   "REAL",
   "FIXED"
};
const csm::param::Type
      UsgsAstroLsStateData::PARAM_CHAR_ALL[] =
{
   csm::param::NONE,
   csm::param::FICTITIOUS,
   csm::param::REAL,
   csm::param::FIXED
};


std::string UsgsAstroLsStateData::toJson() const {
    json state = {
        {STATE_KEYWORD[STA_SENSOR_MODEL_NAME], SENSOR_MODEL_NAME},
        {STATE_KEYWORD[STA_IMAGE_IDENTIFIER], m_ImageIdentifier},
        {STATE_KEYWORD[STA_SENSOR_TYPE], m_SensorType},
        {STATE_KEYWORD[STA_TOTAL_LINES], m_TotalLines},
        {STATE_KEYWORD[STA_TOTAL_SAMPLES], m_TotalSamples},
        {STATE_KEYWORD[STA_OFFSET_LINES], m_OffsetLines},
        {STATE_KEYWORD[STA_OFFSET_SAMPLES], m_OffsetSamples},
        {STATE_KEYWORD[STA_PLATFORM_FLAG], m_PlatformFlag},
        {STATE_KEYWORD[STA_ABERR_FLAG], m_AberrFlag},
        {STATE_KEYWORD[STA_ATMREF_FLAG], m_AtmRefFlag},
        {STATE_KEYWORD[STA_INT_TIME_LINES], m_IntTimeLines},
        {STATE_KEYWORD[STA_INT_TIME_START_TIMES], m_IntTimeStartTimes},
        {STATE_KEYWORD[STA_INT_TIMES], m_IntTimes},
        {STATE_KEYWORD[STA_STARTING_EPHEMERIS_TIME], m_StartingEphemerisTime},
        {STATE_KEYWORD[STA_CENTER_EPHEMERIS_TIME], m_CenterEphemerisTime},
        {STATE_KEYWORD[STA_DETECTOR_SAMPLE_SUMMING], m_DetectorSampleSumming},
        {STATE_KEYWORD[STA_STARTING_SAMPLE], m_StartingSample},
        {STATE_KEYWORD[STA_IK_CODE], m_IkCode},
        {STATE_KEYWORD[STA_FOCAL], m_Focal},
        {STATE_KEYWORD[STA_ISIS_Z_DIRECTION], m_IsisZDirection},
        {STATE_KEYWORD[STA_IMAGE_FLIP_FLAG] , m_ImageFlipFlag},
        {STATE_KEYWORD[STA_REFERENCE_POINT_XYZ],
          {m_ReferencePointXyz.x,m_ReferencePointXyz.y, m_ReferencePointXyz.z}},
        {STATE_KEYWORD[STA_GSD], m_Gsd},
        {STATE_KEYWORD[STA_FLYING_HEIGHT], m_FlyingHeight},
        {STATE_KEYWORD[STA_HALF_SWATH], m_HalfSwath},
        {STATE_KEYWORD[STA_HALF_TIME], m_HalfTime},
        {STATE_KEYWORD[STA_SEMI_MAJOR_AXIS], m_SemiMajorAxis},
        {STATE_KEYWORD[STA_SEMI_MINOR_AXIS], m_SemiMinorAxis},
        {STATE_KEYWORD[STA_REFERENCE_DATE_AND_TIME], m_ReferenceDateAndTime},
        {STATE_KEYWORD[STA_PLATFORM_IDENTIFIER], m_PlatformIdentifier},
        {STATE_KEYWORD[STA_SENSOR_IDENTIFIER], m_SensorIdentifier},
        {STATE_KEYWORD[STA_TRAJECTORY_IDENTIFIER], m_TrajectoryIdentifier},
        {STATE_KEYWORD[STA_COLLECTION_IDENTIFIER], m_CollectionIdentifier},
        {STATE_KEYWORD[STA_REF_ELEVATION], m_RefElevation},
        {STATE_KEYWORD[STA_MIN_ELEVATION], m_MinElevation},
        {STATE_KEYWORD[STA_MAX_ELEVATION], m_MaxElevation},
        {STATE_KEYWORD[STA_DT_EPHEM], m_DtEphem},
        {STATE_KEYWORD[STA_T0_EPHEM], m_T0Ephem},
        {STATE_KEYWORD[STA_DT_QUAT], m_DtQuat},
        {STATE_KEYWORD[STA_T0_QUAT], m_T0Quat},
        {STATE_KEYWORD[STA_NUM_EPHEM], m_NumEphem},
        {STATE_KEYWORD[STA_NUM_QUATERNIONS], m_NumQuaternions},
        {STATE_KEYWORD[STA_DETECTOR_SAMPLE_ORIGIN], m_DetectorSampleOrigin},
        {STATE_KEYWORD[STA_DETECTOR_LINE_ORIGIN], m_DetectorLineOrigin},
        {STATE_KEYWORD[STA_DETECTOR_LINE_OFFSET], m_DetectorLineOffset},
        {STATE_KEYWORD[STA_OPTICAL_DIST_COEF],
                {m_OpticalDistCoef[0], m_OpticalDistCoef[1], m_OpticalDistCoef[2]}},
        {STATE_KEYWORD[STA_COVARIANCE], m_Covariance},
        {STATE_KEYWORD[STA_I_TRANS_S], {m_ITransS[0], m_ITransS[1], m_ITransS[2]}},
        {STATE_KEYWORD[STA_I_TRANS_L],{m_ITransL[0], m_ITransL[1], m_ITransL[2]}},
        {STATE_KEYWORD[STA_MOUNTING_MATRIX],
             {m_MountingMatrix[0], m_MountingMatrix[1], m_MountingMatrix[2],
              m_MountingMatrix[3], m_MountingMatrix[4], m_MountingMatrix[5],
              m_MountingMatrix[6], m_MountingMatrix[7], m_MountingMatrix[8]}},
        {STATE_KEYWORD[STA_EPHEM_PTS], m_EphemPts},
        {STATE_KEYWORD[STA_EPHEM_RATES], m_EphemRates},
        {STATE_KEYWORD[STA_QUATERNIONS], m_Quaternions},
        {STATE_KEYWORD[STA_PARAMETER_VALS], m_ParameterVals},
        {STATE_KEYWORD[STA_PARAMETER_TYPE], m_ParameterType}
    };
    return state.dump();
}

std::string UsgsAstroLsStateData::toString() const
{
   std::stringstream state_stream(std::ios_base::out);
   state_stream.unsetf (std::ios::floatfield);
   state_stream.precision(14);
   state_stream << STATE_KEYWORD[STA_SENSOR_MODEL_NAME] << " " << SENSOR_MODEL_NAME << "\n";  // 0
   state_stream << STATE_KEYWORD[STA_IMAGE_IDENTIFIER]  << " " << m_ImageIdentifier   << "\n";  // 1
   state_stream << STATE_KEYWORD[STA_SENSOR_TYPE]       << " " << m_SensorType        << "\n";  // 2
   state_stream << STATE_KEYWORD[STA_TOTAL_LINES]       << " " << m_TotalLines        << "\n";  // 3
   state_stream << STATE_KEYWORD[STA_TOTAL_SAMPLES]     << " " << m_TotalSamples      << "\n";  // 4
   state_stream << STATE_KEYWORD[STA_OFFSET_LINES]      << " " << m_OffsetLines       << "\n";  // 7
   state_stream << STATE_KEYWORD[STA_OFFSET_SAMPLES]    << " " << m_OffsetSamples     << "\n";  // 8
   state_stream << STATE_KEYWORD[STA_PLATFORM_FLAG]     << " " << m_PlatformFlag      << "\n";  // 9
   state_stream << STATE_KEYWORD[STA_ABERR_FLAG] << " " << m_AberrFlag         << "\n";  // 10
   state_stream << STATE_KEYWORD[STA_ATMREF_FLAG]<< " " << m_AtmRefFlag        << "\n";  // 11
   state_stream << STATE_KEYWORD[STA_INT_TIME_LINES]    << " " ;
   for (size_t i = 0; i < m_IntTimeLines.size(); i++)
   {
      state_stream << m_IntTimeLines[i] << " ";
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_INT_TIME_START_TIMES] << " " ;
   for (size_t i = 0; i < m_IntTimeStartTimes.size(); i++)
   {
      state_stream << m_IntTimeStartTimes[i] << " ";
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_INT_TIMES]         << " " ;
   for (size_t i = 0; i < m_IntTimes.size(); i++)
   {
      state_stream << m_IntTimes[i] << " ";
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_STARTING_EPHEMERIS_TIME]  << " "
                                           << m_StartingEphemerisTime << "\n";  // 13
   state_stream << STATE_KEYWORD[STA_CENTER_EPHEMERIS_TIME]    << " "
                                           << m_CenterEphemerisTime   << "\n";  // 14
   state_stream << STATE_KEYWORD[STA_DETECTOR_SAMPLE_SUMMING]  << " "
                                           << m_DetectorSampleSumming << "\n";  // 15
   state_stream << STATE_KEYWORD[STA_STARTING_SAMPLE]   << " " << m_StartingSample     << "\n";  // 16
   state_stream << STATE_KEYWORD[STA_IK_CODE]    << " " << m_IkCode             << "\n";  // 17
   state_stream << STATE_KEYWORD[STA_FOCAL]      << " " << m_Focal              << "\n";  // 18
   state_stream << STATE_KEYWORD[STA_ISIS_Z_DIRECTION]  << " " << m_IsisZDirection     << "\n";  // 19
   state_stream << STATE_KEYWORD[STA_OPTICAL_DIST_COEF] << " ";
   for (int i = 0; i < 3; i++) {
      state_stream << m_OpticalDistCoef[i] << " " ;                      // 20
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_I_TRANS_S] << " ";
   for (int i = 0; i < 3; i++) {
      state_stream << m_ITransS[i] << " " ;                              // 21
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_I_TRANS_L]<< " ";
   for (int i = 0; i < 3; i++) {
      state_stream << m_ITransL[i] << " ";                              // 22
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_DETECTOR_SAMPLE_ORIGIN] << " "
                                          << m_DetectorSampleOrigin << "\n";    // 23
   state_stream << STATE_KEYWORD[STA_DETECTOR_LINE_ORIGIN] << " "
                                          << m_DetectorLineOrigin << "\n";      // 24
   state_stream << STATE_KEYWORD[STA_DETECTOR_LINE_OFFSET] << " "
                                          << m_DetectorLineOffset << "\n";      // 25
   state_stream << STATE_KEYWORD[STA_MOUNTING_MATRIX] << " ";
   for (int i =0; i < 9; i++) {
      state_stream << m_MountingMatrix[i] << " ";                       // 26
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_SEMI_MAJOR_AXIS] << " " << m_SemiMajorAxis  << "\n";          // 27
   state_stream << STATE_KEYWORD[STA_SEMI_MINOR_AXIS] << " " << m_SemiMinorAxis << "\n";          // 28
   state_stream << STATE_KEYWORD[STA_REFERENCE_DATE_AND_TIME] << " "
                                           << m_ReferenceDateAndTime << "\n";   // 30
   state_stream << STATE_KEYWORD[STA_PLATFORM_IDENTIFIER] << " "
                                       << m_PlatformIdentifier << "\n";         // 31
   state_stream << STATE_KEYWORD[STA_SENSOR_IDENTIFIER]   << " "
                                       << m_SensorIdentifier << "\n";           // 32
   state_stream << STATE_KEYWORD[STA_TRAJECTORY_IDENTIFIER] << " "
                                       << m_TrajectoryIdentifier << "\n";       // 33
   state_stream << STATE_KEYWORD[STA_COLLECTION_IDENTIFIER] << " "
                                       << m_CollectionIdentifier << "\n";       // 33
   state_stream << STATE_KEYWORD[STA_REF_ELEVATION]   << " " << m_RefElevation   << "\n";  // 34
   state_stream << STATE_KEYWORD[STA_MIN_ELEVATION]   << " " << m_MinElevation   << "\n";  // 34
   state_stream << STATE_KEYWORD[STA_MAX_ELEVATION]   << " " << m_MaxElevation   << "\n";  // 35
   state_stream << STATE_KEYWORD[STA_DT_EPHEM] << " " <<m_DtEphem        << "\n";  // 36
   state_stream << STATE_KEYWORD[STA_T0_EPHEM] << " " <<m_T0Ephem        << "\n";  // 37
   state_stream << STATE_KEYWORD[STA_DT_QUAT]  << " " << m_DtQuat         << "\n";  // 38
   state_stream << STATE_KEYWORD[STA_T0_QUAT]  << " " <<m_T0Quat         << "\n";  // 39
   state_stream << STATE_KEYWORD[STA_NUM_EPHEM]<< " " <<m_NumEphem       << "\n";  // 40
   state_stream << STATE_KEYWORD[STA_NUM_QUATERNIONS] << " " << m_NumQuaternions << "\n";  // 41
   state_stream << STATE_KEYWORD[STA_EPHEM_PTS]       << " " ;
   for (int i = 0; i < 3*m_NumEphem; i++)
   {
      state_stream << m_EphemPts[i] << " ";                             // 42
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_EPHEM_RATES] << " ";
   for (int i = 0; i < 3*m_NumEphem; i++)
   {
      state_stream << m_EphemRates[i] << " ";                           // 43
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_QUATERNIONS] << " ";
   for (int i = 0; i < 4*m_NumQuaternions; i++)
   {
      state_stream << m_Quaternions[i] << " ";                          // 44
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_PARAMETER_VALS]  << " ";
   for (int i = 0; i < NUM_PARAMETERS; i++)
   {
      state_stream << m_ParameterVals[i] << " ";                     // 45
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_PARAMETER_TYPE]  << " ";
   for (int i = 0; i < NUM_PARAMETERS; i++)
   {
      for (int j = 0; j < NUM_PARAM_TYPES; j++)
      {
         if (m_ParameterType[i] == PARAM_CHAR_ALL[j])
         {
            state_stream << PARAM_STRING_ALL[j] << " ";               // 46
            break;
         }
      }
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_REFERENCE_POINT_XYZ]  << " ";
   state_stream << m_ReferencePointXyz.x << " " ;                    // 47
   state_stream << m_ReferencePointXyz.y << " " ;                    // 47
   state_stream << m_ReferencePointXyz.z << "\n" ;                   // 47
   state_stream << STATE_KEYWORD[STA_GSD]    << " " <<m_Gsd           << "\n";  // 48
   state_stream << STATE_KEYWORD[STA_FLYING_HEIGHT] << " " << m_FlyingHeight  << "\n";  // 49
   state_stream << STATE_KEYWORD[STA_HALF_SWATH]    << " " << m_HalfSwath     << "\n";  // 50
   state_stream << STATE_KEYWORD[STA_HALF_TIME]     << " " << m_HalfTime      << "\n";  // 51
   state_stream << STATE_KEYWORD[STA_COVARIANCE] << " ";
   for (int i = 0; i < NUM_PARAMETERS*NUM_PARAMETERS; i++)
   {
      state_stream << m_Covariance[i] << " ";                    // 52
   }
   state_stream << "\n";
   state_stream << STATE_KEYWORD[STA_IMAGE_FLIP_FLAG] << " " << m_ImageFlipFlag << "\n";  // 53
//   state_stream << std::ends;
   return state_stream.str();
}


void UsgsAstroLsStateData::setState(const std::string &stateString )
{
   reset();
   auto j = json::parse(stateString);
   int num_params    = NUM_PARAMETERS;
   int num_params_sq = num_params * num_params;

   m_ImageIdentifier = j["STA_IMAGE_IDENTIFIER"];
   m_SensorType = j["STA_SENSOR_TYPE"];
   m_TotalLines = j["STA_TOTAL_LINES"];
   m_TotalSamples = j["STA_TOTAL_SAMPLES"];
   m_OffsetLines = j["STA_OFFSET_LINES"];
   m_OffsetSamples = j["STA_OFFSET_SAMPLES"];
   m_PlatformFlag = j["STA_PLATFORM_FLAG"];
   m_AberrFlag = j["STA_ABERR_FLAG"];
   m_AtmRefFlag = j["STA_ATMREF_FLAG"];
   m_IntTimeLines = j["STA_INT_TIME_LINES"].get<std::vector<int>>();
   m_IntTimeStartTimes = j["STA_INT_TIME_START_TIMES"].get<std::vector<double>>();
   m_IntTimes = j["STA_INT_TIMES"].get<std::vector<double>>();
   m_StartingEphemerisTime = j["STA_STARTING_EPHEMERIS_TIME"];
   m_CenterEphemerisTime = j["STA_CENTER_EPHEMERIS_TIME"];
   m_DetectorSampleSumming = j["STA_DETECTOR_SAMPLE_SUMMING"];
   m_StartingSample = j["STA_STARTING_SAMPLE"];
   m_IkCode = j["STA_IK_CODE"];
   m_Focal = j["STA_FOCAL"];
   m_IsisZDirection = j["STA_ISIS_Z_DIRECTION"];
   for (int i = 0; i < 3; i++) {
     m_OpticalDistCoef[i] = j["STA_OPTICAL_DIST_COEF"][i];
     m_ITransS[i] = j["STA_I_TRANS_S"][i];
     m_ITransL[i] = j["STA_I_TRANS_L"][i];
   }
   m_DetectorSampleOrigin = j["STA_DETECTOR_SAMPLE_ORIGIN"];
   m_DetectorLineOrigin = j["STA_DETECTOR_LINE_ORIGIN"];
   m_DetectorLineOffset = j["STA_DETECTOR_LINE_OFFSET"];
   for (int i = 0; i < 9; i++) {
       m_MountingMatrix[i] = j["STA_MOUNTING_MATRIX"][i];
   }
   m_SemiMajorAxis = j["STA_SEMI_MAJOR_AXIS"];
   m_SemiMinorAxis = j["STA_SEMI_MINOR_AXIS"];
   m_ReferenceDateAndTime = j["STA_REFERENCE_DATE_AND_TIME"];
   m_PlatformIdentifier = j["STA_PLATFORM_IDENTIFIER"];
   m_SensorIdentifier = j["STA_SENSOR_IDENTIFIER"];
   m_TrajectoryIdentifier = j["STA_TRAJECTORY_IDENTIFIER"];
   m_CollectionIdentifier = j["STA_COLLECTION_IDENTIFIER"];
   m_RefElevation = j["STA_REF_ELEVATION"];
   m_MinElevation = j["STA_MIN_ELEVATION"];
   m_MaxElevation = j["STA_MAX_ELEVATION"];
   m_DtEphem = j["STA_DT_EPHEM"];
   m_T0Ephem = j["STA_T0_EPHEM"];
   m_DtQuat = j["STA_DT_QUAT"];
   m_T0Quat = j["STA_T0_QUAT"];
   m_NumEphem = j["STA_NUM_EPHEM"];
   m_NumQuaternions = j["STA_NUM_QUATERNIONS"];
   m_ReferencePointXyz.x = j["STA_REFERENCE_POINT_XYZ"][0];
   m_ReferencePointXyz.y = j["STA_REFERENCE_POINT_XYZ"][1];
   m_ReferencePointXyz.z = j["STA_REFERENCE_POINT_XYZ"][2];
   m_Gsd = j["STA_GSD"];
   m_FlyingHeight = j["STA_FLYING_HEIGHT"];
   m_HalfSwath = j["STA_HALF_SWATH"];
   m_HalfTime = j["STA_HALF_TIME"];
   m_ImageFlipFlag = j["STA_IMAGE_FLIP_FLAG"];
   // Vector = is overloaded so explicit get with type required.
   m_EphemPts = j["STA_EPHEM_PTS"].get<std::vector<double>>();
   m_EphemRates = j["STA_EPHEM_RATES"].get<std::vector<double>>();
   m_Quaternions = j["STA_QUATERNIONS"].get<std::vector<double>>();
   m_ParameterVals = j["STA_PARAMETER_VALS"].get<std::vector<double>>();
   m_Covariance = j["STA_COVARIANCE"].get<std::vector<double>>();
   for (int i = 0; i < num_params; i++) {
     for (int k = 0; k < NUM_PARAM_TYPES; k++) {
       if (j["STA_PARAMETER_TYPE"][i] == PARAM_STRING_ALL[k]) {
         m_ParameterType[i] = PARAM_CHAR_ALL[k];
         break;
     }
    }
   }
}

std::string UsgsAstroLsStateData::getModelNameFromModelState(
   const std::string& model_state)
{
   // Parse the string to JSON
   auto j = json::parse(model_state);
   // If model name cannot be determined, return a blank string
   std::string model_name;

   if (j.find("STA_SENSOR_MODEL_NAME") != j.end()) {
       model_name = j["STA_SENSOR_MODEL_NAME"];
   } else {
       csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
       std::string aMessage = "No 'STA_SENSOR_MODEL_NAME' key in the model state object.";
       std::string aFunction = "UsgsAstroLsPlugin::getModelNameFromModelState";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   if (model_name != SENSOR_MODEL_NAME){
       csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
       std::string aMessage = "Sensor model not supported.";
       std::string aFunction = "UsgsAstroLsPlugin::getModelNameFromModelState()";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   return model_name;
}


void UsgsAstroLsStateData::reset()
{
   m_ImageIdentifier = "";                    // 1
   m_SensorType    = "USGSAstroLineScanner";  // 2
   m_TotalLines    = 0;                       // 3
   m_TotalSamples  = 0;                       // 4
   m_OffsetLines   = 0.0;                     // 7
   m_OffsetSamples = 0.0;                     // 8
   m_PlatformFlag  = 1;                       // 9
   m_AberrFlag     = 0;                       // 10
   m_AtmRefFlag    = 0;                       // 11
   m_IntTimeLines.clear();
   m_IntTimeStartTimes.clear();
   m_IntTimes.clear();
   m_StartingEphemerisTime = 0.0;             // 13
   m_CenterEphemerisTime = 0.0;               // 14
   m_DetectorSampleSumming = 1.0;             // 15
   m_StartingSample = 1.0;                    // 16
   m_IkCode = -85600;                         // 17
   m_Focal = 350.0;                           // 18
   m_IsisZDirection = 1.0;                    // 19
   m_OpticalDistCoef[0] = 0.0;                // 20
   m_OpticalDistCoef[1] = 0.0;                // 20
   m_OpticalDistCoef[2] = 0.0;                // 20
   m_ITransS[0] = 0.0;                        // 21
   m_ITransS[1] = 0.0;                        // 21
   m_ITransS[2] = 150.0;                      // 21
   m_ITransL[0] = 0.0;                        // 22
   m_ITransL[1] = 150.0;                      // 22
   m_ITransL[2] = 0.0;                        // 22
   m_DetectorSampleOrigin = 2500.0;           // 23
   m_DetectorLineOrigin = 0.0;                // 24
   m_DetectorLineOffset = 0.0;                // 25
   m_MountingMatrix[0] = 1.0;                 // 26
   m_MountingMatrix[1] = 0.0;                 // 26
   m_MountingMatrix[2] = 0.0;                 // 26
   m_MountingMatrix[3] = 0.0;                 // 26
   m_MountingMatrix[4] = 1.0;                 // 26
   m_MountingMatrix[5] = 0.0;                 // 26
   m_MountingMatrix[6] = 0.0;                 // 26
   m_MountingMatrix[7] = 0.0;                 // 26
   m_MountingMatrix[8] = 1.0;                 // 26
   m_SemiMajorAxis = 3400000.0;               // 27
   m_SemiMinorAxis = 3350000.0;               // 28
   m_ReferenceDateAndTime = "";               // 30
   m_PlatformIdentifier = "";                 // 31
   m_SensorIdentifier = "";                   // 32
   m_TrajectoryIdentifier = "";               // 33
   m_CollectionIdentifier = "";               // 33
   m_RefElevation = 30;                       // 34
   m_MinElevation = -8000.0;                  // 34
   m_MaxElevation =  8000.0;                  // 35
   m_DtEphem = 2.0;                           // 36
   m_T0Ephem = -70.0;                         // 37
   m_DtQuat =  0.1;                           // 38
   m_T0Quat = -40.0;                          // 39
   m_NumEphem = 0;                            // 40
   m_NumQuaternions = 0;                      // 41
   m_EphemPts.clear();                        // 42
   m_EphemRates.clear();                      // 43
   m_Quaternions.clear();                     // 44

   m_ParameterVals.assign(NUM_PARAMETERS,0.0);
   m_ParameterType.assign(NUM_PARAMETERS,csm::param::REAL);

   m_ReferencePointXyz.x = 0.0;             // 47
   m_ReferencePointXyz.y = 0.0;             // 47
   m_ReferencePointXyz.z = 0.0;             // 47
   m_Gsd = 1.0;                             // 48
   m_FlyingHeight = 1000.0;                 // 49
   m_HalfSwath = 1000.0;                    // 50
   m_HalfTime = 10.0;                       // 51

   m_Covariance.assign(NUM_PARAMETERS * NUM_PARAMETERS,0.0); // 52
   for (int i = 0; i < NUM_PARAMETERS; i++)
   {
      m_Covariance[i * NUM_PARAMETERS + i] = 1.0;
   }
   m_ImageFlipFlag = 0;                     // 53

}
