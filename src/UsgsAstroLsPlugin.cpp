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
//  Date        Name        Description
//  ----------- ---------   -----------------------------------------------
//  30-APR-2017 BAE Systems Initial Implementation based on CSM 2.0 code
//  16-OCT-2017 BAE Systems Update for CSM 3.0.3
//
//-----------------------------------------------------------------------------

#define USGSASTROLINESCANNER_LIBRARY

#include "UsgsAstroLsPlugin.h"
#include "UsgsAstroLsISD.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroLsStateData.h"

#ifdef _WIN32
# define DIR_DELIMITER_STR "\\"
#else
# define DIR_DELIMITER_STR  "/"
#endif

//#ifdef WIN32
//#pragma comment(linker, SENCSM_MANIFESTDEPENDENCY_GXP_CSMAPI)
//#endif // WIN32

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <json.hpp>


using json = nlohmann::json;

// Declaration of static variables
static const std::string  PLUGIN_NAME       = "USGS_ASTRO_LINE_SCANNER_PLUGIN";
static const std::string  MANUFACTURER_NAME = "BAE_SYSTEMS_GXP";
static const std::string  RELEASE_DATE      = "20171230";
static const int          N_SENSOR_MODELS   = 1;
const std::string  UsgsAstroLsPlugin::mISD_KEYWORDS[] =
{
   "SENSOR_TYPE",
   "TOTAL_LINES",
   "TOTAL_SAMPLES",
   "PLATFORM",
   "ABERR",
   "ATMREF",
   "INT_TIME",
   "STARTING_EPHEMERIS_TIME",
   "CENTER_EPHEMERIS_TIME",
   "DETECTOR_SAMPLE_SUMMING",
   "STARTING_SAMPLE",
   "IKCODE",
   "FOCAL",
   "ISIS_Z_DIRECTION",
   "OPTICAL_DIST_COEF",
   "ITRANSS",
   "ITRANSL",
   "DETECTOR_SAMPLE_ORIGIN",
   "DETECTOR_LINE_ORIGIN",
   "DETECTOR_LINE_OFFSET",
   "MOUNTING_ANGLES",
   "DT_EPHEM",
   "T0_EPHEM",
   "DT_QUAT",
   "T0_QUAT",
   "NUMBER_OF_EPHEM",
   "NUMBER_OF_QUATERNIONS",
   "EPHEM_PTS",
   "EPHEM_RATES",
   "QUATERNIONS",
   "TRI_PARAMETERS",
   "SEMI_MAJOR_AXIS",
   "ECCENTRICITY",
   // Everything below here is optional
   "REFERENCE_HEIGHT",
   "MIN_VALID_HT",
   "MAX_VALID_HT",
   "IMAGE_ID",
   "SENSOR_ID",
   "PLATFORM_ID",
   "TRAJ_ID",
   "COLL_ID",
   "REF_DATE_TIME"
};

const std::string  UsgsAstroLsPlugin::mSTATE_KEYWORDS[] =
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

//***************************************************************************
// Static instance of itself
//***************************************************************************
const UsgsAstroLsPlugin UsgsAstroLsPlugin::_theRegisteringObject;


//***************************************************************************
// UsgsAstroLsPlugin::UsgsAstroLsPlugin
//***************************************************************************
UsgsAstroLsPlugin::UsgsAstroLsPlugin()
{
}

//*****************************************************************************
// UsgsAstroLsPlugin::~UsgsAstroLsPlugin
//*****************************************************************************
UsgsAstroLsPlugin::~UsgsAstroLsPlugin()
{
}

//***************************************************************************
// UsgsAstroLsPlugin::getPluginName
//***************************************************************************
std::string UsgsAstroLsPlugin::getPluginName() const
{
   return PLUGIN_NAME;
}

//***************************************************************************
// UsgsAstroLsPlugin::getManufacturer
//***************************************************************************
std::string UsgsAstroLsPlugin::getManufacturer() const
{
   return MANUFACTURER_NAME;
}

//***************************************************************************
// UsgsAstroLsPlugin::getReleaseDate
//***************************************************************************
std::string UsgsAstroLsPlugin::getReleaseDate() const
{
   return RELEASE_DATE;
}

//***************************************************************************
// UsgsAstroLsPlugin::getCSMVersion
//***************************************************************************
csm::Version UsgsAstroLsPlugin::getCsmVersion() const
{
   return CURRENT_CSM_VERSION;
}

//***************************************************************************
// UsgsAstroLsPlugin::getNSensorModels
//***************************************************************************
size_t UsgsAstroLsPlugin::getNumModels() const
{
   return N_SENSOR_MODELS;
}

//***************************************************************************
// UsgsAstroLsPlugin::getSensorModelName
//***************************************************************************
std::string UsgsAstroLsPlugin::getModelName(size_t modelIndex) const
{
   // Always return the only sensor model name defined regardless of index
   return UsgsAstroLsStateData::SENSOR_MODEL_NAME;
}


//***************************************************************************
// UsgsAstroLsPlugin::getModelFamily
//***************************************************************************
std::string UsgsAstroLsPlugin::getModelFamily(size_t modelIndex) const
{
   return CSM_RASTER_FAMILY;
}

//***************************************************************************
// UsgsAstroLsPlugin::getSensorModelVersion
//***************************************************************************
csm::Version UsgsAstroLsPlugin::getModelVersion(
   const std::string &model_name) const
{
   if (model_name != UsgsAstroLsStateData::SENSOR_MODEL_NAME)
   {
      csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
      std::string aMessage = " Sensor model not supported: ";
      std::string aFunction = "UsgsAstroLsPlugin::getSensorModelVersion()";
      csm::Error csmErr(aErrorType, aMessage, aFunction);
      throw (csmErr);
   }

   return csm::Version(1, 0, 0);
}


//***************************************************************************
// UsgsAstroLsPlugin::canSensorModelBeConstructedFromState
//***************************************************************************
bool UsgsAstroLsPlugin::canModelBeConstructedFromState(
   const std::string& model_name,
   const std::string& model_state,
   csm::WarningList* warnings) const
{
   // Initialize constructible flag
   bool constructible = true;

   // Get sensor model from sensor model state
   std::string name_from_state;

   try
   {
      name_from_state = getModelNameFromModelState(model_state);
   }
   catch (...)
   {
   }

   // Check if plugin supports sensor model name
   if (model_name != name_from_state)
   {
      constructible = false;
   }

   // Check that all of the necessary state keys are in place
   auto j = json::parse(model_state);
   for (auto &key : mSTATE_KEYWORDS){
       if (j.find(key) == j.end()){
           constructible = false;
       }
   }

   return constructible;
}

//***************************************************************************
// UsgsAstroLsPlugin::canSensorModelBeConstructedFromISD
//***************************************************************************
bool UsgsAstroLsPlugin::canModelBeConstructedFromISD(
   const csm::Isd &image_support_data,
   const std::string& model_name,
   csm::WarningList* warnings) const
{
   // Check file validity
   bool constructible = true;

   std::string value;
   for(auto &key : mISD_KEYWORDS){
       value = image_support_data.param(key);
       if (value.empty()){
           std::cout<<key<<"\n"<<std::endl;
           constructible = false;
       }
   }

   return constructible;
}


//***************************************************************************
// UsgsAstroLsPlugin::constructSensorModelFromState
//***************************************************************************
csm::Model* UsgsAstroLsPlugin::constructModelFromState(
   const std::string& model_state,
   csm::WarningList* warnings ) const
{

    // Get the sensor model name from the sensor model state
    std::string model_name_from_state = getModelNameFromModelState(model_state);

    if (!canModelBeConstructedFromState(model_name_from_state, model_state)){
        csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
        std::string aMessage = "Model state is not valid.";
        std::string aFunction = "UsgsAstroLsPlugin::constructModelFromState()";
        throw csm::Error(aErrorType, aMessage, aFunction);
    }
   UsgsAstroLsStateData data;
   data.setState( model_state );

   UsgsAstroLsSensorModel* sensor_model = new UsgsAstroLsSensorModel();
   // I do not think that exposing a set method is necessarily CSM compliant?
   sensor_model->set( data );

   return sensor_model;
}

//***************************************************************************
// UsgsAstroLsPlugin::constructSensorModelFromISD
//***************************************************************************
csm::Model* UsgsAstroLsPlugin::constructModelFromISD(
   const csm::Isd&    image_support_data,
   const std::string& model_name,
   csm::WarningList*  warnings) const
{
   std::string stateStr = convertISDToModelState(image_support_data, model_name, warnings);
   UsgsAstroLsStateData state(stateStr);

   UsgsAstroLsSensorModel* sm = new UsgsAstroLsSensorModel();
   // I do not see things like flying height getting set properly, are things overflowing?
   sm->set(state);

   csm::Model* sensor_model = sm;
   return sensor_model;
}

//***************************************************************************
// UsgsAstroLsPlugin::getSensorModelNameFromSensorModelState
//***************************************************************************
std::string UsgsAstroLsPlugin::getModelNameFromModelState(
   const std::string& model_state,
   csm::WarningList* warnings) const
{
    return UsgsAstroLsStateData::getModelNameFromModelState( model_state );
}

//***************************************************************************
// UsgsAstroLsPlugin::canISDBeConvertedToSensorModelState
//***************************************************************************
bool UsgsAstroLsPlugin::canISDBeConvertedToModelState(
   const csm::Isd&    image_support_data,
   const std::string& model_name,
   csm::WarningList*  warnings) const
{
   // Check if ISD is supported
   return canModelBeConstructedFromISD(image_support_data, model_name);
}

//***************************************************************************
// UsgsAstroLsPlugin::convertISDToSensorModelState
//***************************************************************************
std::string UsgsAstroLsPlugin::convertISDToModelState(
   const csm::Isd&    image_support_data,
   const std::string& model_name,
   csm::WarningList*  warnings) const
{

   if (!canModelBeConstructedFromISD(image_support_data, model_name)){
       throw csm::Error(csm::Error::ISD_NOT_SUPPORTED,
                        "Sensor model support data provided is not supported by this plugin",
                        "GenericLsPlugin::constructModelFromISD");
   }

   // Instantiate UsgsAstroLineScanner sensor model
   UsgsAstroLsStateData state;
   int num_params = state.NUM_PARAMETERS;
   //int num_params_square = num_params * num_params;

   // Translate the ISD to state data
  // if( .m_image_id.size() > 1 && image_support_data.m_image_id != "UNKNOWN")
   //{
    //  state.m_ImageIdentifier = image_support_data.m_image_id;
   //}

   //else
   //{
    //  state.m_ImageIdentifier = img_rel_name;
   //}

   state.m_ImageIdentifier = image_support_data.param("IMAGE_ID");
   state.m_SensorType = image_support_data.param("SENSOR_TYPE");
   state.m_TotalLines = atoi(image_support_data.param("TOTAL_LINES").c_str());
   state.m_TotalSamples = atoi(image_support_data.param("TOTAL_SAMPLES").c_str());
   state.m_OffsetLines = 0.0;
   state.m_OffsetSamples = 0.0;
   state.m_PlatformFlag = atoi(image_support_data.param("PLATFORM").c_str());
   state.m_AberrFlag = atoi(image_support_data.param("ABERR").c_str());
   state.m_AtmRefFlag = atoi(image_support_data.param("ATMREF").c_str());
   state.m_StartingEphemerisTime = atof(image_support_data.param("STARTING_EPHEMERIS_TIME").c_str());
   state.m_CenterEphemerisTime = atof(image_support_data.param("CENTER_EPHEMERIS_TIME").c_str());
   if (image_support_data.param("NUMBER_OF_INT_TIMES").empty()) {
     state.m_IntTimeLines = {0.5};
     state.m_IntTimeStartTimes = {state.m_StartingEphemerisTime - state.m_CenterEphemerisTime};
     state.m_IntTimes = {atof(image_support_data.param("INT_TIME").c_str())};
   }
   else {
     int numIntTimes = atoi(image_support_data.param("NUMBER_OF_INT_TIMES").c_str());
     for (int i = 0; i < numIntTimes; i++) {
       state.m_IntTimeLines.push_back(atof(image_support_data.param("INT_TIME", i*3).c_str()));
       state.m_IntTimeStartTimes.push_back(atof(image_support_data.param("INT_TIME", i*3 + 1).c_str()));
       state.m_IntTimes.push_back(atof(image_support_data.param("INT_TIME", i*3 + 2).c_str()));
     }
   }
   state.m_CenterEphemerisTime = atof(image_support_data.param("CENTER_EPHEMERIS_TIME").c_str());
   state.m_DetectorSampleSumming = atoi(image_support_data.param("DETECTOR_SAMPLE_SUMMING").c_str());
   state.m_StartingSample = atoi(image_support_data.param("STARTING_SAMPLE").c_str());
   state.m_IkCode = atoi(image_support_data.param("IKCODE").c_str());
   state.m_Focal = atof(image_support_data.param("FOCAL").c_str());
   state.m_IsisZDirection = atof(image_support_data.param("ISIS_Z_DIRECTION").c_str());

   for (int i = 0; i < 3; i++)
   {
      state.m_OpticalDistCoef[i] = atof(image_support_data.param("OPTICAL_DIST_COEF", i).c_str());
      state.m_ITransS[i] = atof(image_support_data.param("ITRANSS", i).c_str());
      state.m_ITransL[i] = atof(image_support_data.param("ITRANSL", i).c_str());
   }

   state.m_DetectorSampleOrigin = atof(image_support_data.param("DETECTOR_SAMPLE_ORIGIN").c_str());
   state.m_DetectorLineOrigin = atof(image_support_data.param("DETECTOR_LINE_ORIGIN").c_str());
   state.m_DetectorLineOffset = atof(image_support_data.param("DETECTOR_LINE_OFFSET").c_str());

   double cos_a = cos(atof(image_support_data.param("MOUNTING_ANGLES", 0).c_str()));
   double sin_a = sin(atof(image_support_data.param("MOUNTING_ANGLES", 0).c_str()));
   double cos_b = cos(atof(image_support_data.param("MOUNTING_ANGLES", 1).c_str()));
   double sin_b = sin(atof(image_support_data.param("MOUNTING_ANGLES", 1).c_str()));
   double cos_c = cos(atof(image_support_data.param("MOUNTING_ANGLES", 2).c_str()));
   double sin_c = sin(atof(image_support_data.param("MOUNTING_ANGLES", 2).c_str()));
   state.m_MountingMatrix[0] = cos_b * cos_c;
   state.m_MountingMatrix[1] = -cos_a * sin_c + sin_a * sin_b * cos_c;
   state.m_MountingMatrix[2] = sin_a * sin_c + cos_a * sin_b * cos_c;
   state.m_MountingMatrix[3] = cos_b * sin_c;
   state.m_MountingMatrix[4] = cos_a * cos_c + sin_a * sin_b * sin_c;
   state.m_MountingMatrix[5] = -sin_a * cos_c + cos_a * sin_b * sin_c;
   state.m_MountingMatrix[6] = -sin_b;
   state.m_MountingMatrix[7] = sin_a * cos_b;
   state.m_MountingMatrix[8] = cos_a * cos_b;

   state.m_DtEphem = atof(image_support_data.param("DT_EPHEM").c_str());
   state.m_T0Ephem = atof(image_support_data.param("T0_EPHEM").c_str());
   state.m_DtQuat = atof(image_support_data.param("DT_QUAT").c_str());
   state.m_T0Quat = atof(image_support_data.param("T0_QUAT").c_str());
   state.m_NumEphem = atoi(image_support_data.param("NUMBER_OF_EPHEM").c_str());
   state.m_NumQuaternions = atoi(image_support_data.param("NUMBER_OF_QUATERNIONS").c_str());

   int numEphem = atoi(image_support_data.param("NUMBER_OF_EPHEM").c_str());
   for (int i=0;i < numEphem * 3; i++){
       state.m_EphemPts.push_back(atof(image_support_data.param("EPHEM_PTS", i).c_str()));
       state.m_EphemRates.push_back(atof(image_support_data.param("EPHEM_RATES", i).c_str()));
   }

   int numQuat = atoi(image_support_data.param("NUMBER_OF_QUATERNIONS").c_str());
   for (int i=0; i < numQuat * 4; i++){
       state.m_Quaternions.push_back(atof(image_support_data.param("QUATERNIONS", i).c_str()));
   }

   //state.m_EphemPts = image_support_data.m_ephem_pts;
   //state.m_EphemRates = image_support_data.m_ephem_rates;
   //state.m_Quaternions = image_support_data.m_quaternions;

   for (int i=0; i < 18;i++){
       state.m_ParameterVals.push_back(atof(image_support_data.param("TRI_PARAMETERS", i).c_str()));
   }
   //state.m_ParameterVals = image_support_data.m_tri_parameters;
   double deltaF = state.m_ParameterVals[num_params - 1] - state.m_Focal;
   if (fabs(deltaF) < 0.4 * state.m_Focal)
      state.m_ParameterVals[num_params - 1] = deltaF;

   // Set the ellipsoid
   state.m_SemiMajorAxis = atof(image_support_data.param("SEMI_MAJOR_AXIS").c_str());
   state.m_SemiMinorAxis =
      state.m_SemiMajorAxis * sqrt(1.0 - atof(image_support_data.param("ECCENTRICITY").c_str()) * atof(image_support_data.param("ECCENTRICITY").c_str()));

   // Now finish setting the state data from the ISD read in

   // set identifiers
   state.m_ReferenceDateAndTime = image_support_data.param("REF_DATE_TIME");
   state.m_PlatformIdentifier   = image_support_data.param("PLATFORM_ID");
   state.m_SensorIdentifier     = image_support_data.param("SENSOR_ID");
   state.m_TrajectoryIdentifier = image_support_data.param("TRAJ_ID");
   state.m_CollectionIdentifier = image_support_data.param("COLL_ID");

   // Ground elevations
   state.m_RefElevation = atof(image_support_data.param("REFERNCE_HEIGHT").c_str());
   state.m_MinElevation = atof(image_support_data.param("MIN_VALID_HT").c_str());
   state.m_MaxElevation = atof(image_support_data.param("MAX_VALID_HT").c_str());

   // Zero parameter values
   for (int i = 0; i < num_params; i++)
   {
      state.m_ParameterVals[i] = 0.0;
      state.m_ParameterType[i] = csm::param::REAL;
   }

   // The state data will still be updated when a sensor model is created since
   // some state data is notin the ISD and requires a SM to compute them.
   return state.toJson();
}
