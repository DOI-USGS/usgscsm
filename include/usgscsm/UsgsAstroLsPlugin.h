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
// Description:
//  This class creates instances of the Astro Line Scanner sensor model.  The
//  sensor model can be created from either image support data or sensor model
//  state data. This plugin is an implementation of the CSM 3.0.3 CSM plugin
//  class. It supports ISD specified by image file name.  It is expected that
//  a support data file exist in the same directory with the images file name
//  with "_keywords.lis" appended instead of the original extension. An optional
//  file containing ellipsoid data (ellipsoid.ell) can also be placed in this
//  directory. Otherwise, the ellipsoid data is found in the .lis file.
//
//  Revision History:
//  Date        Name        Description
//  ----------- ---------   -----------------------------------------------
//  30-APR-2017 BAE Systems Initial Implementation based on CSM 2.0 code
//  16-OCT-2017 BAE Systems Update for CSM 3.0.3
//-----------------------------------------------------------------------------

#ifndef __USGS_ASTRO_LINE_SCANNER_PLUGIN_H
#define __USGS_ASTRO_LINE_SCANNER_PLUGIN_H

#include <string>
#include <csm/Plugin.h>


class UsgsAstroLsPlugin : public csm::Plugin
{
public:

   //---
   // The public interface is inherited from the csm::Plugin class.
   //---

   //--------------------------------------------------------------------------
   // Plugin Interface
   //--------------------------------------------------------------------------
   virtual std::string getPluginName() const;
   //> This method returns the character std::string that identifies the
   //  plugin.
   //<

   //---
   // CSM Plugin Descriptors
   //---
   virtual std::string getManufacturer() const;
   //> This method returns name of the organization that created the plugin.
   //<

   virtual std::string getReleaseDate() const;
   //> This method returns release date of the plugin.
   //  The returned string follows the ISO 8601 standard.
   //
   //-    Precision   Format           Example
   //-    year        yyyy             "1961"
   //-    month       yyyymm           "196104"
   //-    day         yyyymmdd         "19610420"
   //<

   virtual csm::Version getCsmVersion() const;
   //> This method returns the CSM API version that the plugin conforms to.
   //<

   //---
   // Model Availability
   //---
   virtual size_t getNumModels() const;
   //> This method returns the number of types of models that this plugin
   //  can create.
   //<

   virtual std::string getModelName(size_t modelIndex) const;
   //> This method returns the name of the model for the given modelIndex.
   //  The order does not matter - the index is only used to cycle through
   //  all of the model names.
   //
   //  The model index must be less than getNumModels(), or an exception
   //  will be thrown.
   //<

   virtual std::string getModelFamily(size_t modelIndex) const;
   //> This method returns the model "family" for the model for the given
   //  modelIndex.  This should be the same as what is returned from
   //  csm::Model::getFamily() for the model.
   //
   //  SETs can use this information to exclude models when searching for a
   //  model to create.
   //
   //  The model index must be less than getNumModels(), or an exception
   //  will be thrown.
   //<

   //---
   // Model Descriptors
   //---
   virtual csm::Version getModelVersion(const std::string& modelName) const;
   //> This method returns the version of the code for the model given
   //  by modelIndex.  The Version object can be compared to other Version
   //  objects with its comparison operators.  Not to be confused with the
   //  CSM API version.
   //<

   //---
   // Model Construction
   //---
   virtual bool canModelBeConstructedFromState(
      const std::string& modelName,
      const std::string& modelState,
      csm::WarningList* warnings = NULL) const;
   //> This method returns a boolean indicating whether or not a model of the
   //  given modelName can be constructed from the given modelState.
   //
   //  If a non-NULL warnings argument is received, it will be populated
   //  as applicable.
   //<

   virtual bool canModelBeConstructedFromISD(
      const csm::Isd& imageSupportData,
      const std::string& modelName,
      csm::WarningList* warnings = NULL) const;
   //> This method returns a boolean indicating whether or not a model of the
   //  given modelName can be constructed from the given imageSupportData.
   //
   //  If a non-NULL warnings argument is received, it will be populated
   //  as applicable.
   //<

   virtual csm::Model* constructModelFromState(
      const std::string& modelState,
      csm::WarningList* warnings = NULL) const;
   //> This method allocates and initializes an object of the appropriate
   //  derived Model class with the given modelState and returns a pointer to
   //  the Model base class.  The object is allocated by this method using
   //  new; it is the responsibility of the calling application to delete it.
   //
   //  If a non-NULL warnings argument is received, it will be populated
   //  as applicable.
   //<

   virtual csm::Model* constructModelFromISD(
      const csm::Isd& imageSupportData,
      const std::string& modelName,
      csm::WarningList* warnings = NULL) const;
   //> This method allocates and initializes an object of the appropriate
   //  derived Model class with the given imageSupportData and returns a
   //  pointer to the Model base class.  The object is allocated by this
   //  method using new; it is the responsibility of the calling
   //  application to delete it.
   //
   //  If a non-NULL warnings argument is received, it will be populated
   //  as applicable.
   //<

   virtual std::string getModelNameFromModelState(
      const std::string& modelState,
      csm::WarningList* warnings = NULL) const;
   // This method returns the model name for which the given modelState
   // is applicable.
   //
   //  If a non-NULL warnings argument is received, it will be populated
   //  as applicable.
   //<

   //---
   // Image Support Data Conversions
   //---
   virtual bool canISDBeConvertedToModelState(
      const csm::Isd& imageSupportData,
      const std::string& modelName,
      csm::WarningList* warnings = NULL) const;
   //> This method returns a boolean indicating whether or not a model state
   //  of the given modelName can be constructed from the given
   //  imageSupportData.
   //
   //  If a non-NULL warnings argument is received, it will be populated
   //  as applicable.
   //<

   virtual std::string convertISDToModelState(
      const csm::Isd&    imageSupportData,
      const std::string& modelName,
      csm::WarningList*  warnings = NULL) const;
   //> This method returns a model state string for the given modelName,
   //  constructed from the given imageSupportData.
   //
   //  If a non-NULL warnings argument is received, it will be populated
   //  as applicable.
   //<

//private:

   //--------------------------------------------------------------------------
   // Constructors/Destructor
   //--------------------------------------------------------------------------

   UsgsAstroLsPlugin();
   ~UsgsAstroLsPlugin();

private:
   //--------------------------------------------------------------------------
   // Data Members
   //--------------------------------------------------------------------------

   // This is needed to allow the plugin to be registered.
   static const UsgsAstroLsPlugin  _theRegisteringObject;
   static const std::string mISD_KEYWORDS[];
   static const std::string mSTATE_KEYWORDS[];
}; // UsgsAstroLsPlugin

#endif // __USGS_ASTRO_LINE_SCANNER_PLUGIN_H
