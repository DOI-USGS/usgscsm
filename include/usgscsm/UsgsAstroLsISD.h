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
//    FILENAME:          UsgsAstroLsISD.h
//
//    DESCRIPTION:
//
//     Image Support Data (ISD) to build the Astro Line Scanner sensor model.
//     The ISD format is defined by the imagery provider. A few common formats
//     include name/value pairs, XML, NITF TREs, and GeoTiff tags.
//
//     This ISD is formatted as a list of name/value pairs. The order of the
//     list generally does not matter, except that the length of a vector needs
//     to be read in before the vector itself. Extra data in the support data
//     file will be ignored.
//
//     There is data describing the ellipsoid that can optionally be in a
//     seperate file.  This is to support legacy data, but is not encouraged.
//
//     For the Astro LS sensor model there is a close relationship between the
//     ISD and the State data. This is not always the case. So, it is best to
//     seperate the ISD parsing (done here), the translation of ISD to state
//     data (done in the plugin), and the instantiation of the sensor model
//     from state data (done in the sensor model).
//
//    SOFTWARE HISTORY:
//
//    Date          Author       Comment
//    -----------   ----------   -------
//    16-Oct-2017   BAE Systems  Initial Release
//
//#############################################################################
#ifndef __USGS_ASTRO_LINE_SCANNER_ISD_H
#define __USGS_ASTRO_LINE_SCANNER_ISD_H

#include <vector>
#include <string>

#include <csm/csm.h>
#include <csm/Isd.h>
#include <csm/SettableEllipsoid.h>

class UsgsAstroLsISD
{
   public:


   UsgsAstroLsISD()
   {
      reset();
   }

   UsgsAstroLsISD(
      const std::string&   lis_file_name,
      const std::string&   ell_file_name)
   {
      reset();
      setISD(lis_file_name, ell_file_name);
   }

   virtual ~UsgsAstroLsISD() {}

   // Formats the support data as a string
   // This is mainly used to check that the
   // support data is read in correctly.
   std::string toString() const;

   // Initializes the class from ISD as formatted by the image provider.
   //
   // Note that if the ellipsoid data is found in the main ISD file,
   // the ellipsoid data file is not needed.
   void setISD(
      const std::string&   lis_file_name,
      const std::string&   ell_file_name);


   //--------------------------------------------------------------------------
   // Helper Routines
   //--------------------------------------------------------------------------
   // Initial check that support data file exists and looks like Astro LS ISD.
   static bool checkFileValidity(
      const csm::Isd&     image_support_data,
      std::string&        lis_file_name,
      std::string&        ell_file_name,
      std::string&        img_rel_name);


   // ISD elements;
   // The support data is defined by USGS. See the documentation at
   // http://htmlpreview.github.io/?https://github.com/USGS-Astrogeology/socet_set/blob/master/SS4HiRISE/Tutorials/ISD_keyword_examples/AstroLineScanner_ISD_Keywords.ls.htm
   // Even for fields that do not change, it is best to include them in
   // the support data rather than hard code values in the sensor model.
   // Since this is not always done, some fields are optional.
   std::string         m_sensor_type;
   int                 m_total_lines;
   int                 m_total_samples;
   int                 m_platform;
   int                 m_aberr;
   int                 m_atmref;
   double              m_int_time;
   double              m_starting_ephemeris_time;
   double              m_center_ephemeris_time;
   double              m_detector_sample_summing;
   double              m_starting_sample;
   int                 m_ikcode;
   double              m_focal;
   double              m_isis_z_direction;
   double              m_optical_dist_coef[3];
   double              m_itranss[3];
   double              m_itransl[3];
   double              m_detector_sample_origin;
   double              m_detector_line_origin;
   double              m_detector_line_offset;
   double              m_mounting_angles[3];
   double              m_dt_ephem;
   double              m_t0_ephem;
   double              m_dt_quat;
   double              m_t0_quat;
   int                 m_number_of_ephem;
   int                 m_number_of_quaternions;
   std::vector<double> m_ephem_pts;
   std::vector<double> m_ephem_rates;
   std::vector<double> m_quaternions;
   std::vector<double> m_tri_parameters;
   double              m_semi_major_axis;
   double              m_eccentricity;

   // Optional fields
   double              m_ref_height;
   double              m_min_valid_ht;
   double              m_max_valid_ht;
   std::string         m_image_id;
   std::string         m_sensor_id;
   std::string         m_platform_id;
   std::string         m_traj_id;
   std::string         m_coll_id;
   std::string         m_ref_date_time;

   static const std::string mISD_KEYWORDS[];

   protected:
   // Set to default values
   void reset();
};

#endif
