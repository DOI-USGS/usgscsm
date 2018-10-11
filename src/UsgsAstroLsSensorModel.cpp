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

#include <algorithm>
#include <iostream>
#include <sstream>
#include <math.h>

//*****************************************************************************
// UsgsAstroLsSensorModel Constructor
//*****************************************************************************
UsgsAstroLsSensorModel::UsgsAstroLsSensorModel()
{
   _no_adjustment.assign(UsgsAstroLsStateData::NUM_PARAMETERS, 0.0);
}

//*****************************************************************************
// UsgsAstroLsSensorModel Destructor
//*****************************************************************************
UsgsAstroLsSensorModel::~UsgsAstroLsSensorModel()
{
}

//*****************************************************************************
// UsgsAstroLsSensorModel set
//*****************************************************************************
void UsgsAstroLsSensorModel::set(const UsgsAstroLsStateData &state_data)
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

   _no_adjustment.assign(UsgsAstroLsStateData::NUM_PARAMETERS, 0.0);
   _data = state_data;

   // If needed set state data elements that need a sensor model to compute
   // Update if still using default settings
   if (_data.m_Gsd == 1.0  && _data.m_FlyingHeight == 1000.0)
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

//*****************************************************************************
// UsgsAstroLsSensorModel updateState
//*****************************************************************************
void UsgsAstroLsSensorModel::updateState()
{
   // If sensor model is being created for the first time
   // This routine will set some parameters not found in the ISD.

   // Reference point (image center)
   double lineCtr = _data.m_TotalLines / 2.0;
   double sampCtr = _data.m_TotalSamples / 2.0;
   csm::ImageCoord ip(lineCtr, sampCtr);
   double refHeight = _data.m_RefElevation;
   _data.m_ReferencePointXyz = imageToGround(ip, refHeight);
   // Compute ground sample distance
   ip.line += 1;
   ip.samp += 1;
   csm::EcefCoord delta = imageToGround(ip, refHeight);
   double dx = delta.x - _data.m_ReferencePointXyz.x;
   double dy = delta.y - _data.m_ReferencePointXyz.y;
   double dz = delta.z - _data.m_ReferencePointXyz.z;
   _data.m_Gsd = sqrt((dx*dx + dy*dy + dz*dz) / 2.0);

   // Compute flying height
   csm::EcefCoord sensorPos = getSensorPosition(0.0);
   dx = sensorPos.x - _data.m_ReferencePointXyz.x;
   dy = sensorPos.y - _data.m_ReferencePointXyz.y;
   dz = sensorPos.z - _data.m_ReferencePointXyz.z;
   _data.m_FlyingHeight = sqrt(dx*dx + dy*dy + dz*dz);

   // Compute half swath
   _data.m_HalfSwath = _data.m_Gsd * _data.m_TotalSamples / 2.0;

   // Compute half time duration
   double fullImageTime = _data.m_IntTimeStartTimes.back() - _data.m_IntTimeStartTimes.front()
                          + _data.m_IntTimes.back() * (_data.m_TotalLines - _data.m_IntTimeLines.back());
   _data.m_HalfTime = fullImageTime / 2.0;

   // Parameter covariance, hardcoded accuracy values
   int num_params = _data.NUM_PARAMETERS;
   int num_params_square = num_params * num_params;
   double variance = _data.m_Gsd * _data.m_Gsd;
   for (int i = 0; i < num_params_square; i++)
   {
      _data.m_Covariance[i] = 0.0;
   }
   for (int i = 0; i < num_params; i++)
   {
      _data.m_Covariance[i * num_params + i] = variance;
   }

   // Set flag for flipping image along horizontal axis
   int index = int((_data.m_NumEphem - 1) / 2.0) * 3;
   double xs, ys, zs;    // mid sensor position
   xs = _data.m_EphemPts[index];
   ys = _data.m_EphemPts[index + 1];
   zs = _data.m_EphemPts[index + 2];
   double xv, yv, zv;    // mid sensor velocity
   xv = _data.m_EphemRates[index];
   yv = _data.m_EphemRates[index + 1];
   zv = _data.m_EphemRates[index + 2];
   double xm, ym, zm;    // mid line position (mid sample)
   xm = _data.m_ReferencePointXyz.x;
   ym = _data.m_ReferencePointXyz.y;
   zm = _data.m_ReferencePointXyz.z;
   // mid line position (first sample)
   csm::EcefCoord posf = imageToGround(csm::ImageCoord(lineCtr, 0.0), refHeight);
   // mid line position (last sample)
   csm::EcefCoord posl = imageToGround(csm::ImageCoord(lineCtr, _data.m_TotalSamples - 1), refHeight);
   double xd, yd, zd;    // unit vector normal to image footprint
   xd = posl.x - posf.x;
   yd = posl.y - posf.y;
   zd = posl.z - posf.z;
   double xn, yn, zn;
   xn = yv*zd - zv*yd;
   yn = zv*xd - xv*zd;
   zn = xv*yd - yv*xd;
   double nmag = sqrt(xn*xn + yn*yn + zn*zn);
   xn /= nmag;
   yn /= nmag;
   zn /= nmag;
   double xo, yo, zo;   // unit vector ground-to_satellite
   xo = xs - xm;
   yo = ys - ym;
   zo = zs - zm;
   double omag = sqrt(xo*xo + yo*yo + zo*zo);
   xo /= omag;
   yo /= omag;
   zo /= omag;
   double triple_product = xn*xo + yn*yo + zn*zo;
   _data.m_ImageFlipFlag = 0;
   if (triple_product > 0.0)
      _data.m_ImageFlipFlag = 1;
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
   // This method uses an iterative bisection method to search for the image
   // line.
   //
   // For a given search window, this routine involves projecting the
   // ground point onto the focal plane based on the instrument orientation
   // at the start and end of the search window. Then, it computes the focal
   // plane intersection at a mid-point of the search window. Then, it reduces
   // the search window based on the signs of the intersected line offsets from
   // the center of the ccd. For example, if the line offset is -145 at the
   // start of the window, 10 at the mid point, and 35 at the end of the search
   // window, the window will be reduced to the start of the old window to the
   // middle of the old window.
   //
   // In order to achieve faster convergence, the mid point is calculated
   // using the False Position Method instead of simple bisection. This method
   // uses the zero of the line between the two ends of the search window for
   // the mid point instead of a simple bisection. In most cases, this will
   // converge significantly faster, but it can be slower than simple bisection
   // in some situations.

   // Start bisection search on the image lines
   double sampCtr = _data.m_TotalSamples / 2.0;
   double firstTime = getImageTime(csm::ImageCoord(0.0, sampCtr));
   double lastTime = getImageTime(csm::ImageCoord(_data.m_TotalLines, sampCtr));
   double firstOffset = computeViewingPixel(firstTime, ground_pt, adj).line - 0.5;
   double lastOffset = computeViewingPixel(lastTime, ground_pt, adj).line - 0.5;

   // Check if both offsets have the same sign.
   // This means there is not guaranteed to be a zero.
   if ((firstOffset > 0) != (lastOffset < 0)) {
        throw csm::Warning(
           csm::Warning::IMAGE_COORD_OUT_OF_BOUNDS,
           "The image coordinate is out of bounds of the image size.",
           "UsgsAstroLsSensorModel::groundToImage");
   }

   // Convert the ground precision to pixel precision so we can
   // check for convergence without re-intersecting
   csm::ImageCoord approxPoint;
   computeLinearApproximation(ground_pt, approxPoint);
   csm::ImageCoord approxNextPoint = approxPoint;
   if (approxNextPoint.line + 1 < _data.m_TotalLines) {
      ++approxNextPoint.line;
   }
   else {
      --approxNextPoint.line;
   }
   csm::EcefCoord approxIntersect = imageToGround(approxPoint, _data.m_RefElevation);
   csm::EcefCoord approxNextIntersect = imageToGround(approxNextPoint, _data.m_RefElevation);
   double lineDX = approxNextIntersect.x - approxIntersect.x;
   double lineDY = approxNextIntersect.y - approxIntersect.y;
   double lineDZ = approxNextIntersect.z - approxIntersect.z;
   double approxLineRes = sqrt(lineDX * lineDX + lineDY * lineDY + lineDZ * lineDZ);
   // Increase the precision by a small amount to ensure the desired precision is met
   double pixelPrec = desired_precision / approxLineRes * 0.9;

   // Start bisection search for zero
   for (int it = 0; it < 30; it++) {
      double nextTime = ((firstTime * lastOffset) - (lastTime * firstOffset))
                      / (lastOffset - firstOffset);
      double nextOffset = computeViewingPixel(nextTime, ground_pt, adj).line - 0.5;
      // We're looking for a zero, so check that either firstLine and middleLine have
      // opposite signs, or middleLine and lastLine have opposite signs.
      if ((firstOffset > 0) == (nextOffset < 0)) {
         lastTime = nextTime;
         lastOffset = nextOffset;
      }
      else {
         firstTime = nextTime;
         firstOffset = nextOffset;
      }
      if (fabs(lastOffset - firstOffset) < pixelPrec) {
         break;
      }
   }

   // Check that the desired precision was met

   double computedTime = ((firstTime * lastOffset) - (lastTime * firstOffset))
                       / (lastOffset - firstOffset);
   csm::ImageCoord calculatedPixel = computeViewingPixel(computedTime, ground_pt, adj);
   // The computed viewing line is the detector line, so we need to convert that to image lines
   auto referenceTimeIt = std::upper_bound(_data.m_IntTimeStartTimes.begin(),
                                           _data.m_IntTimeStartTimes.end(),
                                           computedTime);
   if (referenceTimeIt != _data.m_IntTimeStartTimes.begin()) {
      --referenceTimeIt;
   }
   size_t referenceIndex = std::distance(_data.m_IntTimeStartTimes.begin(), referenceTimeIt);
   calculatedPixel.line += _data.m_IntTimeLines[referenceIndex] - 1
                         + (computedTime - _data.m_IntTimeStartTimes[referenceIndex])
                         / _data.m_IntTimes[referenceIndex];
   csm::EcefCoord calculatedPoint = imageToGround(calculatedPixel, _data.m_RefElevation);
   double dx = ground_pt.x - calculatedPoint.x;
   double dy = ground_pt.y - calculatedPoint.y;
   double dz = ground_pt.z - calculatedPoint.z;
   double len = dx * dx + dy * dy + dz * dz;

   // Check that the pixel is actually in the image
   if ((calculatedPixel.samp < 0) ||
       (calculatedPixel.samp > _data.m_TotalSamples)) {
      throw csm::Warning(
         csm::Warning::IMAGE_COORD_OUT_OF_BOUNDS,
         "The image coordinate is out of bounds of the image size.",
         "UsgsAstroLsSensorModel::groundToImage");
   }

   // If the final correction is greater than 10 meters,
   // the solution is not valid enough to report even with a warning
   if (len > 100.0) {
      throw csm::Error(
         csm::Error::ALGORITHM,
         "Did not converge.",
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
   if (_data.m_AberrFlag == 1)
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
   const double DELTA_GROUND = _data.m_Gsd;
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
   const double DELTA_GROUND = _data.m_Gsd;

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
   double GND_DELTA = _data.m_Gsd;
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
   double GND_DELTA = _data.m_Gsd;
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

   const double DELTA = _data.m_Gsd;
   std::vector<double> adj(UsgsAstroLsStateData::NUM_PARAMETERS, 0.0);
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
   int index = UsgsAstroLsStateData::NUM_PARAMETERS * index1 + index2;
   return _data.m_Covariance[index];
}

//***************************************************************************
// UsgsAstroLsSensorModel::setParameterCovariance
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterCovariance(
   int index1,
   int index2,
   double covariance)
{
   int index = UsgsAstroLsStateData::NUM_PARAMETERS * index1 + index2;
   _data.m_Covariance[index] = covariance;
}

//---------------------------------------------------------------------------
// Time and Trajectory
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getTrajectoryIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getTrajectoryIdentifier() const
{
   return _data.m_TrajectoryIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getReferenceDateAndTime
//***************************************************************************
std::string UsgsAstroLsSensorModel::getReferenceDateAndTime() const
{
   if (_data.m_ReferenceDateAndTime == "UNKNOWN")
   {
      throw csm::Error(
         csm::Error::UNSUPPORTED_FUNCTION,
         "Unsupported function",
         "UsgsAstroLsSensorModel::getReferenceDateAndTime");
   }
   return _data.m_ReferenceDateAndTime;
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

   double lineCSMFull = line1 + _data.m_OffsetLines;
   double lineUSGSFull = lineCSMFull + 0.5;

   // These calculation assumes that the values in the integration time
   // vectors are in terms of ISIS' pixels
   auto referenceLineIt = std::upper_bound(_data.m_IntTimeLines.begin(),
                                           _data.m_IntTimeLines.end(),
                                           lineUSGSFull);
   if (referenceLineIt != _data.m_IntTimeLines.begin()) {
      --referenceLineIt;
   }
   size_t referenceIndex = std::distance(_data.m_IntTimeLines.begin(), referenceLineIt);

   double time = _data.m_IntTimeStartTimes[referenceIndex]
      + _data.m_IntTimes[referenceIndex] * (lineUSGSFull - _data.m_IntTimeLines[referenceIndex]);

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
   _data.m_ParameterVals[index] = value;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterValue
//***************************************************************************
double UsgsAstroLsSensorModel::getParameterValue(int index) const
{
   return _data.m_ParameterVals[index];
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterName
//***************************************************************************
std::string UsgsAstroLsSensorModel::getParameterName(int index) const
{
   return _data.PARAMETER_NAME[index];
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
   return UsgsAstroLsStateData::NUM_PARAMETERS;
}


//***************************************************************************
// UsgsAstroLsSensorModel::getParameterType
//***************************************************************************
csm::param::Type UsgsAstroLsSensorModel::getParameterType(int index) const
{
   return _data.m_ParameterType[index];
}


//***************************************************************************
// UsgsAstroLsSensorModel::setParameterType
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterType(
   int index, csm::param::Type pType)
{
   _data.m_ParameterType[index] = pType;
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
   return _data.m_ImageIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::setImageIdentifier
//***************************************************************************
void UsgsAstroLsSensorModel::setImageIdentifier(
   const std::string& imageId,
   csm::WarningList* warnings)
{
   // Image id should include the suffix without the path name
   _data.m_ImageIdentifier = imageId;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getSensorIdentifier() const
{
   return _data.m_SensorIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getPlatformIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getPlatformIdentifier() const
{
   return _data.m_PlatformIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::setReferencePoint
//***************************************************************************
void UsgsAstroLsSensorModel::setReferencePoint(const csm::EcefCoord& ground_pt)
{
   _data.m_ReferencePointXyz = ground_pt;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getReferencePoint
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::getReferencePoint() const
{
   // Return ground point at image center
   return _data.m_ReferencePointXyz;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorModelName
//***************************************************************************
std::string UsgsAstroLsSensorModel::getModelName() const
{
   return UsgsAstroLsStateData::SENSOR_MODEL_NAME;
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
   return csm::ImageVector(_data.m_TotalLines, _data.m_TotalSamples);
}

//---------------------------------------------------------------------------
// Sensor Model State
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorModelState
//***************************************************************************
std::string UsgsAstroLsSensorModel::getModelState() const
{
   return _data.toJson();
}

//---------------------------------------------------------------------------
//  Monoscopic Mensuration
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getValidHeightRange
//***************************************************************************
std::pair<double, double> UsgsAstroLsSensorModel::getValidHeightRange() const
{
   return std::pair<double, double>(_data.m_MinElevation, _data.m_MaxElevation);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getValidImageRange
//***************************************************************************
std::pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroLsSensorModel::getValidImageRange() const
{
   return std::pair<csm::ImageCoord, csm::ImageCoord>(
      csm::ImageCoord(0.0, 0.0),
      csm::ImageCoord(_data.m_TotalLines, _data.m_TotalSamples));
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
   return _data.m_CollectionIdentifier;
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

//***************************************************************************
// UsgsAstroLsSensorModel::replaceModelState
//***************************************************************************
void UsgsAstroLsSensorModel::replaceModelState(const std::string& argState)
{
   set(argState);
}


csm::Ellipsoid UsgsAstroLsSensorModel::getEllipsoid() const
{
   return csm::Ellipsoid(_data.m_SemiMajorAxis, _data.m_SemiMinorAxis);
}

void UsgsAstroLsSensorModel::setEllipsoid(
   const csm::Ellipsoid &ellipsoid)
{
   _data.m_SemiMajorAxis = ellipsoid.getSemiMajorRadius();
   _data.m_SemiMinorAxis = ellipsoid.getSemiMinorRadius();
}



double UsgsAstroLsSensorModel::getValue(
   int   index,
   const std::vector<double> &adjustments) const
{
   return _data.m_ParameterVals[index] + adjustments[index];
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
   double&       xl,         // output line-of-sight x coordinate
   double&       yl,         // output line-of-sight y coordinate
   double&       zl) const  // output line-of-sight z coordinate
{
   //# private_func_description
   //  Computes image ray in ecf coordinate system.

   // Compute adjusted sensor position and velocity

   double time = getImageTime(csm::ImageCoord(line, sample));
   getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz);
   // CSM image image convention: UL pixel center == (0.5, 0.5)
   // USGS image convention: UL pixel center == (1.0, 1.0)

   double sampleCSMFull = sample + _data.m_OffsetSamples;
   double sampleUSGSFull = sampleCSMFull + 0.5;
   double fractionalLine = line - floor(line) - 0.5;

   // Compute distorted image coordinates in mm

   double isisDetSample = (sampleUSGSFull - 1.0)
      * _data.m_DetectorSampleSumming + _data.m_StartingSample;
   double m11 = _data.m_ITransL[1];
   double m12 = _data.m_ITransL[2];
   double m21 = _data.m_ITransS[1];
   double m22 = _data.m_ITransS[2];
   double t1 = fractionalLine + _data.m_DetectorLineOffset
               - _data.m_DetectorLineOrigin - _data.m_ITransL[0];
   double t2 = isisDetSample - _data.m_DetectorSampleOrigin - _data.m_ITransS[0];
   double determinant = m11 * m22 - m12 * m21;
   double p11 = m11 / determinant;
   double p12 = -m12 / determinant;
   double p21 = -m21 / determinant;
   double p22 = m22 / determinant;
   double isisNatFocalPlaneX = p11 * t1 + p12 * t2;
   double isisNatFocalPlaneY = p21 * t1 + p22 * t2;

   // Remove lens distortion
   double isisFocalPlaneX = isisNatFocalPlaneX;
   double isisFocalPlaneY = isisNatFocalPlaneY;
   switch (_data.m_IkCode)
   {
   case -85610:
   case -85600:
      isisFocalPlaneY = isisNatFocalPlaneY / (1.0 + _data.m_OpticalDistCoef[0]
         * isisNatFocalPlaneY * isisNatFocalPlaneY);
      break;

   default:
      if (_data.m_OpticalDistCoef[0] != 0.0 ||
         _data.m_OpticalDistCoef[1] != 0.0 ||
         _data.m_OpticalDistCoef[2] != 0.0)
      {
         double rr = isisNatFocalPlaneX * isisNatFocalPlaneX
            + isisNatFocalPlaneY * isisNatFocalPlaneY;
         if (rr > 1.0E-6)
         {
            double dr = _data.m_OpticalDistCoef[0] + (rr * (_data.m_OpticalDistCoef[1]
               + rr * _data.m_OpticalDistCoef[2]));
            isisFocalPlaneX = isisNatFocalPlaneX * (1.0 - dr);
            isisFocalPlaneY = isisNatFocalPlaneY * (1.0 - dr);
         }
      }
      break;
   }

   // Define imaging ray in image space

   double losIsis[3];
   losIsis[0] = -isisFocalPlaneX * _data.m_IsisZDirection;
   losIsis[1] = -isisFocalPlaneY * _data.m_IsisZDirection;
   losIsis[2] = -_data.m_Focal * (1.0 - getValue(15, adj) / _data.m_HalfSwath);
   double isisMag = sqrt(losIsis[0] * losIsis[0]
      + losIsis[1] * losIsis[1]
      + losIsis[2] * losIsis[2]);
   losIsis[0] /= isisMag;
   losIsis[1] /= isisMag;
   losIsis[2] /= isisMag;

   // Apply boresight correction

   double losApl[3];
   losApl[0] =
      _data.m_MountingMatrix[0] * losIsis[0]
      + _data.m_MountingMatrix[1] * losIsis[1]
      + _data.m_MountingMatrix[2] * losIsis[2];
   losApl[1] =
      _data.m_MountingMatrix[3] * losIsis[0]
      + _data.m_MountingMatrix[4] * losIsis[1]
      + _data.m_MountingMatrix[5] * losIsis[2];
   losApl[2] =
      _data.m_MountingMatrix[6] * losIsis[0]
      + _data.m_MountingMatrix[7] * losIsis[1]
      + _data.m_MountingMatrix[8] * losIsis[2];

   // Apply attitude correction

   double aTime = time - _data.m_T0Quat;
   double euler[3];
   double nTime = aTime / _data.m_HalfTime;
   double nTime2 = nTime * nTime;
   euler[0] =
      (getValue(6, adj) + getValue(9, adj)* nTime + getValue(12, adj)* nTime2) / _data.m_FlyingHeight;
   euler[1] =
      (getValue(7, adj) + getValue(10, adj)* nTime + getValue(13, adj)* nTime2) / _data.m_FlyingHeight;
   euler[2] =
      (getValue(8, adj) + getValue(11, adj)* nTime + getValue(14, adj)* nTime2) / _data.m_HalfSwath;
   double cos_a = cos(euler[0]);
   double sin_a = sin(euler[0]);
   double cos_b = cos(euler[1]);
   double sin_b = sin(euler[1]);
   double cos_c = cos(euler[2]);
   double sin_c = sin(euler[2]);
   double plFromApl[9];
   plFromApl[0] = cos_b * cos_c;
   plFromApl[1] = -cos_a * sin_c + sin_a * sin_b * cos_c;
   plFromApl[2] = sin_a * sin_c + cos_a * sin_b * cos_c;
   plFromApl[3] = cos_b * sin_c;
   plFromApl[4] = cos_a * cos_c + sin_a * sin_b * sin_c;
   plFromApl[5] = -sin_a * cos_c + cos_a * sin_b * sin_c;
   plFromApl[6] = -sin_b;
   plFromApl[7] = sin_a * cos_b;
   plFromApl[8] = cos_a * cos_b;
   double losPl[3];
   losPl[0] = plFromApl[0] * losApl[0] + plFromApl[1] * losApl[1]
      + plFromApl[2] * losApl[2];
   losPl[1] = plFromApl[3] * losApl[0] + plFromApl[4] * losApl[1]
      + plFromApl[5] * losApl[2];
   losPl[2] = plFromApl[6] * losApl[0] + plFromApl[7] * losApl[1]
      + plFromApl[8] * losApl[2];

   // Apply rotation matrix from sensor quaternions

   int nOrder = 8;
   if (_data.m_PlatformFlag == 0)
      nOrder = 4;
   int nOrderQuat = nOrder;
   if (_data.m_NumQuaternions < 6 && nOrder == 8)
      nOrderQuat = 4;
   double q[4];
   lagrangeInterp(
      _data.m_NumQuaternions, &_data.m_Quaternions[0], _data.m_T0Quat, _data.m_DtQuat,
      time, 4, nOrderQuat, q);
   double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
   q[0] /= norm;
   q[1] /= norm;
   q[2] /= norm;
   q[3] /= norm;
   double ecfFromPl[9];
   ecfFromPl[0] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
   ecfFromPl[1] = 2 * (q[0] * q[1] - q[2] * q[3]);
   ecfFromPl[2] = 2 * (q[0] * q[2] + q[1] * q[3]);
   ecfFromPl[3] = 2 * (q[0] * q[1] + q[2] * q[3]);
   ecfFromPl[4] = -q[0] * q[0] + q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
   ecfFromPl[5] = 2 * (q[1] * q[2] - q[0] * q[3]);
   ecfFromPl[6] = 2 * (q[0] * q[2] - q[1] * q[3]);
   ecfFromPl[7] = 2 * (q[1] * q[2] + q[0] * q[3]);
   ecfFromPl[8] = -q[0] * q[0] - q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
   xl = ecfFromPl[0] * losPl[0] + ecfFromPl[1] * losPl[1]
      + ecfFromPl[2] * losPl[2];
   yl = ecfFromPl[3] * losPl[0] + ecfFromPl[4] * losPl[1]
      + ecfFromPl[5] * losPl[2];
   zl = ecfFromPl[6] * losPl[0] + ecfFromPl[7] * losPl[1]
      + ecfFromPl[8] * losPl[2];
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
   double ecc_sqr = 1.0 - _data.m_SemiMinorAxis * _data.m_SemiMinorAxis / _data.m_SemiMajorAxis / _data.m_SemiMajorAxis;
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
         r = _data.m_SemiMajorAxis / sqrt(1.0 + ep2 * tt);
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
         r = _data.m_SemiMajorAxis / sqrt(ep2 + cc);
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
   ap = _data.m_SemiMajorAxis + height;
   bp = _data.m_SemiMinorAxis + height;
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

   if (_data.m_AberrFlag == 1)
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
   if (_data.m_PlatformFlag == 0)
      nOrder = 4;
   double sensPosNom[3];
   lagrangeInterp(_data.m_NumEphem, &_data.m_EphemPts[0], _data.m_T0Ephem, _data.m_DtEphem,
      time, 3, nOrder, sensPosNom);
   double sensVelNom[3];
   lagrangeInterp(_data.m_NumEphem, &_data.m_EphemRates[0], _data.m_T0Ephem, _data.m_DtEphem,
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

   double aTime = time - _data.m_T0Ephem;
   double dvi = getValue(3, adj) / _data.m_HalfTime;
   double dvc = getValue(4, adj) / _data.m_HalfTime;
   double dvr = getValue(5, adj) / _data.m_HalfTime;
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
   const std::vector<double>& adj) const
{
   // Get the exterior orientation
   double xc, yc, zc, vx, vy, vz;
   getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz);

   // Compute the look vector
   double bodyLookX = groundPoint.x - xc;
   double bodyLookY = groundPoint.y - yc;
   double bodyLookZ = groundPoint.z - zc;

   // Rotate the look vector into the camera reference frame
   int nOrder = 8;
   if (_data.m_PlatformFlag == 0)
      nOrder = 4;
   int nOrderQuat = nOrder;
   if (_data.m_NumQuaternions < 6 && nOrder == 8)
      nOrderQuat = 4;
   double q[4];
   lagrangeInterp(
      _data.m_NumQuaternions, &_data.m_Quaternions[0], _data.m_T0Quat, _data.m_DtQuat,
      time, 4, nOrderQuat, q);
   double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
   // Divide by the negative norm for 0 through 2 to invert the quaternion
   q[0] /= -norm;
   q[1] /= -norm;
   q[2] /= -norm;
   q[3] /= norm;
   double bodyToCamera[9];
   bodyToCamera[0] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
   bodyToCamera[1] = 2 * (q[0] * q[1] - q[2] * q[3]);
   bodyToCamera[2] = 2 * (q[0] * q[2] + q[1] * q[3]);
   bodyToCamera[3] = 2 * (q[0] * q[1] + q[2] * q[3]);
   bodyToCamera[4] = -q[0] * q[0] + q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
   bodyToCamera[5] = 2 * (q[1] * q[2] - q[0] * q[3]);
   bodyToCamera[6] = 2 * (q[0] * q[2] - q[1] * q[3]);
   bodyToCamera[7] = 2 * (q[1] * q[2] + q[0] * q[3]);
   bodyToCamera[8] = -q[0] * q[0] - q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
   double cameraLookX = bodyToCamera[0] * bodyLookX
                      + bodyToCamera[1] * bodyLookY
                      + bodyToCamera[2] * bodyLookZ;
   double cameraLookY = bodyToCamera[3] * bodyLookX
                      + bodyToCamera[4] * bodyLookY
                      + bodyToCamera[5] * bodyLookZ;
   double cameraLookZ = bodyToCamera[6] * bodyLookX
                      + bodyToCamera[7] * bodyLookY
                      + bodyToCamera[8] * bodyLookZ;

   // Invert the attitude correction
   double aTime = time - _data.m_T0Quat;
   double euler[3];
   double nTime = aTime / _data.m_HalfTime;
   double nTime2 = nTime * nTime;
   euler[0] =
      (getValue(6, adj) + getValue(9, adj)* nTime + getValue(12, adj)* nTime2) / _data.m_FlyingHeight;
   euler[1] =
      (getValue(7, adj) + getValue(10, adj)* nTime + getValue(13, adj)* nTime2) / _data.m_FlyingHeight;
   euler[2] =
      (getValue(8, adj) + getValue(11, adj)* nTime + getValue(14, adj)* nTime2) / _data.m_HalfSwath;
   double cos_a = cos(euler[0]);
   double sin_a = sin(euler[0]);
   double cos_b = cos(euler[1]);
   double sin_b = sin(euler[1]);
   double cos_c = cos(euler[2]);
   double sin_c = sin(euler[2]);
   double attCorr[9];
   attCorr[0] = cos_b * cos_c;
   attCorr[1] = -cos_a * sin_c + sin_a * sin_b * cos_c;
   attCorr[2] = sin_a * sin_c + cos_a * sin_b * cos_c;
   attCorr[3] = cos_b * sin_c;
   attCorr[4] = cos_a * cos_c + sin_a * sin_b * sin_c;
   attCorr[5] = -sin_a * cos_c + cos_a * sin_b * sin_c;
   attCorr[6] = -sin_b;
   attCorr[7] = sin_a * cos_b;
   attCorr[8] = cos_a * cos_b;
   double adjustedLookX = attCorr[0] * cameraLookX
                        + attCorr[3] * cameraLookY
                        + attCorr[6] * cameraLookZ;
   double adjustedLookY = attCorr[1] * cameraLookX
                        + attCorr[4] * cameraLookY
                        + attCorr[7] * cameraLookZ;
   double adjustedLookZ = attCorr[2] * cameraLookX
                        + attCorr[5] * cameraLookY
                        + attCorr[8] * cameraLookZ;

   // Invert the boresight correction
   double correctedLookX = _data.m_MountingMatrix[0] * adjustedLookX
                         + _data.m_MountingMatrix[3] * adjustedLookY
                         + _data.m_MountingMatrix[6] * adjustedLookZ;
   double correctedLookY = _data.m_MountingMatrix[1] * adjustedLookX
                         + _data.m_MountingMatrix[4] * adjustedLookY
                         + _data.m_MountingMatrix[7] * adjustedLookZ;
   double correctedLookZ = _data.m_MountingMatrix[2] * adjustedLookX
                         + _data.m_MountingMatrix[5] * adjustedLookY
                         + _data.m_MountingMatrix[8] * adjustedLookZ;

   // Convert to focal plane coordinate
   double lookScale = _data.m_Focal / correctedLookZ;
   double focalX = correctedLookX * lookScale;
   double focalY = correctedLookY * lookScale;

   // TODO invert distortion here
   // We probably only want to try and invert the distortion if we are
   // reasonably close to the actual CCD because the distortion equations are
   // sometimes only well behaved close to the CCD.

   // Convert to detector line and sample
   double detectorLine = _data.m_ITransL[0]
                       + _data.m_ITransL[1] * focalX
                       + _data.m_ITransL[2] * focalY;
   double detectorSample = _data.m_ITransS[0]
                         + _data.m_ITransS[1] * focalX
                         + _data.m_ITransS[2] * focalY;

   // Convert to image sample line
   double line = detectorLine + _data.m_DetectorLineOrigin - _data.m_DetectorLineOffset
               - _data.m_OffsetLines + 0.5;
   double sample = (detectorSample + _data.m_DetectorSampleOrigin - _data.m_StartingSample)
                 / _data.m_DetectorSampleSumming - _data.m_OffsetSamples + 0.5;
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
      double numRows = _data.m_TotalLines;
      double numCols = _data.m_TotalSamples;
      if (ip.line < 0.0)     ip.line = 0.0;
      if (ip.line > numRows) ip.line = numRows;

      if (ip.samp < 0.0)     ip.samp = 0.0;
      if (ip.samp > numCols) ip.samp = numCols;
   }
   else
   {
      ip.line = _data.m_TotalLines / 2.0;
      ip.samp = _data.m_TotalSamples / 2.0;
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


   double numRows = _data.m_TotalLines;
   double numCols = _data.m_TotalSamples;

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
