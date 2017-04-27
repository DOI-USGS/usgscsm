# CSM-CameraModel

[![Build Status](https://travis-ci.org/USGS-Astrogeology/CSM-CameraModel.svg?branch=master)](https://travis-ci.org/USGS-Astrogeology/CSM-CameraModel)

This repository stores USGS CSM compliant camera models and the associated
plugins.  These camera models are compliant with the [Community Sensor
Model](https://github.com/sminster/csm) and can be utilized within any application supporting the CSM.

The CSM requires associated Image Support Data.  This data is primarily
provided by the NASA Navigation and Ancillary Information (NAIF) SPICE system.
We provide a seperate
[CSM-SpiceISD](https://github.com/USGS-Astrogeology/CSM-SpiceISD) library to interface with the NAIF Spice
functionality and provide a standard ISD file.  
