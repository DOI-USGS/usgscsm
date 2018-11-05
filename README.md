# CSM-CameraModel

Community Sensor Model (CSM) compliant sensor models created by USGS Astrogeology
Science Center.

CSM-CameraModel contains two different sensor models. The first, is a generic
framing camera model written from scratch. The second is a generic line scan
camera model based on code from BAE Systems Information and Electronic Systems
Integration Inc.

## Using CSM-CameraModel

This library is a CSM plugin library that is intended to be dynamically loaded
at run time along side the
[CSM API library](https://github.com/USGS-Astrogeology/csm).

Once, the library is loaded, it can be accessed through the CSM Plugin interface.
For an example of how to do through the CSM c++ interface see the SensorModelFactory
class in [SensorUtils](https://github.com/USGS-Astrogeology/SensorUtils).
For an example of how to do this through the CSM Python bindings see this
[notebook](http://nbviewer.jupyter.org/gist/thareUSGS/4c0eb72799edc33ff4816b2587027148).

From the CSM Plugin interface, a generic framing camera model
(USGS_ASTRO_FRAME_SENSOR_MODEL) or generic line scan camera model
(USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL) can be instantiated from suitable Image
Support Data (ISD). Under the CSM standard, each plugin library can define its
own ISD format. This library uses an auxiliary JSON formatted file that must be
next to the image file passed to the CSM::ISD class. We provide an OpenAPI
server for generating these,
[pfeffernusse](https://github.com/USGS-Astrogeology/pfeffernusse). The swagger
specification is located on
[swaggerhub](https://app.swaggerhub.com/apis/USGS-Astro/pfeffernusse2/0.1.4).

---

## Build Requirements

* cmake 3.10 or newer
* GNU-compatible Make
* a c++11 compliant compiler

This repository has all of its external c++ dependencies included in it. The
excellent header-only JSON library
[JSON for Modern C++](https://github.com/nlohmann/json) is included directly in
the source code. The other two dependencies, the CSM API library, and gtest
are included as git submodules. The library can also be compiled against an
installed versions of the CSM API by setting the BUILD_CSM flag to OFF during
cmake configuration.

## Building CSM-CameraModel

CSM-CameraModel uses a standard cmake build system. To compile the library, and
tests use the following commands:

1. `mkdir build && cd build`
2. `cmake .. && cmake --build .`

## Testing CSM-CameraModel

All of the tests for CSM-CameraModel are written in the googletests framework
and are run via ctest. To run all of the tests simply run `ctest` in the build.

All of the tests are purposefully written to use generic data that values have
been hand validated for. This data can be found under `tests/data`.
