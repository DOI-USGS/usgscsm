<p align="center">
  <img src="docs/USGSCSM_Logo.svg" alt="USGSCSM" width=200>
</p>

# USGSCSM

This library provides *Community Sensor Model (CSM)*-compliant sensor models
created by the USGS Astrogeology Science Center.

USGSCSM contains three different sensor models. The first is a
generic framing camera model written from scratch. The second is a
generic line scan camera model based on code from BAE Systems
Information and Electronic Systems Integration, Inc. The third is a
generic synthetic-aperture radar (SAR) sensor model.

## Using USGSCSM

This library is a CSM plugin library that is intended to be dynamically loaded
at run-time alongside the
[CSM API library](https://github.com/USGS-Astrogeology/csm).

Once the library is loaded, it can be accessed through the CSM plugin interface.
For an example of how to do through the CSM C++ interface see the SensorModelFactory
class in [SensorUtils](https://github.com/USGS-Astrogeology/SensorUtils).
For an example of how to do this through the CSM Python bindings see this
[notebook](http://nbviewer.jupyter.org/gist/thareUSGS/4c0eb72799edc33ff4816b2587027148).

From the CSM plugin interface, a generic framing camera model
(USGS_ASTRO_FRAME_SENSOR_MODEL), line scan camera model
(USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL), or a SAR model
(USGS_ASTRO_SAR_SENSOR_MODEL) can be instantiated from a suitable *Image
Support Data (ISD)* file.

## Camera model format and model state

Under the CSM standard, each plugin library can define its own ISD
camera model format. This library uses an auxiliary JSON formatted file that must
be next to the image file passed to the CSM::ISD class. We provide an
OpenAPI server for generating these,
[pfeffernusse](https://github.com/USGS-Astrogeology/pfeffernusse). The
swagger specification is located on
[swaggerhub](https://app.swaggerhub.com/apis/USGS-Astro/pfeffernusse2/0.1.4-oas3).
You can also use [ALE](https://github.com/USGS-Astrogeology/ale)
directly with metakernels to generate the auxiliary JSON file.

The camera model read from an ISD file is converted at load time to an
internal representation which makes camera operations more
efficient. This optimized *model state* can be saved to disk as a
JSON-formatted file, be used interchangeably with the
original ISD model, and also shared among various photogrammetric
packages.

The camera model state can be modified by an application of a rotation
and translation, which is necessary in order to refine a camera's
position and orientation in photogrammetry, while these operations are
not easy to express in the original ISD format.

This library provides functionality for saving the model state file,
as discussed in the next section.

## Camera model processsing

USGSCSM ships with a program named ``usgscsm_cam_test``, which is
able to load a CSM camera model, whether in the original ISD format or its
model state representation, export the model state, and perform basic
camera operations, as described in its
[documentation](docs/source/tools/usgscsm_cam_test.rst).

## Enabling logging

Logging of the internal operations in the sensor models can be enabled by setting
the `USGSCSM_LOG_FILE` environment variable to the file the log should be written to.
To have the logging information printed to the standard output or standard error, set
this to `stdout` or `stderr`.

You can adjust how much information is logged by setting the `USGSCSM_LOG_LEVEL`
environment variable. The log level is not case sensitive.
The log levels are:

| Level | Description |
| ----- | ----------- |
| trace | Intermediate calculation values |
| debug | All function calls and returns |
| **info** | Only core photogrammetry calls - Default log level |
| warn | CSM warnings |
| err | CSM exceptions |
| critical | Critical errors |
| off | No log messages |

All log messages of level `USGSCSM_LOG_LEVEL` and below will be logged. For example,
setting the log level to *info* will log all messages of types *info*, *warn*, *err*,
*critical*, and *off*. Note that these logs can become several GB in size when the
log level is set to *debug* or *trace*.

---

## Build requirements

* cmake 3.15 or newer
* GNU-compatible Make
* a C++11 compliant compiler

This repository has all of its external C++ dependencies included in it. The
excellent header-only JSON library
[JSON for Modern C++](https://github.com/nlohmann/json) is included directly in
the source code. The other three dependencies, The Abstraction Library for
Ephemerides, the CSM API library, and googletest are included as git submodules.
When you clone this library make sure you add the `--recursive` flag to your
`git clone` command. Alternatively, you can run
`git submodule update --init --recursive` after cloning.

You can also install the build requirements using Conda with the provided
`environment.yml` file. The following commands will create a new environment
to build against. Note that googletest cannot be installed via anaconda and must
be available within the source code. You can remove the googletest dependency
by disabling the tests.

```
conda env create -n usgscsm -f environment.yml
```

## Building USGSCSM

USGSCSM uses a standard cmake build system. To compile the library and
tests use the following commands:

1. `mkdir build && cd build`
2. `cmake .. && cmake --build .`

If you are using external dependencies via Conda or system level installations
add the `-DUSGSCSM_EXTERNAL_DEPS=ON` flag to the cmake command.

You can also disable the tests and the googletest dependency by adding the
`-DUSGSCSM_BUILD_TESTS=OFF` flag to the cmake command.

## Testing USGSCSM

All of the tests for USGSCSM are written in the googletest framework
and are run via ctest. To run all of the tests simply run `ctest` in the build.

All of the tests are purposefully written to use generic data that values have
been hand validated. This data can be found under `tests/data`.

## Code style

This software package uses a modified form of the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

Here are some exceptions:

1. Non-const pass-by-reference is allowed.
2. No copyright notice is necessary
3. Static/global string constants are allowed to be std::strings, rather than C-style strings

To attempt to automatically format any new code to this style, run:
`clang-format -style=Google -i file.cpp`
For more information see: [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html)

To check for compliance, run: `cpplint file.cpp` and ignore errors in the list of exclusions above.
For more information, see: [cpplint](https://github.com/cpplint/cpplint).
