<p align="center">
  <img src="docs/USGSCSM_Logo.svg" alt="USGSCSM" width=200> 
</p>

# USGSCSM

Community Sensor Model (CSM) compliant sensor models created by USGS Astrogeology
Science Center.

USGSCSM contains three different sensor models. The first, is a generic
framing camera model written from scratch. The second is a generic line scan
camera model based on code from BAE Systems Information and Electronic Systems
Integration Inc. The third is a generic SAR sensor model.

## Using USGSCSM

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
[swaggerhub](https://app.swaggerhub.com/apis/USGS-Astro/pfeffernusse2/0.1.4-oas3).
You can also use [ALE](https://github.com/USGS-Astrogeology/ale) directly with
metakernels to generate the auxiliary JSON file.

## Enabling logging

You can enable logging of the internal operations in the sensor models by setting
the `USGSCSM_LOG_FILE` environment variable to the file you would like to log to.
You can also log to standard out by setting it to `stdout` or standard error
by setting it to `stderr`. Note that these logs can become quite large, multiple
GBs.

---

## Build Requirements

* cmake 3.15 or newer
* GNU-compatible Make
* a C++11 compliant compiler

This repository has all of its external c++ dependencies included in it. The
excellent header-only JSON library
[JSON for Modern C++](https://github.com/nlohmann/json) is included directly in
the source code. The other three dependencies, The Abstraction Library for
Ephemerides, the CSM API library, and googletest are included as git submodules.
When you clone this library make sure you add the `--recursive` flag to your
`git clone` command. Alterntively, you can run
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

All of the tests for USGSCSM are written in the googletests framework
and are run via ctest. To run all of the tests simply run `ctest` in the build.

All of the tests are purposefully written to use generic data that values have
been hand validated. This data can be found under `tests/data`.

## Code Style

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
For more information, see: [cpplint](https://github.com/cpplint/cpplint)
