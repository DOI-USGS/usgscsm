/**
 * WebAssembly bindings for USGSCSM using Emscripten Embind
 *
 * This file provides JavaScript-friendly wrapper around the CSM plugin interface.
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/emscripten.h>

#include "Utilities.h"

#include <csm/Error.h>
#include <csm/RasterGM.h>
#include <nlohmann/json.hpp>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>

using namespace emscripten;
using json = nlohmann::json;

/**
 * Wrapper class that provides a simplified JavaScript API for USGSCSM sensor models.
 */
class USGSCSMWrapper {
private:
  std::shared_ptr<csm::RasterGM> model;

public:
  USGSCSMWrapper() {}

  /**
   * Load a sensor model from ISD JSON string.
   *
   * @param isd_json JSON string containing Image Support Data
   * @param model_name Model type (e.g., "USGS_ASTRO_FRAME_SENSOR_MODEL")
   * @return true if model loaded successfully
   */
  bool loadModelFromISD(const std::string& isd_json, const std::string& model_name) {
    try {
      csm::WarningList warnings;

      // Directly construct model from ISD JSON string (no temp file needed)
      csm::RasterGM* raster = getUsgsCsmModelFromIsd(isd_json, model_name, &warnings);

      if (!raster) {
        std::cerr << "Failed to construct model from ISD" << std::endl;
        return false;
      }

      model = std::shared_ptr<csm::RasterGM>(raster);
      return true;

    } catch (const csm::Error& e) {
      // Convert CSM error to JavaScript Error object
      std::string msg = "CSM Error: ";
      msg += e.getMessage();
      msg += " (in ";
      msg += e.getFunction();
      msg += ")";
      std::cerr << msg << std::endl;

      // Create JavaScript Error and throw it
      throw std::runtime_error(msg);

    } catch (const std::exception& e) {
      // Convert std::exception to JavaScript Error object
      std::string msg = "Exception: ";
      msg += e.what();
      std::cerr << msg << std::endl;

      // Create JavaScript Error and throw it
      throw std::runtime_error(msg);

    } catch (...) {
      std::string msg = "Unknown exception in loadModelFromISD";
      std::cerr << msg << std::endl;

      // Create JavaScript Error and throw it
      throw std::runtime_error(msg);
    }
  }

  /**
   * Load a sensor model from model state JSON string.
   *
   * @param state_json JSON string containing model state
   * @return true if model loaded successfully
   */
  bool loadModelFromState(const std::string& state_json) {
    try {
      csm::WarningList warnings;

      // Extract model name from state JSON
      json state = stateAsJson(state_json);
      std::string modelName = state["m_modelName"];

      // Directly construct model from state JSON string (utility function, not plugin)
      csm::RasterGM* raster = getUsgsCsmModelFromJsonState(state_json, modelName, &warnings);

      if (!raster) {
        std::cerr << "Failed to construct model from state" << std::endl;
        return false;
      }

      model = std::shared_ptr<csm::RasterGM>(raster);
      return true;

    } catch (const csm::Error& e) {
      // Convert CSM error to JavaScript Error object
      std::string msg = "CSM Error in loadModelFromState: ";
      msg += e.getMessage();
      msg += " (in ";
      msg += e.getFunction();
      msg += ")";
      std::cerr << msg << std::endl;

      throw std::runtime_error(msg);

    } catch (const std::exception& e) {
      // Convert std::exception to JavaScript Error object
      std::string msg = "Exception in loadModelFromState: ";
      msg += e.what();
      std::cerr << msg << std::endl;

      throw std::runtime_error(msg);

    } catch (...) {
      std::string msg = "Unknown exception in loadModelFromState";
      std::cerr << msg << std::endl;

      throw std::runtime_error(msg);
    }
  }

  /**
   * Load a sensor model from a raw byte buffer, sniffing the format from the
   * leading bytes as usgscsm_cam_test does. Used by the JS fetch helpers in
   * usgscsm_post.js, which do the downloading and hand the bytes here.
   *
   * @param bytes A JavaScript Uint8Array (or other typed array) of file content.
   * @return true if a model was loaded.
   */
  bool loadModelFromBytes(val bytes) {
    // Copy the JS typed array into a std::string (byte buffer).
    const size_t length = bytes["length"].as<size_t>();
    std::string data;
    data.resize(length);
    if (length > 0) {
      val memView = val(typed_memory_view(length,
                                          reinterpret_cast<uint8_t*>(&data[0])));
      memView.call<void>("set", bytes);
    }

    try {
      switch (modelFormatFromBytes(data)) {
        case ModelFormat::Stards: {
#ifdef USGSCSM_ENABLE_STARDS
          // STARDS reads from a path, so stage the bytes in MEMFS first. Emscripten
          // gives every module its own filesystem, so a fixed name cannot collide
          // with another process; the unlink below keeps it from accumulating.
          const std::string tmpPath = "/tmp/usgscsm_load.stards";
          {
            std::ofstream ofs(tmpPath, std::ios::binary);
            ofs.write(data.data(), static_cast<std::streamsize>(data.size()));
          }
          csm::RasterGM* raster = getUsgsCsmModelFromStards(tmpPath, nullptr);
          std::remove(tmpPath.c_str());
          if (!raster) return false;
          model = std::shared_ptr<csm::RasterGM>(raster);
          return true;
#else
          throw std::runtime_error(
              "loadFromBytes: STARDS file, but this build has no STARDS support");
#endif
        }

        case ModelFormat::Msgpack: {
          const char* ptr = data.data();
          json j = json::from_msgpack(ptr, ptr + data.size());
          std::string modelName = j.at("m_modelName").get<std::string>();
          csm::RasterGM* raster =
              getUsgsCsmModelFromJsonState(j.dump(), modelName, nullptr);
          if (!raster) return false;
          model = std::shared_ptr<csm::RasterGM>(raster);
          return true;
        }

        case ModelFormat::Text: {
          // A JSON ISD, or a JSON/.sup model state.
          std::string modelName;
          if (isUsgsCsmIsd(data, modelName)) {
            return loadModelFromISD(data, modelName);
          }
          if (isUsgsCsmState(data, modelName)) {
            return loadModelFromState(data);
          }
          break;
        }

        case ModelFormat::Unknown:
          break;
      }

      std::cerr << "loadFromBytes: unrecognized file format" << std::endl;
      return false;

    } catch (const std::exception& e) {
      std::string msg = "loadFromBytes error: ";
      msg += e.what();
      std::cerr << msg << std::endl;
      throw std::runtime_error(msg);
    }
  }

  /**
   * Get the current model state as a JSON string.
   *
   * @return JSON string containing model state, or empty string if no model loaded
   */
  std::string getModelState() const {
    if (!model) return "";
    return model->getModelState();
  }

  /**
   * Convert image coordinates to ground coordinates (ECEF).
   *
   * @param line Image line coordinate (row)
   * @param sample Image sample coordinate (column)
   * @param height Height above reference ellipsoid (meters)
   * @return JavaScript object with {x, y, z} ECEF coordinates. Throws if no
   *         model is loaded.
   */
  val imageToGround(double line, double sample, double height) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
    }

    try {
      csm::ImageCoord imagePt(line, sample);
      csm::EcefCoord groundPt = model->imageToGround(imagePt, height);

      val result = val::object();
      result.set("x", groundPt.x);
      result.set("y", groundPt.y);
      result.set("z", groundPt.z);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "imageToGround error: ";
      msg += e.what();
      throw std::runtime_error(msg);
    }
  }

  /**
   * Convert ground coordinates (ECEF) to image coordinates.
   *
   * @param x ECEF X coordinate (meters)
   * @param y ECEF Y coordinate (meters)
   * @param z ECEF Z coordinate (meters)
   * @return JavaScript object with {line, samp} pixel coordinates. Throws if no
   *         model is loaded.
   */
  val groundToImage(double x, double y, double z) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
    }

    try {
      csm::EcefCoord groundPt(x, y, z);
      csm::ImageCoord imagePt = model->groundToImage(groundPt);

      val result = val::object();
      result.set("line", imagePt.line);
      result.set("samp", imagePt.samp);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "groundToImage error: ";
      msg += e.what();
      throw std::runtime_error(msg);
    }
  }

  /**
   * Get sensor position for a given image coordinate.
   *
   * @param line Image line coordinate
   * @param sample Image sample coordinate
   * @return JavaScript object with {x, y, z} ECEF coordinates of sensor. Throws
   *         if no model is loaded.
   */
  val getSensorPosition(double line, double sample) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
    }

    try {
      csm::ImageCoord imagePt(line, sample);
      csm::EcefCoord sensorPos = model->getSensorPosition(imagePt);

      val result = val::object();
      result.set("x", sensorPos.x);
      result.set("y", sensorPos.y);
      result.set("z", sensorPos.z);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "getSensorPosition error: ";
      msg += e.what();
      throw std::runtime_error(msg);
    }
  }

  /**
   * Get sensor velocity for a given image coordinate.
   *
   * @param line Image line coordinate
   * @param sample Image sample coordinate
   * @return JavaScript object with {x, y, z} ECEF velocity vector. Throws if no
   *         model is loaded.
   */
  val getSensorVelocity(double line, double sample) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
    }

    try {
      csm::ImageCoord imagePt(line, sample);
      csm::EcefVector velocity = model->getSensorVelocity(imagePt);

      val result = val::object();
      result.set("x", velocity.x);
      result.set("y", velocity.y);
      result.set("z", velocity.z);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "getSensorVelocity error: ";
      msg += e.what();
      throw std::runtime_error(msg);
    }
  }

  /**
   * Get illumination direction (sun vector) for a ground point.
   *
   * @param x ECEF X coordinate (meters)
   * @param y ECEF Y coordinate (meters)
   * @param z ECEF Z coordinate (meters)
   * @return JavaScript object with {x, y, z} unit vector pointing from ground to
   *         sun. Throws if no model is loaded.
   */
  val getIlluminationDirection(double x, double y, double z) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
    }

    try {
      csm::EcefCoord groundPt(x, y, z);
      csm::EcefVector sunVec = model->getIlluminationDirection(groundPt);

      val result = val::object();
      result.set("x", sunVec.x);
      result.set("y", sunVec.y);
      result.set("z", sunVec.z);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "getIlluminationDirection error: ";
      msg += e.what();
      throw std::runtime_error(msg);
    }
  }

  /**
   * Get image dimensions.
   *
   * @return JavaScript object with {line, samp} counts. Throws if no model is
   *         loaded.
   */
  val getImageSize() const {
    if (!model) {
      throw std::runtime_error("No model loaded");
    }

    try {
      csm::ImageVector size = model->getImageSize();
      val result = val::object();
      result.set("line", size.line);
      result.set("samp", size.samp);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "getImageSize error: ";
      msg += e.what();
      throw std::runtime_error(msg);
    }
  }

  /**
   * Get the image start coordinates.
   *
   * @return JavaScript object with {line, samp}. Throws if no model is loaded.
   */
  val getImageStart() const {
    if (!model) {
      throw std::runtime_error("No model loaded");
    }

    try {
      csm::ImageCoord start = model->getImageStart();
      val result = val::object();
      result.set("line", start.line);
      result.set("samp", start.samp);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "getImageStart error: ";
      msg += e.what();
      throw std::runtime_error(msg);
    }
  }

  /**
   * Get the model name (sensor model type).
   *
   * @return Model name string, or empty if no model loaded
   */
  std::string getModelName() const {
    return model ? model->getModelName() : "";
  }

  /**
   * Get the image identifier.
   *
   * @return Image ID string, or empty if no model loaded
   */
  std::string getImageIdentifier() const {
    return model ? model->getImageIdentifier() : "";
  }

  /**
   * Get the sensor identifier.
   *
   * @return Sensor ID string, or empty if no model loaded
   */
  std::string getSensorIdentifier() const {
    return model ? model->getSensorIdentifier() : "";
  }

  /**
   * Get the platform identifier.
   *
   * @return Platform ID string, or empty if no model loaded
   */
  std::string getPlatformIdentifier() const {
    return model ? model->getPlatformIdentifier() : "";
  }

  /**
   * Check if a model is currently loaded.
   *
   * @return true if model is loaded
   */
  bool isLoaded() const {
    return model != nullptr;
  }

  /**
   * Test VariantMap functionality.
   *
   * @return true if VariantMap test passes
   */
  bool testVariantMap() {
    try {
      fprintf(stderr, "[TEST] Creating VariantMap\n");
      fflush(stderr);

      VariantMap vm;

      fprintf(stderr, "[TEST] Adding string to VariantMap\n");
      fflush(stderr);
      vm.set<std::string>("test_string", "hello");

      fprintf(stderr, "[TEST] Adding int to VariantMap\n");
      fflush(stderr);
      vm.set<int>("test_int", 42);

      fprintf(stderr, "[TEST] Adding double to VariantMap\n");
      fflush(stderr);
      vm.set<double>("test_double", 3.14);

      fprintf(stderr, "[TEST] Adding vector to VariantMap\n");
      fflush(stderr);
      std::vector<double> test_vec = {1.0, 2.0, 3.0};
      vm.set<std::vector<double>>("test_vector", test_vec);

      fprintf(stderr, "[TEST] Getting string from VariantMap\n");
      fflush(stderr);
      std::string str = vm.get<std::string>("test_string");

      fprintf(stderr, "[TEST] Getting int from VariantMap\n");
      fflush(stderr);
      int i = vm.get<int>("test_int");

      fprintf(stderr, "[TEST] Getting double from VariantMap\n");
      fflush(stderr);
      double d = vm.get<double>("test_double");

      fprintf(stderr, "[TEST] Getting vector from VariantMap\n");
      fflush(stderr);
      std::vector<double> vec = vm.get<std::vector<double>>("test_vector");

      fprintf(stderr, "[TEST] VariantMap test passed!\n");
      fflush(stderr);

      return str == "hello" && i == 42 && d == 3.14 && vec.size() == 3;

    } catch (std::exception& e) {
      fprintf(stderr, "[TEST] Exception in testVariantMap: %s\n", e.what());
      fflush(stderr);
      return false;
    } catch (...) {
      fprintf(stderr, "[TEST] Unknown exception in testVariantMap\n");
      fflush(stderr);
      return false;
    }
  }

};

// Standalone utility functions for 1-to-1 C++ API mapping

/**
 * Check if a string is a USGS CSM ISD and extract the model name.
 * @param str The string to check
 * @return Object with {isIsd: bool, modelName: string}
 */
val checkIsUsgsCsmIsd(const std::string& str) {
  std::string modelName;
  bool isIsd = isUsgsCsmIsd(str, modelName);

  val result = val::object();
  result.set("isIsd", isIsd);
  result.set("modelName", modelName);
  return result;
}

/**
 * Check if a string is a USGS CSM model state and extract the model name.
 * @param str The string to check
 * @return Object with {isState: bool, modelName: string}
 */
val checkIsUsgsCsmState(const std::string& str) {
  std::string modelName;
  bool isState = isUsgsCsmState(str, modelName);

  val result = val::object();
  result.set("isState", isState);
  result.set("modelName", modelName);
  return result;
}

/**
 * Get model state JSON from a model (utility function).
 * Note: This requires a src/Utilities.cppUSGSCSMModel instance. Use model.getModelState() instead.
 * Kept for API completeness.
 */
std::string utilGetModelJson(USGSCSMWrapper& wrapper) {
  return wrapper.getModelState();
}

// Embind declarations to expose C++ class to JavaScript
EMSCRIPTEN_BINDINGS(usgscsm) {
  // Main model wrapper class
  class_<USGSCSMWrapper>("USGSCSMModel")
    .constructor<>()
    .function("loadFromISD", &USGSCSMWrapper::loadModelFromISD)
    .function("loadFromState", &USGSCSMWrapper::loadModelFromState)
    .function("loadFromBytes", &USGSCSMWrapper::loadModelFromBytes)
    .function("getModelState", &USGSCSMWrapper::getModelState)
    .function("imageToGround", &USGSCSMWrapper::imageToGround)
    .function("groundToImage", &USGSCSMWrapper::groundToImage)
    .function("getSensorPosition", &USGSCSMWrapper::getSensorPosition)
    .function("getSensorVelocity", &USGSCSMWrapper::getSensorVelocity)
    .function("getIlluminationDirection", &USGSCSMWrapper::getIlluminationDirection)
    .function("getImageSize", &USGSCSMWrapper::getImageSize)
    .function("getImageStart", &USGSCSMWrapper::getImageStart)
    .function("getModelName", &USGSCSMWrapper::getModelName)
    .function("getImageIdentifier", &USGSCSMWrapper::getImageIdentifier)
    .function("getSensorIdentifier", &USGSCSMWrapper::getSensorIdentifier)
    .function("getPlatformIdentifier", &USGSCSMWrapper::getPlatformIdentifier)
    .function("isLoaded", &USGSCSMWrapper::isLoaded)
    .function("testVariantMap", &USGSCSMWrapper::testVariantMap);

  // Utility functions (1-to-1 C++ API mapping)
  function("isUsgsCsmIsd", &checkIsUsgsCsmIsd);
  function("isUsgsCsmState", &checkIsUsgsCsmState);
}
