/**
 * WebAssembly bindings for USGSCSM using Emscripten Embind
 *
 * This file provides JavaScript-friendly wrapper around the CSM plugin interface.
 */

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/emscripten.h>

#include "UsgsAstroPlugin.h"
#include "UsgsAstroPluginSupport.h"
#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroPushFrameSensorModel.h"
#include "UsgsAstroSarSensorModel.h"
#include "RasterGM.h"
#include "Utilities.h"

#include <Error.h>
#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <stdexcept>

using namespace emscripten;

/**
 * Wrapper class that provides a simplified JavaScript API for USGSCSM sensor models.
 */
class USGSCSMWrapper {
private:
  std::shared_ptr<csm::RasterGM> model;
  UsgsAstroPlugin plugin;

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
    std::cerr << "=== loadModelFromISD called ===" << std::endl;
    try {
      // Write ISD to virtual filesystem
      std::string temp_file = "/tmp/model.json";
      std::ofstream ofs(temp_file);
      if (!ofs.is_open()) {
        std::cerr << "Failed to create temporary ISD file" << std::endl;
        return false;
      }
      ofs << isd_json;
      ofs.close();

      // Load via CSM plugin interface
      csm::Isd isd(temp_file);
      csm::WarningList warnings;

      // Construct the model (may throw csm::Error)
      std::cerr << "About to call constructModelFromISD..." << std::endl;
      csm::Model* csm_model = plugin.constructModelFromISD(isd, model_name, &warnings);
      std::cerr << "constructModelFromISD returned: " << (void*)csm_model << std::endl;

      if (!csm_model) {
        std::cerr << "Failed to construct model from ISD (returned null)" << std::endl;
        return false;
      }

      std::cerr << "About to dynamic_cast..." << std::endl;
      csm::RasterGM* raster = dynamic_cast<csm::RasterGM*>(csm_model);
      std::cerr << "dynamic_cast returned: " << (void*)raster << std::endl;

      if (!raster) {
        std::cerr << "Failed to cast model to RasterGM" << std::endl;
        return false;
      }

      std::cerr << "About to create shared_ptr..." << std::endl;
      model = std::shared_ptr<csm::RasterGM>(raster);
      std::cerr << "shared_ptr created successfully" << std::endl;

      std::cerr << "=== Model loaded successfully, returning true ===" << std::endl;
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
      return false;  // Never reached, but satisfies compiler

    } catch (const std::exception& e) {
      // Convert std::exception to JavaScript Error object
      std::string msg = "Exception: ";
      msg += e.what();
      std::cerr << msg << std::endl;

      // Create JavaScript Error and throw it
      throw std::runtime_error(msg);
      return false;  // Never reached, but satisfies compiler

    } catch (...) {
      std::string msg = "Unknown exception in loadModelFromISD";
      std::cerr << msg << std::endl;

      // Create JavaScript Error and throw it
      throw std::runtime_error(msg);
      return false;  // Never reached, but satisfies compiler
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
      csm::Model* csm_model = plugin.constructModelFromState(state_json, &warnings);

      if (!csm_model) {
        std::cerr << "Failed to construct model from state" << std::endl;
        return false;
      }

      model = std::shared_ptr<csm::RasterGM>(dynamic_cast<csm::RasterGM*>(csm_model));
      return model != nullptr;

    } catch (const csm::Error& e) {
      // Convert CSM error to JavaScript Error object
      std::string msg = "CSM Error in loadModelFromState: ";
      msg += e.getMessage();
      msg += " (in ";
      msg += e.getFunction();
      msg += ")";
      std::cerr << msg << std::endl;

      throw std::runtime_error(msg);
      return false;  // Never reached

    } catch (const std::exception& e) {
      // Convert std::exception to JavaScript Error object
      std::string msg = "Exception in loadModelFromState: ";
      msg += e.what();
      std::cerr << msg << std::endl;

      throw std::runtime_error(msg);
      return false;  // Never reached

    } catch (...) {
      std::string msg = "Unknown exception in loadModelFromState";
      std::cerr << msg << std::endl;

      throw std::runtime_error(msg);
      return false;  // Never reached
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
   * @return JavaScript object with {x, y, z} ECEF coordinates, or null if no model loaded
   */
  val imageToGround(double line, double sample, double height) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
      return val::null();  // Never reached
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
      return val::null();  // Never reached
    }
  }

  /**
   * Convert ground coordinates (ECEF) to image coordinates.
   *
   * @param x ECEF X coordinate (meters)
   * @param y ECEF Y coordinate (meters)
   * @param z ECEF Z coordinate (meters)
   * @return JavaScript object with {line, sample} pixel coordinates, or null if no model loaded
   */
  val groundToImage(double x, double y, double z) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
      return val::null();  // Never reached
    }

    try {
      csm::EcefCoord groundPt(x, y, z);
      csm::ImageCoord imagePt = model->groundToImage(groundPt);

      val result = val::object();
      result.set("line", imagePt.line);
      result.set("sample", imagePt.samp);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "groundToImage error: ";
      msg += e.what();
      throw std::runtime_error(msg);
      return val::null();  // Never reached
    }
  }

  /**
   * Get sensor position for a given image coordinate.
   *
   * @param line Image line coordinate
   * @param sample Image sample coordinate
   * @return JavaScript object with {x, y, z} ECEF coordinates of sensor, or null if no model loaded
   */
  val getSensorPosition(double line, double sample) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
      return val::null();  // Never reached
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
      return val::null();  // Never reached
    }
  }

  /**
   * Get sensor velocity for a given image coordinate.
   *
   * @param line Image line coordinate
   * @param sample Image sample coordinate
   * @return JavaScript object with {x, y, z} ECEF velocity vector, or null if no model loaded
   */
  val getSensorVelocity(double line, double sample) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
      return val::null();  // Never reached
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
      return val::null();  // Never reached
    }
  }

  /**
   * Get illumination direction (sun vector) for a ground point.
   *
   * @param x ECEF X coordinate (meters)
   * @param y ECEF Y coordinate (meters)
   * @param z ECEF Z coordinate (meters)
   * @return JavaScript object with {x, y, z} unit vector pointing from ground to sun, or null
   */
  val getIlluminationDirection(double x, double y, double z) const {
    if (!model) {
      throw std::runtime_error("No model loaded");
      return val::null();  // Never reached
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
      return val::null();  // Never reached
    }
  }

  /**
   * Get image dimensions.
   *
   * @return JavaScript object with {lines, samples}, or null if no model loaded
   */
  val getImageSize() const {
    if (!model) {
      throw std::runtime_error("No model loaded");
      return val::null();  // Never reached
    }

    try {
      csm::ImageVector size = model->getImageSize();
      val result = val::object();
      result.set("lines", size.line);
      result.set("samples", size.samp);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "getImageSize error: ";
      msg += e.what();
      throw std::runtime_error(msg);
      return val::null();  // Never reached
    }
  }

  /**
   * Get the image start coordinates.
   *
   * @return JavaScript object with {line, sample}, or null if no model loaded
   */
  val getImageStart() const {
    if (!model) {
      throw std::runtime_error("No model loaded");
      return val::null();  // Never reached
    }

    try {
      csm::ImageCoord start = model->getImageStart();
      val result = val::object();
      result.set("line", start.line);
      result.set("sample", start.samp);
      return result;

    } catch (const std::exception& e) {
      std::string msg = "getImageStart error: ";
      msg += e.what();
      throw std::runtime_error(msg);
      return val::null();  // Never reached
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

  /**
   * Test if plugin is loaded correctly.
   *
   * @return plugin name
   */
  std::string testPlugin() {
    try {
      fprintf(stderr, "[TEST] Getting plugin name\n");
      fflush(stderr);
      std::string name = plugin.getPluginName();
      fprintf(stderr, "[TEST] Plugin name: %s\n", name.c_str());
      fflush(stderr);
      return name;
    } catch (std::exception& e) {
      fprintf(stderr, "[TEST] Exception in testPlugin: %s\n", e.what());
      fflush(stderr);
      return "ERROR: " + std::string(e.what());
    } catch (...) {
      fprintf(stderr, "[TEST] Unknown exception in testPlugin\n");
      fflush(stderr);
      return "ERROR: Unknown exception";
    }
  }

  /**
   * Test calling populateModel directly.
   *
   * @return true if test passes
   */
  bool testPopulateModel() {
    try {
      fprintf(stderr, "[TEST] Creating LsSensorModel\n");
      fflush(stderr);
      UsgsAstroLsSensorModel *lsModel = new UsgsAstroLsSensorModel();
      fprintf(stderr, "[TEST] LsSensorModel created at %p\n", (void*)lsModel);
      fflush(stderr);

      fprintf(stderr, "[TEST] Calling reset() directly\n");
      fflush(stderr);
      lsModel->reset();
      fprintf(stderr, "[TEST] reset() completed!\n");
      fflush(stderr);

      delete lsModel;
      fprintf(stderr, "[TEST] Model deleted successfully\n");
      fflush(stderr);
      return true;

    } catch (std::exception& e) {
      fprintf(stderr, "[TEST] Exception in testPopulateModel: %s\n", e.what());
      fflush(stderr);
      return false;
    } catch (...) {
      fprintf(stderr, "[TEST] Unknown exception in testPopulateModel\n");
      fflush(stderr);
      return false;
    }
  }
};

// Embind declarations to expose C++ class to JavaScript
EMSCRIPTEN_BINDINGS(usgscsm) {
  class_<USGSCSMWrapper>("USGSCSMModel")
    .constructor<>()
    .function("loadFromISD", &USGSCSMWrapper::loadModelFromISD)
    .function("loadFromState", &USGSCSMWrapper::loadModelFromState)
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
    .function("testVariantMap", &USGSCSMWrapper::testVariantMap)
    .function("testPlugin", &USGSCSMWrapper::testPlugin)
    .function("testPopulateModel", &USGSCSMWrapper::testPopulateModel);
}
