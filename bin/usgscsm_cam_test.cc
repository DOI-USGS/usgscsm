// A tool to perform some basic tests and operations on a CSM camera model.
// 
// Functionality:
//
// - Load a CSM model in ISD format.
//
// Future functionality:
// 
// - Test projecting rays from the camera to ground and vice-versa.
// - Load a CSM model state (stored in a .json file, just like
//   an ISD model).
// - Ability to export a CSM model in ISD format to a CSM model state file. 

#include <usgscsm/UsgsAstroPlugin.h>
#include <csm/RasterGM.h>

#include <iostream>
int main(int argc, char **argv) {

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <model file>" << std::endl;
    return 1;
  }
  
  std::string model_file = argv[1];
  csm::Isd isd(model_file);

  std::vector<std::string> model_types = {"USGS_ASTRO_FRAME_SENSOR_MODEL",
                                          "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL",
                                          "USGS_ASTRO_SAR_SENSOR_MODEL"};

  // Try to load the model
  bool success = false;
  UsgsAstroPlugin plugin;
  std::shared_ptr<csm::RasterGM> model;
  csm::WarningList* warnings = NULL;
  for (size_t it = 0; it < model_types.size(); it++) {
    if (plugin.canModelBeConstructedFromISD(isd, model_types[it])) {

      csm::Model *csm = plugin.constructModelFromISD(isd, model_types[it], warnings);
      csm::RasterGM *modelPtr = dynamic_cast<csm::RasterGM*>(csm);

      if (modelPtr == NULL) {
        std::cerr << "Could not load correctly a CSM model of type: "
                  << model_types[it] << std::endl;
        return 1;
      } else {
        // Assign it to a smart pointer which will handle its deallocation
        model = std::shared_ptr<csm::RasterGM>(modelPtr);
        success = true;
        std::cout << "Loaded CSM model of type " << model_types[it]
                  << " from " << model_file << ".\n";
      }
    }
  }

  if (!success) {
    std::cerr << "Failed to load a CSM model from: " << model_file << ".\n";
    return 1;
  }

  return 0;
}

