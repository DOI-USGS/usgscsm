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
#include <UsgsAstroLsSensorModel.h>

#include <iostream>
int main(int argc, char **argv) {

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <model file>" << std::endl;
    return 1;
  }

  // This is needed to trigger loading libusgscsm.so. Otherwise 0
  // plugins are detected.
  UsgsAstroLsSensorModel lsModel;

  // Load the isd
  std::string model_file = argv[1];
  csm::Isd isd(model_file);
  std::cout << "Loading model: " << model_file << std::endl;
  
  // Check if loading the model worked
  bool success = false;

  std::shared_ptr<csm::RasterGM> model;
  
  // Try all detected plugins and models for each plugin
  csm::PluginList plugins = csm::Plugin::getList();
  for (auto iter = plugins.begin(); iter != plugins.end(); iter++) {
    
    const csm::Plugin* csm_plugin = (*iter);

    std::cout << "Detected CSM plugin: " << csm_plugin->getPluginName()  << "\n";
    // For each plugin, loop through the available models.
    size_t num_models = csm_plugin->getNumModels();
    std::cout << "Number of models for this plugin: " << num_models << "\n";
    for (size_t i = 0; i < num_models; i++) {

      std::string model_name = (*iter)->getModelName(i);

      csm::WarningList* warnings = NULL;

      if (csm_plugin->canModelBeConstructedFromISD(isd, model_name)) {

        csm::Model *csm = csm_plugin->constructModelFromISD(isd, model_name, warnings);
        csm::RasterGM *modelPtr = dynamic_cast<csm::RasterGM*>(csm);

        if (modelPtr == NULL) {
          std::cerr << "Could not load correctly a CSM model of type: "
                    << model_name << "\n";
          return 1;
        } else {
          // Assign it to a smart pointer which will handle its deallocation
          model = std::shared_ptr<csm::RasterGM>(modelPtr);
          success = true;
          std::cout << "Loaded CSM model of type " << model_name
                    << " from " << model_file << ".\n";
        }
      }
    }
  }
  
  if (!success) {
    std::cerr << "Failed to load a CSM model from: " << model_file << ".\n";
    return 1;
  }

  csm::ImageVector image_size = model->getImageSize();
  std::cout << "Camera image rows and columns: "
            << image_size.samp << ' ' << image_size.line << "\n";

  return 0;
}
