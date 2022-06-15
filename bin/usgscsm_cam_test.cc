// A tool to perform some basic tests and operations on a CSM camera model.
//
// Functionality:
//--------------
//
// - Load a CSM model in ISD format or model state format, via:
//   --model <model file>
//
// - Save the model state if invoked with:
//   --output-model-state <model state .json file>
//
// - Test projecting rays from the camera to ground, then back,
//   and compare with the original pixel values.

#include <UsgsAstroPlugin.h>
#include <RasterGM.h>
#include <UsgsAstroLsSensorModel.h>
#include <Utilities.h>

#include <iostream>
#include <fstream>

struct Options {
  std::string model;              // the .json file in isd or model state format
  std::string output_model_state; // the output model state in .json format
  int sample_rate;
  double subpixel_offset, height_above_datum, desired_precision;
  Options(): sample_rate(0), subpixel_offset(0.0), height_above_datum(0.0), desired_precision(0.0)
  {}
};

void printUsage(std::string const& progName) {
  std::cout << "Usage: " << progName << " --model <model file> [other options]"
            << "\nSee the documentation for more information.\n";
}

// Do some naive parsing. Every option is assumed to start with two
// dashes and have a string value.  The --help option is handled
// separately.
bool parseOptions(int argc, char **argv, Options & opt) {

  std::vector<std::string> params;
  for (int it = 1; it < argc; it++) {
    if (std::string(argv[it]) == std::string("--help")) {
      printUsage(argv[0]);
      return false;
    }
    params.push_back(argv[it]);
  }

  if (params.size() %2 != 0 || params.empty()) {
    std::cout << "Could not parse correctly the input arguments.\n";
    printUsage(argv[0]);
    return false;
  }

  // Collect the parsed options in a map
  std::map<std::string, std::string> parsed_options;
  int num = params.size() / 2;
  for (int it = 0; it < num; it++) {
    std::string opt = params[2*it + 0];
    std::string val = params[2*it + 1];
    if (opt.size() <= 2 || opt[0] != '-' || opt[1] != '-' || val.empty() || val[0] == '-' ) {
      std::cout << "Could not parse correctly the input arguments.\n";
      printUsage(argv[0]);
      return false;
    }
    opt = opt.substr(2); // wipe the dashes
    parsed_options[opt] = val;
  }

  // It is safe to access non-existent values from a map, the result
  // will be an empty string
  opt.model = parsed_options["model"];
  if (opt.model == "") {
    std::cout << "The input model file is empty.\n";
    printUsage(argv[0]);
    return false;
  }

  // Enforce that the sample rate is an integer. Later the user can add
  // a subpixel offset to each sampled pixel, if desired.
  double sample_rate_double = atof(parsed_options["sample-rate"].c_str());
  if (sample_rate_double != round(sample_rate_double)) {
    std::cout << "The value of --sample-rate must be an integer.\n";
    printUsage(argv[0]);
    return false;
  }

  if (parsed_options["desired-precision"].empty())
    parsed_options["desired-precision"] = "0.001"; // set default value

  // Collect all other option values. If not set, the values will default to 0.
  opt.output_model_state = parsed_options["output-model-state"];
  opt.sample_rate        = sample_rate_double;
  opt.subpixel_offset    = atof(parsed_options["subpixel-offset"].c_str());
  opt.height_above_datum = atof(parsed_options["height-above-datum"].c_str());
  opt.desired_precision  = atof(parsed_options["desired-precision"].c_str());

  return true;
}

// Sort the errors and print some stats
void printErrors(std::vector<double> & errors, double desired_precision,
                 double max_achieved_precision) {
  std::sort(errors.begin(), errors.end());

  if (errors.empty()) {
    std::cout << "Empty list of errors.\n";
    return;
  }

  std::cout << "Norm of pixel errors after projecting from camera to ground and back.\n";
  std::cout << "Min:    " << errors[0] << "\n";
  std::cout << "Median: " << errors[errors.size()/2] << "\n";
  std::cout << "Max:    " << errors.back() << "\n";
  std::cout << "Count:  " << errors.size() << "\n";

  std::cout << std::endl;
  std::cout << "Desired precision: " << desired_precision << std::endl;
  std::cout << "Max achieved precision: " << max_achieved_precision << std::endl;
}

double pixDiffNorm(csm::ImageCoord const& a, csm::ImageCoord const& b) {
  return sqrt((a.line - b.line) * (a.line - b.line) + (a.samp - b.samp) * (a.samp - b.samp));
}

// Load a CSM camera model from an ISD or state file. Return true on success.
bool loadCsmCameraModel(std::string const& model_file,
                       std::shared_ptr<csm::RasterGM> & model) {

  // This is needed to trigger loading libusgscsm.so. Otherwise 0
  // plugins are detected.
  UsgsAstroLsSensorModel lsModel;

  // Try to read the model as an ISD
  csm::Isd isd(model_file);

  // Read the model in a string, for potentially finding parsing the
  // model state from it.
  std::string model_state;
  if (!readFileInString(model_file, model_state))
    return false;

  // Remove special characters from string
  sanitize(model_state);

  // Check if loading the model worked
  bool success = false;

  // Try all detected plugins and all models for each plugin.
  csm::PluginList plugins = csm::Plugin::getList();
  for (auto iter = plugins.begin(); iter != plugins.end(); iter++) {

    const csm::Plugin* csm_plugin = (*iter);
    std::cout << "Detected CSM plugin: " << csm_plugin->getPluginName()  << "\n";

    size_t num_models = csm_plugin->getNumModels();
    std::cout << "Number of models for this plugin: " << num_models << "\n";

    // First try to construct the model from isd, and if that fails, from the state
    csm::Model *csm = NULL;
    csm::WarningList* warnings = NULL;
    for (size_t i = 0; i < num_models; i++) {

      std::string model_name = (*iter)->getModelName(i);
      if (csm_plugin->canModelBeConstructedFromISD(isd, model_name, warnings)) {
        // Try to construct the model from the isd
        csm = csm_plugin->constructModelFromISD(isd, model_name, warnings);
        std::cout << "Loaded a CSM model of type " << model_name << " from ISD file "
                  << model_file << ".\n";
        success = true;
      } else if (csm_plugin->canModelBeConstructedFromState(model_name, model_state, warnings)) {
        // Try to construct it from the model state
        csm = csm_plugin->constructModelFromState(model_state, warnings);
        std::cout << "Loaded a CSM model of type " << model_name << " from model state file "
                  << model_file << ".\n";
        success = true;
      } else {
        // No luck so far
        continue;
      }

      csm::RasterGM *modelPtr = dynamic_cast<csm::RasterGM*>(csm);
      if (modelPtr == NULL) {
        // Normally earlier checks should be enough and this should not happen
        std::cerr << "Could not load correctly a CSM model.\n";
        return false;
      } else {
        // Assign to a smart pointer which will handle deallocation
        model = std::shared_ptr<csm::RasterGM>(modelPtr);
        break;
      }
    }
  }

  if (!success) {
    std::cerr << "Failed to load a CSM model from: " << model_file << ".\n";
    return false;
  }

  return true;
}

int main(int argc, char **argv) {

  Options opt;
  if (!parseOptions(argc, argv, opt))
    return 1;

  // Keep the model as smart pointer to the class from which the
  // specific model types inherit.
  std::shared_ptr<csm::RasterGM> model;

  if (!loadCsmCameraModel(opt.model, model))
    return 1;

  if (opt.output_model_state != "") {
    std::cout << "Writing model state: " << opt.output_model_state << "\n";
    std::ofstream ofs(opt.output_model_state.c_str());
    ofs << model->getModelState() << "\n";
    ofs.close();
  }

  if (opt.sample_rate > 0) {
    csm::ImageVector image_size = model->getImageSize();
    std::cout << "\n";
    std::cout << "Camera image rows and columns: "
              << image_size.line << ' ' << image_size.samp << "\n";
    std::cout << "Row and column sample rate: " << opt.sample_rate << "\n";
    std::cout << "Subpixel offset for each pixel: " << opt.subpixel_offset << "\n";
    std::cout << "Ground height (relative to datum): " << opt.height_above_datum << "\n";
    double max_achieved_precision = 0.0;
    std::vector<double> errors;
    for (int samp = 0; samp < image_size.samp; samp += opt.sample_rate) {
      for (int line = 0; line < image_size.line; line += opt.sample_rate) {
        csm::ImageCoord c(line + opt.subpixel_offset, samp + opt.subpixel_offset);

        double achieved_precision = 0.0;
        csm::EcefCoord ground = model->imageToGround(c, opt.height_above_datum,
                                                     opt.desired_precision, &achieved_precision);
        max_achieved_precision = std::max(max_achieved_precision, achieved_precision);

        csm::ImageCoord d = model->groundToImage(ground, opt.desired_precision,
                                                 &achieved_precision);
        max_achieved_precision = std::max(max_achieved_precision, achieved_precision);
        double error = pixDiffNorm(c, d);
        errors.push_back(error);
      }
    }
    printErrors(errors, opt.desired_precision, max_achieved_precision);
  }

  return 0;
}
