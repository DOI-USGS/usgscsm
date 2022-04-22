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

#include <usgscsm/UsgsAstroPlugin.h>
#include <csm/RasterGM.h>
#include <UsgsAstroLsSensorModel.h>

#include <iostream>
#include <fstream>
#include <getopt.h> // for parsing command-line options

struct Options {
  std::string model;              // the .json file in isd or model state format
  std::string output_model_state; // the output model state in .json format
  int sample_rate;
  double subpixel_offset, height_above_datum;
  Options(): sample_rate(0), subpixel_offset(0.0), height_above_datum(0.0) {}
};
  
// Parse the input options with getopt.
int parse_options(int argc, char **argv, Options & opt) {

  // Collect the parsed options in a map
  std::map<std::string, std::string> parsed_options;
  
  int digit_optind = 0;
  while (1) {
    int this_option_optind = optind ? optind : 1;
    std::string short_options = ""; // no short options
    int option_index = 0;
    static struct option long_options[] = {
      {"model",              required_argument, 0,  0},
      {"output-model-state", required_argument, 0,  0},
      {"sample-rate",        required_argument, 0,  0},
      {"subpixel-offset",    required_argument, 0,  0},
      {"height-above-datum", required_argument, 0,  0},
      {"help",               no_argument,       0,  0}
    };

    int c = getopt_long(argc, argv, short_options.c_str(), long_options, &option_index);
    if (c == -1) // done parsing all options
      break;

    std::string opt_name = long_options[option_index].name;
    if (c == 0) {
      if (optarg) {
        // The option has a value
        parsed_options[opt_name] = optarg;
      } else {
        // The option has no value
        parsed_options[opt_name] = "";
      }
    } else {
      // Something went wrong in parsing. The parser will print a message. Just add to it.
      std::cout << "Cannot continue.\n";
      return 1;
    }
  }

  // Print unexpected input arguments
  if (optind < argc) {
    printf("Unexpected argument: ");
    while (optind < argc)
      printf("%s ", argv[optind++]);
    printf("\n");

    return 1;
  }

  // See if the user asked for help
  bool print_help_and_exit = (parsed_options.find("help") != parsed_options.end());
  
  // It is safe to access non-existent values from a map, the result
  // will be an empty string
  opt.model = parsed_options["model"];
  if (opt.model == "") 
    print_help_and_exit = true;

  // Enforce that the sample rate is an integer
  double sample_rate_double = atof(parsed_options["sample-rate"].c_str());
  if (sample_rate_double != round(sample_rate_double)) {
    std::cout << "The value of --sample-rate must be an integer.\n";
    print_help_and_exit = true;
  }
  
  // Print the help and exit
  if (print_help_and_exit) {
    std::cout << "Usage: " << argv[0] << " --model <model file> [other options]"
              << "\nSee the documentation for more information.\n";
    return 1;
  }

  // Collect all other option values. If not set the values will default to 0.
  opt.output_model_state = parsed_options["output-model-state"];
  opt.sample_rate        = sample_rate_double;
  opt.subpixel_offset    = atof(parsed_options["subpixel-offset"].c_str());
  opt.height_above_datum = atof(parsed_options["height-above-datum"].c_str());

  return 0;
}

// Read a file's content in a single string
void readFileInString(std::string const& filename, std::string & str) {

  str.clear(); // clear the output

  std::ifstream ifs(filename.c_str());
  ifs.seekg(0, std::ios::end);   
  str.reserve(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  str.assign((std::istreambuf_iterator<char>(ifs)),
             std::istreambuf_iterator<char>());
  ifs.close();
}

// Sort the errors and print some stats
void printErrors(std::vector<double> & errors) {
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
}

double pixDiffNorm(csm::ImageCoord const& a, csm::ImageCoord const& b) {
  return sqrt((a.line - b.line) * (a.line - b.line) + (a.samp - b.samp) * (a.samp - b.samp));
}

// Load a CSM camera model from an ISD or state file. Return 0 on success.
int load_csm_camera_model(std::string const& model_file,
                          std::shared_ptr<csm::RasterGM> & model) {

  // This is needed to trigger loading libusgscsm.so. Otherwise 0
  // plugins are detected.
  UsgsAstroLsSensorModel lsModel;

  // Try to read the model as an ISD
  csm::Isd isd(model_file);
  
  // Read the model in a string, for potentially finding parsing the
  // model state from it.
  std::string model_state;
  readFileInString(model_file, model_state);
  
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
        model_name = csm->getModelName(); // ensure the name is right
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
        return 1;
      } else {
        // Assign to a smart pointer which will handle deallocation
        model = std::shared_ptr<csm::RasterGM>(modelPtr);
        break;
      }
    }
  }
  
  if (!success) {
    std::cerr << "Failed to load a CSM model from: " << model_file << ".\n";
    return 1;
  }
  
  return 0;
}

int main(int argc, char **argv) {

  Options opt;
  if (parse_options(argc, argv, opt) != 0)
    return 1;

  // Keep the model as smart pointer to the class from which the
  // specific model types inherit.
  std::shared_ptr<csm::RasterGM> model;

  if (load_csm_camera_model(opt.model, model) != 0)
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
    std::vector<double> errors;
    for (int samp = 0; samp < image_size.samp; samp += opt.sample_rate) {
      for (int line = 0; line < image_size.line; line += opt.sample_rate) {
        csm::ImageCoord c(line + opt.subpixel_offset, samp + opt.subpixel_offset);
        csm::EcefCoord ground = model->imageToGround(c, opt.height_above_datum);
        csm::ImageCoord d = model->groundToImage(ground);
        double error = pixDiffNorm(c, d);
        errors.push_back(error);
      }   
    }
    printErrors(errors);
  }
    
  return 0;
}
