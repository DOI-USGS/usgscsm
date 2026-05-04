// A tool to perform some basic tests and operations on a CSM camera model.
//
// Functionality:
//--------------
//
// - Load a CSM model in ISD format, model state, binary msgpack model state,
//   or GXP .sup file format, via:
//   --model <model file>
//
// - Save the model state in JSON format:
//   --output-model-state <model state .json file>
//
// - Save the model state in binary msgpack format:
//   --output-binary-model-state <model state .isd file>
//
// - Modify GXP .sup file's model state with input model:
//   --modify-sup-file <GXP .sup file>
//
// - Test projecting rays from the camera to ground, then back,
//   and compare with the original pixel values.

#include <UsgsAstroPlugin.h>
#include <UsgsAstroPluginSupport.h>
#include <RasterGM.h>
#include <UsgsAstroLsSensorModel.h>
#include <Utilities.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <iostream>
#include <fstream>

struct Options {
  std::string model;               // .json file in isd or model state format
  std::string modify_sup_file;     // .sup file needing a modified model state
  std::string output_model_state;  // output model state in .json format
  std::string output_binary_state; // output model state in msgpack binary format
  int sample_rate;
  bool verbose;
  double subpixel_offset, height_above_datum, desired_precision;
  Options(): sample_rate(0), subpixel_offset(0.0), height_above_datum(0.0), desired_precision(0.0), verbose(false)
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
    std::string arg = std::string(argv[it]);
    if (arg == std::string("--help")) {
      printUsage(argv[0]);
      return false;
    }
    if (arg == std::string("--verbose") || arg == std::string("-v")) {
      opt.verbose = true;
      continue;
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
  opt.modify_sup_file     = parsed_options["modify-sup-file"];
  opt.output_model_state  = parsed_options["output-model-state"];
  opt.output_binary_state = parsed_options["output-binary-model-state"];
  opt.sample_rate         = sample_rate_double;
  opt.subpixel_offset     = atof(parsed_options["subpixel-offset"].c_str());
  opt.height_above_datum  = atof(parsed_options["height-above-datum"].c_str());
  opt.desired_precision   = atof(parsed_options["desired-precision"].c_str());

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
                        std::shared_ptr<csm::RasterGM> & model,
                        Options & opt) {

  // This is needed to trigger loading libusgscsm.so. Otherwise 0
  // plugins are detected. Do not remove this.
  UsgsAstroLsSensorModel lsModel;

  // Binary model state: detect by first byte, load directly via populateModel
  if (isMsgpack(model_file)) {
    std::cout << "Detected msgpack binary model state: " << model_file << "\n";
    std::ifstream ifs(model_file, std::ios::binary);
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());
    nlohmann::json j = nlohmann::json::from_msgpack(data);
    std::string model_name = j.at("m_modelName").get<std::string>();
    model = std::shared_ptr<csm::RasterGM>(getUsgsCsmModelFromJsonState(j.dump(), model_name, NULL));
    std::cout << "Loaded model of type " << model_name
              << " from binary state.\n";
    return true;
  }

  // Read the file into a string
  std::string model_state;
  if (!readFileInString(model_file, model_state))
    return false;

  // Quick peek: if this is a JSON ISD, we know the model name without
  // expensive trial-and-error. Construct only the matching model.
  std::string isd_model_name;
  if (isUsgsCsmIsd(model_state, isd_model_name)) {
    std::cout << "Detected JSON ISD with model: " << isd_model_name << "\n";
    csm::Isd isd(model_file);
    UsgsAstroPlugin cameraPlugin;
    csm::Model *csm = cameraPlugin.constructModelFromISD(isd, isd_model_name, NULL);
    if (!csm) {
      std::cerr << "Failed to construct model from ISD: " << model_file << ".\n";
      return false;
    }
    model = std::shared_ptr<csm::RasterGM>(dynamic_cast<csm::RasterGM*>(csm));
    std::cout << "Loaded a CSM model of type " << isd_model_name
              << " from ISD file " << model_file << ".\n";
    return true;
  }

  // Quick peek for model state (JSON state or .sup)
  std::string state_model_name;
  if (isUsgsCsmState(model_state, state_model_name)) {
    std::cout << "Detected model state with model: " << state_model_name << "\n";
    UsgsAstroPlugin cameraPlugin;
    csm::Model *csm = cameraPlugin.constructModelFromState(model_state, NULL);
    if (!csm) {
      std::cerr << "Failed to construct model from state: " << model_file << ".\n";
      return false;
    }
    model = std::shared_ptr<csm::RasterGM>(dynamic_cast<csm::RasterGM*>(csm));
    std::cout << "Loaded a CSM model of type " << state_model_name
              << " from model state file " << model_file << ".\n";
    return true;
  }

  std::cerr << "Failed to load a CSM model from: " << model_file << ".\n";
  return false;
}

bool updateSupModel(std::string& sup_string, std::string model) {
  // grab the state JSON out of the original sup file to determine length of string to replace
  std::string sup_state = stateAsJson(sup_string).dump().c_str();

  // add back in the BEL characters in json state for GXP to read .sup file
  std::replace(model.begin(), model.end(), '\n', '\a');

  // update the .sup file SENSOR_STATE_LENGTH with new state length
  std::string sensor_length = "SENSOR_STATE_LENGTH ";
  size_t start_len_pos = sup_string.find(sensor_length) + sensor_length.length();
  size_t end_len_pos = sup_string.find("SENSOR_STATE ") - 1;
  sup_string.replace(start_len_pos, end_len_pos - start_len_pos, std::to_string(model.length()));

  //replace the camera state in .sup file at the state model name which start with "USGS_ASTRO"
  size_t start_pos = sup_string.find("USGS_ASTRO");
  std::size_t end_pos = sup_string.find_last_of("}");

  if(start_pos == std::string::npos)
    return false;

  sup_string.replace(start_pos, (end_pos - start_pos) + 1, model);
  return true;
}


int main(int argc, char **argv) {

  Options opt;
  if (!parseOptions(argc, argv, opt))
    return 1;

  // Keep the model as smart pointer to the class from which the
  // specific model types inherit.
  std::shared_ptr<csm::RasterGM> model;

  if (!loadCsmCameraModel(opt.model, model, opt))
    return 1;

  if (opt.output_model_state != "") {
    std::cout << "Writing model state: " << opt.output_model_state << "\n";
    std::ofstream ofs(opt.output_model_state.c_str());
    ofs << model->getModelState() << "\n";
    ofs.close();
  }

  if (opt.output_binary_state != "") {
    std::cout << "Writing model state in msgpack binary format: "
              << opt.output_binary_state << "\n";
    nlohmann::json j = json::parse(getUsgsCsmModelJson(model.get()));
    std::vector<uint8_t> msgpack_data = nlohmann::json::to_msgpack(j);
    std::ofstream ofs(opt.output_binary_state, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(msgpack_data.data()), msgpack_data.size());
    ofs.close();
    std::cout << "Binary model state size: " << msgpack_data.size() << " bytes\n";
  }

  if (opt.modify_sup_file != "") {
    std::string sup_string;
    readFileInString(opt.modify_sup_file, sup_string);

    if (!updateSupModel(sup_string, model->getModelState()))
      return 1;

    std::cout << "Updating model state for sup file: " << opt.modify_sup_file << "\n";
    std::ofstream ofs_sup(opt.modify_sup_file.c_str());
    ofs_sup << sup_string << "\n";
    ofs_sup.close();
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
