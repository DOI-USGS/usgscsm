// csm_translate: convert a CSM camera model / state between file formats. The
// pivot is always a CSM model. See printUsage() below for the interface.

#include <UsgsAstroPlugin.h>
#include <Utilities.h>

#include <csm/RasterGM.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

void printUsage(const std::string &progName) {
  std::cout <<
"Usage: " << progName << " <from> <to> [options]\n"
"\n"
"Convert a CSM camera model/state between file formats. The input is loaded\n"
"into a CSM model and written back out in the output format. The input format is\n"
"detected from the file's contents; the output format from its extension. Either\n"
"can be overridden.\n"
"\n"
"Formats (name, and the extensions that imply it):\n"
"  json            .json          CSM model state as JSON (a JSON ISD is also\n"
"                                 accepted as input and constructed into a state)\n"
"  msgpack         .isd .msgpack  CSM model state in binary MessagePack\n"
"                  .mp\n"
"  stards          .stards        CSM model state in the STARDS binary format\n"
"                                 (requires a build with STARDS support)\n"
"\n"
"Options:\n"
"  -i, --input-format <fmt>\n"
"                  Override the input format (json, msgpack, stards).\n"
"  -o, --output-format <fmt>\n"
"                  Override the output format (json, msgpack, stards).\n"
"  --set key=val   Set an advanced, format-specific option (repeatable).\n"
"                  STARDS output supports:\n"
"                    compression=<none|gzip|zstd|lz4|gzip-shuffle|lz4-shuffle>\n"
"                    block-size=<bytes>       compressed block size\n"
"                    array-threshold=<count>  values longer than this go to\n"
"                                             array storage, shorter ones to the\n"
"                                             metadata block\n"
"  -h, --help      Show this help message and exit.\n"
"\n"
"Examples:\n"
"  " << progName << " image.json state.msgpack\n"
"  " << progName << " state.json state.stards\n"
"  " << progName << " state.json state.stards --set compression=gzip-shuffle\n"
"  " << progName << " state.stards state.json\n"
;
}

std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}

std::string fileExtension(const std::string &path) {
  size_t slash = path.find_last_of("/\\");
  size_t dot = path.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
    return "";
  }
  return toLower(path.substr(dot + 1));
}

// -i/-o names and file extensions both resolve to ModelFormat, so the two cannot
// disagree about which formats exist. ModelFormat::Text is "json" here: a JSON ISD
// and a JSON model state are told apart by content, not by name. Stards resolves
// even without STARDS support so the error can say so instead of "unknown format".
ModelFormat formatFromName(const std::string &name) {
  if (name == "json") return ModelFormat::Text;
  if (name == "msgpack") return ModelFormat::Msgpack;
  if (name == "stards") return ModelFormat::Stards;
  return ModelFormat::Unknown;
}

ModelFormat formatFromExtension(const std::string &ext) {
  if (ext == "isd" || ext == "mp") return ModelFormat::Msgpack;
  return formatFromName(ext);
}

const char *formatName(ModelFormat format) {
  switch (format) {
    case ModelFormat::Text:    return "json";
    case ModelFormat::Msgpack: return "msgpack";
    case ModelFormat::Stards:  return "stards";
    case ModelFormat::Unknown: break;
  }
  return "unknown";
}

// Takes ownership of m. Returns nullptr, having deleted m, if it is not a raster
// model -- the only kind this tool can write back out.
std::shared_ptr<csm::RasterGM> asRasterGM(csm::Model *m) {
  csm::RasterGM *raster = dynamic_cast<csm::RasterGM *>(m);
  if (m && !raster) {
    std::cerr << "Loaded model is not a raster GM; cannot convert it.\n";
    delete m;
  }
  return std::shared_ptr<csm::RasterGM>(raster);
}

// Returns nullptr on failure. inputFormat picks the reader; an ISD is still
// distinguished from a model state by content.
std::shared_ptr<csm::RasterGM> loadModel(const std::string &path,
                                         ModelFormat inputFormat) {
  if (inputFormat == ModelFormat::Stards) {
#ifdef USGSCSM_ENABLE_STARDS
    return asRasterGM(getUsgsCsmModelFromStards(path, NULL));
#else
    std::cerr << "Reading STARDS requires a build with STARDS support.\n";
    return nullptr;
#endif
  }

  if (inputFormat == ModelFormat::Msgpack) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
      std::cerr << "Could not open input file: " << path << "\n";
      return nullptr;
    }
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());
    const char *dataPtr = reinterpret_cast<const char *>(data.data());
    json j = json::from_msgpack(dataPtr, dataPtr + data.size());
    std::string modelName = j.at("m_modelName").get<std::string>();
    return asRasterGM(getUsgsCsmModelFromJsonState(j.dump(), modelName, NULL));
  }

  // json: either a CSM model state or a JSON ISD (detected by content).
  std::string contents;
  if (!readFileInString(path, contents)) {
    std::cerr << "Could not read input file: " << path << "\n";
    return nullptr;
  }

  std::string modelName;
  UsgsAstroPlugin plugin;
  if (isUsgsCsmIsd(contents, modelName)) {
    csm::Isd isd(path);
    return asRasterGM(plugin.constructModelFromISD(isd, modelName, NULL));
  }
  if (isUsgsCsmState(contents, modelName)) {
    return asRasterGM(plugin.constructModelFromState(contents, NULL));
  }

  std::cerr << "Input file is not a recognized CSM ISD or model state: " << path
            << "\n";
  return nullptr;
}

// Write the model in the requested output format. Returns true on success.
bool writeModel(csm::RasterGM *model, const std::string &path,
                ModelFormat outputFormat,
                const std::map<std::string, std::string> &advanced) {
  if (outputFormat == ModelFormat::Text) {
    std::ofstream ofs(path);
    if (!ofs) {
      std::cerr << "Could not open output file: " << path << "\n";
      return false;
    }
    // getModelState() prefixes the model-name line the JSON reader expects.
    ofs << model->getModelState() << "\n";
    return true;
  }

  if (outputFormat == ModelFormat::Msgpack) {
    // A msgpack map cannot carry the model-name preamble, so the name travels as
    // the m_modelName key instead -- hence the bare-JSON accessor here.
    json j = json::parse(getUsgsCsmModelJson(model));
    std::vector<uint8_t> bytes = json::to_msgpack(j);
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
      std::cerr << "Could not open output file: " << path << "\n";
      return false;
    }
    ofs.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    return true;
  }

  if (outputFormat == ModelFormat::Stards) {
#ifdef USGSCSM_ENABLE_STARDS
    std::string compression = STARDS_DEFAULT_COMPRESSION;
    size_t blockSize = STARDS_DEFAULT_BLOCK_SIZE;
    size_t arrayThreshold = STARDS_DEFAULT_ARRAY_THRESHOLD;
    for (const auto &kv : advanced) {
      if (kv.first == "compression") {
        compression = kv.second;
      } else if (kv.first == "block-size" || kv.first == "block_size") {
        blockSize = std::stoul(kv.second);
      } else if (kv.first == "array-threshold" || kv.first == "array_threshold") {
        arrayThreshold = std::stoul(kv.second);
      } else {
        std::cerr << "Unknown STARDS option '" << kv.first << "'\n";
        return false;
      }
    }
    writeUsgsCsmModelToStards(model, path, compression, blockSize, arrayThreshold);
    return true;
#else
    (void)advanced;
    std::cerr << "Writing STARDS requires a build with STARDS support.\n";
    return false;
#endif
  }

  std::cerr << "Unsupported output format: " << formatName(outputFormat) << "\n";
  return false;
}

}  // namespace

int main(int argc, char **argv) {
  std::string inputPath, outputPath;
  std::string inputFormatOverride, outputFormatOverride;
  std::map<std::string, std::string> advanced;
  std::vector<std::string> positionals;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "-i" || arg == "--input-format") {
      if (i + 1 >= argc) { std::cerr << arg << " requires a value\n"; return 1; }
      inputFormatOverride = toLower(argv[++i]);
    } else if (arg == "-o" || arg == "--output-format") {
      if (i + 1 >= argc) { std::cerr << arg << " requires a value\n"; return 1; }
      outputFormatOverride = toLower(argv[++i]);
    } else if (arg == "--set") {
      if (i + 1 >= argc) { std::cerr << "--set requires key=value\n"; return 1; }
      std::string kv = argv[++i];
      size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        std::cerr << "--set expects key=value, got: " << kv << "\n";
        return 1;
      }
      advanced[toLower(kv.substr(0, eq))] = kv.substr(eq + 1);
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    } else {
      positionals.push_back(arg);
    }
  }

  if (positionals.size() != 2) {
    std::cerr << "Expected exactly two file arguments (from, to).\n";
    printUsage(argv[0]);
    return 1;
  }
  inputPath = positionals[0];
  outputPath = positionals[1];

  // Input: an override wins, else sniff the content -- authoritative for the
  // binary formats and needs no extension. Output: the file does not exist yet,
  // so only the extension is available.
  ModelFormat inputFormat = inputFormatOverride.empty()
                                ? modelFormatOfFile(inputPath)
                                : formatFromName(inputFormatOverride);
  ModelFormat outputFormat = outputFormatOverride.empty()
                                 ? formatFromExtension(fileExtension(outputPath))
                                 : formatFromName(outputFormatOverride);

  if (inputFormat == ModelFormat::Unknown) {
    std::cerr << "Could not determine input format for '" << inputPath
              << "'. Use -i to specify it.\n";
    return 1;
  }
  if (outputFormat == ModelFormat::Unknown) {
    std::cerr << "Could not determine output format for '" << outputPath
              << "'. Use -o to specify it.\n";
    return 1;
  }

  try {
    std::shared_ptr<csm::RasterGM> model = loadModel(inputPath, inputFormat);
    if (!model) {
      std::cerr << "Failed to load model from: " << inputPath << "\n";
      return 1;
    }

    if (!writeModel(model.get(), outputPath, outputFormat, advanced)) {
      std::cerr << "Failed to write: " << outputPath << "\n";
      return 1;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  std::cout << "Converted " << inputPath << " (" << formatName(inputFormat)
            << ") -> " << outputPath << " (" << formatName(outputFormat) << ")\n";
  return 0;
}
