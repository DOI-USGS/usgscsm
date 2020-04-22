#define _USE_MATH_DEFINES

#include "UsgsAstroSarSensorModel.h"

#include "Warning.h"

#include <json/json.hpp>
#include <gtest/gtest.h>

#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

TEST(SarTests, stateFromIsd) {
  std::ifstream isdFile("data/orbitalSar.json");
  json isdJson;
  isdFile >> isdJson;
  std::string isdString = isdJson.dump();
  csm::WarningList warnings;
  std::string stateString;
  try {
    stateString = UsgsAstroSarSensorModel::constructStateFromIsd(isdString, &warnings);
  }
  catch(...) {
    for (auto &warn: warnings) {
      std::cerr << "Warning in " << warn.getFunction() << std::endl;
      std::cerr << "  " << warn.getMessage() << std::endl;
    }
    FAIL() << "constructStateFromIsd errored";
  }
  EXPECT_TRUE(warnings.empty());
}


