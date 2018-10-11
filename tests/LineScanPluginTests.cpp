#include "UsgsAstroLsPlugin.h"
#include "UsgsAstroLsSensorModel.h"

#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(LineScanPluginTests, PluginName) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ("USGS_ASTRO_LINE_SCANNER_PLUGIN", testPlugin.getPluginName());;
}

TEST(LineScanPluginTests, ManufacturerName) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ("BAE_SYSTEMS_GXP", testPlugin.getManufacturer());;
}

TEST(LineScanPluginTests, ReleaseDate) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ("20171230", testPlugin.getReleaseDate());;
}

TEST(LineScanPluginTests, NumModels) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ(1, testPlugin.getNumModels());;
}
