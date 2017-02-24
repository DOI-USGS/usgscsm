#include <MdisPlugin.h>
#include <MdisNacSensorModel.h>

#include <csm/Error.h>
#include <csm/Isd.h>
#include <csm/Model.h>
#include <csm/Version.h>

#include <gtest/gtest.h>

class MdisPluginTest : public ::testing::Test {

  protected:
    virtual void SetUp() {
      mdisNacName = MdisNacSensorModel::_SENSOR_MODEL_NAME;
    }

    MdisPlugin defaultMdisPlugin;

    std::string mdisNacName;

};

TEST_F(MdisPluginTest, getPluginName) {
  EXPECT_EQ(defaultMdisPlugin.getPluginName(), "UsgsAstroFrameMdisPluginCSM");
}

TEST_F(MdisPluginTest, getManufacturer) {
  EXPECT_EQ(defaultMdisPlugin.getManufacturer(), "UsgsAstrogeology");
}

TEST_F(MdisPluginTest, getReleaseDate) {
  EXPECT_EQ(defaultMdisPlugin.getReleaseDate(), "TBA");
}

TEST_F(MdisPluginTest, getCsmVersion) {
  const csm::Version v(3, 1, 0);
  // const discard qualifier if we don't use the version() method on our csm::Version's
  EXPECT_EQ(defaultMdisPlugin.getCsmVersion().version(), v.version());
}

TEST_F(MdisPluginTest, getNumModels) {
  EXPECT_EQ(defaultMdisPlugin.getNumModels(), 1);
}

TEST_F(MdisPluginTest, getModelName) {
  EXPECT_EQ(defaultMdisPlugin.getModelName(0), mdisNacName);
}

TEST_F(MdisPluginTest, getModelFamily) {
  EXPECT_EQ(defaultMdisPlugin.getModelFamily(0), "Raster");
}

TEST_F(MdisPluginTest, getModelVersion) {
  const csm::Version v(1, 0, 0);
  // const discard qualifier if we don't use the version() method on our csm::Version's
  EXPECT_EQ(defaultMdisPlugin.getModelVersion("blah").version(), v.version());
}

TEST_F(MdisPluginTest, constructModelFromISD) {
  // Empty (i.e. invalid) ISD
  EXPECT_THROW({
    csm::Isd emptyIsd;
    csm::Model *model = defaultMdisPlugin.constructModelFromISD(emptyIsd, mdisNacName);
  },
  csm::Error);

  // ISD is not supported (wrong sensor model name)
  EXPECT_THROW({
    csm::Isd catSensor;
    csm::Model *model = defaultMdisPlugin.constructModelFromISD(catSensor, "catCamera");
  },
  csm::Error);
}
