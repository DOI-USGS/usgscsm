#ifndef MdisNacSensorModelTest_h
#define MdisNacSensorModelTest_h

#include <sstream>
#include <string>

#include <csm/csm.h>
#include <csm/Isd.h>
#include <csm/Model.h>

#include <MdisNacSensorModel.h>
#include <MdisPlugin.h>

// runTest.cpp defines this global string to a data directory
extern std::string g_dataPath;

/**
 * Sub-class MdisNacSensorModel to get test its protected linear algebra methods.
 *
 * We should be testing the protected methods of MdisNacSensorModel since imageToGround
 * depends on intersect, which depends on project, etc.
 */
class TestableMdisNacSensorModel : public MdisNacSensorModel {
  // Give linear algebra methods public accessing when using instances of this class.
  public:
    using MdisNacSensorModel::setFocalPlane;
};


// Set up a fixture (i.e. objects we can use throughout test)
class MdisNacSensorModelTest : public ::testing::Test {
  protected:

    // Per test-case setup and teardown (e.g. once for this MdisNacSensorModelTest)
    static void SetUpTestCase() {
      if (g_dataPath != "") {
        std::cout << g_dataPath << std::endl;
        dataFile = g_dataPath + "/EN1007907102M.json";
        std::cout << "dataFile: " << dataFile << std::endl;
      }
      isd = readISD(dataFile);
      
      // Make sure the isd was read correctly.
      if (isd == nullptr) {
        setupFixtureFailed = true;
        std::stringstream ss;
        ss << "Could not create isd from file: " << dataFile << "\nError: " << strerror(errno);
        setupFixtureError = ss.str();
        return;
      }
      
      // printISD(*isd);

      // Create a model from the ISD so we can test a valid image.
      std::string modelName = MdisNacSensorModel::_SENSOR_MODEL_NAME;
      csm::Model *validModel = mdisPlugin.constructModelFromISD(*isd, modelName);

      // We could static_cast, but may be hard to debug if it doesn't correctly cast.
      mdisModel = dynamic_cast<MdisNacSensorModel *>(validModel);
      std::cout << "Construction model: " << mdisModel << "\n";

      // Fatal failure if the downcast doesn't work
      if (mdisModel == nullptr) {
        setupFixtureFailed = true;
        setupFixtureError = "Could not downcast Model* to MdisNacSensorModel*.";
        return;
      }
    }

    static void TearDownTestCase() {
      delete isd;
      delete mdisModel;
    }

    static bool setupFixtureFailed;       // Work-around gtest issue #247.
    static std::string setupFixtureError; // ^
    static csm::Isd *isd;                 // ISD converted from JSON to use for creating model.
    static std::string dataFile;          // JSON data file to be converted to ISD for testing.
    static MdisPlugin mdisPlugin;         // Plugin used to create a model from ISD.
    static MdisNacSensorModel *mdisModel; // MDIS-NAC sensor model created with ISD.


    // Per test setup and teardown (e.g. each TEST_F)
    virtual void SetUp() {
      tolerance = 0.00001;
    }

    virtual void TearDown() {}

    double tolerance;                     // Tolerance to be used for double comparison.
    MdisNacSensorModel defaultMdisNac;    // A default constructed MdisNacSensorModel.
    TestableMdisNacSensorModel testMath;  // Subclassed MdisNacSensorModel for protected methods.
};

#endif
