#include <string>

#include <gtest/gtest.h>

// Set up a global path to a test data directory
std::string g_dataPath;

// Run the tests (see CMakeLists.txt)
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // If we have an argument, this is for the test data directory
  // Root CMakeLists.txt defines the test data directory
  if (argc == 2) {
    g_dataPath = argv[1];
  }
  return RUN_ALL_TESTS();
}
