#include "UsgsAstroFramePlugin.h"

#include <gtest/gtest.h>

TEST(SimpleTests, MyFirstTest) {
   EXPECT_EQ(0, 0);
}

TEST(SimpleTests, MySecondTest) {
   EXPECT_EQ(0, 1);
}

TEST(FramePluginTests, Name) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_EQ("UsgsAstroFramePluginCSM", testPlugin.getPluginName());;
}

int main(int argc, char **argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
