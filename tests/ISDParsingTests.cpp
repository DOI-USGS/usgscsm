#include "Utilities.h"

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(MetricConversion, DistanceConversion) {
  EXPECT_EQ(1,    metric_conversion(1000, "m", "km"));
  EXPECT_EQ(1000, metric_conversion(1000, "m", "m"));
  EXPECT_EQ(1000, metric_conversion(1, "km", "m"));
  EXPECT_EQ(1,    metric_conversion(1, "km", "km"));
}
