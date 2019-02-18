#include "Utilities.h"

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(MetricConversion, DistanceConversion) {
  EXPECT_EQ(1,    metric_conversion(1000, "m", "km"));
  EXPECT_EQ(1000, metric_conversion(1000, "m", "m"));
  EXPECT_EQ(1000, metric_conversion(1, "km", "m"));
  EXPECT_EQ(1,    metric_conversion(1, "km", "km"));
}

TEST(ISDParsing, TotalLines) {
  json isd = {
    {"image_lines", 16}
  };
  EXPECT_EQ(16, getTotalLines(isd));
}

TEST(ISDParsing, TotalSamples) {
  json isd = {
    {"image_samples", 16}
  };
  EXPECT_EQ(16, getTotalSamples(isd));
}

TEST(ISDParsing, StartTime) {
  json isd = {
    {"starting_ephemeris_time", -10}
  };
  EXPECT_EQ(-10, getStartingTime(isd));
}

TEST(ISDParsing, CenterTime) {
  json isd = {
    {"center_ephemeris_time", -10}
  };
  EXPECT_EQ(-10, getCenterTime(isd));
}

TEST(ISDParsing, IntegrationStartLines) {
  json isd = {
    {"line_scan_rate", {
      {0, 1, 2},
      {3, 4, 5},
      {6, 7, 8}}
    }
  };
  std::vector<double> startLines = {0, 3, 6};
  EXPECT_EQ(startLines, getIntegrationStartLines(isd));
}

TEST(ISDParsing, IntegrationStartTimes) {
  json isd = {
    {"line_scan_rate", {
      {0, 1, 2},
      {3, 4, 5},
      {6, 7, 8}}
    }
  };
  std::vector<double> startTimes = {1, 4, 7};
  EXPECT_EQ(startTimes, getIntegrationStartTimes(isd));
}

TEST(ISDParsing, IntegrationTimes) {
  json isd = {
    {"line_scan_rate", {
      {0, 1, 2},
      {3, 4, 5},
      {6, 7, 8}}
    }
  };
  std::vector<double> times = {2, 5, 8};
  EXPECT_EQ(times, getIntegrationTimes(isd));
}

TEST(ISDParsing, SampleSumming) {
  json isd = {
    {"detector_sample_summing", 4}
  };
  EXPECT_EQ(4, getSampleSumming(isd));
}

TEST(ISDParsing, LineSumming) {
  json isd = {
    {"detector_line_summing", 4}
  };
  EXPECT_EQ(4, getLineSumming(isd));
}

TEST(ISDParsing, FocalLength) {
  json isd = {
    {"focal_length_model", {
      {"focal_length", 2}}
    }
  };
  EXPECT_EQ(2, getFocalLength(isd));
}

TEST(ISDParsing, FocalLengthEpsilon) {
  json isd = {
    {"focal_length_model", {
      {"focal_epsilon", 0.1}}
    }
  };
  EXPECT_EQ(0.1, getFocalLengthEpsilon(isd));
}

TEST(ISDParsing, Focal2PixelLines) {
  json isd = {
    {"focal2pixel_lines", {0, 1, 2}}
  };
  std::vector<double> coefficients = {0, 1, 2};
  EXPECT_EQ(coefficients, getFocal2PixelLines(isd));
}

TEST(ISDParsing, Focal2PixelSamples) {
  json isd = {
    {"focal2pixel_samples", {0, 1, 2}}
  };
  std::vector<double> coefficients = {0, 1, 2};
  EXPECT_EQ(coefficients, getFocal2PixelSamples(isd));
}

TEST(ISDParsing, DetectorCenter) {
  json isd = {
    {"detector_center", {
      {"line", 2},
      {"sample", 3}}
    }
  };
  csm::ImageCoord value = getDetectorCenter(isd);
  EXPECT_EQ(2, value.line);
  EXPECT_EQ(3, value.samp);
}

TEST(ISDParsing, MinHeight) {
  json isd = {
    {"reference_height", {
      {"minheight", -1},
      {"unit", "km"}}
    }
  };
  EXPECT_EQ(-1000, getMinHeight(isd));
}

TEST(ISDParsing, MaxHeight) {
  json isd = {
    {"reference_height", {
      {"maxheight", 10},
      {"unit", "km"}}
    }
  };
  EXPECT_EQ(10000, getMaxHeight(isd));
}

TEST(ISDParsing, SemiMajor) {
  json isd = {
    {"radii", {
      {"semimajor", 2},
      {"unit", "km"}}
    }
  };
  EXPECT_EQ(2000, getSemiMajorRadius(isd));
}

TEST(ISDParsing, SemiMinor) {
  json isd = {
    {"radii", {
      {"semiminor", 1},
      {"unit", "km"}}
    }
  };
  EXPECT_EQ(1000, getSemiMinorRadius(isd));
}

TEST(ISDParsing, TransverseX) {
  json isd = {
    {"optical_distortion", {
      {"transverse", {
        {"x", {-1, 2, 4}}}}
      }
    }
  };
  std::vector<double> coefficients = {-1, 2, 4, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_EQ(coefficients, getTransverseDistortionX(isd));
}

TEST(ISDParsing, TransverseY) {
  json isd = {
    {"optical_distortion", {
      {"transverse", {
        {"y", {-11, 21, 24, 16, 20}}}}
      }
    }
  };
  std::vector<double> coefficients = {-11, 21, 24, 16, 20, 0, 0, 0, 0, 0};
  EXPECT_EQ(coefficients, getTransverseDistortionY(isd));
}

TEST(ISDParsing, Radial) {
  json isd = {
    {"optical_distortion", {
      {"radial", {
        {"coefficients", {0, 1, 2}}}}
      }
    }
  };
  std::vector<double> coefficients = {0, 1, 2};
  EXPECT_EQ(coefficients, getRadialDistortion(isd));
}

TEST(ISDParsing, SensorPosition) {
  json isd = {
    {"sensor_position", {
      {"positions", {
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8}}},
      {"unit", "km"}}
    }
  };
  std::vector<double> positions = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
  EXPECT_EQ(positions, getSensorPositions(isd));
}

TEST(ISDParsing, SensorVelocities) {
  json isd = {
    {"sensor_position", {
      {"velocities", {
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8}}},
      {"unit", "km"}}
    }
  };
  std::vector<double> velocity = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
  EXPECT_EQ(velocity, getSensorVelocities(isd));
}

TEST(ISDParsing, SensorOrientations) {
  json isd = {
    {"sensor_orientation", {
      {"quaternions", {
        {0, 1, 2, 3},
        {4, 5, 6, 7},
        {8, 9, 10, 11}}}}
    }
  };
  std::vector<double> rotations = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  EXPECT_EQ(rotations, getSensorOrientations(isd));
}