#include "Utilities.h"

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(MetricConversion, DistanceConversion) {
  EXPECT_EQ(1,     metric_conversion(1000, "m", "km"));
  EXPECT_EQ(1000,  metric_conversion(1000, "m", "m"));
  EXPECT_EQ(1000,  metric_conversion(1, "km", "m"));
  EXPECT_EQ(1,     metric_conversion(1, "km", "km"));
  EXPECT_EQ(0,     metric_conversion(0, "m", "km"));
  EXPECT_EQ(0,     metric_conversion(0, "m", "m"));
  EXPECT_EQ(0,     metric_conversion(0, "km", "m"));
  EXPECT_EQ(0,     metric_conversion(0, "km", "km"));
  EXPECT_EQ(-1,    metric_conversion(-1000, "m", "km"));
  EXPECT_EQ(-1000, metric_conversion(-1000, "m", "m"));
  EXPECT_EQ(-1000, metric_conversion(-1, "km", "m"));
  EXPECT_EQ(-1,    metric_conversion(-1, "km", "km"));
}

TEST(ISDParsing, ModelName) {
  json isd = {
    {"name_model", "Test"}
  };
  EXPECT_EQ("Test", getSensorModelName(isd));
}

TEST(ISDParsing, ImageIdentifier) {
  json isd = {
    {"image_identifier", "Test"}
  };
  EXPECT_EQ("Test", getImageId(isd));
}

TEST(ISDParsing, SensorName) {
  json isd = {
    {"name_sensor", "Test"}
  };
  EXPECT_EQ("Test", getSensorName(isd));
}

TEST(ISDParsing, PlatformName) {
  json isd = {
    {"name_platform", "Test"}
  };
  EXPECT_EQ("Test", getPlatformName(isd));
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

TEST(ISDParsing, DetectorCenterLine) {
  json isd = {
    {"detector_center", {
      {"line", 2}}
    }
  };
  EXPECT_EQ(2, getDetectorCenterLine(isd));
}

TEST(ISDParsing, DetectorCenterSample) {
  json isd = {
    {"detector_center", {
      {"sample", 3}}
    }
  };
  EXPECT_EQ(3, getDetectorCenterSample(isd));
}

TEST(ISDParsing, DetectorStartingLine) {
  json isd = {
    {"starting_detector_line", 1}
  };
  EXPECT_EQ(1, getDetectorStartingLine(isd));
}

TEST(ISDParsing, DetectorStartingSample) {
  json isd = {
    {"starting_detector_sample", 2}
  };
  EXPECT_EQ(2, getDetectorStartingSample(isd));
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

TEST(ISDParsing, TransverseDistortion) {
  json isd = {
    {"optical_distortion", {
      {"transverse", {
        {"y", {-11, 21, 24}},
        {"x", {-1, 2, 4}}}}
      }
    }
  };
  std::vector<double> coefficients = {-1, 2, 4, 0, 0, 0, 0, 0, 0, 0,
                                       -11, 21, 24, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_EQ(coefficients, getDistortionCoeffs(isd));
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
  EXPECT_EQ(coefficients, getDistortionCoeffs(isd));
}

TEST(ISDParsing, SunPosition) {
  json isd = {
    {"sun_position", {
      {"positions", {
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8}}},
      {"unit", "km"}}
    }
  };
  std::vector<double> positions = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
  EXPECT_EQ(positions, getSunPositions(isd));
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

TEST(ISDParsing, getExposureDuration) {
  json isd = {{"line_exposure_duration", 0.45}};
  EXPECT_EQ(getExposureDuration(isd), 0.45);
}

TEST(ISDParsing, getScaledPixelWidth) {
  json isd = {{"scaled_pixel_width", 7.5}};
  EXPECT_EQ(getScaledPixelWidth(isd), 7.5);
}

TEST(ISDParsing, getScaleConversionCoefficients) {
  json isd = {{"range_conversion_coefficients",
               {{300, 1, 0.1, 0.01},
                {400, 2, 0.2, 0.02},
                {500, 3, 0.3, 0.03}}}};
  std::vector<double> coefficients = getScaleConversionCoefficients(isd);
  ASSERT_EQ(coefficients.size(), 12);
  EXPECT_EQ(coefficients[0], 300);
  EXPECT_EQ(coefficients[1], 1);
  EXPECT_EQ(coefficients[2], 0.1);
  EXPECT_EQ(coefficients[3], 0.01);
  EXPECT_EQ(coefficients[4], 400);
  EXPECT_EQ(coefficients[5], 2);
  EXPECT_EQ(coefficients[6], 0.2);
  EXPECT_EQ(coefficients[7], 0.02);
  EXPECT_EQ(coefficients[8], 500);
  EXPECT_EQ(coefficients[9], 3);
  EXPECT_EQ(coefficients[10], 0.3);
  EXPECT_EQ(coefficients[11], 0.03);
}

TEST(ISDParsing, getScaleConversionTimes) {
  json isd = {{"range_conversion_times",
               {100, 200, 300, -400}}};
  std::vector<double> times = getScaleConversionTimes(isd);
  ASSERT_EQ(times.size(), 4);
  EXPECT_EQ(times[0], 100);
  EXPECT_EQ(times[1], 200);
  EXPECT_EQ(times[2], 300);
  EXPECT_EQ(times[3], -400);
}
