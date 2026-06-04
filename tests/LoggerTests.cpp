/**
 * Tests for usgscsm_logger - Simple SPDLOG replacement
 */

#include "usgscsm/usgscsm_logger.h"

#include <gtest/gtest.h>
#include <sstream>
#include <fstream>

TEST(LoggerTests, LogLevelFromString) {
    EXPECT_EQ(spdlog::from_str("trace"), spdlog::level::trace);
    EXPECT_EQ(spdlog::from_str("debug"), spdlog::level::debug);
    EXPECT_EQ(spdlog::from_str("info"), spdlog::level::info);
    EXPECT_EQ(spdlog::from_str("warn"), spdlog::level::warn);
    EXPECT_EQ(spdlog::from_str("error"), spdlog::level::err);
    EXPECT_EQ(spdlog::from_str("critical"), spdlog::level::critical);
    EXPECT_EQ(spdlog::from_str("off"), spdlog::level::off);

    // Case insensitive
    EXPECT_EQ(spdlog::from_str("DEBUG"), spdlog::level::debug);
    EXPECT_EQ(spdlog::from_str("INFO"), spdlog::level::info);

    // Default
    EXPECT_EQ(spdlog::from_str("unknown"), spdlog::level::info);
}

TEST(LoggerTests, CreateAndGetLogger) {
    auto logger = spdlog::stdout_color_mt("test_logger");
    ASSERT_NE(logger, nullptr);

    auto retrieved = spdlog::get("test_logger");
    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(logger, retrieved);

    spdlog::drop("test_logger");
    auto after_drop = spdlog::get("test_logger");
    EXPECT_EQ(after_drop, nullptr);
}

TEST(LoggerTests, LoggingLevels) {
    std::ostringstream oss;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(oss);
    auto logger = spdlog::sinks::create_ostream_logger("level_test", sink);

    logger->set_level(spdlog::level::warn);

    logger->log(spdlog::level::debug, "debug message");
    EXPECT_TRUE(oss.str().empty()); // Below threshold, should not log

    logger->log(spdlog::level::warn, "warn message");
    EXPECT_FALSE(oss.str().empty()); // At threshold, should log
    EXPECT_NE(oss.str().find("warn message"), std::string::npos);

    oss.str("");
    logger->log(spdlog::level::err, "error message");
    EXPECT_FALSE(oss.str().empty()); // Above threshold, should log
    EXPECT_NE(oss.str().find("error message"), std::string::npos);

    spdlog::drop("level_test");
}

TEST(LoggerTests, FormatStringSimple) {
    std::ostringstream oss;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(oss);
    auto logger = spdlog::sinks::create_ostream_logger("format_test", sink);
    logger->set_level(spdlog::level::info);

    logger->log(spdlog::level::info, "Value: {}", 42);
    EXPECT_NE(oss.str().find("Value: 42"), std::string::npos);

    oss.str("");
    logger->log(spdlog::level::info, "String: {}", "test");
    EXPECT_NE(oss.str().find("String: test"), std::string::npos);

    spdlog::drop("format_test");
}

TEST(LoggerTests, FormatStringMultipleArgs) {
    std::ostringstream oss;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(oss);
    auto logger = spdlog::sinks::create_ostream_logger("multi_format_test", sink);
    logger->set_level(spdlog::level::info);

    logger->log(spdlog::level::info, "Values: {}, {}, {}", 1, 2.5, "three");
    EXPECT_NE(oss.str().find("Values: 1, 2.5, three"), std::string::npos);

    spdlog::drop("multi_format_test");
}

TEST(LoggerTests, FileOutput) {
    const char* test_file = "/tmp/usgscsm_logger_test.log";

    // Clean up any existing file
    std::remove(test_file);

    auto logger = spdlog::basic_logger_mt("file_logger", test_file);
    logger->set_level(spdlog::level::debug);
    logger->log(spdlog::level::info, "Test message to file");

    spdlog::drop("file_logger");

    // Verify file was created and contains message
    std::ifstream infile(test_file);
    ASSERT_TRUE(infile.is_open());

    std::string content((std::istreambuf_iterator<char>(infile)),
                        std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("Test message to file"), std::string::npos);
    EXPECT_NE(content.find("INFO"), std::string::npos);

    infile.close();
    std::remove(test_file);
}

TEST(LoggerTests, StdoutLogger) {
    // Just verify it creates without crashing
    auto logger = spdlog::stdout_color_mt("stdout_test");
    ASSERT_NE(logger, nullptr);
    logger->set_level(spdlog::level::info);

    // This will output to actual stdout, but won't fail the test
    logger->log(spdlog::level::info, "Stdout test");

    spdlog::drop("stdout_test");
}

TEST(LoggerTests, StderrLogger) {
    // Just verify it creates without crashing
    auto logger = spdlog::stderr_color_mt("stderr_test");
    ASSERT_NE(logger, nullptr);
    logger->set_level(spdlog::level::err);

    // This will output to actual stderr, but won't fail the test
    logger->log(spdlog::level::err, "Stderr test");

    spdlog::drop("stderr_test");
}

TEST(LoggerTests, OstreamSink) {
    std::ostringstream oss;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(oss);
    ASSERT_NE(sink, nullptr);

    auto logger = spdlog::sinks::create_ostream_logger("ostream_test", sink);
    ASSERT_NE(logger, nullptr);

    logger->set_level(spdlog::level::trace);
    logger->log(spdlog::level::trace, "Trace message");

    EXPECT_FALSE(oss.str().empty());
    EXPECT_NE(oss.str().find("Trace message"), std::string::npos);
    EXPECT_NE(oss.str().find("TRACE"), std::string::npos);

    spdlog::drop("ostream_test");
}

TEST(LoggerTests, LoggerReuse) {
    auto logger1 = spdlog::stdout_color_mt("reuse_test");
    ASSERT_NE(logger1, nullptr);

    // Getting again should return the same logger
    auto logger2 = spdlog::get("reuse_test");
    EXPECT_EQ(logger1, logger2);

    spdlog::drop("reuse_test");
}
