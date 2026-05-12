/**
 * Simple logging implementation for USGSCSM
 */

#ifndef USGSCSM_LOGGING_H_
#define USGSCSM_LOGGING_H_

#include <iostream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <iomanip>

#ifndef PROJECT_NAME
#define PROJECT_NAME "USGSCSM"
#endif

namespace usgscsm {
namespace logger {
    enum LogLevel { TRACE=0, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF };
    static inline LogLevel current_log_level = INFO;

    static constexpr const char* LOG_LEVEL_STRINGS[] = {
        "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "CRITICAL", "OFF"
    };

    /**
     * @brief Base implementation for logging a single value
     * @param os Output stream to write to
     * @param first Value to log
     */
    template<typename T>
    void log_impl(std::ostream& os, const T& first) {
        os << first;
    }

    /**
     * @brief Recursive implementation for logging multiple values
     * @param os Output stream to write to
     * @param first First value to log
     * @param args Remaining values to log
     */
    template<typename T, typename... Args>
    void log_impl(std::ostream& os, const T& first, const Args&... args) {
        os << first;
        log_impl(os, args...);
    }

    /**
     * @brief Internal logging function that formats and outputs log messages
     * @param level Log level of the message
     * @param line Source code line number
     * @param func Function name
     * @param args Values to log
     */
    template<typename... Args>
    void log_internal(LogLevel level, int line, const char* func, const Args&... args) {
        if (level >= current_log_level) {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);

            std::stringstream ss;
            ss << "[" << PROJECT_NAME << "][" << LOG_LEVEL_STRINGS[level] << "]"
               << "[" << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << "]"
               << "[" << func << ":" << line << "] ";
            log_impl(ss, args...);
            ss << std::endl;

            std::cerr << ss.str();
        }
    }

    /**
     * @brief Sets the current log level
     * @param level New log level to use
     */
    inline void set_log_level(LogLevel level) {
        current_log_level = level;
    }

    /**
     * @brief Gets the current log level
     * @return Current log level
     */
    inline LogLevel get_log_level() {
        return current_log_level;
    }

    /**
     * @brief Parse log level from string (case-insensitive)
     * @param level_str String representation of log level
     * @return Parsed log level, defaults to INFO if invalid
     */
    inline LogLevel level_from_string(const std::string& level_str) {
        std::string lower = level_str;
        for (auto& c : lower) c = std::tolower(c);

        if (lower == "trace") return TRACE;
        if (lower == "debug") return DEBUG;
        if (lower == "info") return INFO;
        if (lower == "warn" || lower == "warning") return WARN;
        if (lower == "err" || lower == "error") return ERROR;
        if (lower == "critical") return CRITICAL;
        if (lower == "off") return OFF;

        return INFO; // Default
    }
}
}

// Logging macros for different severity levels
#define MESSAGE_LOG(level, ...) \
    do { \
        if (level >= usgscsm::logger::current_log_level) { \
            usgscsm::logger::log_internal(level, __LINE__, __func__, __VA_ARGS__); \
        } \
    } while(0)

#define LOG_TRACE(...) MESSAGE_LOG(usgscsm::logger::TRACE, __VA_ARGS__)
#define LOG_DEBUG(...) MESSAGE_LOG(usgscsm::logger::DEBUG, __VA_ARGS__)
#define LOG_INFO(...)  MESSAGE_LOG(usgscsm::logger::INFO, __VA_ARGS__)
#define LOG_WARN(...)  MESSAGE_LOG(usgscsm::logger::WARN, __VA_ARGS__)
#define LOG_ERROR(...) MESSAGE_LOG(usgscsm::logger::ERROR, __VA_ARGS__)
#define LOG_CRITICAL(...) MESSAGE_LOG(usgscsm::logger::CRITICAL, __VA_ARGS__)

#endif  // USGSCSM_LOGGING_H_
