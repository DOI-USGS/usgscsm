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
    static inline LogLevel current_log_level = OFF;

    static constexpr const char* LOG_LEVEL_STRINGS[] = {
        "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "CRITICAL", "OFF"
    };

    /**
     * @brief Helper to convert value to string for formatting
     */
    template<typename T>
    std::string to_string_helper(const T& value) {
        std::ostringstream oss;
        oss << value;
        return oss.str();
    }

    // Specialization for string types to avoid extra quotes
    inline std::string to_string_helper(const std::string& value) {
        return value;
    }

    inline std::string to_string_helper(const char* value) {
        return std::string(value);
    }

    /**
     * @brief Base case: no more arguments to substitute
     */
    inline void format_impl(std::ostream& os, const char* fmt) {
        // Output any remaining format string
        while (*fmt) {
            if (*fmt == '{' && *(fmt + 1) == '}') {
                // Found placeholder but no more args - just output it literally
                os << "{}";
                fmt += 2;
            } else {
                os << *fmt++;
            }
        }
    }

    /**
     * @brief Recursive implementation for format string with {} placeholders
     * @param os Output stream to write to
     * @param fmt Format string with {} placeholders
     * @param first First value to substitute
     * @param args Remaining values to substitute
     */
    template<typename T, typename... Args>
    void format_impl(std::ostream& os, const char* fmt, const T& first, const Args&... args) {
        while (*fmt) {
            if (*fmt == '{' && *(fmt + 1) == '}') {
                // Found placeholder - substitute the value
                os << to_string_helper(first);
                // Continue with remaining args
                format_impl(os, fmt + 2, args...);
                return;
            } else {
                os << *fmt++;
            }
        }
        // If we get here, format string ended but we still have args - just ignore them
    }

    /**
     * @brief Internal logging function that formats and outputs log messages
     * Supports both legacy concatenation and {} placeholder formatting
     * @param level Log level of the message
     * @param line Source code line number
     * @param func Function name
     * @param fmt Format string (can contain {} placeholders)
     * @param args Values to substitute or concatenate
     */
    template<typename... Args>
    void log_internal(LogLevel level, int line, const char* func, const char* fmt, const Args&... args) {
        if (level >= current_log_level) {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            std::tm tm_buf;

#ifdef _WIN32
            localtime_s(&tm_buf, &time);
#else
            localtime_r(&time, &tm_buf);
#endif

            std::stringstream ss;
            ss << "[" << PROJECT_NAME << "][" << LOG_LEVEL_STRINGS[level] << "]"
               << "[" << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "]"
               << "[" << func << ":" << line << "] ";

            format_impl(ss, fmt, args...);
            ss << std::endl;

            std::cerr << ss.str();
        }
    }

    /**
     * @brief Overload for single string (no formatting)
     */
    inline void log_internal(LogLevel level, int line, const char* func, const char* msg) {
        if (level >= current_log_level) {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            std::tm tm_buf;

#ifdef _WIN32
            localtime_s(&tm_buf, &time);
#else
            localtime_r(&time, &tm_buf);
#endif

            std::stringstream ss;
            ss << "[" << PROJECT_NAME << "][" << LOG_LEVEL_STRINGS[level] << "]"
               << "[" << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "]"
               << "[" << func << ":" << line << "] ";
            ss << msg << std::endl;

            std::cerr << ss.str();
        }
    }

    /**
     * @brief Overload for std::string (no formatting)
     */
    inline void log_internal(LogLevel level, int line, const char* func, const std::string& msg) {
        log_internal(level, line, func, msg.c_str());
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
