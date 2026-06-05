/**
 * USGSCSM Simple Logger - Single-header SPDLOG replacement
 *
 * Pure macro-based logging with no initialization required.
 * Reads USGSCSM_LOG_FILE and USGSCSM_LOG_LEVEL from environment on first use.
 */

#ifndef USGSCSM_LOGGER_H
#define USGSCSM_LOGGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <mutex>


namespace spdlog {

// Log levels
enum class level {
    trace = 0,
    debug = 1,
    info = 2,
    warn = 3,
    err = 4,
    critical = 5,
    off = 6
};

// Convert string to log level
inline level from_str(const std::string& str) {
    std::string lower = str;
    for (char& c : lower) c = std::tolower(c);

    if (lower == "trace") return level::trace;
    if (lower == "debug") return level::debug;
    if (lower == "info") return level::info;
    if (lower == "warn" || lower == "warning") return level::warn;
    if (lower == "err" || lower == "error") return level::err;
    if (lower == "critical") return level::critical;
    if (lower == "off") return level::off;

    return level::info; // default
}

// Global logging configuration - lazy-initialized
inline std::ostream& get_log_stream() {
    static std::ostream* stream = nullptr;
    static std::ofstream file_stream;
    static std::mutex init_mutex;

    if (stream == nullptr) {
        std::lock_guard<std::mutex> lock(init_mutex);
        if (stream == nullptr) {
            const char* log_file = std::getenv("USGSCSM_LOG_FILE");
            std::string file = log_file ? log_file : "stdout";

            if (file == "stdout") {
                stream = &std::cout;
            } else if (file == "stderr") {
                stream = &std::cerr;
            } else if (!file.empty()) {
                file_stream.open(file, std::ios::app);
                stream = file_stream.is_open() ? &file_stream : &std::cout;
            } else {
                stream = &std::cout;
            }
        }
    }
    return *stream;
}

inline level& get_log_level() {
    static level current_level = level::off;
    static bool initialized = false;
    static std::mutex init_mutex;

    if (!initialized) {
        std::lock_guard<std::mutex> lock(init_mutex);
        if (!initialized) {
            const char* log_level_env = std::getenv("USGSCSM_LOG_LEVEL");
            current_level = log_level_env ? from_str(log_level_env) : level::info;
            initialized = true;
        }
    }
    return current_level;
}

inline std::mutex& get_log_mutex() {
    static std::mutex log_mutex;
    return log_mutex;
}

inline const char* level_string(level lvl) {
    switch (lvl) {
        case level::trace: return "TRACE";
        case level::debug: return "DEBUG";
        case level::info: return "INFO";
        case level::warn: return "WARN";
        case level::err: return "ERROR";
        case level::critical: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

inline std::string timestamp() {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf;

#ifdef _WIN32
    localtime_s(&tm_buf, &now);
#else
    localtime_r(&now, &tm_buf);
#endif

    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_buf);
    return std::string(buf);
}

// Format string helpers
namespace detail {
    inline std::string format_one(const std::string& fmt, const std::string& value) {
        size_t pos = fmt.find("{}");
        if (pos == std::string::npos) return fmt;
        return fmt.substr(0, pos) + value + fmt.substr(pos + 2);
    }

    template<typename T>
    inline std::string to_string(const T& value) {
        std::ostringstream oss;
        oss << value;
        return oss.str();
    }

    inline std::string format_message(const std::string& fmt) {
        return fmt;
    }

    template<typename T, typename... Args>
    inline std::string format_message(const std::string& fmt, const T& value, Args&&... args) {
        return format_message(format_one(fmt, to_string(value)), std::forward<Args>(args)...);
    }
}

// Core logging function
template<typename... Args>
inline void log_message(level lvl, const char* fmt, Args&&... args) {
    if (lvl < get_log_level()) return;

    std::string message = detail::format_message(std::string(fmt), std::forward<Args>(args)...);

    std::lock_guard<std::mutex> lock(get_log_mutex());
    get_log_stream() << "[" << timestamp() << "] [" << level_string(lvl) << "] "
                     << message << std::endl;
}

inline void log_message(level lvl, const char* msg) {
    if (lvl < get_log_level()) return;

    std::lock_guard<std::mutex> lock(get_log_mutex());
    get_log_stream() << "[" << timestamp() << "] [" << level_string(lvl) << "] "
                     << msg << std::endl;
}

inline void log_message(level lvl, const std::string& msg) {
    log_message(lvl, msg.c_str());
}

// Set level dynamically
inline void set_level(level lvl) {
    get_log_level() = lvl;
}

inline level get_level() {
    return get_log_level();
}

} // namespace spdlog

#define NOOP
// MESSAGE_LOG macro for compatibility with existing code
#define MESSAGE_LOG(...) \
	NOOP;	
//if (m_logger) { m_logger->log(__VA_ARGS__); } 

// Dummy logger class for API compatibility
namespace spdlog {
class logger {
public:
    logger(const std::string&) {}

    template<typename... Args>
    void log(level lvl, const char* fmt, Args&&... args) {
        log_message(lvl, fmt, std::forward<Args>(args)...);
    }

    void log(level lvl, const char* msg) { log_message(lvl, msg); }
    void log(level lvl, const std::string& msg) { log_message(lvl, msg); }

    // Overloads that default to info level when no level provided
    template<typename... Args>
    void log(const char* fmt, Args&&... args) {
        log_message(level::info, fmt, std::forward<Args>(args)...);
    }

    void log(const char* msg) { log_message(level::info, msg); }
    void log(const std::string& msg) { log_message(level::info, msg); }

    template<typename... Args>
    void trace(const char* fmt, Args&&... args) { log_message(level::trace, fmt, std::forward<Args>(args)...); }
    void trace(const std::string& msg) { log_message(level::trace, msg); }

    template<typename... Args>
    void debug(const char* fmt, Args&&... args) { log_message(level::debug, fmt, std::forward<Args>(args)...); }
    void debug(const std::string& msg) { log_message(level::debug, msg); }

    template<typename... Args>
    void info(const char* fmt, Args&&... args) { log_message(level::info, fmt, std::forward<Args>(args)...); }
    void info(const std::string& msg) { log_message(level::info, msg); }

    template<typename... Args>
    void warn(const char* fmt, Args&&... args) { log_message(level::warn, fmt, std::forward<Args>(args)...); }
    void warn(const std::string& msg) { log_message(level::warn, msg); }

    template<typename... Args>
    void error(const char* fmt, Args&&... args) { log_message(level::err, fmt, std::forward<Args>(args)...); }
    void error(const std::string& msg) { log_message(level::err, msg); }

    template<typename... Args>
    void critical(const char* fmt, Args&&... args) { log_message(level::critical, fmt, std::forward<Args>(args)...); }
    void critical(const std::string& msg) { log_message(level::critical, msg); }

    void set_level(level lvl) { spdlog::set_level(lvl); }
    level get_level() const { return spdlog::get_level(); }
    std::string name() const { return "usgscsm_logger"; }
    void flush() {}
};

// Stub functions - always return the same shared dummy logger
inline std::shared_ptr<logger> get(const std::string&) {
    static auto dummy = std::make_shared<logger>("dummy");
    return dummy;
}

inline std::shared_ptr<logger> stdout_color_mt(const std::string&) {
    return get("");
}

inline std::shared_ptr<logger> stderr_color_mt(const std::string&) {
    return get("");
}

inline std::shared_ptr<logger> basic_logger_mt(const std::string&, const std::string&) {
    return get("");
}

inline void register_logger(std::shared_ptr<logger>) {}
inline void drop(const std::string&) {}

// Ostream sink compatibility - for tests only
namespace sinks {
    class ostream_sink_mt {
    public:
        ostream_sink_mt(std::ostream& stream) : stream_(stream) {}
        std::ostream& stream() { return stream_; }
    private:
        std::ostream& stream_;
    };

    // Test logger that writes to a specific ostream
    class ostream_logger : public logger {
    public:
        ostream_logger(const std::string& name, std::ostream* stream)
            : logger(name), stream_(stream), min_level_(level::trace) {}

        void set_level(level lvl) { min_level_ = lvl; }

        template<typename... Args>
        void log(level lvl, const char* fmt, Args&&... args) {
            if (lvl < min_level_ || !stream_) return;
            std::string message = detail::format_message(std::string(fmt), std::forward<Args>(args)...);
            std::lock_guard<std::mutex> lock(mutex_);
            *stream_ << "[" << timestamp() << "] [" << level_string(lvl) << "] " << message << std::endl;
        }

        void log(level lvl, const char* msg) {
            if (lvl < min_level_ || !stream_) return;
            std::lock_guard<std::mutex> lock(mutex_);
            *stream_ << "[" << timestamp() << "] [" << level_string(lvl) << "] " << msg << std::endl;
        }

        void log(level lvl, const std::string& msg) { log(lvl, msg.c_str()); }

    private:
        std::ostream* stream_;
        level min_level_;
        std::mutex mutex_;
    };

    inline std::shared_ptr<ostream_logger> create_ostream_logger(const std::string& name,
                                                                  std::shared_ptr<ostream_sink_mt> sink) {
        return std::make_shared<ostream_logger>(name, &sink->stream());
    }
}

} // namespace spdlog

#endif // USGSCSM_LOGGER_H
