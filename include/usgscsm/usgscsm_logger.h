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

// Global logging configuration - lazy-initialized with std::call_once
inline std::ostream& get_log_stream() {
    static std::once_flag init_flag;
    static std::ostream* stream = nullptr;
    static std::ofstream* file_stream = nullptr;  // Heap-allocated to avoid destruction order issues

    std::call_once(init_flag, []() {
        const char* log_file_env = std::getenv("USGSCSM_LOG_FILE");
        std::string file = log_file_env ? std::string(log_file_env) : "stdout";

        if (file == "stdout") {
            stream = &std::cout;
        } else if (file == "stderr") {
            stream = &std::cerr;
        } else if (!file.empty()) {
            file_stream = new std::ofstream(file, std::ios::app);  // Intentional leak - safer than static destruction
            stream = file_stream->is_open() ? file_stream : &std::cout;
        } else {
            stream = &std::cout;
        }
    });

    return *stream;
}

inline std::atomic<level>& get_log_level_atomic() {
    static std::once_flag init_flag;
    static std::atomic<level> current_level{level::off};

    std::call_once(init_flag, []() {
        const char* log_level_env = std::getenv("USGSCSM_LOG_LEVEL");
        std::string level_str = log_level_env ? std::string(log_level_env) : "";
        current_level.store(level_str.empty() ? level::info : from_str(level_str), std::memory_order_release);
    });

    return current_level;
}

inline level get_log_level() {
    return get_log_level_atomic().load(std::memory_order_acquire);
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

// Format string helpers - iterative version to avoid recursion
namespace detail {
    template<typename T>
    inline std::string to_string(const T& value) {
        std::ostringstream oss;
        oss.imbue(std::locale::classic());  // Use C locale to avoid corrupting PROJ's locale/FP state
        oss << value;
        return oss.str();
    }

    // Base case - no arguments
    inline std::string format_message(const std::string& fmt) {
        return fmt;
    }

    // Variadic case - iteratively replace all {} with arguments
    template<typename... Args>
    inline std::string format_message(const std::string& fmt, Args&&... args) {
        if (sizeof...(Args) == 0) return fmt;  // Handle empty args case

        std::string result = fmt;
        std::string values[] = {to_string(std::forward<Args>(args))...};
        size_t arg_index = 0;
        size_t pos = 0;

        while (arg_index < sizeof...(Args) && (pos = result.find("{}", pos)) != std::string::npos) {
            result.replace(pos, 2, values[arg_index]);
            pos += values[arg_index].length();
            arg_index++;
        }

        return result;
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

// Overloads that default to info level when no level provided
template<typename... Args>
inline void log_message(const char* fmt, Args&&... args) {
    log_message(level::info, fmt, std::forward<Args>(args)...);
}

inline void log_message(const char* msg) {
    log_message(level::info, msg);
}

inline void log_message(const std::string& msg) {
    log_message(level::info, msg);
}

// Set level dynamically
inline void set_level(level lvl) {
    get_log_level_atomic().store(lvl, std::memory_order_release);
}

inline level get_level() {
    return get_log_level();
}

} // namespace spdlog

// MESSAGE_LOG macro for compatibility with existing code - now just calls log_message directly
#define MESSAGE_LOG(...) \
    spdlog::log_message(__VA_ARGS__);

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
