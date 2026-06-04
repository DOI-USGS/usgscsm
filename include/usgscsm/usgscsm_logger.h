/**
 * USGSCSM Simple Logger - Single-header SPDLOG replacement
 *
 * Minimal logging implementation compatible with SPDLOG API.
 * Supports format strings with {} placeholders.
 */

#ifndef USGSCSM_LOGGER_H
#define USGSCSM_LOGGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <map>
#include <mutex>
#include <ctime>

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

// Logger class
class logger {
public:
    logger(const std::string& name)
        : name_(name), level_(level::info), output_type_(OutputType::NONE), ostream_ptr_(nullptr) {}

    ~logger() {
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
    }

    void set_level(level lvl) {
        std::lock_guard<std::mutex> lock(mutex_);
        level_ = lvl;
    }

    level get_level() const {
        return level_;
    }

    std::string name() const {
        return name_;
    }

    // Main logging method - no arguments
    void log(level lvl, const char* msg) {
        if (lvl < level_ || output_type_ == OutputType::NONE) return;

        std::lock_guard<std::mutex> lock(mutex_);
        write_log(lvl, msg);
    }

    // Overload for std::string
    void log(level lvl, const std::string& msg) {
        log(lvl, msg.c_str());
    }

    // Format string helper - recursion base case
    template<typename T>
    std::string format_impl(const std::string& fmt, size_t pos, const T& value) {
        size_t placeholder = fmt.find("{}", pos);
        if (placeholder == std::string::npos) {
            return fmt;
        }

        std::ostringstream oss;
        oss << value;

        std::string result = fmt.substr(0, placeholder) + oss.str();
        result += fmt.substr(placeholder + 2);
        return result;
    }

    // Format string helper - recursive case
    template<typename T, typename... Args>
    std::string format_impl(const std::string& fmt, size_t pos, const T& value, Args&&... args) {
        size_t placeholder = fmt.find("{}", pos);
        if (placeholder == std::string::npos) {
            return fmt;
        }

        std::ostringstream oss;
        oss << value;

        std::string result = fmt.substr(0, placeholder) + oss.str() + fmt.substr(placeholder + 2);
        return format_impl(result, placeholder + oss.str().length(), std::forward<Args>(args)...);
    }

    // Logging method with format arguments
    template<typename... Args>
    void log(level lvl, const char* fmt, Args&&... args) {
        if (lvl < level_ || output_type_ == OutputType::NONE) return;

        std::string formatted = format_impl(std::string(fmt), 0, std::forward<Args>(args)...);

        std::lock_guard<std::mutex> lock(mutex_);
        write_log(lvl, formatted.c_str());
    }

    void set_output_stdout() {
        std::lock_guard<std::mutex> lock(mutex_);
        output_type_ = OutputType::STDOUT;
    }

    void set_output_stderr() {
        std::lock_guard<std::mutex> lock(mutex_);
        output_type_ = OutputType::STDERR;
    }

    void set_output_file(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
        file_stream_.open(path, std::ios::app);
        if (file_stream_.is_open()) {
            output_type_ = OutputType::FILE;
        }
    }

    void set_output_ostream(std::ostream* stream) {
        std::lock_guard<std::mutex> lock(mutex_);
        ostream_ptr_ = stream;
        output_type_ = OutputType::OSTREAM;
    }

    // Convenience methods for each log level
    template<typename... Args>
    void trace(const char* fmt, Args&&... args) {
        log(level::trace, fmt, std::forward<Args>(args)...);
    }

    void trace(const std::string& msg) {
        log(level::trace, msg.c_str());
    }

    template<typename... Args>
    void debug(const char* fmt, Args&&... args) {
        log(level::debug, fmt, std::forward<Args>(args)...);
    }

    void debug(const std::string& msg) {
        log(level::debug, msg.c_str());
    }

    template<typename... Args>
    void info(const char* fmt, Args&&... args) {
        log(level::info, fmt, std::forward<Args>(args)...);
    }

    void info(const std::string& msg) {
        log(level::info, msg.c_str());
    }

    template<typename... Args>
    void warn(const char* fmt, Args&&... args) {
        log(level::warn, fmt, std::forward<Args>(args)...);
    }

    void warn(const std::string& msg) {
        log(level::warn, msg.c_str());
    }

    template<typename... Args>
    void error(const char* fmt, Args&&... args) {
        log(level::err, fmt, std::forward<Args>(args)...);
    }

    void error(const std::string& msg) {
        log(level::err, msg.c_str());
    }

    template<typename... Args>
    void critical(const char* fmt, Args&&... args) {
        log(level::critical, fmt, std::forward<Args>(args)...);
    }

    void critical(const std::string& msg) {
        log(level::critical, msg.c_str());
    }

    // Flush method (for compatibility)
    void flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (output_type_ == OutputType::FILE && file_stream_.is_open()) {
            file_stream_.flush();
        }
        // stdout/stderr/ostream flush automatically after each log
    }

private:
    enum class OutputType { NONE, STDOUT, STDERR, FILE, OSTREAM };

    std::string name_;
    level level_;
    OutputType output_type_;
    std::ofstream file_stream_;
    std::ostream* ostream_ptr_;
    std::mutex mutex_;

    std::string level_string(level lvl) const {
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

    std::string timestamp() const {
        std::time_t now = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        return std::string(buf);
    }

    void write_log(level lvl, const char* msg) {
        std::ostringstream line;
        line << "[" << timestamp() << "] [" << level_string(lvl) << "] " << msg << "\n";

        std::string output = line.str();

        switch (output_type_) {
            case OutputType::STDOUT:
                std::cout << output << std::flush;
                break;
            case OutputType::STDERR:
                std::cerr << output << std::flush;
                break;
            case OutputType::FILE:
                if (file_stream_.is_open()) {
                    file_stream_ << output << std::flush;
                }
                break;
            case OutputType::OSTREAM:
                if (ostream_ptr_) {
                    *ostream_ptr_ << output << std::flush;
                }
                break;
            case OutputType::NONE:
                break;
        }
    }
};

// Global logger registry
class logger_registry {
public:
    static logger_registry& instance() {
        static logger_registry registry;
        return registry;
    }

    std::shared_ptr<logger> get(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = loggers_.find(name);
        if (it != loggers_.end()) {
            return it->second;
        }
        return nullptr;
    }

    void register_logger(const std::string& name, std::shared_ptr<logger> logger_ptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        loggers_[name] = logger_ptr;
    }

    void drop(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        loggers_.erase(name);
    }

private:
    std::map<std::string, std::shared_ptr<logger>> loggers_;
    std::mutex mutex_;
};

// Public API functions
inline std::shared_ptr<logger> get(const std::string& name) {
    return logger_registry::instance().get(name);
}

inline std::shared_ptr<logger> stdout_color_mt(const std::string& name) {
    auto logger_ptr = std::make_shared<logger>(name);
    logger_ptr->set_output_stdout();
    logger_registry::instance().register_logger(name, logger_ptr);
    return logger_ptr;
}

inline std::shared_ptr<logger> stderr_color_mt(const std::string& name) {
    auto logger_ptr = std::make_shared<logger>(name);
    logger_ptr->set_output_stderr();
    logger_registry::instance().register_logger(name, logger_ptr);
    return logger_ptr;
}

inline std::shared_ptr<logger> basic_logger_mt(const std::string& name, const std::string& path) {
    auto logger_ptr = std::make_shared<logger>(name);
    logger_ptr->set_output_file(path);
    logger_registry::instance().register_logger(name, logger_ptr);
    return logger_ptr;
}

inline void register_logger(std::shared_ptr<logger> logger_ptr) {
}

inline void drop(const std::string& name) {
    logger_registry::instance().drop(name);
}

// Compatibility namespace for test sinks
namespace sinks {
    // Ostream sink for test compatibility
    class ostream_sink_mt {
    public:
        ostream_sink_mt(std::ostream& stream) : stream_(stream) {}
        std::ostream& stream() { return stream_; }
    private:
        std::ostream& stream_;
    };

    // Factory function to create logger with ostream sink
    inline std::shared_ptr<logger> create_ostream_logger(const std::string& name,
                                                          std::shared_ptr<ostream_sink_mt> sink) {
        auto logger_ptr = std::make_shared<logger>(name);
        logger_ptr->set_output_ostream(&sink->stream());
        logger_registry::instance().register_logger(name, logger_ptr);
        return logger_ptr;
    }
}

} // namespace spdlog

#endif // USGSCSM_LOGGER_H
