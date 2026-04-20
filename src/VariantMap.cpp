#include "VariantMap.h"
#include <variant>
#include <map>
#include <stdexcept>
#include <typeinfo>
#include <sstream>

using VariantValue = std::variant<
  std::string,
  int,
  double,
  bool,
  std::vector<double>,
  std::vector<int>
>;

class VariantMapImpl {
public:
  std::map<std::string, VariantValue> data;
};

// Helper to overload lambdas for std::visit
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Macro to define get<T> with error checking
#define VARIANT_MAP_GET(TYPE, TYPE_NAME) \
template<> \
TYPE VariantMap::get<TYPE>(const std::string& key) const { \
  try { \
    return std::get<TYPE>(impl_->data.at(key)); \
  } catch (const std::out_of_range&) { \
    throw std::runtime_error("Key '" + key + "' not found in VariantMap"); \
  } catch (const std::exception& e) { \
    throw std::runtime_error("Bad variant access for key '" + key + "': expected " TYPE_NAME " but got different type"); \
  } \
}

// Macro to define get<T> with default value and error checking
#define VARIANT_MAP_GET_DEFAULT(TYPE, TYPE_NAME) \
template<> \
TYPE VariantMap::get<TYPE>(const std::string& key, const TYPE& defaultValue) const { \
  auto it = impl_->data.find(key); \
  if (it == impl_->data.end()) return defaultValue; \
  try { \
    return std::get<TYPE>(it->second); \
  } catch (const std::exception& e) { \
    throw std::runtime_error("Bad variant access for key '" + key + "': expected " TYPE_NAME " but got different type"); \
  } \
}

VariantMap::VariantMap() : impl_(new VariantMapImpl()) {}
VariantMap::~VariantMap() = default;
VariantMap::VariantMap(const VariantMap& other)
  : impl_(new VariantMapImpl(*other.impl_)) {}

VariantMap::VariantMap(VariantMap&& other) noexcept
  : impl_(std::move(other.impl_)) {}

VariantMap& VariantMap::operator=(VariantMap other) {
  std::swap(impl_, other.impl_);
  return *this;
}

template<>
void VariantMap::set<std::string>(const std::string& key, const std::string& value) {
  impl_->data[key] = value;
}

template<>
void VariantMap::set<int>(const std::string& key, const int& value) {
  impl_->data[key] = value;
}

template<>
void VariantMap::set<double>(const std::string& key, const double& value) {
  impl_->data[key] = value;
}

template<>
void VariantMap::set<bool>(const std::string& key, const bool& value) {
  impl_->data[key] = value;
}

template<>
void VariantMap::set<std::vector<double>>(const std::string& key,
                                          const std::vector<double>& value) {
  impl_->data[key] = value;
}

template<>
void VariantMap::set<std::vector<int>>(const std::string& key,
                                       const std::vector<int>& value) {
  impl_->data[key] = value;
}

// Generate get<T>() specializations with error checking
VARIANT_MAP_GET(std::string, "string")
VARIANT_MAP_GET(bool, "bool")

// Special handling for int - can retrieve int or double and cast
template<>
int VariantMap::get<int>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw std::runtime_error("Key '" + key + "' not found in VariantMap");
  }
  try {
    return std::get<int>(it->second);
  } catch (const std::exception&) {
    // Try to get as double and cast to int
    try {
      return static_cast<int>(std::get<double>(it->second));
    } catch (const std::exception&) {
      throw std::runtime_error("Bad variant access for key '" + key + "': expected numeric type (int or double) but got different type");
    }
  }
}

// Special handling for double - can retrieve double or int and cast
template<>
double VariantMap::get<double>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw std::runtime_error("Key '" + key + "' not found in VariantMap");
  }
  try {
    return std::get<double>(it->second);
  } catch (const std::exception&) {
    // Try to get as int and cast to double
    try {
      return static_cast<double>(std::get<int>(it->second));
    } catch (const std::exception&) {
      throw std::runtime_error("Bad variant access for key '" + key + "': expected numeric type (int or double) but got different type");
    }
  }
}

// Special handling for vector<int> - can retrieve vector<int> or vector<double> and cast
template<>
std::vector<int> VariantMap::get<std::vector<int>>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw std::runtime_error("Key '" + key + "' not found in VariantMap");
  }
  try {
    return std::get<std::vector<int>>(it->second);
  } catch (const std::exception&) {
    // Try to get as vector<double> and cast each element to int
    try {
      const std::vector<double>& doubles = std::get<std::vector<double>>(it->second);
      std::vector<int> ints;
      ints.reserve(doubles.size());
      for (double d : doubles) {
        ints.push_back(static_cast<int>(d));
      }
      return ints;
    } catch (const std::exception&) {
      throw std::runtime_error("Bad variant access for key '" + key + "': expected numeric vector type (vector<int> or vector<double>) but got different type");
    }
  }
}

// Special handling for vector<double> - can retrieve vector<double> or vector<int> and cast
template<>
std::vector<double> VariantMap::get<std::vector<double>>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw std::runtime_error("Key '" + key + "' not found in VariantMap");
  }
  try {
    return std::get<std::vector<double>>(it->second);
  } catch (const std::exception&) {
    // Try to get as vector<int> and cast each element to double
    try {
      const std::vector<int>& ints = std::get<std::vector<int>>(it->second);
      std::vector<double> doubles;
      doubles.reserve(ints.size());
      for (int i : ints) {
        doubles.push_back(static_cast<double>(i));
      }
      return doubles;
    } catch (const std::exception&) {
      throw std::runtime_error("Bad variant access for key '" + key + "': expected numeric vector type (vector<int> or vector<double>) but got different type");
    }
  }
}

// Generate get<T>(key, default) specializations with error checking
VARIANT_MAP_GET_DEFAULT(std::string, "string")
VARIANT_MAP_GET_DEFAULT(bool, "bool")
VARIANT_MAP_GET_DEFAULT(std::vector<double>, "vector<double>")
VARIANT_MAP_GET_DEFAULT(std::vector<int>, "vector<int>")

// Special handling for int with default - can retrieve int or double and cast
template<>
int VariantMap::get<int>(const std::string& key, const int& defaultValue) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) return defaultValue;
  try {
    return std::get<int>(it->second);
  } catch (const std::exception&) {
    // Try to get as double and cast to int
    try {
      return static_cast<int>(std::get<double>(it->second));
    } catch (const std::exception&) {
      throw std::runtime_error("Bad variant access for key '" + key + "': expected numeric type (int or double) but got different type");
    }
  }
}

// Special handling for double with default - can retrieve double or int and cast
template<>
double VariantMap::get<double>(const std::string& key, const double& defaultValue) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) return defaultValue;
  try {
    return std::get<double>(it->second);
  } catch (const std::exception&) {
    // Try to get as int and cast to double
    try {
      return static_cast<double>(std::get<int>(it->second));
    } catch (const std::exception&) {
      throw std::runtime_error("Bad variant access for key '" + key + "': expected numeric type (int or double) but got different type");
    }
  }
}

// Helper: converts variant index to ValueType
static VariantMap::ValueType indexToValueType(size_t idx) {
  switch (idx) {
    case 0: return VariantMap::ValueType::String;
    case 1: return VariantMap::ValueType::Int;
    case 2: return VariantMap::ValueType::Double;
    case 3: return VariantMap::ValueType::Bool;
    case 4: return VariantMap::ValueType::VectorDouble;
    case 5: return VariantMap::ValueType::VectorInt;
    default: return VariantMap::ValueType::Unknown;
  }
}

VariantMap::ValueType VariantMap::getValueType(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    return ValueType::Unknown;
  }
  return indexToValueType(it->second.index());
}

bool VariantMap::contains(const std::string& key) const {
  return impl_->data.find(key) != impl_->data.end();
}

void VariantMap::erase(const std::string& key) {
  impl_->data.erase(key);
}

void VariantMap::clear() {
  impl_->data.clear();
}

size_t VariantMap::size() const {
  return impl_->data.size();
}

bool VariantMap::empty() const {
  return impl_->data.empty();
}

std::vector<std::string> VariantMap::keys() const {
  std::vector<std::string> result;
  for (const auto& pair : impl_->data) {
    result.push_back(pair.first);
  }
  return result;
}

std::string VariantMap::dumps() const {
  std::ostringstream oss;
  oss << "VariantMap contents (" << size() << " entries):\n";

  for (const auto& pair : impl_->data) {
    oss << "  \"" << pair.first << "\": ";

    std::visit(overloaded{
      [&oss](const std::string& val) { oss << "\"" << val << "\" (string)"; },
      [&oss](int val) { oss << val << " (int)"; },
      [&oss](double val) { oss << val << " (double)"; },
      [&oss](bool val) { oss << (val ? "true" : "false") << " (bool)"; },
      [&oss](const std::vector<double>& val) {
        oss << "[";
        for (size_t i = 0; i < val.size(); ++i) {
          if (i > 0) oss << ", ";
          oss << val[i];
        }
        oss << "] (vector<double>, size=" << val.size() << ")";
      },
      [&oss](const std::vector<int>& val) {
        oss << "[";
        for (size_t i = 0; i < val.size(); ++i) {
          if (i > 0) oss << ", ";
          oss << val[i];
        }
        oss << "] (vector<int>, size=" << val.size() << ")";
      }
    }, pair.second);

    oss << "\n";
  }

  return oss.str();
}

// Clean up internal macros
#undef VARIANT_MAP_GET
#undef VARIANT_MAP_GET_DEFAULT
