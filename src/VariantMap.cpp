#include "VariantMap.h"
#include <variant>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>
#include <sstream>

using VariantValue = std::variant<
  std::string,
  int,
  double,
  bool,
  std::vector<double>,
  std::vector<int>,
  std::vector<std::string>
>;

class VariantMapImpl {
public:
  std::map<std::string, VariantValue> data;
};

// Helper to overload lambdas for std::visit
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

static VariantMapTypeError typeError(const std::string& key, const char* expected) {
  return VariantMapTypeError("Bad variant access for key '" + key +
                             "': expected " + expected +
                             " but got different type");
}

namespace {

// int and double are interchangeable on read, so each numeric getter also accepts
// the other and casts. `Other` names that counterpart.
template<typename T>
using Other = std::conditional_t<std::is_same_v<T, int>, double, int>;

template<typename To, typename From>
std::vector<To> castVector(const std::vector<From>& in) {
  std::vector<To> out;
  out.reserve(in.size());
  for (From v : in) out.push_back(static_cast<To>(v));
  return out;
}

template<typename T>
T numericScalar(const VariantValue& value, const std::string& key) {
  if (std::holds_alternative<T>(value)) return std::get<T>(value);
  if (std::holds_alternative<Other<T>>(value)) {
    return static_cast<T>(std::get<Other<T>>(value));
  }
  throw typeError(key, "numeric type (int or double)");
}

// Also accepts a bare scalar: STARDS stores everything as an NDArray, so a
// scalar and a size-1 array are indistinguishable on the way back in.
template<typename T>
std::vector<T> numericVector(const VariantValue& value, const std::string& key) {
  if (std::holds_alternative<std::vector<T>>(value)) {
    return std::get<std::vector<T>>(value);
  }
  if (std::holds_alternative<std::vector<Other<T>>>(value)) {
    return castVector<T>(std::get<std::vector<Other<T>>>(value));
  }
  if (std::holds_alternative<T>(value)) return std::vector<T>{std::get<T>(value)};
  if (std::holds_alternative<Other<T>>(value)) {
    return std::vector<T>{static_cast<T>(std::get<Other<T>>(value))};
  }
  throw typeError(key, "numeric vector type (vector<int> or vector<double>)");
}

}  // namespace

// Macro to define get<T> with error checking
#define VARIANT_MAP_GET(TYPE, TYPE_NAME) \
template<> \
TYPE VariantMap::get<TYPE>(const std::string& key) const { \
  auto it = impl_->data.find(key); \
  if (it == impl_->data.end()) { \
    throw VariantMapKeyError("Key '" + key + "' not found in VariantMap"); \
  } \
  if (!std::holds_alternative<TYPE>(it->second)) throw typeError(key, TYPE_NAME); \
  return std::get<TYPE>(it->second); \
}

// Macro to define get<T> with default value and error checking
#define VARIANT_MAP_GET_DEFAULT(TYPE, TYPE_NAME) \
template<> \
TYPE VariantMap::get<TYPE>(const std::string& key, const TYPE& defaultValue) const { \
  auto it = impl_->data.find(key); \
  if (it == impl_->data.end()) return defaultValue; \
  if (!std::holds_alternative<TYPE>(it->second)) throw typeError(key, TYPE_NAME); \
  return std::get<TYPE>(it->second); \
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

template<>
void VariantMap::set<std::vector<std::string>>(const std::string& key,
                                               const std::vector<std::string>& value) {
  impl_->data[key] = value;
}

// Generate get<T>() specializations with error checking
VARIANT_MAP_GET(std::string, "string")
VARIANT_MAP_GET(bool, "bool")

template<>
int VariantMap::get<int>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw VariantMapKeyError("Key '" + key + "' not found in VariantMap");
  }
  return numericScalar<int>(it->second, key);
}

template<>
double VariantMap::get<double>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw VariantMapKeyError("Key '" + key + "' not found in VariantMap");
  }
  return numericScalar<double>(it->second, key);
}

template<>
std::vector<int> VariantMap::get<std::vector<int>>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw VariantMapKeyError("Key '" + key + "' not found in VariantMap");
  }
  return numericVector<int>(it->second, key);
}

template<>
std::vector<double> VariantMap::get<std::vector<double>>(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    throw VariantMapKeyError("Key '" + key + "' not found in VariantMap");
  }
  return numericVector<double>(it->second, key);
}

// vector<string> - no cross-type coercion (used to carry nested model states)
VARIANT_MAP_GET(std::vector<std::string>, "vector<string>")

// Generate get<T>(key, default) specializations with error checking
VARIANT_MAP_GET_DEFAULT(std::string, "string")
VARIANT_MAP_GET_DEFAULT(bool, "bool")
VARIANT_MAP_GET_DEFAULT(std::vector<double>, "vector<double>")
VARIANT_MAP_GET_DEFAULT(std::vector<int>, "vector<int>")
VARIANT_MAP_GET_DEFAULT(std::vector<std::string>, "vector<string>")

template<>
int VariantMap::get<int>(const std::string& key, const int& defaultValue) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) return defaultValue;
  return numericScalar<int>(it->second, key);
}

template<>
double VariantMap::get<double>(const std::string& key, const double& defaultValue) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) return defaultValue;
  return numericScalar<double>(it->second, key);
}

// Dispatches on the stored type rather than the variant's index, so reordering
// VariantValue's alternatives cannot silently remap these.
static VariantMap::ValueType valueTypeOf(const VariantValue& value) {
  return std::visit(overloaded{
    [](const std::string&) { return VariantMap::ValueType::String; },
    [](int) { return VariantMap::ValueType::Int; },
    [](double) { return VariantMap::ValueType::Double; },
    [](bool) { return VariantMap::ValueType::Bool; },
    [](const std::vector<double>&) { return VariantMap::ValueType::VectorDouble; },
    [](const std::vector<int>&) { return VariantMap::ValueType::VectorInt; },
    [](const std::vector<std::string>&) { return VariantMap::ValueType::VectorString; }
  }, value);
}

VariantMap::ValueType VariantMap::getValueType(const std::string& key) const {
  auto it = impl_->data.find(key);
  if (it == impl_->data.end()) {
    return ValueType::Unknown;
  }
  return valueTypeOf(it->second);
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
      },
      [&oss](const std::vector<std::string>& val) {
        oss << "[" << val.size() << " entries] (vector<string>)";
      }
    }, pair.second);

    oss << "\n";
  }

  return oss.str();
}

// Clean up internal macros
#undef VARIANT_MAP_GET
#undef VARIANT_MAP_GET_DEFAULT
