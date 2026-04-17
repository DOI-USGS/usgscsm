#ifndef USGSCSM_VARIANTMAP_H
#define USGSCSM_VARIANTMAP_H

#include <string>
#include <vector>
#include <memory>

class VariantMapImpl;

class VariantMap {
public:
  VariantMap();
  ~VariantMap();
  VariantMap(const VariantMap& other);
  VariantMap& operator=(const VariantMap& other);

  template<typename T>
  void set(const std::string& key, const T& value);

  template<typename T>
  T get(const std::string& key) const;

  template<typename T>
  T get(const std::string& key, const T& defaultValue) const;

  // Type introspection
  enum class ValueType {
    String,
    Int,
    Double,
    Bool,
    VectorDouble,
    VectorInt,
    Unknown
  };

  ValueType getValueType(const std::string& key) const;

  bool contains(const std::string& key) const;
  void erase(const std::string& key);
  void clear();
  size_t size() const;
  bool empty() const;
  std::vector<std::string> keys() const;

  // Returns string representation of all keys and values
  std::string dumps() const;

private:
  std::unique_ptr<VariantMapImpl> impl_;

  friend class VariantMapImpl;
};

// Explicit specialization declarations at namespace scope
// These prevent implicit instantiation and must appear before first use
template<> void VariantMap::set<std::string>(const std::string& key, const std::string& value);
template<> void VariantMap::set<int>(const std::string& key, const int& value);
template<> void VariantMap::set<double>(const std::string& key, const double& value);
template<> void VariantMap::set<bool>(const std::string& key, const bool& value);
template<> void VariantMap::set<std::vector<double>>(const std::string& key, const std::vector<double>& value);
template<> void VariantMap::set<std::vector<int>>(const std::string& key, const std::vector<int>& value);

template<> std::string VariantMap::get<std::string>(const std::string& key) const;
template<> int VariantMap::get<int>(const std::string& key) const;
template<> double VariantMap::get<double>(const std::string& key) const;
template<> bool VariantMap::get<bool>(const std::string& key) const;
template<> std::vector<double> VariantMap::get<std::vector<double>>(const std::string& key) const;
template<> std::vector<int> VariantMap::get<std::vector<int>>(const std::string& key) const;

template<> std::string VariantMap::get<std::string>(const std::string& key, const std::string& defaultValue) const;
template<> int VariantMap::get<int>(const std::string& key, const int& defaultValue) const;
template<> double VariantMap::get<double>(const std::string& key, const double& defaultValue) const;
template<> bool VariantMap::get<bool>(const std::string& key, const bool& defaultValue) const;
template<> std::vector<double> VariantMap::get<std::vector<double>>(const std::string& key, const std::vector<double>& defaultValue) const;
template<> std::vector<int> VariantMap::get<std::vector<int>>(const std::string& key, const std::vector<int>& defaultValue) const;

#endif
