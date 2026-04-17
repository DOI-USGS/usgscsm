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

  // Explicit specialization declarations for set()
  template<>
  void set<std::string>(const std::string& key, const std::string& value);

  template<>
  void set<int>(const std::string& key, const int& value);

  template<>
  void set<double>(const std::string& key, const double& value);

  template<>
  void set<bool>(const std::string& key, const bool& value);

  template<>
  void set<std::vector<double>>(const std::string& key, const std::vector<double>& value);

  template<>
  void set<std::vector<int>>(const std::string& key, const std::vector<int>& value);

  // Explicit specialization declarations for get() without default
  template<>
  std::string get<std::string>(const std::string& key) const;

  template<>
  int get<int>(const std::string& key) const;

  template<>
  double get<double>(const std::string& key) const;

  template<>
  bool get<bool>(const std::string& key) const;

  template<>
  std::vector<double> get<std::vector<double>>(const std::string& key) const;

  template<>
  std::vector<int> get<std::vector<int>>(const std::string& key) const;

  // Explicit specialization declarations for get() with default
  template<>
  std::string get<std::string>(const std::string& key, const std::string& defaultValue) const;

  template<>
  int get<int>(const std::string& key, const int& defaultValue) const;

  template<>
  double get<double>(const std::string& key, const double& defaultValue) const;

  template<>
  bool get<bool>(const std::string& key, const bool& defaultValue) const;

  template<>
  std::vector<double> get<std::vector<double>>(const std::string& key, const std::vector<double>& defaultValue) const;

  template<>
  std::vector<int> get<std::vector<int>>(const std::string& key, const std::vector<int>& defaultValue) const;

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

#endif
