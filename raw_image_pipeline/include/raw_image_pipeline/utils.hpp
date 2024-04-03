#pragma once
#include <yaml-cpp/yaml.h>
#include <boost/array.hpp>
#include <opencv2/opencv.hpp>

namespace YAML {

template <>
struct convert<cv::Matx14d> {
  static bool decode(const YAML::Node& node, cv::Matx14d& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }
    rhs = cv::Matx14d{node[0].as<double>(), node[1].as<double>(), node[2].as<double>(), node[3].as<double>()};
    return true;
  }
};

template <>
struct convert<cv::Matx31d> {
  static bool decode(const YAML::Node& node, cv::Matx31d& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }
    rhs = cv::Matx31d{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
    return true;
  }
};

template <>
struct convert<cv::Matx33d> {
  static bool decode(const YAML::Node& node, cv::Matx33d& rhs) {
    if (!node.IsSequence() || node.size() != 9) {
      return false;
    }
    rhs = cv::Matx33d{node[0].as<double>(), node[1].as<double>(), node[2].as<double>(), node[3].as<double>(), node[4].as<double>(),
                      node[5].as<double>(), node[6].as<double>(), node[7].as<double>(), node[8].as<double>()};
    return true;
  }
};

template <>
struct convert<cv::Matx34d> {
  static bool decode(const YAML::Node& node, cv::Matx34d& rhs) {
    if (!node.IsSequence() || node.size() != 12) {
      return false;
    }
    rhs = cv::Matx34d{node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),  node[3].as<double>(),
                      node[4].as<double>(), node[5].as<double>(), node[6].as<double>(),  node[7].as<double>(),
                      node[8].as<double>(), node[9].as<double>(), node[10].as<double>(), node[11].as<double>()};
    return true;
  }
};

}  // namespace YAML

namespace raw_image_pipeline {
namespace utils {

// Wrapper for yaml-cpp to provide default values if value is not found
template <typename T>
T get(const YAML::Node& node, const std::string& param, const T& default_value) {
  T out;
  try {
    if (!node[param]) {
      out = default_value;
    }
    out = node[param].as<T>();
  } catch (const YAML::InvalidNode::Exception& e) {
    out = default_value;
  }
  // std::cout << "Reading parameter [" << param << "]: " << std::endl;
  return out;
}

// Convert mat to std vector
// Source: https://stackoverflow.com/a/56600115
template <typename T>
static std::vector<T> toStdVector(const cv::Mat& m) {
  cv::Mat m_aux = m.isContinuous() ? m : m.clone();
  std::vector<T> vec = m_aux.reshape(1, m_aux.total() * m_aux.channels());
  return vec;
}

template <typename T, std::size_t N>
static boost::array<T, N> toBoostArray(const cv::Mat& m) {
  // TODO: this could be optimized
  boost::array<T, N> vec;
  cv::Mat m_aux = m.isContinuous() ? m : m.clone();
  m_aux = m_aux.reshape(1, m_aux.total() * m_aux.channels());

  for (size_t i = 0; i < N; i++) vec[i] = m_aux.at<T>(i);
  return vec;
}

template <typename T, std::size_t N>
static std::array<T, N> toStdArray(const cv::Mat& m) {
  // TODO: this could be optimized
  std::array<T, N> vec;
  cv::Mat m_aux = m.isContinuous() ? m : m.clone();
  m_aux = m_aux.reshape(1, m_aux.total() * m_aux.channels());

  for (size_t i = 0; i < N; i++) vec[i] = m_aux.at<T>(i);
  return vec;
}

}  // namespace utils
}  // namespace raw_image_pipeline