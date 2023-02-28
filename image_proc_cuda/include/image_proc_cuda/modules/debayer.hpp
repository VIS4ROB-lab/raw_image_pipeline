// Author: Matias Mattamala

#pragma once

#include <opencv2/opencv.hpp>

#ifdef HAS_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

namespace image_proc_cuda {

class DebayerModule {
 public:
  DebayerModule();
  void enable(bool enabled);
  bool enabled() const;

  //-----------------------------------------------------------------------------
  // Setters
  //-----------------------------------------------------------------------------
  void setEncoding(const std::string& encoding);

  //-----------------------------------------------------------------------------
  // Main interface
  //-----------------------------------------------------------------------------
  template <typename T>
  bool apply(T& image);

  //-----------------------------------------------------------------------------
  // Helper methods (CPU)
  //-----------------------------------------------------------------------------
 private:
  bool isBayerEncoding(const std::string& encoding) const;

  void debayer(cv::Mat& image);
#ifdef HAS_CUDA
  void debayer(cv::cuda::GpuMat& image);
#endif

  //-----------------------------------------------------------------------------
  // Variables
  //-----------------------------------------------------------------------------
  bool enabled_;
  std::string encoding_;

  std::vector<std::string> BAYER_TYPES = {"bayer_bggr8",
                                          "bayer_gbrg8",
                                          "bayer_grbg8",
                                          "bayer_rggb8"
                                          "bayer_bggr16",
                                          "bayer_gbrg16",
                                          "bayer_grbg16",
                                          "bayer_rggb16"};
};

}  // namespace image_proc_cuda