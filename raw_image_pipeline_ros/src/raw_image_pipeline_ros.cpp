//
// Copyright (c) 2021-2023, ETH Zurich, Robotic Systems Lab, Matias Mattamala,
// Timon Homberger. All rights reserved. Licensed under the MIT license. See
// LICENSE file in the project root for details.
//

#include <raw_image_pipeline_ros/raw_image_pipeline_ros.hpp>

namespace raw_image_pipeline {

RawImagePipelineRos::RawImagePipelineRos(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("raw_image_pipeline_ros", options), skipped_images_(0) {
  one_off_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
        loadParams();
        setupRos();
        one_off_timer_->cancel();
      });
}

RawImagePipelineRos::~RawImagePipelineRos() {}

void RawImagePipelineRos::loadParams() {
  // Topic options
  readRequiredParameter("input_topic", input_topic_);
  readRequiredParameter("output_prefix", output_prefix_);

  pub_debayer_ = readParameter("publish_debayer", false);
  pub_color_ = readParameter("publish_color", false);
  pub_rect_ = readParameter("publish_rect", false);

  image_transport_ =
      std::make_shared<image_transport::ImageTransport>(shared_from_this());

  // Get transport hint
  transport_ = getTransportHintFromTopic(input_topic_);

  // Other parameters
  output_encoding_ = readParameter("output_encoding", std::string("BGR"));
  output_frame_ = readParameter("output_frame", std::string("passthrough"));
  skip_number_of_images_ = readParameter("skip_number_of_images", -1);

  // Read GPU parameter and initialize image proc
  bool use_gpu = readParameter("use_gpu", true);
  raw_image_pipeline_ = std::make_unique<RawImagePipeline>(use_gpu);

  // Debug param
  bool debug = readParameter("debug", false);
  raw_image_pipeline_->setDebug(debug);

  // Debayer
  bool run_debayer = readParameter("debayer/enabled", true);
  raw_image_pipeline_->setDebayer(run_debayer);

  std::string debayer_encoding =
      readParameter("debayer/encoding", std::string("auto"));
  raw_image_pipeline_->setDebayerEncoding(debayer_encoding);

  // Output options
  bool needs_rotation = readParameter("flip/enabled", false);
  raw_image_pipeline_->setFlip(needs_rotation);

  int flip_angle = readParameter("flip/angle", 0);
  raw_image_pipeline_->setFlipAngle(flip_angle);

  // White balancing params
  bool run_white_balance = readParameter("white_balance/enabled", false);
  raw_image_pipeline_->setWhiteBalance(run_white_balance);

  std::string white_balance_method =
      readParameter("white_balance/method", std::string("simple"));
  raw_image_pipeline_->setWhiteBalanceMethod(white_balance_method);

  double white_balance_clipping_percentile =
      readParameter("white_balance/clipping_percentile", 10.0);
  raw_image_pipeline_->setWhiteBalancePercentile(
      white_balance_clipping_percentile);

  double white_balance_saturation_bright_thr =
      readParameter("white_balance/saturation_bright_thr", 0.9);
  double white_balance_saturation_dark_thr =
      readParameter("white_balance/saturation_dark_thr", 0.1);
  raw_image_pipeline_->setWhiteBalanceSaturationThreshold(
      white_balance_saturation_bright_thr, white_balance_saturation_dark_thr);

  bool white_balance_temporal_consistency =
      readParameter("white_balance/temporal_consistency", false);
  raw_image_pipeline_->setWhiteBalanceTemporalConsistency(
      white_balance_temporal_consistency);

  // Color calibration
  bool run_color_calibration =
      readParameter("color_calibration/enabled", false);
  raw_image_pipeline_->setColorCalibration(run_color_calibration);

  if (run_color_calibration) {
    // Check if we have the calibration file path
    std::string color_calibration_file =
        readParameter("color_calibration/calibration_file", std::string(""));
    raw_image_pipeline_->loadColorCalibration(color_calibration_file);

    if (color_calibration_file.empty()) {
      std::vector<double> color_calibration_matrix = readParameter(
          "color_calibration/calibration_matrix/data",
          std::vector<double>({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}));
      raw_image_pipeline_->setColorCalibrationMatrix(color_calibration_matrix);

      std::vector<double> color_calibration_bias =
          readParameter("color_calibration/calibration_bias/data",
                        std::vector<double>({0.0, 0.0, 0.0}));
      raw_image_pipeline_->setColorCalibrationBias(color_calibration_bias);
    }
  }

  // Gamma correction
  bool run_gamma_correction = readParameter("gamma_correction/enabled", false);
  raw_image_pipeline_->setGammaCorrection(run_gamma_correction);

  std::string gamma_correction_method =
      readParameter("gamma_correction/method", std::string("default"));
  raw_image_pipeline_->setGammaCorrectionMethod(gamma_correction_method);

  double gamma_correction_k = readParameter("gamma_correction/k", 0.8);
  raw_image_pipeline_->setGammaCorrectionK(gamma_correction_k);

  // Vignetting correction
  bool run_vignetting_correction =
      readParameter("vignetting_correction/enabled", false);
  raw_image_pipeline_->setVignettingCorrection(run_vignetting_correction);

  double vignetting_correction_scale =
      readParameter("vignetting_correction/scale", 1.0);
  double vignetting_correction_a2 =
      readParameter("vignetting_correction/a2", 1.0);
  double vignetting_correction_a4 =
      readParameter("vignetting_correction/a4", 1.0);
  raw_image_pipeline_->setVignettingCorrectionParameters(
      vignetting_correction_scale, vignetting_correction_a2,
      vignetting_correction_a4);

  // Color enhancer
  bool run_color_enhancer = readParameter("color_enhancer/enabled", false);
  raw_image_pipeline_->setColorEnhancer(run_color_enhancer);

  double color_enhancer_hue_gain =
      readParameter("color_enhancer/hue_gain", 1.0);
  raw_image_pipeline_->setColorEnhancerHueGain(color_enhancer_hue_gain);

  double color_enhancer_saturation_gain =
      readParameter("color_enhancer/saturation_gain", 1.0);
  raw_image_pipeline_->setColorEnhancerSaturationGain(
      color_enhancer_saturation_gain);

  double color_enhancer_value_gain =
      readParameter("color_enhancer/value_gain", 1.0);
  raw_image_pipeline_->setColorEnhancerValueGain(color_enhancer_value_gain);

  // Undistortion parameters
  bool run_undistortion = readParameter("undistortion/enabled", false);
  raw_image_pipeline_->setUndistortion(run_undistortion);

  double undistortion_balance = readParameter("undistortion/balance", 0.0);
  raw_image_pipeline_->setUndistortionBalance(undistortion_balance);

  double undistortion_fov_scale = readParameter("undistortion/fov_scale", 1.0);
  raw_image_pipeline_->setUndistortionFovScale(undistortion_fov_scale);

  // Check if we have the calibration file path
  std::string calibration_file =
      readParameter("undistortion/calibration_file", std::string(""));
  raw_image_pipeline_->loadCameraCalibration(calibration_file);

  cv::Mat rect_projection_matrix =
      raw_image_pipeline_->getRectProjectionMatrix();
  cv::Mat rect_distortion_coeff =
      raw_image_pipeline_->getRectDistortionCoefficients();
  std::cerr << "K: [";
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      if ((i == 0) && (j == 0)) {
      } else {
        std::cerr << ", ";
      }
      std::cerr << std::fixed << std::setprecision(10)
                << rect_projection_matrix.at<double>(i, j);
    }
  }
  std::cerr << "]\n";
  std::cerr << "D: [0.0, 0.0, 0.0, 0.0]\n";

  if (calibration_file.empty()) {
    int image_width = readParameter("undistortion/image_width", 640);
    int image_height = readParameter("undistortion/image_width", 480);
    raw_image_pipeline_->setUndistortionImageSize(image_width, image_height);

    std::vector<double> camera_matrix = readParameter(
        "undistortion/camera_matrix/data",
        std::vector<double>({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}));
    raw_image_pipeline_->setUndistortionCameraMatrix(camera_matrix);

    std::vector<double> distortion_coeff =
        readParameter("undistortion/distortion_coefficients/data",
                      std::vector<double>({0.0, 0.0, 0.0, 0.0}));
    raw_image_pipeline_->setUndistortionDistortionCoefficients(
        distortion_coeff);

    std::string distortion_model =
        readParameter("undistortion/distortion_model", std::string("none"));
    raw_image_pipeline_->setUndistortionDistortionModel(distortion_model);

    std::vector<double> rectification_matrix = readParameter(
        "undistortion/rectification_matrix/data",
        std::vector<double>({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}));
    raw_image_pipeline_->setUndistortionRectificationMatrix(
        rectification_matrix);

    std::vector<double> projection_matrix =
        readParameter("undistortion/projection_matrix/data",
                      std::vector<double>({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                           0.0, 0.0, 0.0, 1.0, 0.0}));
    raw_image_pipeline_->setUndistortionProjectionMatrix(projection_matrix);

    // Init undistortion module
    raw_image_pipeline_->initUndistortion();
  }
}

void RawImagePipelineRos::setupRos() {
  constexpr size_t ros_queue_size =
      1u;  // We always process the most updated frame

  // Set up the raw image subscriber.

  auto transport_hint =
      std::make_shared<image_transport::TransportHints>(this, transport_);

  // Subscribe image
  sub_raw_image_ = image_transport_->subscribe(
      input_topic_,                               // topic
      ros_queue_size,                             // queue size
      &RawImagePipelineRos::imageCallback, this,  // callback
      transport_hint.get()                        // hints
  );

  // Set up the processed image publisher.
  if (raw_image_pipeline_->isUndistortionEnabled() && pub_rect_) {
    pub_image_rect_ = image_transport_->advertiseCamera(
        output_prefix_ + "/rectified/image", ros_queue_size);
  }

  if (raw_image_pipeline_->isDebayerEnabled() && pub_debayer_) {
    pub_image_debayered_ = image_transport_->advertiseCamera(
        output_prefix_ + "/debayered/image", ros_queue_size);
  }

  if ((raw_image_pipeline_->isFlipEnabled() ||
       raw_image_pipeline_->isWhiteBalanceEnabled() ||
       raw_image_pipeline_->isColorCalibrationEnabled() ||
       raw_image_pipeline_->isGammaCorrectionEnabled() ||
       raw_image_pipeline_->isVignettingCorrectionEnabled() ||
       raw_image_pipeline_->isColorEnhancerEnabled()) &&
      pub_color_) {
    pub_image_color_ = image_transport_->advertiseCamera(
        output_prefix_ + "/color/image", ros_queue_size);
  }

}  // namespace raw_image_pipeline

void RawImagePipelineRos::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
  bool rect_subs = pub_image_rect_.getNumSubscribers() > 0;
  bool debayer_subs = pub_image_debayered_.getNumSubscribers() > 0;
  bool colour_subs = pub_image_color_.getNumSubscribers() > 0;

  if (skipped_images_ >= skip_number_of_images_) {
    skipped_images_ = 0;
  } else {
    skipped_images_++;
    return;
  }

  // early return if there is no subscribers
  if (!rect_subs && !debayer_subs && !colour_subs) {
    return;
  }

  // Copy Ros msg to opencv
  // CHECK_NOTNULL(image_msg);
  cv_bridge::CvImagePtr cv_ptr_processed;

  if (transport_ != "raw")
    cv_ptr_processed = cv_bridge::toCvCopy(image_msg, "bgr8");
  else
    cv_ptr_processed = cv_bridge::toCvCopy(image_msg, image_msg->encoding);

  // CHECK_NOTNULL(cv_ptr_processed);

  if (cv_ptr_processed->image.empty()) {
    RCLCPP_WARN(this->get_logger(), "image empty");
    return;
  }

  // Run image proc cuda pipeline
  raw_image_pipeline_->apply(cv_ptr_processed->image,
                             cv_ptr_processed->encoding);

  // Publish undistorted only when there is a subscriber
  if (raw_image_pipeline_->isUndistortionEnabled() && pub_rect_ && rect_subs) {
    // Publish undistorted
    publishColorImage(
        cv_ptr_processed,  // Processed
        image_msg,         // Original image
        raw_image_pipeline_->getRectImageHeight(),
        raw_image_pipeline_->getRectImageWidth(),  // Dimensions
        raw_image_pipeline_->getRectDistortionModel(),
        raw_image_pipeline_
            ->getRectDistortionCoefficients(),  // Distortion stuff
        raw_image_pipeline_->getRectCameraMatrix(),
        raw_image_pipeline_->getRectRectificationMatrix(),
        raw_image_pipeline_->getRectProjectionMatrix(),  // Pinhole stuff
        pub_image_rect_                                  // Publishers
    );
  }

  if (raw_image_pipeline_->isDebayerEnabled() && pub_debayer_ && debayer_subs) {
    // Publish debayered image
    cv_ptr_processed->image = raw_image_pipeline_->getDistDebayeredImage();
    publishColorImage(
        cv_ptr_processed,  // Processed
        image_msg,         // Original image
        raw_image_pipeline_->getDistImageHeight(),
        raw_image_pipeline_->getDistImageWidth(),  // Dimensions
        raw_image_pipeline_->getDistDistortionModel(),
        raw_image_pipeline_
            ->getDistDistortionCoefficients(),  // Distortion stuff
        raw_image_pipeline_->getDistCameraMatrix(),
        raw_image_pipeline_->getDistRectificationMatrix(),
        raw_image_pipeline_->getDistProjectionMatrix(),  // Pinhole stuff
        pub_image_debayered_                             // Publishers
    );
  }

  if ((raw_image_pipeline_->isFlipEnabled() ||
       raw_image_pipeline_->isWhiteBalanceEnabled() ||
       raw_image_pipeline_->isColorCalibrationEnabled() ||
       raw_image_pipeline_->isGammaCorrectionEnabled() ||
       raw_image_pipeline_->isVignettingCorrectionEnabled() ||
       raw_image_pipeline_->isColorEnhancerEnabled()) &&
      pub_color_ && colour_subs) {
    // Publish color image
    cv_ptr_processed->image = raw_image_pipeline_->getDistColorImage();
    publishColorImage(
        cv_ptr_processed,  // Processed
        image_msg,         // Original image
        raw_image_pipeline_->getDistImageHeight(),
        raw_image_pipeline_->getDistImageWidth(),  // Dimensions
        raw_image_pipeline_->getDistDistortionModel(),
        raw_image_pipeline_
            ->getDistDistortionCoefficients(),  // Distortion stuff
        raw_image_pipeline_->getDistCameraMatrix(),
        raw_image_pipeline_->getDistRectificationMatrix(),
        raw_image_pipeline_->getDistProjectionMatrix(),  // Pinhole stuff
        pub_image_color_                                 // Publishers
    );
  }
}

void RawImagePipelineRos::publishColorImage(
    const cv_bridge::CvImagePtr& cv_ptr_processed,              //
    const sensor_msgs::msg::Image::ConstSharedPtr& orig_image,  //
    int image_height, int image_width,                          // Dimensions
    const std::string& distortion_model,
    const cv::Mat& distortion_coefficients,  //
    const cv::Mat& camera_matrix, const cv::Mat& rectification_matrix,
    const cv::Mat& projection_matrix,  //
    image_transport::CameraPublisher& camera_publisher) {
  // Note: Image is BGR
  // Convert image to output encoding
  if (output_encoding_ == "RGB") {
    cv::cvtColor(cv_ptr_processed->image, cv_ptr_processed->image,
                 cv::COLOR_BGR2RGB);
  }

  // Copy to ROS
  // CHECK_NOTNULL(cv_ptr_processed);
  // CHECK_NOTNULL(orig_image);
  const sensor_msgs::msg::Image::SharedPtr color_img_msg =
      cv_ptr_processed->toImageMsg();
  // CHECK_NOTNULL(color_img_msg);

  // Set output encoding
  if (cv_ptr_processed->image.channels() == 3) {
    if (output_encoding_ == "RGB")
      color_img_msg->encoding = sensor_msgs::image_encodings::RGB8;
    else if (output_encoding_ == "BGR")
      color_img_msg->encoding = sensor_msgs::image_encodings::BGR8;
    else
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Found invalid image encoding: "
                              << output_encoding_
                              << ", make sure to set a supported ouput "
                                 "encoding (either 'RGB' or 'BGR')");
  }

  sensor_msgs::msg::CameraInfo::SharedPtr color_camera_info_msg(
      new sensor_msgs::msg::CameraInfo());

  // Fix output frame if required
  if (output_frame_ == "passthrough") {
    color_camera_info_msg->header.frame_id = orig_image->header.frame_id;
  } else {
    color_img_msg->header.frame_id = output_frame_;
    color_camera_info_msg->header.frame_id = output_frame_;
  }
  color_camera_info_msg->header.stamp = orig_image->header.stamp;
  color_camera_info_msg->height = image_height;
  color_camera_info_msg->width = image_width;

  // Fix distortion model if it's none
  if (color_camera_info_msg->distortion_model == "none") {
    color_camera_info_msg->distortion_model = "plumb_bob";
  } else {
    color_camera_info_msg->distortion_model = distortion_model;
  }

  // Other calibration stuff
  color_camera_info_msg->d =
      utils::toStdVector<double>(distortion_coefficients);
  color_camera_info_msg->k = utils::toStdArray<double, 9>(camera_matrix);
  color_camera_info_msg->r = utils::toStdArray<double, 9>(rectification_matrix);
  color_camera_info_msg->p = utils::toStdArray<double, 12>(projection_matrix);

  // Assign the original timestamp and publish
  color_img_msg->header.stamp = orig_image->header.stamp;

  // Publish to topic
  camera_publisher.publish(color_img_msg, color_camera_info_msg);
}

std::string RawImagePipelineRos::getTransportHintFromTopic(
    std::string& image_topic) {
  // This method modifies the input topic

  std::string transport_hint = "compressed";
  std::size_t ind = image_topic.find(
      transport_hint);  // Find if compressed is in the topic name

  if (ind != std::string::npos) {
    transport_hint = image_topic.substr(
        ind, image_topic.length());  // Get the hint as the last part
    image_topic.erase(
        ind - 1, image_topic.length());  // We remove the hint from the topic
  } else {
    transport_hint = "raw";  // In the default case we assume raw topic
  }
  return transport_hint;
}

std::vector<double> RawImagePipelineRos::readParameter(
    const std::string& param, std::vector<double> default_value) {
  this->declare_parameter(param, default_value);

  std::stringstream ss;  // Prepare default string
  for (auto v : default_value) {
    ss << v << " ";
  }
  std::vector<double> value;
  if (!this->get_parameter(param, value)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "could not get ["
                                               << param << "], using default "
                                               << ss.str());
  }
  return value;
}

}  // namespace raw_image_pipeline

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(raw_image_pipeline::RawImagePipelineRos)
