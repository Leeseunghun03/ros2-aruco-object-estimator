#include "../include/object_estimator/qnode.hpp"

bool camera_info_received_ = false;
bool roi_init_done = false;

QNode::QNode()
{
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("object_estimator");

  image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      "camera/camera/color/image_raw", 10, std::bind(&QNode::imageCallback, this, std::placeholders::_1));

  camera_info_subscriber_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera/camera/color/camera_info", 10, std::bind(&QNode::cameraInfoCallback, this, std::placeholders::_1));

  image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);

  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::spin(node);
  Q_EMIT rosShutDown();
}

void QNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (camera_info_received_)
  {
    return;
  }

  if (msg == nullptr)
  {
    RCLCPP_ERROR(node->get_logger(), "Received null camera info message!");
    return;
  }

  camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
  distortion_coefficients_ = cv::Mat(1, msg->d.size(), CV_64F, const_cast<double *>(msg->d.data())).clone();

  camera_info_received_ = true;

  std::ostringstream camera_matrix_ss;
  camera_matrix_ss << camera_matrix_;
  camera_matrix_str_ = camera_matrix_ss.str();

  std::ostringstream distortion_coefficients_ss;
  distortion_coefficients_ss << distortion_coefficients_;
  distortion_coefficients_str_ = distortion_coefficients_ss.str();

  std::cout << "Camera matrix: " << std::endl
            << camera_matrix_ << std::endl;
  std::cout << "Distortion coefficients: " << distortion_coefficients_ << std::endl;

  camera_info_subscriber_.reset();
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg == nullptr)
  {
    RCLCPP_ERROR(node->get_logger(), "Received null image message!");
    return;
  }

  if (!camera_info_received_)
  {
    RCLCPP_WARN(node->get_logger(), "Camera info not yet received. Skipping image processing.");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  cv::Mat raw_image = cv_ptr->image.clone();
  cv::resize(raw_image, resize_img, cv::Size(320, 240), 0, 0, CV_INTER_LINEAR);

  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  cv::aruco::detectMarkers(resize_img, dictionary, marker_corners, marker_ids);
  if (marker_ids.empty())
  {
    // RCLCPP_WARN(node->get_logger(), "No ArUco markers detected in the image.");
  }
  else if (!roi_init_done)
  {
    aruco_img = resize_img.clone();
    cv::aruco::drawDetectedMarkers(aruco_img, marker_corners, marker_ids);
    getROI();
  }
  else
  {
    estimateProcess();
  }

  Q_EMIT imgSignalEmit();
}

void QNode::estimateProcess()
{
  std::vector<int> object_marker_ids = {1, 2, 3};

  cv::warpPerspective(resize_img, roi_img, perspectiveMatrix, roi_size);
  cv::resize(roi_img, roi_img, cv::Size(480, 480), 0, 0, cv::INTER_LINEAR);

  cv::aruco::detectMarkers(roi_img, dictionary, marker_corners, marker_ids);

  if (marker_ids.empty())
  {
    RCLCPP_WARN(node->get_logger(), "No markers detected in the ROI.");
    return;
  }

  cv::aruco::drawDetectedMarkers(roi_img, marker_corners, marker_ids, cv::Scalar(0, 255, 0));

  for (size_t i = 0; i < marker_ids.size(); ++i)
  {
    int detected_id = marker_ids[i];

    if (std::find(object_marker_ids.begin(), object_marker_ids.end(), detected_id) != object_marker_ids.end())
    {
      cv::Point2f center(0, 0);
      for (const auto &corner : marker_corners[i])
      {
        center += corner;
      }
      center *= 0.25f;

      cv::line(roi_img, cv::Point(0, 0), center, cv::Scalar(255, 0, 0), 2);

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.037, camera_matrix_, distortion_coefficients_, rvecs, tvecs);

      cv::Vec3d rvec = rvecs[i];
      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);

      double yaw = std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
      double yaw_degrees = yaw * 180.0 / CV_PI;

      RCLCPP_INFO(node->get_logger(), "Yaw of object marker ID %d: %.2f degrees", detected_id, yaw_degrees);
    }
  }
}

void QNode::getROI()
{
  int target_marker_id = 0;
  auto it = std::find(marker_ids.begin(), marker_ids.end(), target_marker_id);
  if (it == marker_ids.end())
  {
    RCLCPP_WARN(node->get_logger(), "Target ArUco marker ID %d not detected.", target_marker_id);
  }
  else
  {
    size_t index = std::distance(marker_ids.begin(), it);

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.037, camera_matrix_, distortion_coefficients_, rvecs, tvecs);

    cv::Vec3d rvec = rvecs[index];
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    double yaw = std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
    double yaw_degrees = yaw * 180.0 / CV_PI;

    if (std::abs(yaw_degrees) <= 1.0)
    {
      cv::Point2f top_left_corner = marker_corners[index][0];
      top_left_corner_result = cv::Point2i(static_cast<int>(top_left_corner.x), static_cast<int>(top_left_corner.y));
      RCLCPP_INFO(node->get_logger(), "Top-left corner of ArUco marker ID %d: (%d, %d)", target_marker_id, top_left_corner_result.x, top_left_corner_result.y);

      float marker_real_size = 37.0;
      float roi_size_mm = 450.0;

      float marker_image_size = cv::norm(marker_corners[index][0] - marker_corners[index][1]);

      float scale = marker_real_size / marker_image_size;

      float roi_size_px = roi_size_mm / scale;

      int roi_x = static_cast<int>(top_left_corner.x);
      int roi_y = static_cast<int>(top_left_corner.y);

      cv::Mat rotationMatrix = cv::getRotationMatrix2D(top_left_corner, -yaw_degrees, 1.0);

      cv::Point2f top_right(roi_x + roi_size_px, roi_y);
      cv::Point2f bottom_left(roi_x, roi_y + roi_size_px);
      cv::Point2f bottom_right(roi_x + roi_size_px, roi_y + roi_size_px);

      std::vector<cv::Point2f> roi_corners = {top_left_corner, top_right, bottom_right, bottom_left};
      std::vector<cv::Point2f> rotated_corners;
      cv::transform(roi_corners, rotated_corners, rotationMatrix);

      std::vector<cv::Point2f> dst_points = {cv::Point2f(0, 0), cv::Point2f(roi_size_px, 0),
                                             cv::Point2f(roi_size_px, roi_size_px),
                                             cv::Point2f(0, roi_size_px)};
      perspectiveMatrix = cv::getPerspectiveTransform(rotated_corners, dst_points);
      roi_size = cv::Size(roi_size_px, roi_size_px);

      if (rotated_corners[0].x >= 0 && rotated_corners[0].y >= 0 && rotated_corners[2].x <= resize_img.cols && rotated_corners[2].y <= resize_img.rows)
      {
        roi_init_done = true;
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "ROI exceeds image boundaries.");
      }
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "Yaw of ArUco marker ID %d: %.2f degrees", target_marker_id, yaw_degrees);
    }

    RCLCPP_INFO(node->get_logger(), "Yaw of ArUco marker ID %d: %.2f degrees", target_marker_id, yaw_degrees);
    cv::aruco::drawAxis(aruco_img, camera_matrix_, distortion_coefficients_, rvec, tvecs[index], 0.1);
  }
}