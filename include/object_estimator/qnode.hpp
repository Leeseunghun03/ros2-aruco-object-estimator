/**
 * @file /include/object_estimator/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef object_estimator_QNODE_HPP_
#define object_estimator_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();
  cv::Mat resize_img;
  cv::Mat aruco_img;
  cv::Mat roi_img;

  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

  std::string camera_matrix_str_;
  std::string distortion_coefficients_str_;

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_publisher_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void publishPoints(const std::vector<cv::Point2f> &points, float scale);

  void getROI();
  void estimateProcess();

  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Point2i top_left_corner_result;

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;

  cv::Size roi_size;
  cv::Mat perspectiveMatrix;

Q_SIGNALS:
  void rosShutDown();
  void imgSignalEmit();
};

#endif /* object_estimator_QNODE_HPP_ */
