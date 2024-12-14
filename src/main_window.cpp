/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/object_estimator/main_window.hpp"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(imgSignalEmit()), this, SLOT(imageUpdateSlot()));
}

void MainWindow::imageUpdateSlot()
{
  QImage qraw_img(
      (const unsigned char *)(qnode->resize_img.data),
      qnode->resize_img.cols,
      qnode->resize_img.rows,
      QImage::Format_RGB888);
  ui->img->setPixmap(QPixmap::fromImage(qraw_img.rgbSwapped()));

  QImage qaruco_img(
      (const unsigned char *)(qnode->aruco_img.data),
      qnode->aruco_img.cols,
      qnode->aruco_img.rows,
      QImage::Format_RGB888);
  ui->aruco->setPixmap(QPixmap::fromImage(qaruco_img.rgbSwapped()));

  QImage qresult_img(
      (const unsigned char *)(qnode->roi_img.data),
      qnode->roi_img.cols,
      qnode->roi_img.rows,
      QImage::Format_RGB888);
  ui->result->setPixmap(QPixmap::fromImage(qresult_img.rgbSwapped()));

  ui->camera_matrix->setText(QString(qnode->camera_matrix_str_.c_str()));
  ui->distortion_coefficients->setText(QString(qnode->distortion_coefficients_str_.c_str()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}
