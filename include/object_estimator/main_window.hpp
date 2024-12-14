/**
 * @file /include/object_estimator/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef object_estimator_MAIN_WINDOW_H
#define object_estimator_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QMainWindow>
#include <QImage>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

public Q_SLOTS:
  void imageUpdateSlot();

private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
};

#endif  // object_estimator_MAIN_WINDOW_H
