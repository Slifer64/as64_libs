#ifndef APRILTAGS_ROS_GUI_MAIN_WINDOW_H
#define APRILTAGS_ROS_GUI_MAIN_WINDOW_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QMainWindow>
#include <QRadioButton>
#include <QDialogButtonBox>
#include <QButtonGroup>

#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>

namespace apriltags_ros
{

class AprilTagDetector; // forward decleration

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(AprilTagDetector *tag_detector, QWidget *parent = 0);
    ~MainWindow();

signals:
  void closeSignal();

private:

  AprilTagDetector *tag_detector;

  QCheckBox *apply_filt_ckbox;
  QLineEdit *ap_le;
  QLineEdit *aq_le;
  QLineEdit *miss_frames_tol_le;

  QCheckBox *publish_ckbox;
  QCheckBox *publish_tag_tf_ckbox;
  QCheckBox *publish_tag_im_ckbox;
  QLineEdit *pub_rate_le;

  QWidget *central_widget;

}; // class MainWindow

} // namespace apriltags_ros

#endif // APRILTAGS_ROS_GUI_MAIN_WINDOW_H
