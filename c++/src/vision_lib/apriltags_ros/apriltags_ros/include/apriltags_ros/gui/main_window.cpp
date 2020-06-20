#include <apriltags_ros/gui/main_window.h>
#include <apriltags_ros/apriltag_detector.h>

#include <QDebug>
#include <iostream>

namespace apriltags_ros
{

MainWindow::MainWindow(AprilTagDetector *tag_detector_, QWidget *parent): QMainWindow(parent)
{
  this->tag_detector = tag_detector_;

  this->resize(200,200);
  this->setWindowTitle("Apriltag Detector");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  // status_bar = new QStatusBar(this);
  // this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  // =============  Fonts  ================

  QFont font1("Ubuntu", 17, QFont::DemiBold);
  QFont font2("Ubuntu", 15, QFont::DemiBold);
  QFont font3("Ubuntu", 14, 57);
  QFont font4("Ubuntu", 12, QFont::Normal);


  // =============  Widgets  ================

  // miss_frames_tol: 5
  // pub_rate_sec: 0.033

  // ======================================================

  apply_filt_ckbox = new QCheckBox("apply filter");
  apply_filt_ckbox->setFont(font2);
  apply_filt_ckbox->setChecked(tag_detector->apply_filter);

  QLabel *ap_lb = new QLabel("a_p:");
  ap_lb->setFont(font2);
  ap_le = new QLineEdit; // (QString::number(tag_detector->a_p.get()));
  ap_le->setFont(font2);
  ap_le->setAlignment(Qt::AlignCenter);

  QLabel *aq_lb = new QLabel("a_q:");
  aq_lb->setFont(font2);
  aq_le = new QLineEdit; // (QString::number(tag_detector->a_q.get()));
  aq_le->setFont(font2);
  aq_le->setAlignment(Qt::AlignCenter);

  QGridLayout *filter_layout = new QGridLayout;
  filter_layout->addWidget(apply_filt_ckbox, 0,0);
  filter_layout->addWidget(ap_lb, 1,0);
  filter_layout->addWidget(ap_le, 1,1);
  filter_layout->addWidget(aq_lb, 2,0);
  filter_layout->addWidget(aq_le, 2,1);

  QFrame *filter_frame = new QFrame;
  filter_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  filter_frame->setLineWidth(1);
  filter_frame->setLayout(filter_layout);

  // ======================================================

  publish_ckbox = new QCheckBox("Publish");
  publish_ckbox->setFont(font2);
  publish_ckbox->setChecked(tag_detector->publish_);
  publish_tag_tf_ckbox = new QCheckBox("Publish tags TF");
  publish_tag_tf_ckbox->setFont(font2);
  publish_tag_tf_ckbox->setChecked(tag_detector->publish_tag_tf && tag_detector->publish_);
  publish_tag_im_ckbox = new QCheckBox("Publish tags image");
  publish_tag_im_ckbox->setFont(font2);
  publish_tag_im_ckbox->setChecked(tag_detector->publish_tag_im && tag_detector->publish_);

  QLabel *pub_rate_lb = new QLabel("Publish rate (sec):");
  pub_rate_lb->setFont(font2);
  pub_rate_le = new QLineEdit("0");
  pub_rate_le->setFont(font2);
  pub_rate_le->setAlignment(Qt::AlignCenter);

  QVBoxLayout *pub_layout = new QVBoxLayout;
  pub_layout->addWidget(publish_ckbox);
  pub_layout->addWidget(publish_tag_tf_ckbox);
  pub_layout->addWidget(publish_tag_im_ckbox);
  QHBoxLayout *pub_rate_layout = new QHBoxLayout;
  pub_rate_layout->addWidget(pub_rate_lb);
  pub_rate_layout->addWidget(pub_rate_le);
  pub_rate_layout->addStretch(0);
  pub_layout->addLayout(pub_rate_layout);
  pub_layout->addStretch(0);

  QFrame *pub_frame = new QFrame;
  pub_frame->setFrameStyle(QFrame::Box | QFrame::Raised);
  pub_frame->setLineWidth(1);
  pub_frame->setLayout(pub_layout);

  // ======================================================

  QHBoxLayout *main_layout = new QHBoxLayout(central_widget);
  main_layout->addWidget(pub_frame);
  main_layout->addWidget(filter_frame);
  main_layout->addStretch(0);

  // ================  Connections  ====================

  QObject::connect( apply_filt_ckbox, &QCheckBox::stateChanged, [this]()
  {
    bool set = apply_filt_ckbox->isChecked();
    ap_le->setEnabled(set);
    aq_le->setEnabled(set);
    tag_detector->setFilter(set);
    ap_le->setText(QString::number(tag_detector->a_p.get()));
    aq_le->setText(QString::number(tag_detector->a_q.get()));
  });

  QObject::connect( ap_le, &QLineEdit::editingFinished, [this]()
  { tag_detector->a_p.set(ap_le->text().toDouble()); });

  QObject::connect( aq_le, &QLineEdit::editingFinished, [this]()
  { tag_detector->a_q.set(aq_le->text().toDouble()); });


  QObject::connect( publish_ckbox, &QCheckBox::stateChanged, [this]()
  {
    bool set = publish_ckbox->isChecked();
    tag_detector->publish_ = set;

    if (!set)
    {
      publish_tag_tf_ckbox->setChecked(false);
      emit publish_tag_tf_ckbox->stateChanged(Qt::Unchecked);
      publish_tag_im_ckbox->setChecked(false);
      emit publish_tag_im_ckbox->stateChanged(Qt::Unchecked);
    }
    else tag_detector->launchPublishThread();

    publish_tag_tf_ckbox->setEnabled(set);
    publish_tag_im_ckbox->setEnabled(set);
  });

  QObject::connect( publish_tag_tf_ckbox, &QCheckBox::stateChanged, [this]()
  {
    bool set = publish_tag_tf_ckbox->isChecked();
    tag_detector->publish_tag_tf = set;
  });

  QObject::connect( publish_tag_im_ckbox, &QCheckBox::stateChanged, [this]()
  {
    bool set = publish_tag_im_ckbox->isChecked();
    tag_detector->publish_tag_im = set;
  });

  QObject::connect( this, SIGNAL(closeSignal()), this, SLOT(close()) );

  // ===============  initialize  =================
  emit publish_ckbox->stateChanged(Qt::PartiallyChecked);
  emit apply_filt_ckbox->stateChanged(Qt::PartiallyChecked);
}

MainWindow::~MainWindow()
{}

}