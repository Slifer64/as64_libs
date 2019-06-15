#ifndef AS64_QTPLOT_H
#define AS64_QTPLOT_H

#include <QDialog>
#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QVector>
#include <QApplication>
#include <QMainWindow>

#include <vector>
#include <cstring>
#include <exception>
#include <memory>
#include <armadillo>

#include "qcustomplot.h"

#include <plot_lib/utils.h>

// Q_DECLARE_METATYPE(QVector<QString>);

namespace as64_
{

namespace pl_
{

class Figure; // forward declaration
class Axes; // forward declaration

enum PROPERTY
{
  Color_,
  LineWidth_,
  LineStyle_,
  MarkerSize_,
  MarkerStyle_,
  FontSize_,
  FontWeight_,
  FontFamily_,
};

enum Color {
  BLUE = 0,        //   0   0   255
  GREEN = 1,       //   0  255    0
  MAGENTA = 2,     // 255    0  255
  BROWN = 3,       // 153   51    0
  CYAN = 4,        //   0  255  255
  RED = 5,         // 255    0    0
  YELLOW = 6,      // 230  230    0
  LIGHT_BROWN = 7, // 217   84   26
  PURPLE = 8,      // 125   46  143
  MUSTARD = 9,     // 237  176   33
  PINK = 10,        // 255  153  199
  BLACK = 11,       //   0    0    0
  GREY = 12,        // 200  200  200
};

enum LineStyle {
  NoLine = 0,
  SolidLine = 1,
  DashLine = 2,
  DotLine = 3,
  DashDotLine = 4,
  DashDotDotLine = 5
};

enum MarkerStyle {
  ssNone,
  ssDot,
  ssCross,
  ssPlus,
  ssCircle,
  ssDisc,
  ssSquare,
  ssDiamond,
  ssStar,
  ssTriangle,
  ssTriangleInverted,
  ssCrossSquare,
  ssPlusSquare,
  ssCrossCircle,
  ssPlusCircle,
  ssPeace
};

enum FontWeight {
  Light = 25,
  Normal = 50,
  DemiBold = 63,
  Bold = 75,
  Black = 87
};

class Graph : public QWidget {

Q_OBJECT

public:
  Graph(QCPGraph *qcp_graph, Axes *parent = 0);

  ~Graph();

  template<typename T, typename... Arguments>
  void setProperty(pl_::PROPERTY p, T p_value, Arguments... parameters)
  {
    setPropertyHelper(p, p_value);
    setProperty(parameters...);
  }

  void setColor(Color color);

  void setLineStyle(LineStyle style);

  void setLineWidth(double width);

  void setMarkerStyle(MarkerStyle type);

  void setMarkerSize(double size);

signals:

  void setColorSignal(int color);

  void setLineStyleSignal(int style);

  void setLineWidthSignal(double width);

  void setMarkerStyleSignal(int type);

  void setMarkerSizeSignal(double size);

private slots:

  void setColorSlot(int color);

  void setLineStyleSlot(int style);

  void setLineWidthSlot(double width);

  void setMarkerStyleSlot(int type);

  void setMarkerSizeSlot(double size);

private:
  void setProperty();
  void setPropertyHelper(pl_::PROPERTY p, pl_::Color p_value);
  void setPropertyHelper(pl_::PROPERTY p, double p_value);
  void setPropertyHelper(pl_::PROPERTY p, pl_::LineStyle p_value);
  void setPropertyHelper(pl_::PROPERTY p, pl_::MarkerStyle p_value);

  Axes *parent;
  QCPGraph *qcp_graph;

  Semaphore sem;
};

class Axes : public QCustomPlot {
Q_OBJECT

public:

  Axes(Figure *parent=0);

  ~Axes();

  void hold(bool set);

  void grid(bool set);

  Graph *plot(const arma::vec &data);
  Graph *plot(const arma::rowvec &data);

  Graph *plot(const arma::vec &x_data, const arma::vec &y_data);
  Graph *plot(const arma::rowvec &x_data, const arma::rowvec &y_data);

  template<typename... Arguments>
  Graph *plot(const arma::rowvec  &x, const arma::rowvec  &y, Arguments... properties)
  {
    Graph *graph = plot(x, y);
    graph->setProperty(properties...);
    return graph;
  }

  template<typename... Arguments>
  Graph *plot(const arma::vec  &x, const arma::vec  &y, Arguments... properties)
  {
    Graph *graph = plot(x, y);
    graph->setProperty(properties...);
    return graph;
  }

  void title(const std::string &title);

  void xlabel(const std::string &label);

  void ylabel(const std::string &label);

  void legend(const std::vector<std::string> &legend_labels);

  void drawnow();

signals:

  void holdSignal(bool set);

  void gridSignal(bool set);

  void plotSignal(const void *x_data, const void *y_data);

  void setTitleSignal(const QString &title);

  void setXLabelSignal(const QString &label);

  void setYLabelSignal(const QString &label);

  void setLegendSignal(const QVector<QString> &legend_labels);

  void drawnowSignal();

private slots:

  void holdSlot(bool set);

  void gridSlot(bool set);

  void plotSlot(const void *x_data, const void *y_data);

  void setTitleSlot(const QString &title);

  void setXLabelSlot(const QString &label);

  void setYLabelSlot(const QString &label);

  void setLegendSlot(const QVector<QString> &legend_labels);

  void drawnowSlot();


private:
  bool hold_on;
  Figure *parent;
  std::vector<Graph *> graphs;

  QCPAxisRect *axes;
  QCPLayoutGrid *axes_grid;

  int color_ind;

  Graph *last_graph;

  Semaphore sem;
};

class Figure : public QMainWindow {
Q_OBJECT

public:

  Figure(QWidget *parent = 0);

  ~Figure();

  Axes *getAxes(int k);

  Axes *getAxes(int row, int col);

  void setAxes(int n1, int n2);

  void clearAxes(int k = -1);

signals:

  void setAxesSignal(int n1, int n2);

  void clearAxesSignal(int k = -1);

private slots:

  void setAxesSlot(int n1, int n2);

  void clearAxesSlot(int k = -1);

  void closeEvent(QCloseEvent *event) override;

private:
  QGridLayout *grid_layout;
  std::vector<Axes *> axes;
  QWidget *central_widget;
  int n1;
  int n2;

  int getAxesIndex(int i, int j) { return j + i * n2; }

  Semaphore sem;
};

class QtPlot : public QWidget {
Q_OBJECT

public:
  QtPlot(QWidget *parent = 0);

  ~QtPlot();

  static void init(QWidget *parent = 0);
  static void terminate();

  static Figure *figure();

  static int fig_count;

  static QColor getQColor(Color c);

  static const int N_COLORS;

signals:

  void figureSignal(Figure **);
  void terminateSignal();

private slots:

  void figureSlot(Figure **);

private:
  static bool initialized;
  Semaphore sem;
  static QtPlot *QtPlot_;
  static std::map<Color, QColor> color;
};

} // namespace pl_

} // namespace as64_

#endif // AS64_QTPLOT_H
