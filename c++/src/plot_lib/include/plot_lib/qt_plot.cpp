#include "qt_plot.h"
#include <QDebug>
#include <QCoreApplication>

#include <thread>


// ===========================================================

namespace as64_
{

namespace pl_
{

QtPlot *QtPlot::QtPlot_;
bool QtPlot::initialized = false;
int QtPlot::fig_count = 0;
std::map<Color, QColor> QtPlot::color;
const int QtPlot::N_COLORS = 13;

void QtPlot::init(QWidget *parent)
{
  if (!initialized)
  {
    // if (!QCoreApplication::instance()) throw std::runtime_error("[QtPlot::init]: There is no QApplication...");

    if (!QCoreApplication::instance())
    {
      Semaphore sem;

      std::thread([&sem]()
      {
        int argc = 0;
        char **argv = 0;
        QApplication app(argc, argv);
        QThread::currentThread()->setPriority(QThread::LowestPriority);
        app.setQuitOnLastWindowClosed(false);
//        QMainWindow *win = new QMainWindow();
//        win->setVisible(false);
        QtPlot::QtPlot_ = new QtPlot();
        QObject::connect(QtPlot_, SIGNAL(terminateSignal()), &app, SLOT(quit()));
        sem.notify();
        app.exec();

        std::cerr << "[QtPlot::init::thread]: Finished exec!\n";

        //delete (win);
        delete (QtPlot_);
      }).detach();

       sem.wait();
    }
    else
    {
      if (QCoreApplication::instance()->thread()->currentThreadId() != QThread::currentThread()->currentThreadId())
      {
        throw std::runtime_error("[QtPlot::init]: Must be called from the QApplication thread!");
      }
      QtPlot::QtPlot_ = new QtPlot(parent);
      // QtPlot::QtPlot_->moveToThread( QApplication::instance()->thread() );
    }

    initialized = true;
  }
  else
  {
    std::cerr << "\033[1m" << "\033[33m" << "[WARNING]: QtPlot has already been initialized!\033[0m\n";
  }
}

void QtPlot::terminate()
{
  QtPlot::QtPlot_->terminateSignal();
}

QColor QtPlot::getQColor(Color c)
{
  return (QtPlot::color.find(static_cast<Color>(c)))->second;
}

Figure *QtPlot::figure()
{
  if (!QtPlot::initialized) throw std::runtime_error("[QtPlot::figure]: QtPlot has not been initialized...");

  Figure *fig;
  QtPlot::QtPlot_->figureSignal(&fig);
  return fig;
}

QtPlot::QtPlot(QWidget *parent)
{
  QtPlot::color[BLUE] = QColor(0, 0, 255);
  QtPlot::color[GREEN] = QColor(0, 255, 0);
  QtPlot::color[BROWN] = QColor(153, 51, 0);
  QtPlot::color[MAGENTA] = QColor(255, 0, 255);
  QtPlot::color[CYAN] = QColor(0, 255, 255);
  QtPlot::color[RED] = QColor(255, 0, 0);
  QtPlot::color[YELLOW] = QColor(230, 230, 0);
  QtPlot::color[LIGHT_BROWN] = QColor(217, 84, 26);
  QtPlot::color[PURPLE] = QColor(125, 46, 143);
  QtPlot::color[MUSTARD] = QColor(237, 176, 33);
  QtPlot::color[PINK] = QColor(255, 153, 199);
  QtPlot::color[BLACK] = QColor(0, 0, 0);
  QtPlot::color[GREY] = QColor(200, 200, 200);

  QObject::connect(this, SIGNAL(figureSignal(Figure **)), this, SLOT(figureSlot(Figure **)), Qt::BlockingQueuedConnection);
}

QtPlot::~QtPlot()
{
   // std::cerr << "[QtPlot::~QtPlot]: deleting self...\n";
}

void QtPlot::figureSlot(Figure **fig)
{
  *fig = new Figure(this);
  QtPlot::fig_count++;
}


// ===========================================
// =============   Figure   ==================
// ===========================================


Figure::Figure(QWidget *parent) : QMainWindow(parent)
{
  QObject::connect(this, &Figure::setAxesSignal, this, &Figure::setAxesSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Figure::clearAxesSignal, this, &Figure::clearAxesSlot, Qt::BlockingQueuedConnection);

  // this->moveToThread( QApplication::instance()->thread() );

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  QPalette pal = palette();
  pal.setColor(QPalette::Background, Qt::white);
  central_widget->setAutoFillBackground(true);
  central_widget->setPalette(pal);

  if (this->objectName().isEmpty()) this->setObjectName(QStringLiteral("Figure"));
  this->resize(500, 400);
  grid_layout = new QGridLayout(central_widget);
  grid_layout->setObjectName(QStringLiteral("gridLayout"));

  this->setWindowTitle(QString("Figure ") + QString().setNum(QtPlot::fig_count+1));
  this->setAttribute(Qt::WA_QuitOnClose, true);

  setAxesSlot(1,1);

  // this->setWindowTitle(QApplication::translate("PlotDialog", "Dialog", 0));
  show();
}

Figure::~Figure()
{
  // std::cerr << "[Figure::~Figure]: deleting self...\n";
  clearAxesSlot();
  // delete grid_layout;
  QtPlot::fig_count--;
}

void Figure::closeEvent(QCloseEvent *event)
{
  delete this;
}

Axes *Figure::getAxes(int k)
{
  return axes[k];
}

Axes *Figure::getAxes(int row, int col)
{
  return axes[col + row*n2];
}

void Figure::setAxes(int n1, int n2)
{
  setAxesSignal(n1,n2);
}

void Figure::clearAxes(int k)
{
  clearAxesSignal(k);
}

void Figure::setAxesSlot(int n1, int n2)
{
  this->n1 = n1;
  this->n2 = n2;

  clearAxesSlot();
  delete grid_layout;
  grid_layout = new QGridLayout(central_widget);
  // this->setLayout(grid_layout);

  axes.resize(n1*n2);
  for (int i=0; i<n1; i++)
  {
    for (int j=0; j<n2; j++)
    {
      int k = getAxesIndex(i,j);
      axes[k] = new Axes;
      grid_layout->addWidget(axes[k], i, j, 1, 1);
    }
  }
}

void Figure::clearAxesSlot(int k)
{
  if (k < 0)
  {
    for (int i=0;i<axes.size();i++) delete axes[i];
  }
  else
  {
    axes.erase(axes.begin()+k);
  }
}


// =========================================
// =============   Axes   ==================
// =========================================

Axes::Axes(Figure *parent): QCustomPlot(parent)
{
  // qDebug() << "[Axes::Figure]: " << QThread::currentThread() << "\n";

  this->parent = parent;

  qRegisterMetaType<QVector<QString>>("QVector<QString>");

  QObject::connect(this, &Axes::holdSignal, this, &Axes::holdSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Axes::gridSignal, this, &Axes::gridSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Axes::setTitleSignal, this, &Axes::setTitleSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Axes::setXLabelSignal, this, &Axes::setXLabelSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Axes::setYLabelSignal, this, &Axes::setYLabelSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Axes::setLegendSignal, this, &Axes::setLegendSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, SIGNAL(plotSignal(const void *, const void *)), this, SLOT(plotSlot(const void *, const void *)), Qt::BlockingQueuedConnection);
  QObject::connect(this, &Axes::drawnowSignal, this, &Axes::drawnowSlot, Qt::BlockingQueuedConnection);

  this->resize(667, 452);

  this->setAttribute(Qt::WA_DeleteOnClose);

  // cplot = new QCustomPlot;
  // set locale to english, so we get english decimal separator:
  this->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
  // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
  this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

  color_ind = 0;

  holdSlot(false);
  gridSlot(false);

  this->show();
}

Axes::~Axes()
{
  // std::cerr << "[Axes::~Axes]: deleting self...\n";
    // delete this;
}

void Axes::hold(bool set)
{
  holdSignal(set);
}

void Axes::grid(bool set)
{
  gridSignal(set);
}

Graph *Axes::plot(const arma::vec &data)
{
  arma::vec x_data(data.size());
  for (int i=0; i<data.size(); i++) x_data(i) = i;
  return plot(x_data, data);
}

Graph *Axes::plot(const arma::vec &x_data, const arma::vec &y_data)
{
  int n_data = x_data.size();

  if (y_data.size() != n_data) throw std::runtime_error("[Axes::plot]: Incompatible sizes between x_data and y_data");

  QVector<double> qx_data(n_data);
  QVector<double> qy_data(n_data);
  for (int i=0; i<n_data; i++)
  {
    qx_data[i] = x_data(i);
    qy_data[i] = y_data(i);
  }

  plotSignal(reinterpret_cast<const void *>(&qx_data), reinterpret_cast<const void *>(&qy_data));

  return last_graph;
}

Graph *Axes::plot(const arma::rowvec &data)
{
  arma::vec data2 = data.t();
  return plot(data2);
}

Graph *Axes::plot(const arma::rowvec &x_data, const arma::rowvec &y_data)
{
  arma::vec x_data2 = x_data.t();
  arma::vec y_data2 = y_data.t();
  return plot(x_data2, y_data2);
}


TextLabel *Axes::title(const std::string &title_text)
{
  setTitleSignal(QString(title_text.c_str()));
  return this->title_elem;
}

void Axes::xlabel(const std::string &label)
{
  setXLabelSignal(QString(label.c_str()));
}

void Axes::ylabel(const std::string &label)
{
  setYLabelSignal(QString(label.c_str()));
}

void Axes::legend(const std::vector<std::string> &legend_labels)
{
  QVector<QString> qlegend_labels(legend_labels.size());
  for (int i=0; i<legend_labels.size(); i++) qlegend_labels[i] = legend_labels[i].c_str();
  setLegendSignal(qlegend_labels);
}

void Axes::drawnow()
{
  drawnowSignal();
}

void Axes::plotSlot(const void *x_data, const void *y_data)
{
  // qDebug() << "[Axes::plotSlot]: " << QThread::currentThread() << "\n";
  const QVector<double> &qx_data = *reinterpret_cast<const QVector<double> *>(x_data);
  const QVector<double> &qy_data = *reinterpret_cast<const QVector<double> *>(y_data);

  if (!hold_on)
  {
    this->clearGraphs();
    color_ind = 0;
  }

  QCPGraph *graph = new QCPGraph(this->xAxis, this->yAxis);
  // graph->setLineStyle(QCPGraph::lsLine); // connect points with straight lines
  graph->setData(qx_data, qy_data);

  graphs.push_back( new Graph(graph, this) );
  last_graph = graphs.back();
  last_graph->setColorSlot(static_cast<Color>(color_ind));
  color_ind = (color_ind+1)%QtPlot::N_COLORS;
  last_graph->setLineStyleSlot(SolidLine);
  last_graph->setLineWidthSlot(2.0);
  last_graph->setMarkerStyleSlot(ssDot);
  last_graph->setMarkerSizeSlot(2.0);
  //graph->setPen(QPen(QBrush(c), 2.0, Qt::SolidLine)); // line color blue for first graph
  // graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, 4)); // set marker type and size. The width is determined by the linewidth
  this->rescaleAxes();
}

void Axes::holdSlot(bool set)
{
  hold_on = set;
}

void Axes::gridSlot(bool set)
{
  this->xAxis->grid()->setVisible(set);
  this->yAxis->grid()->setVisible(set);
}

void Axes::setTitleSlot(const QString &title)
{
  int fontsize = 16;
  QString font_family = "Arial";
  this->plotLayout()->insertRow(0);
  QCPTextElement *qcp_title = new QCPTextElement(this, title, QFont(font_family, fontsize, QFont::Normal));
  this->plotLayout()->addElement(0, 0, qcp_title);

  this->title_elem = new TextLabel(qcp_title);
}

void Axes::setXLabelSlot(const QString &label)
{
  // qDebug() << "[Axes::setXLabelSlot]: " << QThread::currentThread() << "\n";
  int fontsize = 14;
  QString font_family = "Arial";
  QCPAxis *x_axis = this->xAxis;
  x_axis->setLabel(label);
  x_axis->setLabelColor(QColor(0,0,0));
  x_axis->setLabelFont(QFont(font_family, fontsize, QFont::Normal));
}

void Axes::setYLabelSlot(const QString &label)
{
  int fontsize = 14;
  QString font_family = "Arial";

  QCPAxis *y_axis = this->yAxis;
  y_axis->setLabel(label);
  y_axis->setLabelColor(QColor(0,0,0));
  y_axis->setLabelFont(QFont(font_family, fontsize, QFont::Normal));
}

void Axes::setLegendSlot(const QVector<QString> &legend_labels)
{
  // qDebug() << "[Axes::setLegendSlot]: " << QThread::currentThread() << "\n";
  int fontsize = 14;
  QString font_family = "Arial";

  int n_graphs = this->graphCount();
  int n_labels = legend_labels.size();

  if (n_labels > n_graphs)
  {
      qDebug() << "Extra legend labels will be ignored.";
      n_labels = n_graphs;
  }

  for (int i=0; i<n_labels; i++) this->graph(i)->setName(legend_labels[i]);

  this->QCustomPlot::legend->setVisible(true);
  this->QCustomPlot::legend->setFont(QFont(font_family, fontsize, QFont::Normal));
  this->QCustomPlot::legend->setBrush(QBrush(QColor(255,255,255,230)));
  this->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
}

void Axes::drawnowSlot()
{
  this->replot();
}


// ==========================================
// =============   Graph   ==================
// ==========================================


Graph::Graph(QCPGraph *qcp_graph, Axes *parent)
{
  QObject::connect(this, &Graph::setColorSignal, this, &Graph::setColorSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Graph::setLineStyleSignal, this, &Graph::setLineStyleSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Graph::setLineWidthSignal, this, &Graph::setLineWidthSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Graph::setMarkerStyleSignal, this, &Graph::setMarkerStyleSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &Graph::setMarkerSizeSignal, this, &Graph::setMarkerSizeSlot, Qt::BlockingQueuedConnection);

  this->qcp_graph = qcp_graph;
}

Graph::~Graph()
{

}

void Graph::setProperty() {}

void Graph::setPropertyHelper(pl_::PROPERTY p, pl_::Color p_value)
{
  if (p == pl_::Color_) setColor(p_value);
  else std::cerr << "[Graph::setPropertyHelper::Color_]: ** Invalid graph property **\n";
}

void Graph::setPropertyHelper(pl_::PROPERTY p, const QColor &p_value)
{
  if (p == pl_::Color_) setColor(p_value);
  else std::cerr << "[Graph::setPropertyHelper::Color_]: ** Invalid graph property **\n";
}

void Graph::setPropertyHelper(pl_::PROPERTY p, double p_value)
{
  if (p == pl_::LineWidth_) setLineWidth(p_value);
  else if (p == pl_::MarkerSize_) setMarkerSize(p_value);
  else std::cerr << "[Graph::setPropertyHelper::LineWidth_|MarkerSize_]: ** Invalid graph property **\n";
}

void Graph::setPropertyHelper(pl_::PROPERTY p, pl_::LineStyle p_value)
{
  if (p == pl_::LineStyle_) setLineStyle(p_value);
  else std::cerr << "[Graph::setPropertyHelper::LineStyle_]: ** Invalid graph property **\n";
}

void Graph::setPropertyHelper(pl_::PROPERTY p, pl_::MarkerStyle p_value)
{
  if (p == pl_::MarkerStyle_) setMarkerStyle(p_value);
  else std::cerr << "[Graph::setPropertyHelper::MarkerStyle_]: ** Invalid graph property **\n";
}

void Graph::setColor(Color color)
{
  setColorSignal(QtPlot::getQColor(static_cast<Color>(color)));
}

void Graph::setColor(const QColor &color)
{
  setColorSignal(color);
}

void Graph::setLineStyle(LineStyle style)
{
  setLineStyleSignal(style);
}

void Graph::setLineWidth(double width)
{
  setLineWidthSignal(width);
}

void Graph::setMarkerStyle(MarkerStyle type)
{
  setMarkerStyleSignal(type);
}

void Graph::setMarkerSize(double size)
{
  setMarkerSizeSignal(size);
}

void Graph::setColorSlot(const QColor &color)
{
  QPen pen = qcp_graph->pen();
  pen.setColor(color);
  qcp_graph->setPen(pen);
}

void Graph::setLineStyleSlot(int style)
{
  if (style == NoLine)
  {
    qcp_graph->setLineStyle(QCPGraph::lsNone);
  }
  else
  {
    qcp_graph->setLineStyle(QCPGraph::lsLine);
    QPen pen = qcp_graph->pen();
    pen.setStyle(static_cast<Qt::PenStyle>(style));
    qcp_graph->setPen(pen);
  }
}

void Graph::setLineWidthSlot(double width)
{
  QPen pen = qcp_graph->pen();
  pen.setWidth(width);
  qcp_graph->setPen(pen);
}

void Graph::setMarkerStyleSlot(int type)
{
  QCPScatterStyle marker = qcp_graph->scatterStyle();
  marker.setShape( static_cast<QCPScatterStyle::ScatterShape>(type));
  qcp_graph->setScatterStyle(marker);
}

void Graph::setMarkerSizeSlot(double size)
{
  QCPScatterStyle marker = qcp_graph->scatterStyle();
  marker.setSize(size);
  qcp_graph->setScatterStyle(marker);
}


// ===============================================
// =============   Text Label   ==================
// ===============================================

TextLabel::TextLabel(QCPTextElement *qcp_text)
{
  this->qcp_text = qcp_text;

  QObject::connect(this, &TextLabel::setTextSignal, this, &TextLabel::setTextSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &TextLabel::setColorSignal, this, &TextLabel::setColorSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &TextLabel::setFontSizeSignal, this, &TextLabel::setFontSizeSlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &TextLabel::setFontFamilySignal, this, &TextLabel::setFontFamilySlot, Qt::BlockingQueuedConnection);
  QObject::connect(this, &TextLabel::setFontWeightSignal, this, &TextLabel::setFontWeightSlot, Qt::BlockingQueuedConnection);

}

void TextLabel::setText(const std::string &s)
{
  setTextSignal(QString(s.c_str()));
}

void TextLabel::setColor(Color c)
{
  setColorSignal(QtPlot::getQColor(static_cast<Color>(c)));
}

void TextLabel::setColor(const QColor &c)
{
  setColorSignal(c);
}

void TextLabel::setFontSize(int size)
{
  setFontSizeSignal(size);
}

void TextLabel::setFontFamily(const std::string &family)
{
  setFontFamilySignal(QString(family.c_str()));
}

void TextLabel::setFontWeight(FontWeight fweight)
{
  setFontWeightSignal(fweight);
}

void TextLabel::setProperty() {}

void TextLabel::setPropertyHelper(pl_::PROPERTY p, pl_::Color p_value)
{
  if (p == pl_::Color_) setColor(p_value);
  else std::cerr << "** Invalid text property **\n";
}

void TextLabel::setPropertyHelper(pl_::PROPERTY p, const QColor &p_value)
{
  if (p == pl_::Color_) setColor(p_value);
  else std::cerr << "** Invalid text property **\n";
}

void TextLabel::setPropertyHelper(pl_::PROPERTY p, int p_value)
{
  if (p == pl_::FontSize_) setFontSize(p_value);
  else std::cerr << "** Invalid text property **\n";
}

void TextLabel::setPropertyHelper(pl_::PROPERTY p, const std::string &p_value)
{
  if (p == pl_::FontFamily_) setFontFamily(p_value);
  else std::cerr << "** Invalid text property **\n";
}

void TextLabel::setPropertyHelper(pl_::PROPERTY p, FontWeight p_value)
{
  if (p == pl_::FontWeight_) setFontWeight(p_value);
  else std::cerr << "** Invalid text property **\n";
}

void TextLabel::setTextSlot(const QString &s)
{
  qcp_text->setText(s);
}

void TextLabel::setColorSlot(const QColor &c)
{
  qcp_text->setTextColor(c);
}

void TextLabel::setFontSizeSlot(int size)
{
  QFont font = qcp_text->font();
  font.setPointSize(size);
  qcp_text->setFont(font);
}

void TextLabel::setFontFamilySlot(const QString &family)
{
  QFont font = qcp_text->font();
  font.setFamily(family);
  qcp_text->setFont(font);
}

void TextLabel::setFontWeightSlot(FontWeight fweight)
{
  QFont font = qcp_text->font();
  font.setWeight(fweight);
  qcp_text->setFont(font);
}



  } // namespace pl_

} // namespace as64_
