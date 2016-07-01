#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <algorithm>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <particle_filter.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

static int NUM_HISTS = 20;
static double euclidean_dist(int x1, int y1, int x2, int y2);
static double correlation(QVector<double> conf, QVector<double> dists);

static double euclidean_dist(int x1, int y1, int x2, int y2) {

    double delta_x_2 = (x2 - x1) * (x2 - x1);
    double delta_y_2 = (y2 - y1) * (y2 - y1);

    double res = sqrt(delta_x_2 + delta_y_2);

    return res;
}

static double correlation(QVector<double> Qconf, QVector<double> Qdists) {

    /* Calculate means */
    double sum_conf = std::accumulate(Qconf.begin(), Qconf.end(), 0.0);
    double sum_distances = std::accumulate(Qdists.begin(), Qdists.end(), 0.0);

    double mean_conf = sum_conf / Qconf.size();
    double mean_dist = sum_distances / Qdists.size();

    /* Calculate correlations */

    double cov = 0.0; /* covariance */
    double var_conf = 0.0; /* variance confidence */
    double var_dists = 0.0; /* variance distances */
    for (int i = 0; i < std::min(100, Qconf.size()); ++i) {
        cov += (Qconf.at(i) - mean_conf) * (Qdists.at(i) - mean_dist);
        var_conf += (Qconf.at(i) - mean_conf) * (Qconf.at(i) - mean_conf);
        var_dists += (Qdists.at(i) - mean_dist) * (Qdists.at(i) - mean_dist);
    }

    double corr = cov / (sqrt(var_conf) * sqrt(var_dists));

    std::cout << "cov is " << cov << std::endl;
    std::cout << "var conf is " << var_conf << std::endl;
    std::cout << "var dists is " << var_dists << std::endl;
    std::cout << "corr is " << corr;

    return corr;

}

MainWindow *w = NULL;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->plot->addGraph();

    //ui->plot->axisRect()->setBackground(QPixmap("/home/pold/from_bebop/img_00195.png"));
    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, QColor("red"), 30));

    ui->plot->xAxis->setRange(0, 1000);
    ui->plot->yAxis->setRange(0, 1000);

    ui->plot->addGraph();
    ui->plot->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, QColor("blue"), 30));

    /* Confidence */
    ui->confBar->addGraph();
    ui->confBar->xAxis->setRange(0, 245);
    ui->confBar->yAxis->setRange(0, 3000);

    /* Distances */
    ui->distPlot->addGraph();
    ui->distPlot->xAxis->setRange(0, 245);
    ui->distPlot->yAxis->setRange(0, 600);

    Qtimes.push_back(0.0);

    /* Histogram */
    QCPBars *bars = new QCPBars(ui->histogram->xAxis, ui->histogram->yAxis);
    ui->histogram->addPlottable(bars);

    QVector<double> datax(NUM_HISTS);
    QVector<double> datay(NUM_HISTS);
    for (int i = 0; i < NUM_HISTS; ++i) {
        datax[i] = i;
        datay[i] = i;
    }

    bars->setData(datax, datay);
    bars->setBrush(QColor(0, 0, 255, 50));
    bars->setPen(QColor(0, 0, 255));
    bars->setWidth(1.0);

    ui->histogram->xAxis->setRange(0.1, 20.9);
    ui->histogram->yAxis->setRange(0, 0.5);
    ui->histogram->xAxis->setAutoTickStep(false);
    ui->histogram->xAxis->setTickStep(1);


}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::updateCoords(QString x_trexton, QString y_trexton, QString x_optitrack,
                              QString y_optitrack, QString entropy, QString x_uncertainty,
                              QString y_uncertainty, QVector<double> Qhists)
{

    this->ui->lOptitrackX->setText(x_optitrack);
    this->ui->lOptitrackY->setText(y_optitrack);

    this->ui->lTrexton_x->setText(x_trexton);
    this->ui->lTrexton_y->setText(y_trexton);


    /* Convert uncertainty to integer */
    QString x_unc_s = QString::number((int) x_uncertainty.toDouble());
    QString y_unc_s = QString::number((int) y_uncertainty.toDouble());

    this->ui->confVal_x->setText(x_unc_s);
    this->ui->confVal_y->setText(y_unc_s);

    QVector<double> Qx_trexton(1);
    QVector<double> Qy_trexton(1);

    QVector<double> Qx_optitrack(1);
    QVector<double> Qy_optitrack(1);

    Qx_trexton[0] = x_trexton.toDouble();
    Qy_trexton[0] = y_trexton.toDouble();

    Qx_optitrack[0] = x_optitrack.toDouble();
    Qy_optitrack[0] = y_optitrack.toDouble();

    ui->plot->graph(0)->setData(Qx_trexton, Qy_trexton);
    ui->plot->graph(1)->setData(Qx_optitrack, Qy_optitrack);

    printf("Qx_trexton: %f %f\n", x_trexton.toDouble(), y_trexton.toDouble());

    /* Confidence */
    Qconfidence.push_back((x_uncertainty.toDouble() + y_uncertainty.toDouble()) / 2.0);

    Qtimes.push_back(Qtimes.last() + 1);

    ui->confBar->graph(0)->setData(Qtimes, Qconfidence);
    ui->confBar->replot();


    /* Calculate Euclidean distance between OptiTrack and treXton */

    double dist = euclidean_dist(x_optitrack.toInt(), y_optitrack.toInt(),
                                 x_trexton.toInt(), y_trexton.toInt());

    if (Qdistances.size() < 224)
        Qdistances.push_back(dist);

    /* Calculate average euclidean distance */

    double sum_distances = std::accumulate(Qdistances.begin(), Qdistances.end(), 0.0);
    double mean_distances = sum_distances / Qdistances.size();

    ui->dist_Mean->setText(QString::number(mean_distances));
    ui->dist_Val->setText(QString::number(dist));


    /* Calculate correlation between Euclidean distances and confidence */

    double corr = correlation(Qconfidence, Qdistances);
    ui->corrVal->setText(QString::number(corr));

    /* Plot distances */

    ui->distPlot->graph(0)->setData(Qtimes, Qdistances);
    ui->distPlot->replot();
    ui->plot->replot();

    QVector<double> datax(NUM_HISTS);
    for (int i = 0; i < NUM_HISTS; ++i) {
        datax[i] = i;
    }

    /* Histogram */
    ui->histogram->removePlottable(0);

    QCPBars *bars = new QCPBars(ui->histogram->xAxis, ui->histogram->yAxis);
    ui->histogram->addPlottable(bars);

    bars->setBrush(QColor(0, 0, 255, 50));
    bars->setPen(QColor(0, 0, 255));
    bars->setWidth(1.0);

    bars->setData(datax, Qhists);
    ui->histogram->replot();

}
