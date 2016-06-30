#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

namespace Ui {
class MainWindow;

}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    //void start_loop();
    void updateCoords(QString x_trexton, QString y_trexton, QString x_optitrack,
                      QString y_optitrack, QString entropy, QString x_uncertainty,
                      QString y_uncertainty, QVector<double> hists);

    QVector<double> Qconfidence;
    QVector<double> Qdistances;
    QVector<double> Qtimes;

private:
    Ui::MainWindow *ui;

};

extern MainWindow *w;

#endif // MAINWINDOW_H
