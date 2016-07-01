/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionExit;
    QWidget *centralWidget;
    QLabel *lOptitrackX;
    QLabel *lOptitrackY;
    QCustomPlot *plot;
    QLabel *label;
    QLabel *xPos;
    QLabel *yPos;
    QLabel *lOptitrack;
    QLabel *ltreXton;
    QLabel *lTrexton_x;
    QLabel *lTrexton_y;
    QLabel *lUncertainty;
    QLabel *confVal_x;
    QLabel *confVal_y;
    QFrame *line;
    QCustomPlot *confBar;
    QCustomPlot *histogram;
    QCustomPlot *distPlot;
    QLabel *lDistance;
    QLabel *dist_Val;
    QLabel *dist_Mean;
    QLabel *lCurrent;
    QLabel *lMean;
    QFrame *line_2;
    QLabel *lCorr;
    QLabel *corrVal;
    QFrame *line_3;
    QLabel *label_2;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1469, 713);
        MainWindow->setStyleSheet(QStringLiteral("background-color: rgb(255, 255, 255);"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        lOptitrackX = new QLabel(centralWidget);
        lOptitrackX->setObjectName(QStringLiteral("lOptitrackX"));
        lOptitrackX->setGeometry(QRect(130, 530, 121, 17));
        lOptitrackY = new QLabel(centralWidget);
        lOptitrackY->setObjectName(QStringLiteral("lOptitrackY"));
        lOptitrackY->setGeometry(QRect(130, 560, 111, 17));
        plot = new QCustomPlot(centralWidget);
        plot->setObjectName(QStringLiteral("plot"));
        plot->setGeometry(QRect(30, 20, 641, 481));
        plot->setStyleSheet(QStringLiteral(""));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(513, -10, 351, 51));
        QFont font;
        font.setFamily(QStringLiteral("URW Chancery L"));
        font.setPointSize(36);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        label->setFont(font);
        label->setStyleSheet(QStringLiteral("color: rgb(0, 0, 0);"));
        xPos = new QLabel(centralWidget);
        xPos->setObjectName(QStringLiteral("xPos"));
        xPos->setGeometry(QRect(20, 530, 67, 17));
        yPos = new QLabel(centralWidget);
        yPos->setObjectName(QStringLiteral("yPos"));
        yPos->setGeometry(QRect(20, 560, 67, 17));
        lOptitrack = new QLabel(centralWidget);
        lOptitrack->setObjectName(QStringLiteral("lOptitrack"));
        lOptitrack->setGeometry(QRect(130, 490, 91, 17));
        ltreXton = new QLabel(centralWidget);
        ltreXton->setObjectName(QStringLiteral("ltreXton"));
        ltreXton->setGeometry(QRect(280, 490, 61, 16));
        lTrexton_x = new QLabel(centralWidget);
        lTrexton_x->setObjectName(QStringLiteral("lTrexton_x"));
        lTrexton_x->setGeometry(QRect(260, 530, 121, 17));
        lTrexton_y = new QLabel(centralWidget);
        lTrexton_y->setObjectName(QStringLiteral("lTrexton_y"));
        lTrexton_y->setGeometry(QRect(260, 560, 91, 17));
        lUncertainty = new QLabel(centralWidget);
        lUncertainty->setObjectName(QStringLiteral("lUncertainty"));
        lUncertainty->setGeometry(QRect(20, 570, 91, 17));
        confVal_x = new QLabel(centralWidget);
        confVal_x->setObjectName(QStringLiteral("confVal_x"));
        confVal_x->setGeometry(QRect(30, 600, 67, 17));
        confVal_y = new QLabel(centralWidget);
        confVal_y->setObjectName(QStringLiteral("confVal_y"));
        confVal_y->setGeometry(QRect(30, 620, 67, 17));
        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(81, 510, 301, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        confBar = new QCustomPlot(centralWidget);
        confBar->setObjectName(QStringLiteral("confBar"));
        confBar->setGeometry(QRect(150, 580, 531, 71));
        histogram = new QCustomPlot(centralWidget);
        histogram->setObjectName(QStringLiteral("histogram"));
        histogram->setGeometry(QRect(670, 70, 641, 251));
        distPlot = new QCustomPlot(centralWidget);
        distPlot->setObjectName(QStringLiteral("distPlot"));
        distPlot->setGeometry(QRect(750, 460, 541, 191));
        lDistance = new QLabel(centralWidget);
        lDistance->setObjectName(QStringLiteral("lDistance"));
        lDistance->setGeometry(QRect(1086, 350, 131, 17));
        dist_Val = new QLabel(centralWidget);
        dist_Val->setObjectName(QStringLiteral("dist_Val"));
        dist_Val->setGeometry(QRect(1156, 400, 67, 17));
        dist_Mean = new QLabel(centralWidget);
        dist_Mean->setObjectName(QStringLiteral("dist_Mean"));
        dist_Mean->setGeometry(QRect(1156, 420, 67, 17));
        lCurrent = new QLabel(centralWidget);
        lCurrent->setObjectName(QStringLiteral("lCurrent"));
        lCurrent->setGeometry(QRect(1056, 400, 91, 17));
        lMean = new QLabel(centralWidget);
        lMean->setObjectName(QStringLiteral("lMean"));
        lMean->setGeometry(QRect(1056, 420, 91, 17));
        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(1036, 370, 211, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        lCorr = new QLabel(centralWidget);
        lCorr->setObjectName(QStringLiteral("lCorr"));
        lCorr->setGeometry(QRect(530, 350, 321, 20));
        corrVal = new QLabel(centralWidget);
        corrVal->setObjectName(QStringLiteral("corrVal"));
        corrVal->setGeometry(QRect(660, 400, 67, 17));
        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(520, 370, 331, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(560, 0, 31, 31));
        label_2->setFont(font);
        label_2->setStyleSheet(QLatin1String("color: rgb(0, 0, 0);\n"
"color: rgb(24, 136, 0);"));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1469, 22));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(actionExit);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0));
        lOptitrackX->setText(QApplication::translate("MainWindow", "xPosOptitrack", 0));
        lOptitrackY->setText(QApplication::translate("MainWindow", "yPosOptitrack", 0));
        label->setText(QApplication::translate("MainWindow", "treXton evaluation", 0));
        xPos->setText(QApplication::translate("MainWindow", "x-Position:", 0));
        yPos->setText(QApplication::translate("MainWindow", "y-Position", 0));
        lOptitrack->setText(QApplication::translate("MainWindow", "Optitrack", 0));
        ltreXton->setText(QApplication::translate("MainWindow", "treXton", 0));
        lTrexton_x->setText(QApplication::translate("MainWindow", "xPosTreXton", 0));
        lTrexton_y->setText(QApplication::translate("MainWindow", "yPosTreXton", 0));
        lUncertainty->setText(QApplication::translate("MainWindow", "Uncertainty", 0));
        confVal_x->setText(QApplication::translate("MainWindow", "confVal", 0));
        confVal_y->setText(QApplication::translate("MainWindow", "confVal", 0));
        lDistance->setText(QApplication::translate("MainWindow", "Euclidean distance", 0));
        dist_Val->setText(QApplication::translate("MainWindow", "dist_Val", 0));
        dist_Mean->setText(QApplication::translate("MainWindow", "dist_Mean", 0));
        lCurrent->setText(QApplication::translate("MainWindow", "current:", 0));
        lMean->setText(QApplication::translate("MainWindow", "mean:", 0));
        lCorr->setText(QApplication::translate("MainWindow", "Correlation between distance and confidence", 0));
        corrVal->setText(QApplication::translate("MainWindow", "dist_Val", 0));
        label_2->setText(QApplication::translate("MainWindow", "X", 0));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
