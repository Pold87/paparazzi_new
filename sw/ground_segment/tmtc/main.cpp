#include <iostream>

#include "mainwindow.h"
#include <QApplication>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <particle_filter.h>
#include "main.h"
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

int NUM_HISTS = 20;
int DEBUG = 0;

void on_position_estimate(IvyClientPtr app, void *user_data, int argc, char *argv[]){

    if (DEBUG) {
        printf("%s", argv[0]);
    } else {

        printf("id:%s x_trexton:%s y_trexton: %s x_optitrack: %s, y_optitrack: %s, entropy: %s, x_uncertainty: %s, y_uncertainty: %s\n",
               argv[0], argv[1], argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);

        /* Calculate Euclidean distance  */

        /* Update text fields */

        QString x_trexton = argv[1];
        QString y_trexton = argv[2];
        QString x_optitrack = argv[3];
        QString y_optitrack = argv[4];
        QString entropy = argv[5];
        QString x_uncertainty = argv[6];
        QString y_uncertainty = argv[7];

        /* Read histogram values */
        QVector<double> Qhists(NUM_HISTS);

        for (int h = 0; h < NUM_HISTS; ++h) {
            QString hVal = argv[8 + h];
            Qhists[h] = hVal.toDouble();
            printf("%s -- %f ", argv[8 + h], Qhists[h]);
            printf("\n");
        }

        w->updateCoords(x_trexton, y_trexton, x_optitrack, y_optitrack,
                        entropy, x_uncertainty, y_uncertainty, Qhists);
    }
}


void start_loop(){
    GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

    IvyInit ("TreXton visualizer", "TreXton Visualizer READY", NULL, NULL, NULL, NULL);

    if (DEBUG) {
        // Important: for debugging the next line will show the entire message!
        IvyBindMsg(on_position_estimate, NULL, "(.*)");
    } else {
        IvyBindMsg(on_position_estimate, NULL, "^(\\S*) TREXTON (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
    }
    IvyStart("127.255.255.255");

    g_main_loop_run(ml);

}

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow win;
    win.show();
    w = &win;

    std::cout << "Starting loop" << std::endl;
    start_loop();
    std::cout << "Finished loop" << std::endl;

    return a.exec();
}
