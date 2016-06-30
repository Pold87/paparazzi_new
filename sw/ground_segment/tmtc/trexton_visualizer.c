#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <particle_filter.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

static void on_position_estimate(IvyClientPtr app, void *user_data, int argc, char *argv[]);

static void on_position_estimate(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  printf("id:%s x_trexton:%s y_trexton: %s x_optitrack: %s, y_optitrack: %s, entropy: %s, x_uncertainty: %s, y_uncertainty: %s\n", argv[0], argv[1], argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);
  /*  visualize_simple(atoi(argv[1]), atoi(argv[2]));*/

  /* Calculate Euclidean distance  */ 
  
  visualize_optitrack(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atof(argv[6]));
}


int main ( int argc, char** argv) {
  printf("Starting TreXton visualizer");
  init_visualize();
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("TreXton visualizer", "TreXton Visualizer READY", NULL, NULL, NULL, NULL);
  /* IvyBindMsg(on_position_estimate, NULL, "^(\\S*) TREXTON (\\S*) (\\S*) .*"); */
  IvyBindMsg(on_position_estimate, NULL, "^(\\S*) TREXTON (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  IvyStart("127.255.255.255");

  g_main_loop_run(ml);

  return 0;
}
