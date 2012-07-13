#include <ros/ros.h>
#include "std_msgs/String.h"

#include <sl_msgs/SLState.h>
#include <sl_msgs/SLEnvDescription.h>
#include <sl_msgs/SLAction.h>
#include <sl_msgs/SLSpeech.h>

#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include "Environment.hh"
#include "Random.h"

#include "SpeechArmTable.hh"

#include <getopt.h>
#include <stdlib.h>


static ros::Publisher out_env_desc;
static ros::Publisher out_state;

Environment* e;
Random rng;
bool PRINTS = false;//true;
int seed = 1;

// some default parameters
bool stochastic = true;
int width = 5;
int nobjects = 3;
float speech_radius = 0.6;


void displayHelp();
void publishState();
void processAction(const sl_msgs::SLAction::ConstPtr &actionIn);
void initEnvironment();
int main(int argc, char *argv[]);
