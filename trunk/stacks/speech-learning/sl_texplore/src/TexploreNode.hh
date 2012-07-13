#include <ros/ros.h>
#include "std_msgs/String.h"

#include <sl_msgs/SLState.h>
#include <sl_msgs/SLGoal.h>
#include <sl_msgs/SLAction.h>
#include <sl_msgs/SLSpeech.h>
#include <sl_msgs/SLEnvDescription.h>

#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include "ModelBasedAgent.hh"
#include "ETUCTGivenGoal.hh"
#include "Random.h"

#include <getopt.h>
#include <stdlib.h>


static ros::Publisher out_action;
static ros::Publisher out_speech;
static ros::Publisher out_done;
static ros::Publisher out_goal; // if we want to control speech learner

Agent* a = NULL;
ETUCTGivenGoal* planner = NULL;
Random rng;
bool PRINTS = false;//true;
int seed = 1;
bool learningActive = false; // so we know if we're running or the speech learner is running
std::vector<float> lastState;

// default parameters
float vcoeff = 5;
float ncoeff = 5;
int maxsteps = 20;
float actrate = 5;
bool master = false;

// track which step we're on for current goal
int istep = 0;

// current goal
std::vector<float> goal;
std::vector<bool> mask;
sl_msgs::SLGoal lastGoalMsg;

// list of available speeches sent from speech learner
std::vector<std::pair<float,float> > speeches;

void displayHelp();
void processState(const sl_msgs::SLAction::ConstPtr &actionIn);
void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processEnvDesc(const sl_msgs::SLEnvDescription::ConstPtr &envIn);
void processSpeech(const sl_msgs::SLGoal::ConstPtr &speechIn);
void processDone(const std_msgs::Empty::ConstPtr &doneIn);
int main(int argc, char *argv[]);
