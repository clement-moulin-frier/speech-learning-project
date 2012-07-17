#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <sl_msgs/SLState.h>
#include <sl_msgs/SLGoal.h>
#include <sl_msgs/SLSpeech.h>
#include <sl_msgs/SLEnvDescription.h>

#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include <getopt.h>
#include <stdlib.h>

// possible outputs
ofstream* statesOut = NULL;
ofstream* goalsSelected = NULL;
ofstream* goalsReached = NULL;
ofstream* speechesTried = NULL;
ofstream* evalPerf = NULL;
ofstream* allPerf = NULL;
ofstream* texploreSpeeches = NULL;

// some counters
int ngoals = 0;
int nstates = 0;
int nspeeches = 0;
int nevalgoals = 0;
int ntexspeeches = 0;
int ntexspeechlearners = 0;

void displayHelp();

void processState(const sl_msgs::SLState::ConstPtr &stateIn);
void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processSpeech(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processSpeechFromTexplore(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processGoalFromTexplore(const sl_msgs::SLGoal::ConstPtr &goalIn);

int main(int argc, char *argv[]);

