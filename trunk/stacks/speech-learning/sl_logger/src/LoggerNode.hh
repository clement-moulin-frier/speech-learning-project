#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <sl_msgs/SLState.h>
#include <sl_msgs/SLGoal.h>
#include <sl_msgs/SLSpeech.h>
#include <sl_msgs/SLEnvDescription.h>
#include <sl_msgs/SLResult.h>

#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include <getopt.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
using namespace std;


// possible outputs
ofstream* statesOut = NULL;
ofstream* goalsSelected = NULL;
ofstream* goalsReached = NULL;
ofstream* speechesTried = NULL;
ofstream* evalPerf = NULL;
ofstream* allPerf = NULL;
ofstream* texploreSpeeches = NULL;
std::string filePrefix;

// flags to turn on/off logging each of these
bool logStates = true;
bool logGoalsSelected = true;
bool logGoalsReached = true;
bool logSpeeches = true;
bool logEvalPerf = true;
bool logAllPerf = true;
bool logTexploreSpeeches = true;

bool PRINTS = false;

// some counters
int ngoals = 0;
int nstates = 0;
int nspeeches = 0;
int nevalgoals = 0;
int ntexspeeches = 0;
int ntexspeechlearners = 0;

bool lastGoalWasEvalOnly = false;
std::vector<float> evalResults;

void displayHelp();
void initOutputFiles();

void processState(const sl_msgs::SLState::ConstPtr &stateIn);
void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processSpeech(const sl_msgs::SLSpeech::ConstPtr &speechIn);
void processSpeechFromTexplore(const sl_msgs::SLSpeech::ConstPtr &speechIn);
void processGoalFromTexplore(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processResult(const sl_msgs::SLResult::ConstPtr &resultIn);
void processEnvDesc(const sl_msgs::SLEnvDescription::ConstPtr &envIn);


int main(int argc, char *argv[]);

