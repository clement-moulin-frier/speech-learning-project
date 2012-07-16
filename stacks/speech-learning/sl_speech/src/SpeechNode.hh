#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <sl_msgs/SLState.h>
#include <sl_msgs/SLGoal.h>
#include <sl_msgs/SLSpeech.h>

#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include "Random.h"

#include <getopt.h>
#include <stdlib.h>


static ros::Publisher out_speech;
static ros::Publisher out_speech_tex;

static ros::Publisher out_done;
static ros::Publisher out_done_tex;


Random rng;
bool PRINTS = false;//true;
int seed = 1;
bool learningActive = false; // so we know if we're running or TEXPLORE
std::vector<float> lastState;

// track which step we're on for current goal
int istep = 0;
int nsteps = 10;

// current goal
std::vector<float> goal;
std::vector<bool> mask;
int goalFrom = 0;
int goalIndex = -1;
bool evalOnly = false;

int tryThresh = 10; // how many times we should try a speech
float minRate = 0.82; // how successful does it need to be?

struct goalInfo {
  std::vector<float> goal;
  std::vector<bool> mask;

  float best_f1;
  float best_f2;

  int nSuccess;
  int nTry;

  bool sentToTexplore;
};

int matchGoal(const sl_msgs::SLGoal::ConstPtr &goalIn);


std::vector<goalInfo> goalsTried;

void displayHelp();
void processState(const sl_msgs::SLState::ConstPtr &stateIn);
void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processGoalFromES(const sl_msgs::SLGoal::ConstPtr &goalIn);
void processGoalFromTexplore(const sl_msgs::SLGoal::ConstPtr &goalIn);
int main(int argc, char *argv[]);

void tryNextAction();
