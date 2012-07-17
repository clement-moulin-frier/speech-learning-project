#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <sl_msgs/SLState.h>
#include <sl_msgs/SLGoal.h>
#include <sl_msgs/SLEnvDescription.h>
#include <sl_msgs/SLResult.h>

#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include "EffectSpaceGoalSelector.hh"
#include "Random.h"

#include <getopt.h>
#include <stdlib.h>


static ros::Publisher out_goal_tex;
static ros::Publisher out_goal_speech;
static ros::Publisher out_result;

EffectSpaceGoalSelector* selector;

// things we need to keep track of
EffectSpaceGoalSelector::goal currentGoal;
int currentGoalID;
std::vector<float> lastState;
bool firstGoalSent = false;

Random rng;
bool PRINTS = false;//true;

// default parameters
int goalSelect = EffectSpaceGoalSelector::SaggRiac;
int learnerSelect = EffectSpaceGoalSelector::Competence;
int tau = 2;
int theta = 3; // although domain is stochastic, taking n-steps to reach goal should be achievable deterministically
int evalFreq = 25;

void displayHelp();
void processState(const sl_msgs::SLState::ConstPtr &stateIn);
void processEnvDesc(const sl_msgs::SLEnvDescription::ConstPtr &envIn);
void processDone(const std_msgs::Empty::ConstPtr &doneIn);

int main(int argc, char *argv[]);
