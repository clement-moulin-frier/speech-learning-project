#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <sl_msgs/SLState.h>
#include <sl_msgs/SLGoal.h>
#include <sl_msgs/SLEnvDescription.h>

#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include "EffectSpaceGoalSelector.hh"
#include "Random.h"

#include <getopt.h>
#include <stdlib.h>


static ros::Publisher out_goal_tex;
static ros::Publisher out_goal_speech;

EffectSpaceGoalSelector* selector;

// things we need to keep track of
EffectSpaceGoalSelector::goal currentGoal;
std::vector<float> lastState;
bool firstGoalSent = false;

Random rng;
bool PRINTS = false;//true;

// default parameters
int goalSelect = EffectSpaceGoalSelector::RandomGoal;
int learnerSelect = EffectSpaceGoalSelector::TexploreOnly;
int tau = 10;
int theta = 10;

void displayHelp();
void processState(const sl_msgs::SLState::ConstPtr &stateIn);
void processEnvDesc(const sl_msgs::SLEnvDescription::ConstPtr &envIn);
void processDone(const std_msgs::Empty::ConstPtr &doneIn);

int main(int argc, char *argv[]);
