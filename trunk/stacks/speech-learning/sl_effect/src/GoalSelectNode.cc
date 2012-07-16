/** \file Main file that starts the Goal selection node
    \author Todd Hester
*/

#include "GoalSelectNode.hh"


#define NODE "SLGoalSelect"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sstream>
#include <iostream>


void displayHelp(){
  cout << "\n Options:\n";
  cout << "--seed value (integer seed for random number generator)\n";
  cout << "--randomgoal (select goals randomly)\n";
  cout << "--progressgoal (select goals with competence progress)\n";
  cout << "--randomlearner (select learner randomly)\n";
  cout << "--texplorelearner (always select texplore as learner)\n";
  cout << "--competencelearner (select learner with highest competence)\n";
  cout << "--progresslearner (select learner with highest competence progress)\n";
  cout << "--tau value (Compare with competence estimate tau steps ago)\n";
  cout << "--theta value (Average this many values together to estimate competence)\n";
  cout << "--prints (turn on debug printing of actions/states)\n";
  exit(-1);
}

void processEnvDesc(const sl_msgs::SLEnvDescription::ConstPtr &envIn){

  // init the goal selection module
  selector = new EffectSpaceGoalSelector(envIn->num_objects, envIn->width, envIn->min_state_range, envIn->max_state_range, goalSelect, learnerSelect, tau, theta, PRINTS, rng);


}


void selectGoal(){

  // ask for a goal
  int learner = -1;
  currentGoal = selector->selectGoal(lastState, &learner);

  // send appropriate message (not eval mode)
  currentGoal.goalMsg.evaluateOnly = false;

  // send to correct learner
  if (learner == EffectSpaceGoalSelector::TexploreLearner){
    out_goal_tex.publish(currentGoal.goalMsg);
  } else {
    // TODO: for now, just report back bad results
    selector->updateGoal(lastState);
    selectGoal();
    //out_goal_speech.publish(currentGoal.goalMsg);
  }
}

void processState(const sl_msgs::SLState::ConstPtr &stateIn){
  lastState = stateIn->state;

  if (!firstGoalSent){
    firstGoalSent = true;
    selectGoal();
  }
}
 

void processDone(const std_msgs::Empty::ConstPtr &doneIn){

  // update progress of last goal
  selector->updateGoal(lastState);

  // select new goal
  selectGoal();
}
   


/** Main function to start the agent node. */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  int seed = 1;

  // now parse other options
  char ch;
  const char* optflags = "ds:";
  int option_index = 0;
  static struct option long_options[] = {
    {"randomgoal", 0, 0, 'r'},
    {"progressgoal", 0, 0, 'g'},
    {"randomlearner", 0, 0, 'l'},
    {"texplorelearner", 0, 0, 't'},
    {"competencelearner", 0, 0, 'c'},
    {"progresslearner", 0, 0, 'o'},
    {"tau", 1, 0, 'a'},
    {"theta", 1, 0, 'e'},
    {"seed", 1, 0, 'x'},
    {"prints", 0, 0, 'p'},
    {"help", 0, 0, 'h'}
  };

  while(-1 != (ch = getopt_long_only(argc, argv, optflags, long_options, &option_index))) {
    switch(ch) {

    case 'x':
      seed = std::atoi(optarg);
      cout << "seed: " << seed << endl;
      break;

    case 'a':
      tau = std::atoi(optarg);
      cout << "tau: " << tau << endl;
      break;

    case 'e':
      theta = std::atoi(optarg);
      cout << "theta: " << theta << endl;
      break;

    case 'r':
      goalSelect = EffectSpaceGoalSelector::RandomGoal;
      cout << "Select Goals Randomly" << endl;
      break;

    case 'g':
      goalSelect = EffectSpaceGoalSelector::SaggRiac;
      cout << "Select Goals with SAGG-RIAC" << endl;
      break;

    case 'l':
      learnerSelect = EffectSpaceGoalSelector::RandomLearner;
      cout << "Select Learner Randomly" << endl;
      break;

    case 't':
      learnerSelect = EffectSpaceGoalSelector::TexploreOnly;
      cout << "Always Select TEXPLORE as Learner" << endl;
      break;

    case 'c':
      learnerSelect = EffectSpaceGoalSelector::Competence;
      cout << "Select Learner with highest competence" << endl;
      break;
    
    case 'o':
      learnerSelect = EffectSpaceGoalSelector::CompProgress;
      cout << "Select Learner with highest competence progress" << endl;
      break;

    case 'p':
      PRINTS = true;
      break;

    case 'h':
    case '?':
    case 0:
    default:
      displayHelp();
      break;
    }
  }


  int qDepth = 1;

  rng = Random(1+seed);

  // Set up Publishers
  ros::init(argc, argv, "my_tf_broadcaster");
  tf::Transform transform;

  // Set up Publishers
  // send new goals to texplore or sagg-riac 
  out_goal_tex = node.advertise<sl_msgs::SLGoal>("sl_effect/tex_goal",qDepth,false);
  out_goal_speech = node.advertise<sl_msgs::SLGoal>("sl_effect/speech_goal",qDepth,false);

  // Set up subscribers
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber sl_env =  node.subscribe("sl_env/env_description", qDepth, processEnvDesc, noDelay);
  ros::Subscriber sl_state = node.subscribe("sl_env/state", qDepth, processState, noDelay);
  ros::Subscriber sl_sp_done = node.subscribe("sl_speech/done", qDepth, processDone, noDelay);
  ros::Subscriber sl_tex_done = node.subscribe("sl_texplore/done", qDepth, processDone, noDelay);


  ROS_INFO(NODE ": starting main loop");
  
  ros::spin();                          // handle incoming data
  //while (ros::ok()){
  //  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  //}

  return 0;
}




