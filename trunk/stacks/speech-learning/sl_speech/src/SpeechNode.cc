/** \file Starts a dummy speech learning agent
    \author Todd Hester
*/

#include "SpeechNode.hh"


#define NODE "SLSpeech"


void displayHelp(){
  cout << "\n Options:\n";
  cout << "--seed value (integer seed for random number generator)\n";
  cout << "--nsteps (how many speeches it can attempt to reach goal)\n";
  cout << "--prints (turn on debug printing of actions/states)\n";
  exit(-1);
}

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sstream>
#include <iostream>


void processGoalFromES(const sl_msgs::SLGoal::ConstPtr &goalIn){
  if (PRINTS) cout << "Received goal from: effect space goal selector" <<endl;
  goalFrom = 0;
  processGoal(goalIn);
}

void processGoalFromTexplore(const sl_msgs::SLGoal::ConstPtr &goalIn){
  if (PRINTS) cout << "Received goal from TEXPLORE" << endl;
  goalFrom = 1;
  processGoal(goalIn);
}

void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn){
  learningActive = true;
  istep = 0;

  goal.resize(goalIn->goal_state.size(), 0);
  mask.resize(goalIn->goal_state.size(), false);

  for (unsigned i = 0; i < goal.size(); i++){
    goal[i] = goalIn->goal_state[i];
    mask[i] = goalIn->goal_mask[i];
  }

  // print new goal
  if (PRINTS){
    cout << "Set goal: " << endl << flush;
    for (unsigned i = 0; i < goal.size(); i++){
      if (mask[i])
        cout << "  Feat " << i << ": Goal value: " << goal[i] << endl << flush;
    }
  }

  tryNextAction();
}

void processState(const sl_msgs::SLState::ConstPtr &stateIn){
  //if (PRINTS) cout << "Received state message" << endl;
  lastState = stateIn->state;

  // do nothing if we're not the learner right now
  if (!learningActive) return;
  
  tryNextAction();
}

void tryNextAction(){

  istep++;
  
  // check if we're at the goal, or have reached the maximum number of steps
  // check if at goal
  bool goalMatched = true;
  for (unsigned i = 0; i < goal.size(); i++){
    if (mask[i] && goal[i] != lastState[i]){
      goalMatched = false;
      break;
    } 
  }

  if (goalMatched && PRINTS){
    cout << "Reached GOAL!!!" << endl;
  }

  if (istep >= nsteps || goalMatched){
    learningActive = false;
    std_msgs::Empty doneMsg;
    // send done back to whomever sent us the goal
    if (goalFrom == 0)
      out_done.publish(doneMsg);
    else
      out_done_tex.publish(doneMsg);
    return;
  }

  // sample a random speech sound
  sl_msgs::SLSpeech speechMsg;
  speechMsg.f1 = rng.uniform(3, 6.5);
  speechMsg.f2 = rng.uniform(2.5, 4);

  if (PRINTS) cout << "Try speech: " << speechMsg.f1 << ", " << speechMsg.f2 << endl;
  
  // TODO: note, not doing anything smart here, just random sampling
  // TODO: and no attempt to actually create this sound with vocal tract
  // TODO: also need to decide if we should add this speech to TEXPLORE
  out_speech.publish(speechMsg);

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
    {"nsteps", 1, 0, 's'},
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

    case 's':
      nsteps = std::atoi(optarg);
      cout << "nsteps: " << nsteps << endl;
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
  // send speeches to the environment
  out_speech = node.advertise<sl_msgs::SLSpeech>("sl_speech/speech",qDepth,false);
  // and sending a new speech for texplore to use as a discrete action
  out_speech_tex = node.advertise<sl_msgs::SLSpeech>("sl_speech/new_speech",qDepth,false);

  // tell goal selector we're done with this goal
  out_done = node.advertise<std_msgs::Empty>("sl_speech/done",qDepth, false);
  // tell texplore we're done with goal that it gave us
  out_done_tex = node.advertise<std_msgs::Empty>("sl_speech/tex_done",qDepth, false);


  // Set up subscribers
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber sl_state = node.subscribe("sl_env/state", qDepth, processState, noDelay);
  ros::Subscriber sl_goal = node.subscribe("sl_effect/speech_goal", qDepth, processGoalFromES, noDelay);
  ros::Subscriber sl_goal_tex = node.subscribe("sl_texplore/speech_goal", qDepth, processGoalFromTexplore, noDelay);

  ROS_INFO(NODE ": starting main loop");
  
  ros::spin();                          // handle incoming data
  //while (ros::ok()){
  //  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  //}

  return 0;
}




