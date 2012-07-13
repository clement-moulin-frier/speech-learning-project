/** \file Main file that starts the TEXPLORE agent to reach a given goal
    \author Todd Hester
*/

#include "TexploreNode.hh"


#define NODE "SLTexplore"


void displayHelp(){
  cout << "\n Options:\n";
  cout << "--seed value (integer seed for random number generator)\n";
  cout << "--nsteps (how many steps TEXPLORE gets to reach the goal)\n";
  cout << "--actrate (how many actions/sec TEXPLORE should take)\n";
  cout << "--v (variance coefficient for exploration)\n";
  cout << "--n (novelty coefficient for exploration)\n";
  cout << "--master (TEXPLORE decides when to call speech learner)\n";
  cout << "--prints (turn on debug printing of actions/states)\n";
  exit(-1);
}

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sstream>
#include <iostream>


/** Publish our action message. */
void publishAction(int action){

  if (action < numactions){
    // normal discrete action
    if (PRINTS) cout << "Normal discrete action: " << action << endl;
    sl_msgs::SLAction actMsg;
    actMsg.action = action;
    out_action.publish(actMsg);
  }
  else if (action == numactions && master){
    // calling sagg-riac speech learner
    if (PRINTS) cout << "TEXPLORE is calling SAGG-RIAC speech learner" << action << endl;
    out_goal.publish(lastGoalMsg);
    // no learning on our side until speech learning is done
    learningActive = false;
  } else {
    // calling one of our saved speeches
    int speechIndex = action - numactions - master;
    if (PRINTS) cout << "TEXPLORE calling speech action " << speechIndex << ": f1: " << speeches[speechIndex].first << ", " << speeches[speechIndex].second << endl;
    sl_msgs::SLSpeech speechMsg;
    speechMsg.f1 = speeches[speechIndex].first;
    speechMsg.f2 = speeches[speechIndex].second;
    out_speech.publish(speechMsg);
  }

}

/** Process done */
void processDone(const std_msgs::Empty::ConstPtr &doneIn){
  if (PRINTS) cout << "Speech learner is done, we are active again" << endl;
  learningActive = true;
  selectNextAction();
}

/** Process a new speech message. */
void processSpeech(const sl_msgs::SLSpeech::ConstPtr &speechIn){
  // if we have too many, reject
  if (numspeeches >= maxspeeches){
    if (PRINTS) cout << "Already too many speeches!" << endl;
    return;
  }

  std::pair<float, float> speech(speechIn->f1, speechIn->f2);

  // make sure its not too close to ones we already have
  for (int i = 0; i < numspeeches; i++){
    if (fabs(speeches[i].first - speechIn->f1) < 0.1 && fabs(speeches[i].second - speechIn->f2) < 0.1){
      if (PRINTS) cout << "New speech too close to speech " << i << endl;
      return;
    }
  }

  speeches.push_back(speech);
  if (PRINTS){
    cout << endl << "  Add SPEECH: " << speech.first << ", " << speech.second << " as action " << (numactions + numspeeches + master) << endl;
  }
  numspeeches++;
  planner->setUsableActions(numactions+numspeeches+master);
}

/** Process a new state message. */
void processState(const sl_msgs::SLState::ConstPtr &stateIn){
  lastState = stateIn->state;

  // do nothing if we're not the learner right now
  if (!learningActive) return;
  
  selectNextAction();
}

void selectNextAction(){
  
  istep++;
  
  int a = agent->next_action(0, lastState);

  // check if we're at the goal, or have reached the maximum number of steps
  // check if at goal
  bool goalMatched = true;
  for (unsigned i = 0; i < goal.size(); i++){
    if (mask[i] && goal[i] != lastState[i]){
      goalMatched = false;
      break;
    } 
  }

  if (istep >= nsteps || goalMatched){
    learningActive = false;
    std_msgs::Empty doneMsg;
    out_done.publish(doneMsg);
    return;
  }

  // otherwise, keep on acting
  publishAction(a);
  
}

/** Process a new goal message. */
void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn){
  lastGoalMsg = *goalIn;
  
  for (unsigned i = 0; i < goal.size(); i++){
    goal[i] = goalIn->goal_state[i];
    mask[i] = goalIn->goal_mask[i];
  }

  // print new goal
  if (PRINTS){
    cout << "Set goal: " << endl;
    for (unsigned i = 0; i < goal.size(); i++){
      if (mask[i])
        cout << "  Feat " << i << ": Goal value: " << goal[i] << endl;
    }
  }

  // reset # steps counter
  istep = 0;

  // set agent eval mode
  planner->setEvaluationMode(goalIn->evaluateOnly);

  // set goal for agent
  planner->setGoal(goal, mask);
  learningActive = true;

  // call first action
  int a = agent->first_action(lastState);

  publishAction(a);
}

/** process env description. init agent. */
void processEnv(const sl_msgs::SLEnvDescription::ConstPtr &envIn){
  if (PRINTS) cout << "Got envrionment " << envIn->title << endl;

  numactions = envIn->num_actions;
  numspeeches = 0;
  const int maxactions = numactions + master + maxspeeches;
  float gamma = 0.98; //0.998;

  // lets just check this for now
  for (unsigned i = 0; i < envIn->min_state_range.size(); i++){
    if (PRINTS) cout << "Feat " << i << " min: " << envIn->min_state_range[i]
                     << " max: " << envIn->max_state_range[i] << endl;
  }

  // get max/min reward for the domain
  float rMax = envIn->max_reward;
  float rMin = envIn->max_reward - envIn->reward_range;
  float rRange = envIn->reward_range;
  if (PRINTS) cout << "Min Reward: " << rMin
                   << ", Max Reward: " << rMax << endl;


  // default for gridworld like domains
  float lambda = 0.05;

  std::vector<int> statesPerDim;
  statesPerDim.resize(envIn->min_state_range.size(), 0);
  goal.resize(envIn->min_state_range.size(), 0);
  mask.resize(envIn->min_state_range.size(), false);

  // Construct agent here.
  ModelBasedAgent* agent;
  if (PRINTS) cout << "Agent: Model Based" << endl;
  if (PRINTS) cout << "Lambda: " << lambda << endl;
  if (PRINTS) cout << "Act Rate: " << actrate << " Hz, seconds: " << (1.0/actrate) << endl;
  agent = new ModelBasedAgent(maxactions,
                              gamma,
                              rMax, rRange,
                              C45TREE,
                              DIFF_AND_NOVEL_BONUS,
                              AVERAGE, 5,
                              0,
                              0.1, // epsilon
                              lambda,
                              (1.0/actrate), //0.1, //0.1, //0.01, // max time
                              10,
                              envIn->min_state_range, envIn->max_state_range,
                              statesPerDim,//0,
                              0, vcoeff, ncoeff,
                              false, true, 0.2, true, false,
                              rng);

  ETUCTGivenGoal* planner = NULL;
  planner = new ETUCTGivenGoal(maxactions, gamma, rRange, lambda, 500000, (1.0/actrate), 100, C45TREE, envIn->max_state_range, envIn->min_state_range, statesPerDim, true, 0, rng);

  delete agent->planner;
  agent->planner = (Planner*)planner;
  planner->setModel(agent->model);
  if (master) planner->setSpeechLearner(numactions);
  planner->setUsableActions(numactions+master);

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
    {"nsteps", 0, 0, 's'},
    {"actrate", 0, 0, 'a'},
    {"v", 1, 0, 'v'},
    {"n", 1, 0, 'n'},
    {"master", 1, 0, 'm'},
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

    case 'a':
      actrate = std::atof(optarg);
      cout << "actrate: "<< actrate << endl;
      break;

   case 'v':
      vcoeff = std::atof(optarg);
      cout << "v coeff: "<< vcoeff << endl;
      break;

   case 'n':
      ncoeff = std::atof(optarg);
      cout << "n coeff: "<< ncoeff << endl;
      break;

    case 'm':
      master = true;
      cout << "TEXPLORE controls when speech learner is called" << endl;
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
  // send actions and speeches to the environment
  out_action = node.advertise<sl_msgs::SLAction>("sl_texplore/action",qDepth,true);
  out_speech = node.advertise<sl_msgs::SLSpeech>("sl_texplore/speech",qDepth,false);
  out_done = node.advertise<std_msgs::Empty>("sl_texplore/done",qDepth, false);
  // send new goals to sagg-riac speech learner
  out_goal = node.advertise<sl_msgs::SLSpeech>("sl_texplore/speech_goal",qDepth,false);

  // Set up subscribers
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber sl_env =  node.subscribe("sl_env/env_description", qDepth, processEnv, noDelay);
  ros::Subscriber sl_state = node.subscribe("sl_env/state", qDepth, processState, noDelay);
  ros::Subscriber sl_goal = node.subscribe("sl_effect/tex_goal", qDepth, processGoal, noDelay);
  ros::Subscriber sl_speech = node.subscribe("sl_splearner/new_speech", qDepth, processSpeech, noDelay);
  ros::Subscriber sl_done = node.subscribe("sl_splearner/tex_done", qDepth, processDone, noDelay);


  ROS_INFO(NODE ": starting main loop");
  
  ros::spin();                          // handle incoming data
  //while (ros::ok()){
  //  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  //}

  return 0;
}




