#include "env.h"


#define NODE "SLEnvironment"



void displayHelp(){
  cout << "\n Options:\n";
  cout << "--seed value (integer seed for random number generator)\n";
  cout << "--deterministic (deterministic version of domain)\n";
  cout << "--stochastic (stochastic version of domain)\n";
  cout << "--width value (width of domain)\n";
  cout << "--nobjects value (# of objects in the domain)\n";
  cout << "--speech_radius value (how close does speech have to be to be counted)\n";
  cout << "--prints (turn on debug printing of actions/states)\n";
  exit(-1);
}

void publishState(){

  sl_msgs::SLState stateMsg;
  stateMsg.state = e->sensation();

  // publish the state message
  if (PRINTS){ 
    cout << "Now at state: ";
    for (unsigned i = 0; i < stateMsg.state.size(); i++){
      cout << stateMsg.state[i] << ", ";
    }
    cout << endl;
  }

  out_state.publish(stateMsg);

}


/** process action from the agent */
void processAction(const sl_msgs::SLAction::ConstPtr &actionIn){
  if (PRINTS) cout << "Got action " << actionIn->action << endl;

  // process action from the agent, affecting the environment
  e->apply(actionIn->action);

  publishState();
}

/** process speech action from the agent */
void processSpeech(const sl_msgs::SLSpeech::ConstPtr &speechIn){
  if (PRINTS) cout << "Got speech " << speechIn->f1 << ", " << speechIn->f2 << endl;

  // process action from the agent, affecting the environment
  e->applySpeechAction(speechIn->f1, speechIn->f2);

  publishState();
}

/** init the environment, publish a description. */
void initEnvironment(){

  // init the environment
  e = NULL;
  sl_msgs::SLEnvDescription desc;
  
  desc.title = "Speech Arm Table Domain\n";
  e = new SpeechArmTable(rng, stochastic, width, nobjects, speech_radius);

  // fill in some parameters
  desc.width = width;
  desc.num_objects = nobjects;
  desc.stochastic = stochastic;
  desc.num_speeches = nobjects;


  // fill in some more description info
  desc.num_actions = e->getNumActions();
  desc.episodic = e->isEpisodic();

  std::vector<float> maxFeats;
  std::vector<float> minFeats;

  e->getMinMaxFeatures(&minFeats, &maxFeats);
  desc.num_states = minFeats.size();
  desc.num_state_features = minFeats.size();
  desc.min_state_range = minFeats;
  desc.max_state_range = maxFeats;

  float minReward;
  float maxReward;
  e->getMinMaxReward(&minReward, &maxReward);
  desc.max_reward = maxReward;
  desc.reward_range = maxReward - minReward;

  cout << desc.title << endl;

  // publish environment description
  out_env_desc.publish(desc);

  sleep(1);

  // now send first state message
  sl_msgs::SLState stateMsg;
  stateMsg.state = e->sensation();
  out_state.publish(stateMsg);
  
}


/** Main function to start the env node. */
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
    {"deterministic", 0, 0, 'd'},
    {"stochastic", 0, 0, 's'},
    {"nobjects", 1, 0, 'o'},
    {"width", 1, 0, 'w'},
    {"speech_radius", 1, 0, 'r'},
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

    case 'd':
      stochastic = false;
      cout << "stochastic: " << stochastic << endl;
      break;

    case 's':
      stochastic = true;
      cout << "stochastic: " << stochastic << endl;
      break;

    case 'o':
      nobjects = std::atoi(optarg);
      cout << "nobjects: "<< nobjects << endl;
      break;

    case 'w':
      width = std::atoi(optarg);
      cout << "width: "<< width << endl;
      break;

    case 'r':
      speech_radius = std::atof(optarg);
      cout << "speech_radius: "<< speech_radius << endl;
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

  // Set up Publishers
  ros::init(argc, argv, "my_tf_broadcaster");
  tf::Transform transform;

  // Set up Publishers
  out_env_desc = node.advertise<sl_msgs::SLEnvDescription>("sl_env/env_description",qDepth,true);
  out_state = node.advertise<sl_msgs::SLState>("sl_env/state",qDepth,false);

  // Set up subscribers
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber slt_action =  node.subscribe("sl_texplore/action", qDepth, processAction, noDelay);
  ros::Subscriber slt_speech = node.subscribe("sl_texplore/speech", qDepth, processSpeech, noDelay);
  ros::Subscriber sls_speech = node.subscribe("sl_splearner/speech", qDepth, processSpeech, noDelay);

  // publish env description, first state
  // Setup RL World
  rng = Random(1+seed);
  initEnvironment();

  ROS_INFO(NODE ": starting main loop");
  
  ros::spin();                          // handle incoming data
  //while (ros::ok()){
  //  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  //}

  return 0;
}




