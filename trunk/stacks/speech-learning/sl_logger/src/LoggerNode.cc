/** \file Starts a logging node
    \author Todd Hester
*/

#include "LoggerNode.hh"


#define NODE "SLLogger"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sstream>
#include <iostream>


void displayHelp(){
  cout << "\n Options:\n";
  cout << "Turn on/off logging of various things:\n";
  cout << "--states 0/1\n";
  cout << "--goalstried 0/1\n";
  cout << "--goalsreached 0/1\n";
  cout << "--speeches 0/1\n";
  cout << "--texplore 0/1\n";
  cout << "--eval 0/1\n";
  cout << "--perf 0/1\n";
  cout << "--filename string (start log files with this string)\n";
  cout << "--prints (turn on debug printing of actions/states)\n";
  exit(-1);
}



void processState(const sl_msgs::SLState::ConstPtr &stateIn){}
void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn){}
void processSpeech(const sl_msgs::SLSpeech::ConstPtr &speechIn){}
void processSpeechFromTexplore(const sl_msgs::SLSpeech::ConstPtr &speechIn){}
void processGoalFromTexplore(const sl_msgs::SLGoal::ConstPtr &goalIn){}
void processResult(const sl_msgs::SLResult::ConstPtr &resultIn){}
void processEnvDesc(const sl_msgs::SLEnvDescription::ConstPtr &envIn){}


void initFiles(){

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
    {"states", 1, 0, 's'},
    {"goalstried", 1, 0, 'g'},
    {"goalsreached", 1, 0, 'r'},
    {"speeches", 1, 0, 'v'},
    {"texplore", 1, 0, 't'},
    {"eval", 1, 0, 'e'},
    {"perf", 1, 0, 'a'},
    {"filename", 1, 0, 'f'},
    {"prints", 0, 0, 'p'},
    {"help", 0, 0, 'h'}
  };

  while(-1 != (ch = getopt_long_only(argc, argv, optflags, long_options, &option_index))) {
    switch(ch) {

    case 's':
      logStates = std::atoi(optarg);
      cout << "Log states? " << logStates << endl;
      break;

    case 'g':
      logGoalsSelected = std::atoi(optarg);
      cout << "Log goals tried? " << logGoalsSelected << endl;
      break;

    case 'r':
      logGoalsReached = std::atoi(optarg);
      cout << "Log goals reached? " << logGoalsReached << endl;
      break;

    case 'e':
      logEvalPerf = std::atoi(optarg);
      cout << "Log eval? " << logEvalPerf << endl;
      break;

    case 'v':
      logSpeeches = std::atoi(optarg);
      cout << "Log speeches? " << logSpeeches << endl;
      break;

    case 't':
      logTexploreSpeeches = std::atoi(optarg);
      cout << "Log Texplore calls to speeches vs speech learner? " << logTexploreSpeeches << endl;
      break;

    case 'a':
      logAllPerf = std::atoi(optarg);
      cout << "Log all perf? " << logAllPerf << endl;
      break;

    case 'f':
      filePrefix = optarg;
      cout << "File prefix: " << filePrefix << endl;
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

  // Set up subscribers
  int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber sl_state = node.subscribe("sl_env/state", qDepth, processState, noDelay);
  ros::Subscriber sl_goal_s = node.subscribe("sl_effect/speech_goal", qDepth, processGoal, noDelay);
  ros::Subscriber sl_goal_t = node.subscribe("sl_effect/tex_goal", qDepth, processGoal, noDelay);
  ros::Subscriber sl_goal_tex = node.subscribe("sl_texplore/speech_goal", qDepth, processGoalFromTexplore, noDelay);
  ros::Subscriber sl_speech_t = node.subscribe("sl_texplore/speech", qDepth, processSpeechFromTexplore, noDelay);
  ros::Subscriber sl_speech_s = node.subscribe("sl_speech/speech", qDepth, processSpeech, noDelay);
  ros::Subscriber sl_result = node.subscribe("sl_effect/result", qDepth, processResult, noDelay);
  ros::Subscriber sl_env = node.subscribe("sl_env/env_description", qDepth, processEnvDesc, noDelay);



  ROS_INFO(NODE ": starting main loop");
  
  ros::spin();                          // handle incoming data
  //while (ros::ok()){
  //  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  //}

  return 0;
}




