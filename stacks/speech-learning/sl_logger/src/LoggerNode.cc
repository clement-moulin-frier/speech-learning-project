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



void processState(const sl_msgs::SLState::ConstPtr &stateIn){
  if (lastGoalWasEvalOnly) return;
  if (PRINTS) cout << "Print " << nstates << "th state to file" << endl;

  // save state to file
  if (states){
    *states << ngoals << "\t" << nstates << "\t";
    printVector(states, stateIn->state);
    *states << endl;
  }

  nstates++;
}

void processGoal(const sl_msgs::SLGoal::ConstPtr &goalIn){
  if (PRINTS) cout << "Print " << ngoals << "th goal, id: " << goalIn->goal_id << endl;

  if (!lastGoalWasEvalOnly && goalIn->evaluateOnly){
    evalIndex = 0;
    nevalreached = 0;
  }

  lastMsg.push_back(*goalIn);
  if (lastMsg.size() > 2) lastMsg.pop_front();
  lastGoalWasEvalOnly = goalIn->evaluateOnly;
  lastGoalID = goalIn->goal_id;

  // save goal to goals attempted file
  if (!goalIn->evaluateOnly && goalsSelected){
    *goalsSelected << ngoals << "\t" << goalIn->goal_id << "\t";
    printBoolVector(goalsSelected, goalIn->goal_mask);
    printVector(goalsSelected, goalIn->goal_state);
    printBoolVector(goalsSelected, goalIn->start_mask);
    printVector(goalsSelected, goalIn->start_state);
    *goalsSelected << goalIn->title << endl << flush;
  }

  if (!goalIn->evaluateOnly)
    ngoals++;

}

void processResult(const sl_msgs::SLResult::ConstPtr &resultIn){
  if (PRINTS) cout << "Process " << nresults << "th result for goal " << resultIn->goal_id << endl;

  // find which last message matches this goal
  int goalMsgIndex = -1;
  for (unsigned i = 0; i < lastMsg.size(); i++){
    if (lastMsg[i].goal_id == resultIn->goal_id){
      goalMsgIndex = i;
      if (PRINTS) cout << "Goal id matches that of message " << i << endl;
      break;
    }
  }
  
  if (goalMsgIndex == -1){
    cout << "ERROR: goal id's do not match: last message 0: " << lastMsg[0].goal_id << ", last message 1: " << lastMsg[1].goal_id << ", result message: " << resultIn->goal_id << endl;
  }


  // if we've gotten all eval results, print the line
  if (!lastMsg[goalMsgIndex].evaluateOnly && lastResultWasEvalOnly && evalPerf){
    float pctReached = (float)nevalreached / (float)evalResults.size();
    *evalPerf << nresults << "\t" << pctReached << "\t";
    printVector(evalPerf, evalResults);
    *evalPerf << endl;
  }
  
  // if eval, then its special
  if (lastMsg[goalMsgIndex].evaluateOnly){
    lastResultWasEvalOnly = true;
    if (evalResults.size() < (1+evalIndex))
      evalResults.push_back(resultIn->goal_dist);
    else
      evalResults[evalIndex] = resultIn->goal_dist;
    evalIndex++;
    if (resultIn->goal_dist == 0)
      nevalreached++;
  }

  // normal result..
  else {
    lastResultWasEvalOnly = false;

    // if dist is 0, goal was reached and we want to save it 
    if (resultIn->goal_dist == 0 && goalsReached){
      *goalsReached << nresults << "\t" << ngoalsreached << "\t";
      *goalsReached << lastMsg[goalMsgIndex].goal_id << "\t";
      printBoolVector(goalsReached, lastMsg[goalMsgIndex].goal_mask);
      printVector(goalsReached, lastMsg[goalMsgIndex].goal_state);
      printBoolVector(goalsReached, lastMsg[goalMsgIndex].start_mask);
      printVector(goalsReached, lastMsg[goalMsgIndex].start_state);
      *goalsReached << lastMsg[goalMsgIndex].title << endl;
    }

    if (resultIn->goal_dist == 0)
      ngoalsreached++;

    // update performance for this goal
    if (allResults[resultIn->goal_id] == 0 && resultIn->goal_dist != 0){
      // no longer reaching this goal
      totalGoalsReached--;
    }
    else if (allResults[resultIn->goal_id] != 0 && resultIn->goal_dist == 0){
      // suddenly we can reach this goal
      totalGoalsReached++;
    }
    allResults[resultIn->goal_id] = resultIn->goal_dist;

    // and save updated results to file
    if (allPerf){
      float pctReached = (float)totalGoalsReached/(float)totalGoals;
      *allPerf << nresults << "\t" << pctReached << "\t";
      printVector(allPerf, allResults);
      *allPerf << endl;
    }
    
    nresults++;
  }

}


void processSpeech(const sl_msgs::SLSpeech::ConstPtr &speechIn){
  if (lastGoalWasEvalOnly) return;
  if (PRINTS) cout << "Log " << nspeeches << " speech called" << endl;

  if (speeches){
    *speeches << ngoals << "\t" << nstates << "\t" << nspeeches << "\t";
    *speeches << speechIn->f1 << "\t" << speechIn->f2 << endl;
  }

  nspeeches++;
}


void processSpeechFromTexplore(const sl_msgs::SLSpeech::ConstPtr &speechIn){
  if (lastGoalWasEvalOnly) return;
  if (PRINTS) cout << "TEXPLORE called speech action" << endl;

  ntexspeeches++;

  // save num speeches called by texplore compared with speech learner calls
  if (texploreSpeeches){
    *texploreSpeeches << ngoals << "\t" << nstates << "\t" << nspeeches << "\t";
    *texploreSpeeches << ntexspeeches << "\t" << ntexspeechlearners;
    *texploreSpeeches << "\t" << speechIn->f1 << "\t" << speechIn->f2 << endl;
  }
  
  processSpeech(speechIn);
}


void processGoalFromTexplore(const sl_msgs::SLGoal::ConstPtr &goalIn){
  if (lastGoalWasEvalOnly) return;
  if (PRINTS) cout << "TEXPLORE called speech learner" << endl;
 
  ntexspeechlearners++;

  // save num speeches called by texplore compared with speech learner calls
  if (texploreSpeeches){
    *texploreSpeeches << ngoals << "\t" << nstates << "\t" << nspeeches << "\t";
    *texploreSpeeches << ntexspeeches << "\t" << ntexspeechlearners;
    *texploreSpeeches << "\t-1\t-1" << endl;
  }
    
}


void printVector(ofstream* out, std::vector<float> vec){
  for (unsigned i = 0; i < vec.size(); i++){
    *out << vec[i] << "\t";
  }
}

void printBoolVector(ofstream* out, std::vector<unsigned char> vec){
  for (unsigned i = 0; i < vec.size(); i++){
    if (vec[i]) *out << "1\t";
    else *out << "0\t";
  }
}

void processEnvDesc(const sl_msgs::SLEnvDescription::ConstPtr &envIn){
  
  // really just want to init the files using some info from this
  initOutputFiles(envIn->num_objects, envIn->width);

  // figure out how many total goals we expect there to be
  int ncells = envIn->width * 2 * envIn->width;
  totalGoals = ncells + envIn->num_objects * ncells * 2 + envIn->num_objects * 8;
  allResults.resize(totalGoals, 1.0);

  if (PRINTS) cout << "Width: " << envIn->width << ", nObjects: "<< envIn->num_objects << " so total goals: "<< totalGoals << endl;
  
}


void initOutputFiles(int numObjects, int width){

  // create base prefix
  ostringstream os(filePrefix);
  os << "speech." << filePrefix << "." << numObjects << "." << width << ".";
  std::string prefix = os.str();

  // now add on things appropriately
  if (logStates){
    ostringstream os1(prefix);
    os1 << prefix << "states";
    std::string filename = os1.str();
    states = new std::ofstream(filename.c_str());
    if (PRINTS) cout << "Log states to file: " << filename << endl;
  }

  if (logGoalsSelected){
    ostringstream os1(prefix);
    os1 << prefix << "goals.selected";
    std::string filename = os1.str();
    goalsSelected = new std::ofstream(filename.c_str());
    if (PRINTS) cout << "Log goals selected to file: " << filename << endl;
  }

  if (logGoalsReached){
    ostringstream os1(prefix);
    os1 << prefix << "goals.reached";
    std::string filename = os1.str();
    goalsReached = new std::ofstream(filename.c_str());
    if (PRINTS) cout << "Log goals reached to file: " << filename << endl;
  }

  if (logSpeeches){
    ostringstream os1(prefix);
    os1 << prefix << "speeches";
    std::string filename = os1.str();
    speeches = new std::ofstream(filename.c_str());
    if (PRINTS) cout << "Log speeches to file: " << filename << endl;
  }

  if (logEvalPerf){
    ostringstream os1(prefix);
    os1 << prefix << "perf.eval";
    std::string filename = os1.str();
    evalPerf = new std::ofstream(filename.c_str());
    if (PRINTS) cout << "Log performance on eval goals to file: " << filename << endl;
  }

  if (logAllPerf){
    ostringstream os1(prefix);
    os1 << prefix << "perf.all";
    std::string filename = os1.str();
    allPerf = new std::ofstream(filename.c_str());
    if (PRINTS) cout << "Log all performance to file: " << filename << endl;
  }

  if (logTexploreSpeeches){
    ostringstream os1(prefix);
    os1 << prefix << "texplore.speeches";
    std::string filename = os1.str();
    texploreSpeeches = new std::ofstream(filename.c_str());
    if (PRINTS) cout << "Log TEXPLORE speech calls to file: " << filename << endl;
  }

}


/** Main function to start the agent node. */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

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




