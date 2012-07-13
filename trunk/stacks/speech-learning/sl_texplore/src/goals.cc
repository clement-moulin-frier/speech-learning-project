/** \file Main file that starts agents and environments
    \author Todd Hester
*/

#include "../Common/Random.h"
#include "../Common/core.hh"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

//////////////////
// Environments //
//////////////////
#include "../Env/SpeechArmTable.hh"

////////////
// Agents //
////////////
#include "../Agent/ModelBasedAgent.hh"

#include "../Planners/ETUCTGivenGoal.hh"


//#include "../FittedRmaxQ/NickAgent.hh"

// hand coded ones
//#include "../Agent/HandStock.hh"
//#include "../Agent/HandCodedFuel.hh"
//#include "../Agent/HandCodedExplore.hh"
//#include "../Agent/HandCoded.hh"


#include <vector>
#include <sstream>
#include <iostream>

#define P_ENV     1
#define P_STOCH   2

#define P_NSTEPS  3
#define P_EXPLORE 4
#define P_VCOEFF  5
#define P_NCOEFF  6
#define P_ACTRATE 7

#define P_SEED    8
#define P_MAX     8

const unsigned MAXGOALS = 1000;
const bool PRINTS = true;


int depTrans = 0;
bool relTrans = true;


double getSeconds();
void logExp(ofstream *v, int epi, int step, std::vector<float> s, int a, float r);

int main(int argc, char **argv) {

  if (argc != P_MAX+1){// || *argv[P_AGENT] != 'm')) {
    std::cerr << "Usage: rl <env> <stoch> <nsteps> <exploreType> <v> <n> <actrate> <seed>\n";
    std::cerr << "Env: table0 table1 table3 table5\n";
    std::cerr << "Stochastic: 0-deterministic, 1-stochastic \n";
    std::cerr << "Nsteps: nsteps to reach each goal\n";
    std::cerr << "Explore: 0-unknown, 3-contbonus, 4-threshbonus, 5-contbonus+R 6-threshbonus+R 7-noexplore 8-epsilon-greedy 19-b+v 27-boltzmann\n";
    std::cerr << "v: variance coefficient\n";
    std::cerr << "n: novelty coefficient\n";
    std::cerr << "Action Hz rate\n";
    exit(0);
  }


  bool stochastic = false;
  if (std::atoi(argv[P_STOCH]) == 1){
    if (PRINTS) cout << "Stohastic\n";
    stochastic = true;
  } else if (std::atoi(argv[P_STOCH]) == 0) {
    if (PRINTS) cout << "Deterministic\n";
    stochastic = false;
  } else {
    std::cerr << "Invalid value for stochastic" << endl;
    exit(-1);
  }

  // define/print model,planner,explore type for modelbasedagent
  int modelType = C45TREE;
  int exploreType = 0;
  int plannerType = ET_UCT;
  int nModels = 5;
  int predType = AVERAGE;
  int history = 0;
  float v = std::atof(argv[P_VCOEFF]);
  float n = std::atof(argv[P_NCOEFF]);
  float featPct = 0.2;
  int nsteps = std::atoi(argv[P_NSTEPS]);

  exploreType = std::atoi(argv[P_EXPLORE]);
  if ((unsigned)exploreType < (sizeof(exploreNames) / sizeof(std::string))) {
    if (PRINTS) cout << "Explore: " << exploreNames[exploreType] << endl;
  }
  else {
    std::cerr << "ERROR: Invalid exploration type\n";
    exit(-1);
  }

  float actrate = std::atof(argv[P_ACTRATE]);

  std::ostringstream expstring2;
  expstring2 << exploreType;

  // create the appropriate condor file for this
  if (std::atoi(argv[P_SEED]) == -1){
    cout << "Seed -1, make condor file" << endl << flush;

    string filename("Condor/condor.goals.");

    for (unsigned i = 1; i <= P_MAX; i++){
      if (i == P_SEED) continue;
      filename += argv[i];
      filename += ".";
    }
    filename += "desc";

    cout << "\nCreating condor file: " << filename << endl << flush;
    cout << endl << "condor_submit " << filename << endl << flush;

    ofstream fout(filename.c_str());
    //ofstream fout("Condor/condor.desc");
    fout << "+Group = \"GRAD\"\n";
    fout << "+Project = \"AI_ROBOTICS\"\n";
    fout << "+ProjectDescription = \"Reinforcement Learning\"\n";
    fout << "universe = vanilla\n";
    fout << "executable = Build/goals\n";
    fout << "Requirements = Memory >= 4000 && Lucid\n";

    fout << "Log = ";
    for (unsigned i = 1; i <= P_MAX; i++){
      if (i == P_SEED) continue;
      fout << argv[i] << ".";
    }
    fout << "condor.goals.log\n";

    fout << "Notification = Never\n";

    fout << "Arguments = ";
    for (unsigned i = 1; i <= P_MAX; i++){
      if (i == P_SEED) fout << "$(Process) ";
      else fout << argv[i] << " ";
    }
    fout << "\n";

    fout << "Output = ";
    for (unsigned i = 1; i <= P_MAX; i++){
      if (i == P_SEED) continue;
      fout << argv[i] << ".";
    }
    fout << "condor.goals.out.$(Process)\n";

    fout << "Error = ";
    for (unsigned i = 1; i <= P_MAX; i++){
      if (i == P_SEED) continue;
      fout << argv[i] << ".";
    }
    fout << "condor.goals.err.$(Process)\n";

    fout << "Queue 15\n";

    fout.close();
    exit(-1);
  }

  Random rng(1 + std::atoi(argv[P_SEED]));

  std::vector<int> statesPerDim;

  // Construct environment here.
  SpeechArmTable* e;
  int nobjects = 0;
  if (strcmp(argv[P_ENV], "table0") == 0){
    e = new SpeechArmTable(rng, stochastic, 5, 0, 0.6);
    nobjects = 0;
  }

  else if (strcmp(argv[P_ENV], "table1") == 0){
    e = new SpeechArmTable(rng, stochastic, 5, 1, 0.6);
    nobjects = 1;
  }

  else if (strcmp(argv[P_ENV], "table3") == 0){
    e = new SpeechArmTable(rng, stochastic, 5, 3, 0.6);
    nobjects = 3;
  }

  else if (strcmp(argv[P_ENV], "table5") == 0){
    e = new SpeechArmTable(rng, stochastic, 5, 5, 0.6);
    nobjects = 5;
  }

  else {
    std::cerr << "Invalid env type" << endl;
    exit(-1);
  }

  //exit(-1);

  int numactions = e->getNumActions(); // Most agents will need this?
  int numspeeches = 0;
  bool speechLearner = true;
  const int maxactions = numactions + speechLearner + 5;
  float gamma = 0.98; //0.998;

  std::vector<std::pair<float,float> > speeches;

  std::vector<float> minValues;
  std::vector<float> maxValues;
  e->getMinMaxFeatures(&minValues, &maxValues);

  // lets just check this for now
  for (unsigned i = 0; i < minValues.size(); i++){
    if (PRINTS) cout << "Feat " << i << " min: " << minValues[i]
                     << " max: " << maxValues[i] << endl;
  }

  // get max/min reward for the domain
  float rMax = 0.0;
  float rMin = -1.0;

  e->getMinMaxReward(&rMin, &rMax);
  float rRange = rMax - rMin;
  if (PRINTS) cout << "Min Reward: " << rMin
                   << ", Max Reward: " << rMax << endl;


  // default for gridworld like domains
  float lambda = 0.05;

  cout << "set statesPerDim to " << 0 << " for all dim" << endl;
  statesPerDim.resize(minValues.size(), 0);

  // Construct agent here.
  ModelBasedAgent* agent;
  if (PRINTS) cout << "Agent: Model Based" << endl;
  if (PRINTS) cout << "Lambda: " << lambda << endl;
  if (PRINTS) cout << "Act Rate: " << actrate << " Hz, seconds: " << (1.0/actrate) << endl;
  agent = new ModelBasedAgent(maxactions,
                              gamma,
                              rMax, rRange,
                              modelType,
                              exploreType,
                              predType,
                              nModels,
                              plannerType,
                              0.1, // epsilon
                              lambda,
                              (1.0/actrate), //0.1, //0.1, //0.01, // max time
                              10,
                              minValues, maxValues,
                              statesPerDim,//0,
                              history, v, n,
                              depTrans, relTrans, featPct, stochastic, false,
                              rng);

  agent->initModel(minValues.size());
  ETUCTGivenGoal* planner = NULL;
  planner = new ETUCTGivenGoal(maxactions, gamma, rRange, lambda, 500000, (1.0/actrate), 100, modelType, maxValues, minValues, statesPerDim, true, history, rng);

  delete agent->planner;
  agent->planner = (Planner*)planner;
  planner->setModel(agent->model);
  planner->setSpeechLearner(numactions);
  planner->setUsableActions(numactions+speechLearner);


  // goal
  std::vector<float> goal(minValues.size(), 0);
  std::vector<bool> mask(minValues.size(), false);

  // first step
  std::vector<float> es = e->sensation();
  int a = agent->first_action(es);
  e->apply(a);

  // loop through, creating goals for the agent
  for (unsigned igoal = 0; igoal < MAXGOALS; igoal++){

    // add speech actions in
    /*
    if (igoal % 2 == 0 && igoal > 1 && (numspeeches+numactions+speechLearner) < maxactions){
      int obj = 0;
      if (nobjects > 0) obj = rng.uniform(0, nobjects-1);
      // add speech near the true one for the given object
      float true_f1 = 0;
      float true_f2 = 0;
      if (obj == 0){
        true_f1 = 6.4;
        true_f2 = 3.0;
      } 
      else if (obj == 1){
        true_f1 = 3.2;
        true_f2 = 3.95;
      } 
      else if (obj == 2){
        true_f1 = 3.3;
        true_f2 = 2.5;
      } 
      else if (obj == 3){
        true_f1 = 4.75;
        true_f2 = 3.75;
      } 
      else if (obj == 4){
        true_f1 = 4.8;
        true_f2 = 2.6;
      } 

      // create speech near this true speech
      std::pair<float,float> speech(rng.uniform(true_f1-0.3, true_f1+0.3), rng.uniform(true_f2-0.3, true_f2+0.3));
      
      speeches.push_back(speech);
      cout << endl << "Add SPEECH: " << speech.first << ", " << speech.second << " as action " << (numactions + numspeeches) << endl;
      numspeeches++;
      planner->setUsableActions(numactions+numspeeches+speechLearner);
    }
    */


    // reset goal
    mask.assign(mask.size(), false);
    bool goalMatched = false;

    // choose goal type randomly:
    int goaltype = 0;
    if (nobjects > 0) goaltype = rng.uniformDiscrete(0,4);

    cout << endl << endl;

    // random goal location for hand
    if (goaltype == 0){
      goal[0] = rng.uniformDiscrete(minValues[0], maxValues[0]);
      goal[1] = rng.uniformDiscrete(minValues[1], maxValues[1]);
      mask[0] = true;
      mask[1] = true;
      cout << "Goal is Hand at: " << goal[0] << ", " << goal[1] << endl;
    }

    // make object reachable/unreachable
    else if (goaltype == 1){
      // choose object randomly
      int obj = 0;
      if (nobjects > 1) obj = rng.uniformDiscrete(0, nobjects-1);
      if (es[3+6*obj+4])
        goal[3+6*obj+4] = 0;
      else
        goal[3+6*obj+4] = 1;
      mask[3+6*obj+4] = true;
      cout << "Goal is object " << obj << " reachable: " << goal[3+6*obj+4] << endl;
    }

    // hand on top of object
    else if (goaltype == 2){
      // choose object randomly
      int obj = 0;
      if (nobjects > 1) obj = rng.uniformDiscrete(0, nobjects-1);
      goal[3+6*obj+2] = 0;
      goal[3+6*obj+3] = 0;
      mask[3+6*obj+2] = true;
      mask[3+6*obj+3] = true;
      cout << "Goal is Hand on object " << obj << endl;
    }

    // object in hand
    else if (goaltype == 3){
      // choose object randomly
      int obj = 0;
      if (nobjects > 1) obj = rng.uniformDiscrete(0, nobjects-1);
      if (es[3+6*obj+5])
        goal[3+6*obj+5] = 0;
      else
        goal[3+6*obj+5] = 1;
      mask[3+6*obj+5] = true;
      cout << "Goal is Object " << obj << " in hand: " << goal[3+6*obj+5] << endl;
    }

    // object at loc
    else if (goaltype == 4){
      // choose object randomly
      int obj = 0;
      if (nobjects > 1) obj = rng.uniformDiscrete(0, nobjects-1);
      goal[3+6*obj+0] = rng.uniformDiscrete(minValues[0], maxValues[0]);
      goal[3+6*obj+1] = rng.uniformDiscrete(minValues[1], maxValues[1]);
      mask[3+6*obj+0] = true;
      mask[3+6*obj+1] = true;
      cout << "Goal is Object " << obj << " at loc: " << goal[3+6*obj+0] << ", " << goal[3+6*obj+1] << endl;
    }
    
    // TODO: try with speech learner as one action
    

    planner->setGoal(goal, mask);


    // now let agent act for nsteps
    for (int istep = 0; istep < nsteps; istep++){
      es = e->sensation();

      // check if at goal
      goalMatched = true;
      for (unsigned i = 0; i < goal.size(); i++){
        if (mask[i] && goal[i] != es[i]){
          goalMatched = false;
          break;
        } 
      }
      
      cout << "Agent at goal: " << goalMatched << ", State: " << es[0] << ", " << es[1] << ", " << es[2] << endl;
      for (int i = 0; i < nobjects; i++){
        cout << "  Object " << i << ": " << es[3+i*6] << ", " << es[3+i*6+1] << ", " << es[3+i*6+2] << ", " << es[3+i*6+3] << ", " << es[3+i*6+4] << ", " << es[3+i*6+5] << endl;
      }
      if (goalMatched) break;

      a = agent->next_action(0, es);

      // normal action
      if (a < numactions){
        cout << "  Action: " << a << endl;
        e->apply(a);
      } 
      // try doing some speech learning!
      else if (speechLearner && a == numactions){
        cout << "  Action: " << a << " try speech learning" << endl;
        // try 3 speech actions
        for (int j = 0; j < 3; j++){
          int obj = rng.uniform(0, 2);
          // add speech near the true one for the given object
          float true_f1 = 0;
          float true_f2 = 0;
          if (obj == 0){
            true_f1 = 6.4;
            true_f2 = 3.0;
          } 
          else if (obj == 1){
            true_f1 = 3.2;
            true_f2 = 3.95;
          } 
          else if (obj == 2){
            true_f1 = 3.3;
            true_f2 = 2.5;
          } 
          else if (obj == 3){
            true_f1 = 4.75;
            true_f2 = 3.75;
          } 
          else if (obj == 4){
            true_f1 = 4.8;
            true_f2 = 2.6;
          } 
        
          // create speech near this true speech
          std::pair<float,float> speech(rng.uniform(true_f1-0.35, true_f1+0.35), rng.uniform(true_f2-0.35, true_f2+0.35));
          
          // try it
          cout << "   Try speech " << speech.first << ", " << speech.second << endl;
          e->applySpeechAction(speech.first, speech.second);
          es = e->sensation();

          // see if we reached goal
          // check if at goal
          goalMatched = true;
          for (unsigned i = 0; i < goal.size(); i++){
            if (mask[i] && goal[i] != es[i]){
              goalMatched = false;
              break;
            } 
          }

          cout << "    Agent at goal: " << goalMatched << ", State: " << es[0] << ", " << es[1] << ", " << es[2] << endl;
          for (int i = 0; i < nobjects; i++){
            cout << "  Object " << i << ": " << es[3+i*6] << ", " << es[3+i*6+1] << ", " << es[3+i*6+2] << ", " << es[3+i*6+3] << ", " << es[3+i*6+4] << ", " << es[3+i*6+5] << endl;
          }

          if (goalMatched){
            cout << "  REACHED GOAL! Add new speech" << endl;
            
            speeches.push_back(speech);
            cout << endl << "  Add SPEECH: " << speech.first << ", " << speech.second << " as action " << (numactions + numspeeches + speechLearner) << endl;
            numspeeches++;
            planner->setUsableActions(numactions+numspeeches+speechLearner);
            break;
          }
        } // loop of 3 speech attempts
      }
      // speech action
      else {
        int index = a-numactions-speechLearner;
        cout << "  Action: " << a << " Speech: " << index << endl << flush;
        cout << "   Speech: " << speeches[index].first << ", " << speeches[index].second << endl;
        e->applySpeechAction(speeches[index].first, speeches[index].second);
      }
        
      } // steps / goal

    cout << "Goal reached? " << goalMatched << endl;

  } // goals


  // clean stuff up
  delete e;

}



void logExp(ofstream *vout, int i, int steps, std::vector<float> es, int a, float r){
  *vout << i << "\t" << steps << "\t";
  for (unsigned jj = 0; jj < es.size(); jj++){
    *vout << es[jj] << "\t";
  }
  *vout << a << "\t" << r << endl;
}




double getSeconds(){
  struct timezone tz;
  timeval timeT;
  gettimeofday(&timeT, &tz);
  return  timeT.tv_sec + (timeT.tv_usec / 1000000.0);
}
