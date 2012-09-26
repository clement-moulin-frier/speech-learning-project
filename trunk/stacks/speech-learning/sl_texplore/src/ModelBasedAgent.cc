/** \file ModelBasedAgent.cc
    Implements the ModelBasedAgent class
    \author Todd Hester
*/

#include "ModelBasedAgent.hh"
#include <algorithm>

#include <sys/time.h>

ModelBasedAgent::ModelBasedAgent(int numactions, float gamma, 
                                 float rmax, float rrange,
                                 int modelType,	int exploreType, 
                                 int predType, int nModels, int plannerType, 
                                 float epsilon, float lambda, float MAX_TIME,
                                 float m, const std::vector<float> &featmin,
                                 const std::vector<float> &featmax, 
                                 std::vector<int> nstatesPerDim, int history, float b, float v,
                                 int depTrans, bool relTrans, float featPct, bool stoch, bool episodic,
                                 Random rng):
  featmin(featmin), featmax(featmax),
  numactions(numactions), gamma(gamma), rmax(rmax), rrange(rrange),
  qmax(rmax/(1.0-gamma)), 
  modelType(modelType), exploreType(exploreType), 
  predType(predType), nModels(nModels), plannerType(plannerType),
  epsilon(epsilon), lambda(lambda), MAX_TIME(MAX_TIME),
  M(m), statesPerDim(nstatesPerDim), history(history), b(b), v(v),
  depTrans(depTrans), relTrans(relTrans), featPct(featPct),
  stoch(stoch), episodic(episodic), rng(rng)
{

  if (statesPerDim[0] > 0){
    cout << "MBA: Planner will use states discretized by various amounts per dim with continuous model" << endl;
  }

  initParams();
  initModel(featmin.size());
}




void ModelBasedAgent::initParams(){

  nstates = 0;
  nactions = 0;

  model = NULL;
  planner = NULL;

  modelUpdateTime = 0.0;
  planningTime = 0.0;
  actionTime = 0.0;
  
  modelChanged = false;

  
  BATCH_FREQ = 1; //50;

  TIMEDEBUG = false; //true;
  AGENTDEBUG = false;
  ACTDEBUG = false;//true;
  SIMPLEDEBUG = false; //true; //false; //true;

  seeding = false;
  
  if (SIMPLEDEBUG)
    cout << "qmax: " << qmax  << endl;

}

ModelBasedAgent::~ModelBasedAgent() {
  delete planner;
  delete model;
  featmin.clear();
  featmax.clear();
  prevstate.clear();
}

int ModelBasedAgent::first_action(const std::vector<float> &s) {
  if (AGENTDEBUG) cout << "first_action(s)" << endl;

  if (model == NULL){
    initModel(s.size());
  }

  planner->setFirst();
 
  // in case we didn't do it already
  if (plannerType == PARALLEL)
    planner->planOnNewModel();

  // choose an action
  int act = chooseAction(s);

  // save curr state/action for next time
  saveStateAndAction(s, act);

  if (ACTDEBUG)
    cout << "Took action " << act << " from state " 
	 << s[0] << "," << s[1] 
	 << endl;

  // return that action
  return act;

}

int ModelBasedAgent::next_action(float r, const std::vector<float> &s) {
  if (AGENTDEBUG) {
    cout << "next_action(r = " << r 
	 << ", s = " << &s << ")" << endl;
  }
  
  if (SIMPLEDEBUG) cout << "Got Reward " << r;
 
  // update our models
  // this is where we possibly plan again if model changes
  updateWithNewExperience(prevstate, s, prevact, r, false);

  // choose an action
  int act = chooseAction(s);
  
  // save curr state/action for next time
  saveStateAndAction(s, act);

  if (ACTDEBUG){
    cout << "Took action " << act << " from state " 
	 << (s)[0];
    for (unsigned i = 1; i < s.size(); i++){
      cout << "," << (s)[i];
    }
    cout << endl;
  }

  // return that action
  return act;

}

void ModelBasedAgent::last_action(float r) {
  if (AGENTDEBUG) cout << "last_action(r = " << r
		    << ")" << endl;

  if (AGENTDEBUG) cout << "Got Reward " << r;

  // update our models
  // this is where we possibly plan again if model changes
  updateWithNewExperience(prevstate, prevstate, prevact, r, true);

}



/////////////////////////////
// Functional functions :) //
/////////////////////////////


void ModelBasedAgent::initModel(int nfactors){
  if ( AGENTDEBUG) cout << "initModel nfactors: " << nfactors << endl;
 
  bool needConf = true;

  std::vector<float> featRange(featmax.size(), 0);
  for (unsigned i = 0; i < featmax.size(); i++){
    featRange[i] = featmax[i] - featmin[i];
    cout << "feature " << i << " has range " << featRange[i] << endl;
  }
  cout << "reward range: " << rrange << endl;

  float treeRangePct = 0.0001;
  
  // just to ensure the diff models are on different random values
  for (int i = 0; i < modelType; i++){
    rng.uniform(0, 1);
  }

  model = new FactoredModel(0,numactions, M, modelType, predType, exploreType, nModels, treeRangePct, featRange, rrange, needConf, depTrans, relTrans, featPct, stoch, episodic, rng);

  
  MDPModel* m2 = model;
  
  cout << "Creating exploration model" << endl;
  model = new ExplorationModel(m2, modelType, exploreType,
                               predType, nModels, M, numactions,
                               rmax, qmax, rrange, nfactors, b,v,
                               featmax, featmin, rng);
  
  initPlanner();
  planner->setModel(model);

}

void ModelBasedAgent::initPlanner(){
  if (AGENTDEBUG) cout << "InitPlanner type: " << plannerType << endl;

  int max_path = 100; 

  if (plannerType == PARALLEL){
    planner = new ParallelETUCTGivenGoal(numactions, gamma, rrange, lambda, 500000, MAX_TIME, max_path, modelType, featmax, featmin, statesPerDim, false, rng);
  } else {
    planner = new ETUCTGivenGoal(numactions, gamma, rrange, lambda, 500000, MAX_TIME, max_path, modelType, featmax, featmin, statesPerDim, false, rng);
  }

}

void ModelBasedAgent::updateWithNewExperience(const std::vector<float> &last, 
                                              const std::vector<float> &curr, 
                                              int lastact, float reward, 
                                              bool terminal){
  if (AGENTDEBUG) cout << "updateWithNewExperience(last = " << &last 
                       << ", curr = " << &curr
                       << ", lastact = " << lastact 
                       << ", r = " << reward
                       << ", t = " << terminal
                       << ")" << endl;
  
  double initTime = 0;
  double timeTwo = 0;
  double timeThree = 0;

  if (model == NULL)
    initModel(last.size());

  // update our models and see if they change
  if (false || TIMEDEBUG) initTime = getSeconds();

  modelChanged = planner->updateModelWithExperience(last, lastact, curr, reward, terminal) || modelChanged;

  if (false || TIMEDEBUG) timeTwo = getSeconds();

  if (AGENTDEBUG) cout << "Agent Added exp: " << modelChanged << endl;

  // tell the planner to update with the updated model
  planner->planOnNewModel();
  modelChanged = false;

  if (TIMEDEBUG){

    timeThree = getSeconds();
    
    planningTime += (timeThree-timeTwo);
    modelUpdateTime += (timeTwo - initTime);
    
    if (nactions % 10 == 0){
      cout << nactions 
	   << " UpdateModel " << modelUpdateTime/ (float)nactions
	   << " createPolicy " << planningTime/(float)nactions << endl;
      
    }
  }


}


int ModelBasedAgent::chooseAction(const std::vector<float> &s){
  if (AGENTDEBUG) cout << "chooseAction(s = " << &s 
		    << ")" << endl;

  double initTime = 0;
  double timeTwo = 0;

  // get action to take from planner
  if (TIMEDEBUG) initTime = getSeconds();
  int act = planner->getBestAction(s);

  // convert to actual action if boss augmetned model
  if (predType == BOSS_COMBO){
    int trueAct = act % numactions;
    if (AGENTDEBUG) cout << "Using BOSS model, convert action: " << act << " to true action: " << trueAct << endl;
    act = trueAct;
  }

  if (TIMEDEBUG) {
    timeTwo = getSeconds();
    planningTime += (timeTwo - initTime);
  }

  if ((exploreType == EPSILONGREEDY && rng.bernoulli(epsilon)) || exploreType == RANDOM_EXPLORE){
    //if (true) cout << "Random action" << endl;
    act = rng.uniformDiscrete(0, numactions-1);
  }

  if (SIMPLEDEBUG){
    cout << endl << "Action " << nactions
	 << ": State " << (s)[0];
    for (unsigned i = 1; i < s.size(); i++){
      cout << "," << (s)[i];
    }
    cout << ", Took action " << act << ", ";
  }

  nactions++;

  // return index of action
  return act;
}

void ModelBasedAgent::saveStateAndAction(const std::vector<float> &s, int act){
  if (AGENTDEBUG) cout << "saveStateAndAction(s = " << &s 
		    << ", act = " << act
		    << ")" << endl;
  prevstate = s;
  prevact = act;

}





double ModelBasedAgent::getSeconds(){
  struct timezone tz;
  timeval timeT;
  gettimeofday(&timeT, &tz);
  return  timeT.tv_sec + (timeT.tv_usec / 1000000.0);
}


void ModelBasedAgent::seedExp(std::vector<experience> seeds){
  if (AGENTDEBUG) cout << "seed experiences" << endl;

  if (seeds.size() == 0) return;

  if (model == NULL)
    initModel(seeds[0].s.size());

  seeding = true;
  planner->setSeeding(true);

  // for each seeding experience, update our model
  for (unsigned i = 0; i < seeds.size(); i++){
    experience e = seeds[i];

    // update our models
    // this is where we possibly run qmax again if model(s) change
    updateWithNewExperience(e.s, e.next, e.act, e.reward, e.terminal);

    /*
    cout << "Seeding with experience " << i << endl;
    cout << "last: " << (*curr)[0] << ", " << (*curr)[1] << ", " 
	 << (*curr)[2] << endl;
    cout << "act: " << e.act << " r: " << e.reward << endl;
    cout << "next: " << (*next)[0] << ", " << (*next)[1] << ", " 
	 << (*next)[2] << ", " << e.terminal << endl;
    */

  }

  seeding = false;
  planner->setSeeding(false);

  if (seeds.size() > 0)
    planner->planOnNewModel();

}


 void ModelBasedAgent::setDebug(bool d){
   AGENTDEBUG = d;
 }

void ModelBasedAgent::savePolicy(const char* filename){
  planner->savePolicy(filename);
}



void ModelBasedAgent::logValues(ofstream *of, int xmin, int xmax, int ymin, int ymax){

  //  planner->logValues(of, xmin, xmax, ymin, ymax);

}
