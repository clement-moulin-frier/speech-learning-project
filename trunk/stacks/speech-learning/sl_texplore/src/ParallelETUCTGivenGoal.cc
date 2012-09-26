/** \file ParallelETUCTGivenGoal.cc
    Implements my real-time model-based RL architecture which uses UCT with eligiblity traces for planning.
    The modified version of UCT used is presented in:
    L. Kocsis and C. Szepesv´ari, "Bandit based monte-carlo planning," in
    ECML-06. Number 4212 in LNCS. Springer, 2006, pp. 282-293.
    The real-time architecture is presented in: TODO
    \author Todd Hester
*/

#include "ParallelETUCTGivenGoal.hh"
#include <algorithm>

#include <sys/time.h>


ParallelETUCTGivenGoal::ParallelETUCTGivenGoal(int numactions, float gamma, float rrange, float lambda,
                                               int MAX_ITER, float MAX_TIME, int MAX_DEPTH, int modelType,
                                               const std::vector<float> &fmax, const std::vector<float> &fmin,
                                               const std::vector<int> &nstatesPerDim, bool trackActual, Random r):
  numactions(numactions), gamma(gamma), rrange(rrange), lambda(lambda),
  MAX_ITER(MAX_ITER), MAX_TIME(MAX_TIME),
  MAX_DEPTH(MAX_DEPTH), modelType(modelType), statesPerDim(nstatesPerDim),
  trackActual(trackActual),
  CLEAR_SIZE(25)
{
  rng = r;

  nstates = 0;
  nsaved = 0;
  nactions = 0;
  lastUpdate = -1;

  seedMode = false;
  timingType = true;

  model = NULL;
  planTime = getSeconds();
  initTime = getSeconds();
  setTime = getSeconds();

  PLANNERDEBUG = false;
  POLICYDEBUG = false; //true; //false; //true; //false;
  ACTDEBUG = false; //true;
  MODELDEBUG = false;
  UCTDEBUG = false;
  PTHREADDEBUG = false;
  ATHREADDEBUG = false;//true;
  MTHREADDEBUG = false; //true;
  TIMINGDEBUG = false;
  REALSTATEDEBUG = false;
  GOALDEBUG = true;

  // init goal stuff
  goalState.resize(featmax.size(), 0);
  goalMask.resize(featmax.size(), false);

  // init some other things
  numUsableActions = numactions;
  evaluationMode = false;

  if (statesPerDim[0] > 0){
    cout << "Planner Parallel ETUCT using discretization of " << statesPerDim[0] << endl;
  }
  if (trackActual){
    cout << "Parallel ETUCT tracking real state values" << endl;
  }

  featmax = fmax;
  featmin = fmin;
  intrinsicRewards.resize(featmax.size()+1,0);

  pthread_mutex_init(&update_mutex, NULL);
  pthread_mutex_init(&nactions_mutex, NULL);
  pthread_mutex_init(&plan_state_mutex, NULL);
  pthread_mutex_init(&statespace_mutex, NULL);
  pthread_mutex_init(&model_mutex, NULL);
  pthread_mutex_init(&list_mutex, NULL);
  pthread_cond_init(&list_cond, NULL);

  pthread_mutex_init(&goal_mutex, NULL);
  pthread_mutex_init(&usable_actions_mutex, NULL);
  pthread_mutex_init(&eval_mode_mutex, NULL);

  pthread_mutex_init(&rollout_mutex, NULL);
  pthread_cond_init(&rollout_cond, NULL);
  currentRollout = false;

  // start parallel search thread
  actualPlanState = std::vector<float>(featmax.size());
  discPlanState = NULL;
  doRandom = true;
  modelThreadStarted = false;
  planThreadStarted = false;
  expList.clear();

  //  initStates();
}

ParallelETUCTGivenGoal::~ParallelETUCTGivenGoal() {
  // join threads

  //pthread_kill(planThread);
  //pthread_kill(modelThread);

  pthread_detach(planThread);//, NULL);
  pthread_detach(modelThread);//, NULL);

  pthread_cancel(planThread);//, NULL);
  pthread_cancel(modelThread);//, NULL);

  //pthread_join(planThread, NULL);
  //pthread_join(modelThread, NULL);

  //pthread_detach(planThread);//, NULL);
  //pthread_detach(modelThread);//, NULL);


  pthread_mutex_lock(&plan_state_mutex);
  pthread_mutex_lock(&statespace_mutex);
  pthread_mutex_lock(&model_mutex);
  pthread_mutex_lock(&list_mutex);

  // delete exp list
  expList.clear();

  for (std::map<state_t, state_info>::iterator i = statedata.begin();
       i != statedata.end(); i++){

    // get state's info
    //cout << "  planner got info" << endl;
    state_info* info = &((*i).second);

    deleteInfo(info);
  }

  featmax.clear();
  featmin.clear();
  intrinsicRewards.clear();

  statespace.clear();
  statedata.clear();

  pthread_mutex_unlock(&plan_state_mutex);
  pthread_mutex_unlock(&statespace_mutex);
  pthread_mutex_unlock(&model_mutex);
  pthread_mutex_unlock(&list_mutex);

}

void ParallelETUCTGivenGoal::setModel(MDPModel* m){
  pthread_mutex_lock(&model_mutex);

  model = m;

  pthread_mutex_unlock(&model_mutex);

}


/////////////////////////////
// Functional functions :) //
/////////////////////////////



/** Use the latest experience to update state info and the model. */
bool ParallelETUCTGivenGoal::updateModelWithExperience(const std::vector<float> &laststate,
                                                       int lastact,
                                                       const std::vector<float> &currstate,
                                                       float reward, bool term){
  //  if (PLANNERDEBUG) cout << "updateModelWithExperience(last = " << &laststate
  //     << ", curr = " << &currstate
  //        << ", lastact = " << lastact
  //     << ", r = " << reward
  //     << ", term = " << term
  //     << ")" << endl;

  //cout << "updateModel" << endl << flush;

  if (!timingType)
    planTime = getSeconds();
  initTime = getSeconds();

  pthread_mutex_lock(&eval_mode_mutex);
  if (evaluationMode){
    pthread_mutex_unlock(&eval_mode_mutex);
    planTime = getSeconds();
    return false;
  }
  pthread_mutex_unlock(&eval_mode_mutex);


  // canonicalize these things
  state_t last = canonicalize(laststate);

  prevstate = last;
  prevact = lastact;

  if (MODELDEBUG){
    cout << "Update with exp from state: ";
    for (unsigned i = 0; i < last->size(); i++){
      cout << (laststate)[i] << ", ";
    }
    cout << " action: " << lastact;
    cout << " to state: ";
    for (unsigned i = 0; i < currstate.size(); i++){
      cout << (currstate)[i] << ", ";
    }
    cout << " and reward: " << reward << endl;
  }

  // add experiences to list to later be updated into model
  if (ATHREADDEBUG)
    cout << "*** Action thread wants list lock ***" << endl << flush;
  if (TIMINGDEBUG) cout << "Want list mutex, time: " << (getSeconds()-initTime) << endl;
  pthread_mutex_lock(&list_mutex);
  if (TIMINGDEBUG) cout << "got list mutex, time: " << (getSeconds()-initTime) << endl;
  experience e;
  e.s = laststate;
  e.next = currstate;
  e.act = lastact;
  e.reward = reward;
  e.terminal = term;

  expList.push_back(e);
  //expfile.saveExperience(e);
  if (ATHREADDEBUG || MTHREADDEBUG)
    cout << "added exp to list, size: " << expList.size() << endl << flush;
  if (TIMINGDEBUG) cout << "list updated, time: " << (getSeconds()-initTime) << endl;
  pthread_cond_signal(&list_cond);
  pthread_mutex_unlock(&list_mutex);

  /*
    if (e.reward > -0.5 && e.reward < 0){
    expfile.saveExperience(e);
    nsaved++;
    cout << "Saved Experience " << e.reward << endl;
    }
  */

  if (timingType)
    planTime = getSeconds();

  if (TIMINGDEBUG) cout << "leaving updateModel, time: " << (getSeconds()-initTime) << endl;


  return false;

}

/** Update a single state-action from the model */
void ParallelETUCTGivenGoal::updateStateActionFromModel(state_t s, int a, state_info* info){

  pthread_mutex_lock(&info->statemodel_mutex);

  StateActionInfo* newModel = &(info->model[a]);

  pthread_mutex_lock(&model_mutex);

  model->getStateActionInfo(*s, a, newModel);

  pthread_mutex_lock(&nactions_mutex);
  newModel->frameUpdated = nactions;
  pthread_mutex_unlock(&nactions_mutex);

  pthread_mutex_unlock(&model_mutex);

  pthread_mutex_unlock(&info->statemodel_mutex);

}



void ParallelETUCTGivenGoal::canonNextStates(StateActionInfo* modelInfo){


  // loop through all next states
  for (std::map<std::vector<float>, float>::iterator outIt
         = modelInfo->transitionProbs.begin();
       outIt != modelInfo->transitionProbs.end(); outIt++){

    std::vector<float> nextstate = (*outIt).first;
    bool badState = false;

    // check that it is valid, otherwise replace with current
    for (unsigned j = 0; j < nextstate.size(); j++){
      float factor = EPSILON;
      if (statesPerDim[j] > 0)
        factor = (featmax[j] - featmin[j]) / (float)statesPerDim[j];
      if (nextstate[j] < (featmin[j]-factor)
          || nextstate[j] > (featmax[j]+factor)){
        //cout << "next state out of range " << nextstate[j] << endl;
        badState = true;
        break;
      }
    }

    if (!badState){

      canonicalize(nextstate);
    }
  }
}




/** Choose the next action */
int ParallelETUCTGivenGoal::getBestAction(const std::vector<float> &state){
  //  if (PLANNERDEBUG) cout << "getBestAction(s = " << &state << ")" << endl;

  pthread_mutex_lock(&nactions_mutex);
  nactions++;
  pthread_mutex_unlock(&nactions_mutex);

  if (TIMINGDEBUG) cout << "getBestAction, time: " << (getSeconds()-initTime) << endl;


  state_t s = canonicalize(state);

  // set plan state so uct will search from here
  if (ATHREADDEBUG)
    cout << "*** Action thread wants plan state lock ***" << endl << flush;
  if (TIMINGDEBUG) cout << "want planStateMut, time: " << (getSeconds()-initTime) << endl;

  pthread_mutex_lock(&(plan_state_mutex));
  if (TIMINGDEBUG) cout << "got planStateMut, time: " << (getSeconds()-initTime) << endl;

  doRandom = false;
  actualPlanState = state;
  discPlanState = s;
  setTime = getSeconds();

  if (ATHREADDEBUG){
    cout << "Set planning state as: ";
    for (unsigned i = 0; i < state.size(); i++){
      cout << state[i] << ", ";
    }
    cout << endl << flush;
  }

  // call uct search on it
  pthread_mutex_unlock(&(plan_state_mutex));
  if (TIMINGDEBUG) cout << "set planState, time: " << (getSeconds()-initTime) << endl;

  // get state info
  pthread_mutex_lock(&statespace_mutex);
  state_info* info = &(statedata[s]);
  pthread_mutex_unlock(&statespace_mutex);

  // wait a bit for some planning from this state

  // depending on how you run the code, this has to be setup differently
  // if someone else calls this method at the appropriate rate, do nothing here

  // or this can be where we wait to ensure we run at some rate:
  while (((getSeconds()- initTime) < MAX_TIME)){
    if (TIMINGDEBUG)
      cout << "waiting for time: " << (getSeconds()-initTime) << endl;

    pthread_yield();
  }

  if (TIMINGDEBUG) cout << "time up: " << (getSeconds()-initTime) << endl;

  if (TIMINGDEBUG && (getSeconds()-initTime) > 0.15) cout << "**********" << endl;

  pthread_mutex_lock(&info->stateinfo_mutex);

  // Get Q values
  std::vector<float> &Q = info->Q;


  // Choose an action
  pthread_mutex_lock(&usable_actions_mutex);
  const std::vector<float>::iterator a =
    random_max_element(Q.begin(), Q.begin() + numUsableActions); // Choose maximum
  int act = a - Q.begin();
  pthread_mutex_unlock(&usable_actions_mutex);

  if (ATHREADDEBUG) {
    cout << "Getting best action " << act << " from state ";
    for (unsigned i = 0; i < s->size(); i++){
      cout << (*s)[i] << ", ";
    }
    cout << " sampled " << info->uctVisits << " times. " << endl << flush;
  }

  if (TIMINGDEBUG) cout << "got action: " << (getSeconds()-initTime) << endl;

  pthread_mutex_unlock(&info->stateinfo_mutex);

  // return index of action
  return act;
}






void ParallelETUCTGivenGoal::planOnNewModel(){
  //return;
  //  cout << "planOnNewModel" << endl << flush;
  // start model learning thread here
  if (!modelThreadStarted){
    modelThreadStarted = true;
    pthread_create(&modelThread, NULL, parallelGGModelLearningStart, this);
  }

  if (!planThreadStarted){
    planThreadStarted = true;
    pthread_create(&(planThread), NULL, parallelGGSearchStart, this);
  }

}


void* parallelGGModelLearningStart(void* arg){
  cout << "Start model learning thread" << endl << flush;
  ParallelETUCTGivenGoal* pe = reinterpret_cast<ParallelETUCTGivenGoal*>(arg);
  while(true){
    pe->parallelModelLearning();
    /*
      if (!pe->planThreadStarted){
      pe->planThreadStarted = true;
      pthread_create(&(pe->planThread), NULL, parallelSearchStart, pe);
      }
    */
  }
  return NULL;
}

void ParallelETUCTGivenGoal::parallelModelLearning(){
  //while(true){

  // wait for experience list to be non-empty
  pthread_mutex_lock(&list_mutex);
  while (expList.size() == 0){
    pthread_cond_wait(&list_cond,&list_mutex);
  }
  pthread_mutex_unlock(&list_mutex);

  // copy over experience list
  std::vector<experience> updateList;
  if (MTHREADDEBUG) cout << "  *** Model thread wants list lock ***" << endl << flush;
  pthread_mutex_lock(&list_mutex);
  updateList = expList;
  expList.clear();
  if (MTHREADDEBUG) cout << "  *** Model thread done with list lock ***" << endl << flush;
  pthread_mutex_unlock(&list_mutex);

  /*
  // update model
  //cout << "*** Model thread wants tree lock ***" << endl << flush;
  pthread_mutex_lock(&model_mutex);
  if (MTHREADDEBUG) cout << "  Model thread: going to update model with " << updateList.size() << " new experiences" << endl << flush;
  //cout << "****update tree with " << updateList.size() << endl << flush;
  bool modelChanged = model->updateWithExperiences(updateList);
  if (MTHREADDEBUG) cout << "  Model updated" << endl << flush;
  pthread_mutex_unlock(&model_mutex);
  */

  modelcopy = model->getCopy();
  //if (COPYDEBUG) cout << "*** PO: model copied" << endl;

  // update model copy with new experience
  bool modelChanged = modelcopy->updateWithExperiences(updateList);


  // set model pointer to point at copy, delete original model                    cout << "acquire model_mutex for update" << endl;
  pthread_mutex_lock(&model_mutex);
  //cout << "model_mutex acquired for update" << endl;
  //if (COPYDEBUG) cout << "*** PO: delete original model and change pointer" << endl;

  delete model;
  model = modelcopy;

  if (MTHREADDEBUG) cout << "  Model updated" << endl << flush;
  //if (COPYDEBUG) cout << "*** PO: pointer set to updated model copy" << endl;
  pthread_mutex_unlock(&model_mutex);



  // if it changed, reset counts, update state actions
  if (modelChanged) resetAndUpdateStateActions();

  pthread_yield();

  //}// while loop
} // method


void ParallelETUCTGivenGoal::setBetweenEpisodes(){
  // TODO: for now, I know this means we just ended an episode, lets plan
  // from a different random state (or a state we know to be the initial one)
  if (ATHREADDEBUG) cout << "*** Action thread wants planning state lock (bet eps)***" << endl;
  pthread_mutex_lock(&(plan_state_mutex));

  doRandom = true;
  discPlanState = NULL;

  // call uct search on it
  pthread_mutex_unlock(&(plan_state_mutex));

}




void ParallelETUCTGivenGoal::resetAndUpdateStateActions(){
  //cout << "*** Model changed, updating state actions ***" << endl << flush;
  const int MIN_VISITS = 10;

  pthread_mutex_lock(&nactions_mutex);
  int updateTime = nactions;
  pthread_mutex_unlock(&nactions_mutex);

  // loop through here

  pthread_mutex_lock(&statespace_mutex);

  for (std::set<std::vector<float> >::iterator i = statespace.begin();
       i != statespace.end(); i++){
    pthread_mutex_unlock(&statespace_mutex);

    state_t s = canonicalize(*i);

    if (MTHREADDEBUG) cout << "  *** Model thread wants search lock ***" << endl;

    if (MTHREADDEBUG) cout << "  *** Model thread got search lock " << endl;

    pthread_mutex_lock(&statespace_mutex);
    state_info* info = &(statedata[s]);
    pthread_mutex_unlock(&statespace_mutex);

    pthread_mutex_lock(&info->stateinfo_mutex);

    if (info->uctVisits > (MIN_VISITS * numactions))
      info->uctVisits = MIN_VISITS * numactions;

    for (int j = 0; j < numactions; j++){
      if (info->uctActions[j] > MIN_VISITS)
        info->uctActions[j] = MIN_VISITS;
      if (info->needsUpdate){
        updateStateActionFromModel(s, j, info);
      }
    }

    info->needsUpdate = false;
    pthread_mutex_unlock(&info->stateinfo_mutex);

    // TODO: clean up unused states without locking issues
    /*
      else {
      // remove it as unnecessary
      deleteInfo(info);
      pthread_mutex_lock(&statespace_mutex);
      statespace.erase(i++);
      statedata.erase(s);
      pthread_mutex_unlock(&statespace_mutex);
      }
    */

    pthread_yield();

    pthread_mutex_lock(&statespace_mutex);

  }
  pthread_mutex_unlock(&statespace_mutex);

  pthread_mutex_lock(&update_mutex);
  lastUpdate = updateTime;
  pthread_mutex_unlock(&update_mutex);

}




////////////////////////////
// Helper Functions       //
////////////////////////////

ParallelETUCTGivenGoal::state_t ParallelETUCTGivenGoal::canonicalize(const std::vector<float> &s) {
  if (PLANNERDEBUG) cout << "canonicalize(s = " << s[0] << ", "
                         << s[1] << ")" << endl;

  // discretize it
  std::vector<float> s2;
  if (statesPerDim[0] > 0){
    s2 = discretizeState(s);
  } else {
    s2 = s;
  }

  pthread_mutex_lock(&statespace_mutex);

  // get state_t for pointer if its in statespace
  const std::pair<std::set<std::vector<float> >::iterator, bool> result =
    statespace.insert(s2);
  state_t retval = &*result.first; // Dereference iterator then get pointer


  // if not, init this new state
  if (result.second) { // s is new, so initialize Q(s,a) for all a
    state_info* info = &(statedata[retval]);
    int id = nstates++;
    pthread_mutex_unlock(&statespace_mutex);
    initStateInfo(retval, info, id);
  } else {
    pthread_mutex_unlock(&statespace_mutex);
  }

  return retval;
}


// init state info
void ParallelETUCTGivenGoal::initStateInfo(state_t s, state_info* info, int id){
  //if (PLANNERDEBUG) cout << "initStateInfo()";

  // init mutex's for this state info
  pthread_mutex_init(&info->statemodel_mutex, NULL);
  pthread_mutex_init(&info->stateinfo_mutex, NULL);

  pthread_mutex_lock(&info->stateinfo_mutex);

  // model data (transition, reward, known)

  pthread_mutex_lock(&info->statemodel_mutex);
  info->model = new StateActionInfo[numactions];
  pthread_mutex_unlock(&info->statemodel_mutex);

  info->id = id;
  if (PLANNERDEBUG) cout << " id = " << info->id << endl;

  // model q values, visit counts
  info->Q.resize(numactions, 0);
  info->uctActions.resize(numactions, 1);
  info->uctVisits = 1;
  info->visited = 0; //false;


  info->needsUpdate = true;

  pthread_mutex_unlock(&info->stateinfo_mutex);

  //if (PLANNERDEBUG) cout << "done with initStateInfo()" << endl;

}


/** Print state info for debugging. */
void ParallelETUCTGivenGoal::printStates(){

  pthread_mutex_lock(&statespace_mutex);
  for (std::set< std::vector<float> >::iterator i = statespace.begin();
       i != statespace.end(); i++){
    pthread_mutex_unlock(&statespace_mutex);

    state_t s = canonicalize(*i);

    pthread_mutex_lock(&statespace_mutex);
    state_info* info = &(statedata[s]);
    pthread_mutex_unlock(&statespace_mutex);

    cout << "State " << info->id << ": ";
    for (unsigned j = 0; j < s->size(); j++){
      cout << (*s)[j] << ", ";
    }
    cout << endl;

    pthread_mutex_lock(&info->stateinfo_mutex);
    //pthread_mutex_lock(&info->statemodel_mutex);
    for (int act = 0; act < numactions; act++){
      cout << " Q: " << info->Q[act] << endl;
      // << " R: " << info->modelInfo[act].reward << endl;
    }
    // pthread_mutex_unlock(&info->statemodel_mutex);
    pthread_mutex_unlock(&info->stateinfo_mutex);

    pthread_mutex_lock(&statespace_mutex);

  }
  pthread_mutex_unlock(&statespace_mutex);

}


void ParallelETUCTGivenGoal::deleteInfo(state_info* info){

  delete [] info->model;

}



double ParallelETUCTGivenGoal::getSeconds(){
  struct timezone tz;
  timeval timeT;
  gettimeofday(&timeT, &tz);
  return  timeT.tv_sec + (timeT.tv_usec / 1000000.0);
}


/** Execute the uct search from state state at depth depth.
    If terminal or at depth, return some reward.
    Otherwise, select an action based on UCB.
    Simulate action to get reward and next state.
    Call search on next state at depth+1 to get reward return from there on.
    Update q value towards new value: reward + gamma * searchReturn
    Update visit counts for confidence bounds
    Return q

    From "Bandit Based Monte Carlo Planning" by Kocsis and Csaba.
*/
float ParallelETUCTGivenGoal::uctSearch(const std::vector<float> &actS, state_t discS, int depth){
  if (UCTDEBUG){
    cout << " uctSearch state ";
    for (unsigned i = 0; i < actS.size(); i++){
      cout << actS[i] << ", ";
    }
    cout << " at depth " << depth << endl;
  }

  pthread_mutex_lock(&statespace_mutex);
  state_info* info = &(statedata[discS]);
  pthread_mutex_unlock(&statespace_mutex);

  // if max depth
  // iterative deepening (probability inversely proportional to visits)
  //float terminateProb = 1.0/(2.0+(float)info->uctVisits);

  // already visited, stop here
  if (depth > MAX_DEPTH){
    pthread_mutex_lock(&info->stateinfo_mutex);

    // return max q value here
    std::vector<float>::iterator maxAct =
      std::max_element(info->Q.begin(),
                       info->Q.end());
    float maxval = *maxAct;

    if (UCTDEBUG)
      cout << "Terminated after depth: " << depth
        //   << " prob: " << terminateProb
           << " Q: " << maxval
           << " visited: " << info->visited << endl;

    pthread_mutex_unlock(&info->stateinfo_mutex);

    return maxval;
  }

  // select action
  int action = selectUCTAction(info);

  // simulate action to get next state and reward
  // depending on exploration, may also terminate us
  float reward = 0;
  bool term = false;

  pthread_mutex_lock(&info->stateinfo_mutex);

  float learnRate;
  //float learnRate = 0.001;
  //float learnRate = 1.0 / info->uctActions[action];
  //    learnRate = 10.0 / (info->uctActions[action] + 100.0);
  learnRate = 10.0 / (info->uctActions[action] + 10.0);
  //if (learnRate < 0.001 && MAX_TIME < 0.5)
  //learnRate = 0.001;
  //learnRate = 0.05;
  //learnRate = 1.0;

  // tell model learning thread to update this state since we've visited it
  info->needsUpdate = true;

  pthread_mutex_unlock(&info->stateinfo_mutex);

  std::vector<float> actualNext = simulateNextState(actS, discS, info, action, &reward, &term);

  // simulate reward from this action
  if (term){
    // this one terminated
    if (UCTDEBUG) cout << "   Terminated on exploration condition" << endl;
    pthread_mutex_lock(&info->stateinfo_mutex);

    info->Q[action] += learnRate * (reward - info->Q[action]);
    info->uctVisits++;
    info->uctActions[action]++;

    if (UCTDEBUG)
      cout << " Depth: " << depth << " Selected action " << action
           << " r: " << reward
           << " StateVisits: " << info->uctVisits
           << " ActionVisits: " << info->uctActions[action] << endl;

    pthread_mutex_unlock(&info->stateinfo_mutex);

    return reward;
  }

  // simulate next state from this action
  state_t discNext = canonicalize(actualNext);

  if (UCTDEBUG)
    cout << " Depth: " << depth << " Selected action " << action
         << " r: " << reward  << endl;

  pthread_mutex_lock(&info->stateinfo_mutex);
  info->visited++; // = true;
  pthread_mutex_unlock(&info->stateinfo_mutex);

  // new q value
  float newQ = reward + gamma * uctSearch(actualNext, discNext, depth+1);

  pthread_mutex_lock(&info->stateinfo_mutex);

  if (info->visited == 1){

    // update q and visit counts
    info->Q[action] += learnRate * (newQ - info->Q[action]);
    info->uctVisits++;
    info->uctActions[action]++;

    if (UCTDEBUG)
      cout << " Depth: " << depth << " newQ: " << newQ
           << " StateVisits: " << info->uctVisits
           << " ActionVisits: " << info->uctActions[action] << endl;

    if (lambda < 1.0){

      // new idea, return max of Q or new q
      std::vector<float>::iterator maxAct =
        std::max_element(info->Q.begin(),
                         info->Q.end());
      float maxval = *maxAct;

      if (UCTDEBUG)
        cout << " Replacing newQ: " << newQ;

      // replace with w avg of maxq and new val
      newQ = (lambda * newQ) + ((1.0-lambda) * maxval);

      if (UCTDEBUG)
        cout << " with wAvg: " << newQ << endl;
    }

  }

  info->visited--;
  pthread_mutex_unlock(&info->stateinfo_mutex);

  // return q
  return newQ;

}


int ParallelETUCTGivenGoal::selectUCTAction(state_info* info){
  //  if (UCTDEBUG) cout << "  selectUCTAction" << endl;

  pthread_mutex_lock(&info->stateinfo_mutex);

  std::vector<float> &Q = info->Q;

  if (info->uctActions.size() < (unsigned)numactions){
    cout << "ERROR: uctActions has size " << info->uctActions.size() << endl << flush;
    info->uctActions.resize(numactions);
  }

  // loop through
  float rewardBound = rrange;
  if (rewardBound < 1.0)
    rewardBound = 1.0;
  rewardBound /= (1.0 - gamma);
  if (UCTDEBUG) cout << "Reward bound: " << rewardBound << endl;

  std::vector<float> uctQ(numactions, 0.0);

  pthread_mutex_lock(&usable_actions_mutex);
  for (int i = 0; i < numUsableActions; i++){

    // this actions value is Q + rMax * 2 sqrt (log N(s) / N(s,a))
    uctQ[i] = Q[i] +
      rewardBound * 2.0 * sqrt(log((float)info->uctVisits) /
                               (float)info->uctActions[i]);

    if (UCTDEBUG)
      cout << "  Action: " << i << " Q: " << Q[i]
           << " visits: " << info->uctActions[i]
           << " value: " << uctQ[i] << endl;
  }

  // max element of uctQ, only up to numUsableActions...
  std::vector<float>::iterator maxAct =
    random_max_element(uctQ.begin(), uctQ.begin() + numUsableActions);
  float maxval = *maxAct;
  int act = maxAct - uctQ.begin();
  pthread_mutex_unlock(&usable_actions_mutex);

  if (UCTDEBUG)
    cout << "  Selected " << act << " val: " << maxval << endl;

  pthread_mutex_unlock(&info->stateinfo_mutex);

  return act;

}

/** sample from next state distribution */
std::vector<float> ParallelETUCTGivenGoal::simulateNextState(const std::vector<float> &actualState, state_t discState, state_info* info, int action, float* reward, bool* term){
  //if (UCTDEBUG) cout << "  simulateNextState" << endl;


  // check if its up to date
  pthread_mutex_lock(&info->statemodel_mutex);
  StateActionInfo* modelInfo = NULL;
  modelInfo = &(info->model[action]);

  pthread_mutex_lock(&update_mutex);
  bool upToDate = modelInfo->frameUpdated >= lastUpdate;
  pthread_mutex_unlock(&update_mutex);

  if (!upToDate){
    updateStateActionFromModel(discState, action, info);
  }

  //*reward = modelInfo->reward;

  // add intrinsic reward
  float intrinsicR = 0;
  for (unsigned i = 0; i < actualState.size(); i++){
    intrinsicR += intrinsicRewards[i] * actualState[i];
  }
  // constant
  intrinsicR += intrinsicRewards[actualState.size()];

  *reward = modelInfo->reward + intrinsicR;

  float randProb = rng.uniform();

  float probSum = 0.0;
  std::vector<float> nextstate;

  if (REALSTATEDEBUG) cout << "randProb: " << randProb << " numNext: " << modelInfo->transitionProbs.size() << endl;

  //if (modelInfo->transitionProbs.size() == 0)
  nextstate = actualState;

  for (std::map<std::vector<float>, float>::iterator outIt
         = modelInfo->transitionProbs.begin();
       outIt != modelInfo->transitionProbs.end(); outIt++){

    float prob = (*outIt).second;
    probSum += prob;
    if (REALSTATEDEBUG) cout << randProb << ", " << probSum << ", " << prob << endl;

    if (randProb <= probSum){
      nextstate = (*outIt).first;
      if (REALSTATEDEBUG) cout << "selected state " << randProb << ", " << probSum << ", " << prob << endl;
      break;
    }
  }

  pthread_mutex_unlock(&info->statemodel_mutex);

  // add goal reward
  bool goalMatched = true;
  // also add up diff in relevant features
  float featDiff = 0;
  pthread_mutex_lock(&goal_mutex);
  for (unsigned i = 0; i < goalState.size(); i++){
    if (goalMask[i] && goalState[i] != nextstate[i]){
      featDiff += fabs(goalState[i] - nextstate[i]);
      //if (GOALDEBUG)
      //cout << "No match on feature " << i << " goal: " << goalState[i] << " actual: " << nextstate[i] << endl;
      goalMatched = false;
    }
  }
  pthread_mutex_unlock(&goal_mutex);

  *term = false;

  // at goal: +10 reward
  if (goalMatched){
    *reward += 10.0;
  }
  // not at goal: -1 reward
  else {
    *reward -= 1.0;
  }

  // add small bias on feature difference
  *reward -= 0.05 * featDiff;



  if (trackActual){

    // find the relative change from discrete center
    std::vector<float> relChange = subVec(nextstate, *discState);

    // add that on to actual current state value
    nextstate = addVec(actualState, relChange);

  }

  // check that next state is valid
  for (unsigned j = 0; j < nextstate.size(); j++){
    float factor = EPSILON;
    if (statesPerDim[j] > 0)
      factor = (featmax[j] - featmin[j]) / (float)statesPerDim[j];
    if (nextstate[j] < (featmin[j]-factor)
        || nextstate[j] > (featmax[j]+factor)){
      return actualState;
    }
  }

  // return new actual state
  return nextstate;

}

// has to have had real visits
std::vector<float> ParallelETUCTGivenGoal::selectRandomState(){

  pthread_mutex_lock(&statespace_mutex);
  if (statespace.size() == 0){
    pthread_mutex_unlock(&statespace_mutex);
    return std::vector<float>(featmax.size());
  }
  pthread_mutex_unlock(&statespace_mutex);

  // take a random state from the space of ones we've visited
  int index = 0;
  state_t s = NULL;
  std::vector<float> state;

  // try 5 times to get one with real visits

  pthread_mutex_lock(&statespace_mutex);
  if (statespace.size() > 1){
    index = rng.uniformDiscrete(0, statespace.size()-1);
  }
  pthread_mutex_unlock(&statespace_mutex);

  int cnt = 0;

  if (PTHREADDEBUG) cout << "*** Planning thread wants search lock (randomstate) ***" << endl << flush;

  pthread_mutex_lock(&statespace_mutex);
  for (std::set<std::vector<float> >::iterator i = statespace.begin();
       i != statespace.end(); i++, cnt++){
    if (cnt == index){
      state = *i;
      break;
    }
  }
  pthread_mutex_unlock(&statespace_mutex);

  s = canonicalize(state);

  pthread_mutex_lock(&statespace_mutex);
  state_info* info = &(statedata[s]);
  pthread_mutex_unlock(&statespace_mutex);

  return state;
}


void* parallelGGSearchStart(void* arg){
  ParallelETUCTGivenGoal* pe = reinterpret_cast<ParallelETUCTGivenGoal*>(arg);

  cout << "start parallel uct planning search thread" << endl << flush;

  while(true){
    pe->parallelSearch();
  }

  return NULL;
}

void ParallelETUCTGivenGoal::parallelSearch(){

  std::vector<float> actS;
  state_t discS;

  // get new planning state
  if (PTHREADDEBUG) {
    cout << "*** Planning thread wants planning state lock ***" << endl << flush;
  }
  pthread_mutex_lock(&(plan_state_mutex));

  // too long on one state, lets do random
  if(!doRandom && (getSeconds()-setTime) > 0.5){
    //cout << (getSeconds()-setTime) << " seconds since plan time." << endl;
    doRandom = true;
  }

  // possibly take random state (bet episodes)
  if (doRandom){
    actS = selectRandomState();
    discS = canonicalize(actS);
    //    cout << "selected random state for search" << endl << flush;
  }
  // or take the state we're in (during episodes)
  else {
    actS = actualPlanState;
    discS = discPlanState;
  }

  // wait for non-null
  if (discS == NULL){
    pthread_mutex_unlock(&(plan_state_mutex));
    return;
  }

  if (PTHREADDEBUG){
    pthread_mutex_lock(&statespace_mutex);
    cout << "  uct search from state s ("
         << statedata[discS].uctVisits <<"): ";
    pthread_mutex_unlock(&statespace_mutex);

    for (unsigned i = 0; i < discS->size(); i++){
      cout << (*discS)[i] << ", ";
    }
    cout << endl << flush;
  }

  // call uct search on it
  pthread_mutex_unlock(&(plan_state_mutex));

  if (PTHREADDEBUG) cout << "*** Planning thread wants search lock ***" << endl;
  pthread_mutex_lock(&rollout_mutex);
  currentRollout = true;
  pthread_mutex_unlock(&rollout_mutex);


  uctSearch(actS, discS, 0);

  // signal that rollout is complete
  pthread_mutex_lock(&rollout_mutex);
  currentRollout = false;
  pthread_cond_signal(&rollout_cond);
  pthread_mutex_unlock(&rollout_mutex);

  pthread_yield();

}


// canonicalize all the states so we already have them in our statespace
void ParallelETUCTGivenGoal::initStates(){
  cout << "init states" << endl;
  std::vector<float> s(featmin.size());

  fillInState(s,0);
}

void ParallelETUCTGivenGoal::fillInState(std::vector<float>s, int depth){

  // if depth == size, canonicalize and return
  if (depth == (int)featmin.size()){
    canonicalize(s);
    return;
  }

  // go through all features at depth
  for (float i = featmin[depth]; i < featmax[depth]+1; i++){
    s[depth] = i;
    fillInState(s, depth+1);
  }
}



void ParallelETUCTGivenGoal::savePolicy(const char* filename){

  ofstream policyFile(filename, ios::out | ios::binary | ios::trunc);

  // first part, save the vector size
  int fsize = featmin.size();
  policyFile.write((char*)&fsize, sizeof(int));

  // save numactions
  policyFile.write((char*)&numactions, sizeof(int));

  // go through all states, and save Q values
  pthread_mutex_lock(&statespace_mutex);

  for (std::set< std::vector<float> >::iterator i = statespace.begin();
       i != statespace.end(); i++){
    pthread_mutex_unlock(&statespace_mutex);

    state_t s = canonicalize(*i);

    pthread_mutex_lock(&statespace_mutex);
    state_info* info = &(statedata[s]);
    pthread_mutex_unlock(&statespace_mutex);

    // save state
    policyFile.write((char*)&((*i)[0]), sizeof(float)*fsize);

    // save q-values
    pthread_mutex_lock(&info->stateinfo_mutex);
    policyFile.write((char*)&(info->Q[0]), sizeof(float)*numactions);
    pthread_mutex_unlock(&info->stateinfo_mutex);

    pthread_mutex_lock(&statespace_mutex);
  }
  pthread_mutex_unlock(&statespace_mutex);

  policyFile.close();
}



void ParallelETUCTGivenGoal::loadPolicy(const char* filename){

  ifstream policyFile(filename, ios::in | ios::binary);

  // first part, save the vector size
  int fsize;
  policyFile.read((char*)&fsize, sizeof(int));
  cout << "Numfeats loaded: " << fsize << endl << flush;

  // save numactions
  int nact;
  policyFile.read((char*)&nact, sizeof(int));
  cout << "nact loaded: " << nact << endl << flush;
  cout << " numactions: " << numactions << endl << flush;

  if (nact != numactions){
    cout << "this policy is not valid loaded nact: " << nact
         << " was told: " << numactions << endl << flush;
    exit(-1);
  }

  // go through all states, loading q values
  while(!policyFile.eof()){
    std::vector<float> state(fsize, 0.0);

    // load state
    policyFile.read((char*)&(state[0]), sizeof(float)*fsize);
    //if (LOADDEBUG){
    //cout << "load policy for state: ";
    // printState(state);
    //}

    state_t s = canonicalize(state);

    pthread_mutex_lock(&statespace_mutex);
    state_info* info = &(statedata[s]);
    pthread_mutex_unlock(&statespace_mutex);

    if (policyFile.eof()) break;

    // load q values
    pthread_mutex_lock(&info->stateinfo_mutex);

    policyFile.read((char*)&(info->Q[0]), sizeof(float)*numactions);

    info->uctVisits = numactions * 100;

    for (int j = 0; j < numactions; j++){
      info->uctActions[j] = 100;
    }

    info->needsUpdate = true;

    pthread_mutex_unlock(&info->stateinfo_mutex);

    //if (LOADDEBUG){
    //cout << "Q values: " << endl;
    //for (int iAct = 0; iAct < numactions; iAct++){
    //  cout << " Action: " << iAct << " val: " << info->Q[iAct] << endl;
    //}
    //}
  }

  policyFile.close();
  cout << "Policy loaded!!!" << endl << flush;
}

void ParallelETUCTGivenGoal::logValues(ofstream *of, int xmin, int xmax, int ymin, int ymax){
  std::vector<float> state(2, 0.0);
  for (int i = xmin ; i < xmax; i++){
    for (int j = ymin; j < ymax; j++){
      state[0] = j;
      state[1] = i;
      state_t s = canonicalize(state);

      pthread_mutex_lock(&statespace_mutex);
      state_info* info = &(statedata[s]);
      pthread_mutex_unlock(&statespace_mutex);

      pthread_mutex_lock(&info->stateinfo_mutex);

      std::vector<float> &Q_s = info->Q;
      const std::vector<float>::iterator max =
        random_max_element(Q_s.begin(), Q_s.end());
      *of << (*max) << ",";

      pthread_mutex_unlock(&info->stateinfo_mutex);

    }
  }
}


// should do it such that an already discretized state stays the same
// mainly the numerical value of each bin should be the average of that bin
std::vector<float> ParallelETUCTGivenGoal::discretizeState(const std::vector<float> &s){
  std::vector<float> ds(s.size());

  for (unsigned i = 0; i < s.size(); i++){

    // since i'm sometimes doing this for discrete domains
    // want to center bins on 0, not edge on 0
    //cout << "feat " << i << " range: " << featmax[i] << " " << featmin[i] << " " << (featmax[i]-featmin[i]) << " n: " << (float)statesPerDim;

    float factor = (featmax[i] - featmin[i]) / (float)statesPerDim[i];
    int bin = 0;
    if (s[i] > 0){
      bin = (int)((s[i]+factor/2) / factor);
    } else {
      bin = (int)((s[i]-factor/2) / factor);
    }

    ds[i] = factor*bin;
    //cout << " factor: " << factor << " bin: " << bin;
    //cout << " Original: " << s[i] << " Discrete: " << ds[i] << endl;
  }

  return ds;
}


std::vector<float> ParallelETUCTGivenGoal::addVec(const std::vector<float> &a, const std::vector<float> &b){
  unsigned fsize = a.size();
  if (a.size() != b.size()){
    //cout << "ERROR: add vector sizes wrong " << a.size() << ", " << b.size() << endl;
    if (b.size() < a.size())
      fsize = b.size();
  }

  std::vector<float> c(fsize, 0.0);
  for (unsigned i = 0; i < fsize; i++){
    c[i] = a[i] + b[i];
  }

  return c;
}

std::vector<float> ParallelETUCTGivenGoal::subVec(const std::vector<float> &a, const std::vector<float> &b){
  unsigned fsize = a.size();
  if (a.size() != b.size()){
    //cout << "ERROR: sub vector sizes wrong " << a.size() << ", " << b.size() << endl;
    if (b.size() < a.size())
      fsize = b.size();
  }

  std::vector<float> c(fsize, 0.0);
  for (unsigned i = 0; i < fsize; i++){
    c[i] = a[i] - b[i];
  }

  return c;
}

void ParallelETUCTGivenGoal::setFirst(){
  return;
}

void ParallelETUCTGivenGoal::setSeeding(bool seeding){

  seedMode = seeding;

}

// fill intrinsicRewards vector with these values
void ParallelETUCTGivenGoal::setIntrinsicRewards(std::vector<float> im){
  pthread_mutex_lock(&plan_state_mutex);
  pthread_mutex_lock(&statespace_mutex);
  for (unsigned i = 0; i < im.size() && i < intrinsicRewards.size(); i++){
    intrinsicRewards[i] = im[i];
  }
  pthread_mutex_unlock(&plan_state_mutex);
  pthread_mutex_unlock(&statespace_mutex);
}

void ParallelETUCTGivenGoal::resetValues(){
  pthread_mutex_lock(&statespace_mutex);

  for (std::set< std::vector<float> >::iterator i = statespace.begin();
       i != statespace.end(); i++){
    pthread_mutex_unlock(&statespace_mutex);

    state_t s = canonicalize(*i);

    pthread_mutex_lock(&statespace_mutex);
    state_info* info = &(statedata[s]);
    pthread_mutex_unlock(&statespace_mutex);

    pthread_mutex_lock(&info->stateinfo_mutex);
    pthread_mutex_lock(&info->statemodel_mutex);


    // reset info
    for (int j = 0; j < numactions; j++){
      info->Q[j] = 0;
      info->uctActions[j] = 0;
      info->model[j] = StateActionInfo();

    }
    info->uctVisits = 0;
    info->visited = false;
    info->needsUpdate = true;

    pthread_mutex_unlock(&info->statemodel_mutex);
    pthread_mutex_unlock(&info->stateinfo_mutex);

    pthread_mutex_lock(&statespace_mutex);

  }
  pthread_mutex_lock(&plan_state_mutex);
  pthread_mutex_lock(&model_mutex);
  pthread_mutex_lock(&list_mutex);

  //statespace.clear();
  //statedata.clear();
  expList.clear();
  pthread_mutex_unlock(&plan_state_mutex);
  pthread_mutex_unlock(&statespace_mutex);
  pthread_mutex_unlock(&model_mutex);
  pthread_mutex_unlock(&list_mutex);
}




void ParallelETUCTGivenGoal::setGoal(std::vector<float> goal, std::vector<bool> mask){

  // lock to make sure uct rollouts aren't happenign during this reset
  pthread_mutex_lock(&rollout_mutex);
  while (currentRollout){
    pthread_cond_wait(&rollout_cond,&rollout_mutex);
  }

  // lock around goal and goal mask
  pthread_mutex_lock(&goal_mutex);
  goalState = goal;
  goalMask = mask;

  if (GOALDEBUG){
    cout << "Set goal: " << endl;
    for (unsigned i = 0; i < goalState.size(); i++){
      if (goalMask[i])
        cout << "  Feat " << i << ": Goal value: " << goalState[i] << endl;
    }
  }
  pthread_mutex_unlock(&goal_mutex);

  // reset q values and uct visit counts for new plan
  pthread_mutex_lock(&statespace_mutex);
  for (std::set<std::vector<float> >::iterator i = statespace.begin();
       i != statespace.end(); i++){
    pthread_mutex_unlock(&statespace_mutex);
    state_t s = canonicalize(*i);
    pthread_mutex_lock(&statespace_mutex);
    state_info* info = &(statedata[s]);
    pthread_mutex_unlock(&statespace_mutex);

    pthread_mutex_lock(&info->stateinfo_mutex);

    info->uctVisits = 1;
    info->visited = false;

    for (int j = 0; j < numactions; j++){
      info->Q[j] = 0;
      info->uctActions[j] = 1;
    }

    pthread_mutex_unlock(&info->stateinfo_mutex);
    pthread_mutex_lock(&statespace_mutex);

  }
  pthread_mutex_unlock(&statespace_mutex);


  pthread_mutex_unlock(&rollout_mutex);

}


// set the number of usable actions
void ParallelETUCTGivenGoal::setUsableActions(int n){
  // lock around usable actions
  cout << "Set usable actions as " << n << endl;
  pthread_mutex_lock(&usable_actions_mutex);
  numUsableActions = n;
  pthread_mutex_unlock(&usable_actions_mutex);
}

// set whether we're in eval mode or not
void ParallelETUCTGivenGoal::setEvaluationMode(bool eval){
  // lock around eval mode
  pthread_mutex_lock(&eval_mode_mutex);
  evaluationMode = eval;
  pthread_mutex_unlock(&eval_mode_mutex);
}
