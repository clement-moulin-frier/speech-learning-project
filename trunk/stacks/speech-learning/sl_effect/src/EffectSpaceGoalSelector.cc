/** \file EffectSpaceGoalSelector.cc
    Implements the EffectSpaceGoalSelector class
    \author Todd Hester
*/

#include "EffectSpaceGoalSelector.hh"
#include <algorithm>

#include <sys/time.h>

EffectSpaceGoalSelector::EffectSpaceGoalSelector(int numobjects, int width, const std::vector<float> &featmin, 
                                                 const std::vector<float> &featmax, int goalSelect,
                                                 int learnerSelect, int tau, int theta, bool debug,
                                                 Random rng):
  numobjects(numobjects), width(width), featmin(featmin), featmax(featmax),
  goalSelect(goalSelect), learnerSelect(learnerSelect), 
  tau(tau), theta(theta), rng(rng)
{

  GOALDEBUG = debug;
  GOALSELECTDEBUG = true;
  evalMode = false;
  lastEvalGoal = -1;

  initGoals();
  initOutputFiles();
}

EffectSpaceGoalSelector::~EffectSpaceGoalSelector() {
  goals.clear();
  featmin.clear();
  featmax.clear();
}


void EffectSpaceGoalSelector::initGoals(){
  nGoalsTried = 0;
  goal newGoal;
  newGoal.goalMsg.start_state.resize(featmax.size(), 0);
  newGoal.goalMsg.start_mask.resize(featmax.size(), false);
  newGoal.goalMsg.goal_state.resize(featmax.size(), 0);
  newGoal.goalMsg.goal_mask.resize(featmax.size(), false);

  // Goal Type 1: Move hand to given x,y location
  for (int i = 0; i < width; i++){
    for (int j = 0; j < width*2; j++){
      newGoal.goalMsg.goal_mask[0] = true;
      newGoal.goalMsg.goal_mask[1] = true;
      newGoal.goalMsg.goal_state[0] = i;
      newGoal.goalMsg.goal_state[1] = j;
      newGoal.goalName.clear();
      ostringstream os(newGoal.goalName);
      os << "Move hand to " << i << ", " << j;
      newGoal.goalName = os.str();

      if (GOALDEBUG){
        cout << "Adding goal " << goals.size() << ": ";
        printGoal(&newGoal);
      }
      goals.push_back(newGoal);
    }
  }

  // Goal Type 2: Make object i reachable/unreachable
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMsg.goal_mask.assign(featmax.size(), false);
    newGoal.goalMsg.start_mask.assign(featmax.size(), false);
    newGoal.goalMsg.goal_mask[3+6*i+4] = true;
    newGoal.goalMsg.start_mask[3+6*i+4] = true;
    
    // reachable or not
    for (int j = 0; j < 2; j++){

      newGoal.goalMsg.goal_state[3+6*i+4] = 1 - j;
      newGoal.goalMsg.start_state[3+6*i+4] = j;
      newGoal.goalName.clear();
      ostringstream os(newGoal.goalName);
      os << "Make object " << i;
      if (newGoal.goalMsg.goal_state[3+6*i+4]) os << " reachable.";
      else os << " UNreachable.";
      newGoal.goalName = os.str();

      if (GOALDEBUG){
        cout << "Adding goal " << goals.size() << ": ";
        printGoal(&newGoal);
      }
      goals.push_back(newGoal);
    }
  }

  // Goal Type 3: Put hand on object
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMsg.goal_mask.assign(featmax.size(), false);
    newGoal.goalMsg.start_mask.assign(featmax.size(), false);
    // care about whether we start from obj already reachable or not
    newGoal.goalMsg.start_mask[3+6*i+4] = true;
    newGoal.goalMsg.goal_state[3+6*i+2] = 0;
    newGoal.goalMsg.goal_state[3+6*i+3] = 0;
    newGoal.goalMsg.goal_mask[3+6*i+2] = true;
    newGoal.goalMsg.goal_mask[3+6*i+3] = true;

    // start reachable or not
    for (int j = 0; j < 2; j++){
      newGoal.goalMsg.start_state[3+6*i+4] = j;
      newGoal.goalName.clear();
      ostringstream os(newGoal.goalName);
      os << "Put hand on ";
      if (newGoal.goalMsg.start_state[3+6*i+4])
        os << "reachable object " << i;
      else
        os << "unreachable object " << i;
      newGoal.goalName = os.str();

      if (GOALDEBUG){
        cout << "Adding goal " << goals.size() << ": ";
        printGoal(&newGoal);
      }
      goals.push_back(newGoal);
    }
  }

  // Goal Type 4: Put object in/out of hand hand
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMsg.goal_mask.assign(featmax.size(), false);
    newGoal.goalMsg.start_mask.assign(featmax.size(), false);
    // care about whether we start from obj already reachable or not
    newGoal.goalMsg.start_mask[3+6*i+4] = true;
    newGoal.goalMsg.goal_mask[3+6*i+5] = true;
    newGoal.goalMsg.start_mask[3+6*i+5] = true;

    // start reachable or not
    for (int j = 0; j < 2; j++){
      newGoal.goalMsg.start_state[3+6*i+4] = j;

      // and goal is in hand or not
      for (int k = 0; k < 2; k++){
        newGoal.goalMsg.goal_state[3+6*i+5] = 1 - k;
        newGoal.goalMsg.start_state[3+6*i+5] = k;
        newGoal.goalName.clear();
        ostringstream os(newGoal.goalName);
        os << "Put ";
        if (newGoal.goalMsg.start_state[3+6*i+4])
          os << "reachable object " << i;
        else
          os << "unreachable object " << i;
        if (newGoal.goalMsg.goal_state[3+6*i+5])
          os << " in hand.";
        else
          os << " down.";
        newGoal.goalName = os.str();

        if (GOALDEBUG){
          cout << "Adding goal " << goals.size() << ": ";
          printGoal(&newGoal);
        }
        goals.push_back(newGoal);
      }
    }
  }
  
  // Goal Type 5: Get object i to location x,y
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMsg.goal_mask.assign(featmax.size(), false);
    newGoal.goalMsg.start_mask.assign(featmax.size(), false);
    // care about starting reachable or not
    newGoal.goalMsg.start_mask[3+6*i+4] = true;
    // and care about final destination 
    newGoal.goalMsg.goal_mask[3+6*i+0] = true;
    newGoal.goalMsg.goal_mask[3+6*i+1] = true;    

    for (int x = 0; x < width; x++){
      for (int y = 0; y < width*2; y++){
        newGoal.goalMsg.goal_state[3+6*i+0] = x;
        newGoal.goalMsg.goal_state[3+6*i+1] = y;

        // start from reachable/unreachable
        for (int j = 0; j < 2; j++){
          newGoal.goalMsg.start_state[3+6*i+4] = j;
          newGoal.goalName.clear();
          ostringstream os(newGoal.goalName);
          os << "Put ";
          if (newGoal.goalMsg.start_state[3+6*i+4])
            os << "reachable object " << i;
          else
            os << "unreachable object " << i;
          os << " at loc: " << x << ", " << y;
          newGoal.goalName = os.str();

          if (GOALDEBUG){
            cout << "Adding goal " << goals.size() << ": ";
            printGoal(&newGoal);
          }
          goals.push_back(newGoal);
        }
      }
    }
  }
}


EffectSpaceGoalSelector::goal EffectSpaceGoalSelector::selectGoal(std::vector<float> currentState, int* learner){

  cout << "nGoals: " << nGoalsTried << endl;
  if (nGoalsTried % 10 == 0 && lastEvalGoal < nGoalsTried){
    int goalIndex = evaluateGoals();
    *learner = TexploreLearner;
    currentGoalIndex = goalIndex;
    startState = currentState;
    whichLearner = *learner;
    goals[goalIndex].goalMsg.evaluateOnly = true;
    if (GOALDEBUG){
      cout << "Selected goal " << goalIndex << ": ";
      printGoal(&(goals[goalIndex]));
    }
    return goals[goalIndex];
  }

  if (GOALDEBUG){
    cout << endl << "Select goal from state: ";
    for (unsigned i = 0; i < currentState.size(); i++){
      cout << currentState[i] << ", ";
    }
    cout << endl;
  }

  int goalIndex = 0;
  if (goalSelect == RandomGoal) {
    goalIndex = selectRandomGoal(currentState);
  } else if (goalSelect == SaggRiac) {
    goalIndex = selectCompetenceProgressGoal(currentState);
  } else {
    cout << "ERROR: invalid goal select type: " << goalSelect << endl;
    exit(-1);
  }

  if (GOALDEBUG){
    cout << "Selected goal " << goalIndex << ": ";
    printGoal(&(goals[goalIndex]));
  }

  // now choose learner
  *learner = chooseLearner(goalIndex);

  // save some stuff so we know what we were doing at the end
  currentGoalIndex = goalIndex;
  startState = currentState;
  whichLearner = *learner;
  goals[goalIndex].goalMsg.evaluateOnly = false;

  return goals[goalIndex];
}


int EffectSpaceGoalSelector::selectRandomGoal(std::vector<float> currentState){
  // select a goal at random
  int goalIndex = -1;
  bool goalValid = false;
  goal* selectedGoal;

  // randomly select goals until we find one that is valid from the current state
  while (!goalValid){
    goalIndex = rng.uniformDiscrete(0, goals.size()-1);
    selectedGoal = &(goals[goalIndex]);

    if (GOALDEBUG){
      cout << " Randomly selected goal " << goalIndex << ": ";
      printGoal(selectedGoal);
    }

    goalValid = isGoalValid(currentState, goalIndex);
  }

  return goalIndex;
}
      



int EffectSpaceGoalSelector::selectCompetenceProgressGoal(std::vector<float> currentState){

  int bestGoal = -1;
  float bestProgress = -1;
  int nTied = 0;

  // go through all goals. find one with highest progress
  for (unsigned i = 0; i < goals.size(); i++){
    // check if its valid
    if (GOALDEBUG){
      cout << "Goal " << i << ": ";
      printGoal(&goals[i]);
    }

    if (!isGoalValid(currentState, i)) continue;

    if (GOALSELECTDEBUG && !GOALDEBUG){
      cout << "Goal " << i << ": ";
      printGoal(&goals[i]);
    }

    // calc competence progress for this goal
    float progress = calcCompetenceProgress(goals[i].allRewards);
    if (GOALDEBUG || GOALSELECTDEBUG) 
      cout << "  Goal " << i << " progress: " << progress << endl;

    if (progress > bestProgress){
      if (GOALDEBUG) cout << "  NEW BEST Goal " << i << " progress: " << progress << endl;
      bestProgress = progress;
      bestGoal = i;
      nTied = 1;
    } else if (progress == bestProgress){
      // randomly select from those with equally best progress
      nTied++;
      if (rng.bernoulli(1.0/(float)nTied))
        bestGoal = i;
      if (GOALDEBUG) cout << "  TIED " << nTied << " Goal " << i << " progress: " << progress << " best: " << bestGoal << endl;
    }
  }

  if (GOALSELECTDEBUG){
    cout << "Selected goal " << bestGoal << " with progress " << bestProgress << " : ";
    printGoal(&(goals[bestGoal]));
    cout << endl;
  }

  return bestGoal;
}


bool EffectSpaceGoalSelector::isGoalValid(std::vector<float> currentState, int goalIndex){
  bool atGoalAlready = true;
  goal* selectedGoal = &(goals[goalIndex]);

  // current state must match start state
  // and be different from goal state
  for (unsigned i = 0; i < currentState.size(); i++){
    // check if start state matches
    if (selectedGoal->goalMsg.start_mask[i] && selectedGoal->goalMsg.start_state[i] != currentState[i]){
      if (GOALDEBUG) cout << "  Goal " << goalIndex << " not valid because feat " << i << " does not match start state" << endl;
      return false;
    }
    // check if we're already at goal
    if (selectedGoal->goalMsg.goal_mask[i] && selectedGoal->goalMsg.goal_state[i] != currentState[i]){
      // it's ok, we do differ from goal state in at least this feature
      atGoalAlready = false;
    }
  } // loop over features

  if (atGoalAlready){
    if (GOALDEBUG) cout << "  Goal " << goalIndex << " not valid because current state already matches goal" << endl;
    return false;
  }

  // its a valid goal!
  return true;
}


int EffectSpaceGoalSelector::chooseLearner(int goalIndex){
  int learner = 0;
  if (learnerSelect == RandomLearner){
    learner = rng.bernoulli(0.5);
  } else if (learnerSelect == TexploreOnly){
    learner = TexploreLearner;
  } else if (learnerSelect == SpeechOnly){
    learner = SpeechLearner;
  } else if (learnerSelect == Competence){
    learner = chooseHighCompetenceLearner(goalIndex);
  } else if (learnerSelect == CompProgress){
    learner = chooseCompetenceProgressLearner(goalIndex);
  } else {
    cout << "ERROR: invalid learner selection method: " << learnerSelect << endl;
    exit(-1);
  }

  if (GOALDEBUG) cout << "Selected Learner " << learner << endl;
  return learner;
}

int EffectSpaceGoalSelector::chooseHighCompetenceLearner(int goalIndex){
  // get average of last theta rewards for each learner
  // if less than theta attempts, be optimistic and assume 0 dist achieved
  float texploreSum = 0;
  for (int i = (int)goals[goalIndex].texploreRewards.size()-1; i >= 0 && i >= (int)goals[goalIndex].texploreRewards.size() - theta; i--){
    texploreSum += goals[goalIndex].texploreRewards[i];
  }
  texploreSum /= (float)theta;

  float speechSum = 0;
  for (int i = (int)goals[goalIndex].speechRewards.size()-1; i >= 0 && i >= (int)goals[goalIndex].speechRewards.size() - theta; i--){
    speechSum += goals[goalIndex].speechRewards[i];
  }
  speechSum /= (float)theta;

  if (GOALDEBUG) cout << "TEXPLORE avg dist: " << texploreSum << ", speech avg dist: " << speechSum << endl;
  if (texploreSum < speechSum)
    return TexploreLearner;
  else if (speechSum < texploreSum)
    return SpeechLearner;
  else
    return rng.bernoulli(0.5);
}

int EffectSpaceGoalSelector::chooseCompetenceProgressLearner(int goalIndex){
  // get current average dist, and average from tau steps ago, for each learner

  // TODO: what do we do if we don't have enough data yet? return random learner?

  float texploreChange = calcCompetenceProgress(goals[goalIndex].texploreRewards);
  float speechChange = calcCompetenceProgress(goals[goalIndex].speechRewards);

  if (GOALDEBUG){
    cout << "TEXPLORE performance change: " << texploreChange << endl;
    cout << "SPEECH performance change: " << speechChange << endl;
  }

  if (texploreChange > speechChange)
    return TexploreLearner;
  else if (speechChange > texploreChange)
    return SpeechLearner;
  else
    return rng.bernoulli(0.5);

}


float EffectSpaceGoalSelector::calcCompetenceProgress(std::deque<float> &results){
  // TODO: what should do in this case with not enough results???
  if (results.size() < (unsigned) (theta+tau)){
    if (GOALDEBUG) cout << "  Only " << results.size() << " results. returning max value " << endl;
    return 10000.0;
  }

  float now = 0;
  for (int i = (int)results.size()-1; i >= 0 && i >= (int)results.size() - theta; i--){
    now += results[i];
  }
  now /= (float)theta;

  float before = 0;
  for (int i = (int)results.size()-1-tau; i >= 0 && i >= (int)results.size() - theta - tau; i--){
    before += results[i];
  }
  before /= (float)theta;
  
  float change = fabs(now-before);

  if (GOALDEBUG)
    cout << "Performance went from " << before << " to " << now << " change: " << change << endl;

  return change;
}


void EffectSpaceGoalSelector::updateGoal(std::vector<float> finalState){
  if (GOALDEBUG) cout << "Goal " << currentGoalIndex << " attempt complete. " << endl;

  // figure out distance to goal achieved
  float dist = calculateDistance(finalState);

  if (evalMode){
    return updateEval(dist);
  }

  // update reward history for this goal
  goal* currentGoal = &(goals[currentGoalIndex]);
  currentGoal->allRewards.push_back(dist);
  if (currentGoal->allRewards.size() > (tau+theta)){
    currentGoal->allRewards.pop_front();
  }
  
  // and for individual learners
  if (whichLearner == TexploreLearner){
    currentGoal->texploreRewards.push_back(dist);
    if (currentGoal->texploreRewards.size() > (tau+theta)){
      currentGoal->texploreRewards.pop_front();
    }
  } 
  else if (whichLearner == SpeechLearner){
    currentGoal->speechRewards.push_back(dist);
    if (currentGoal->speechRewards.size() > (tau+theta)){
      currentGoal->speechRewards.pop_front();
    }
  } 
  else {
    cout << "ERROR: invalid learner type " << whichLearner << endl;
    exit(-1);
  }

  nGoalsTried++;

}
 

float EffectSpaceGoalSelector::calculateDistance(std::vector<float> finalState){
  // lets scale this so distance of 1 if we're still at start state, and dist of 0, if we're at goal
  int nRelevantFeatures = 0;
  float totalDist = 0;
  for (unsigned i = 0; i < finalState.size(); i++){
    if (goals[currentGoalIndex].goalMsg.goal_mask[i]){
      nRelevantFeatures++;
      // distance in this feature from start to goal
      float fullRange = fabs(startState[i] - goals[currentGoalIndex].goalMsg.goal_state[i]);
      if (fullRange < 0.2) fullRange = 0.2; // in case we started at goal in this feature
      // distance in this feature final state is from goal
      float finalDist = fabs(finalState[i] - goals[currentGoalIndex].goalMsg.goal_state[i]);
      if (GOALDEBUG) cout << " Feat " << i << " went from " << startState[i] << " to " << finalState[i] << ", goal: " << goals[currentGoalIndex].goalMsg.goal_state[i] << endl;
      // dist scaled to pct of range
      totalDist += (finalDist / fullRange);
    }
  }
  // normalize by number of features
  float normDist = totalDist / (float) nRelevantFeatures;

  if (GOALDEBUG) cout << "Normalized distance reached to goal " << currentGoalIndex << ": " << normDist << endl;

  return normDist;
}



void EffectSpaceGoalSelector::printGoal(goal* sg){

  cout << sg->goalName << endl;

  /*

  cout << "Goal region is: " << endl;
  for (unsigned i = 0; i < sg->goalMsg.goal_state.size(); i++){
    if (sg->goalMsg.goal_mask[i])
      cout << "  Feat " << i << ": " << sg->goalMsg.goal_state[i] << endl;
  }
  cout << " From start region: " << endl;
  for (unsigned i = 0; i < sg->goalMsg.start_state.size(); i++){
    if (sg->goalMsg.start_mask[i])
      cout << "   Feat " << i << ": " << sg->goalMsg.start_state[i] << endl;
  }

  */
}


void EffectSpaceGoalSelector::initOutputFiles(){

  std::string evalFile;
  ostringstream os(evalFile);
  os << "eval.goals." << numobjects << "." << width << "." << goalSelect << "." << learnerSelect << ".out";
  evalFile = os.str();

  evalGoal = new std::ofstream(evalFile.c_str());

}


void EffectSpaceGoalSelector::updateEval(float dist){
  evalResults[currentEvalGoal] = dist;
  
  // done
  if (currentEvalGoal == 3){
    evalMode = false;
    lastEvalGoal = nGoalsTried;
    (*evalGoal) << lastEvalGoal << "\t" << evalResults[0] << "\t" << evalResults[1] << "\t" << evalResults[2] << "\t" << evalResults[3] << endl;
  }
}

int EffectSpaceGoalSelector::evaluateGoals(){

  if (evalResults.size() != 4)
    evalResults.resize(4, 1.0);

  if (!evalMode){
    evalMode = true;
    currentEvalGoal = 0;
  } 
  else {
    currentEvalGoal++;
  }

  cout << endl << "EVALUATE " << currentEvalGoal << endl << endl;

  // lets evaluate a few arbitrary goals
  // agent going to an arbitrary state
  // goal index 2 (for 2 obj, 4 width)
  if (currentEvalGoal == 0){
    return 2;
  }

  // agent moving to object 0
  // goal index 36 (for 2 obj, 4 width)
  else if (currentEvalGoal == 1){
    return 36;
  }

  // agent moving obj 0 to arbitrary state
  // goal index 66 (for 2 obj, 4 width)
  else if (currentEvalGoal == 2){
    return 66;
  }

  // agent moving obj 1 to arbitrary state on far side
  // goal index 156 (for 2 obj, 4 width)
  else {
    return 156;
  }

}
