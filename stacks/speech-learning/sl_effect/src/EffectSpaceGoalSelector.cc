/** \file EffectSpaceGoalSelector.cc
    Implements the EffectSpaceGoalSelector class
    \author Todd Hester
*/

#include "EffectSpaceGoalSelector.hh"
#include <algorithm>

#include <sys/time.h>

EffectSpaceGoalSelector::EffectSpaceGoalSelector(int numobjects, int width, const std::vector<float> &featmin, 
                                                 const std::vector<float> &featmax, int goalSelect,
                                                 int learnerSelect, int tau, int theta,
                                                 Random rng):
  numobjects(numobjects), width(width), featmin(featmin), featmax(featmax),
  goalSelect(goalSelect), learnerSelect(learnerSelect), 
  tau(tau), theta(theta), rng(rng)
{

  GOALDEBUG = true;

  initGoals();
}

EffectSpaceGoalSelector::~EffectSpaceGoalSelector() {
  goals.clear();
  featmin.clear();
  featmax.clear();
}


void EffectSpaceGoalSelector::initGoals(){
  goal newGoal;
  newGoal.startState.resize(featmax.size(), 0);
  newGoal.startMask.resize(featmax.size(), false);
  newGoal.goalState.resize(featmax.size(), 0);
  newGoal.goalMask.resize(featmax.size(), false);

  // Goal Type 1: Move hand to given x,y location
  for (int i = 0; i < width; i++){
    for (int j = 0; j < width*2; j++){
      newGoal.goalMask[0] = true;
      newGoal.goalMask[1] = true;
      newGoal.goalState[0] = i;
      newGoal.goalState[1] = i;
      if (GOALDEBUG) cout << "Adding goal " << goals.size() << ": Move hand to " << i << ", " << j << endl;
      goals.push_back(newGoal);
    }
  }

  // Goal Type 2: Make object i reachable/unreachable
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMask.assign(featmax.size(), false);
    newGoal.startMask.assign(featmax.size(), false);
    newGoal.goalMask[3+6*i+4] = true;
    newGoal.startMask[3+6*i+4] = true;
    
    // reachable or not
    for (int j = 0; j < 2; j++){

      newGoal.goalState[3+6*i+4] = 1 - j;
      newGoal.startState[3+6*i+4] = j;
      if (GOALDEBUG) cout << "Adding goal " << goals.size() << ": Make object " << i << " reachable: " << newGoal.goalState[3+6*i+4] << endl;
      goals.push_back(newGoal);
    }
  }

  // Goal Type 3: Put hand on object
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMask.assign(featmax.size(), false);
    newGoal.startMask.assign(featmax.size(), false);
    // care about whether we start from obj already reachable or not
    newGoal.startMask[3+6*i+4] = true;
    newGoal.goalState[3+6*i+2] = 0;
    newGoal.goalState[3+6*i+3] = 0;
    newGoal.goalMask[3+6*i+2] = true;
    newGoal.goalMask[3+6*i+3] = true;

    // start reachable or not
    for (int j = 0; j < 2; j++){
      newGoal.startState[3+6*i+4] = j;
      
      if (GOALDEBUG) cout << "Adding goal " << goals.size() << ": Put hand on object " << i << " with it starting reachable: " << newGoal.startState[3+6*i+4] << endl;
      goals.push_back(newGoal);
    }
  }

  // Goal Type 4: Put object in/out of hand hand
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMask.assign(featmax.size(), false);
    newGoal.startMask.assign(featmax.size(), false);
    // care about whether we start from obj already reachable or not
    newGoal.startMask[3+6*i+4] = true;
    newGoal.goalMask[3+6*i+5] = true;
    newGoal.startMask[3+6*i+5] = true;

    // start reachable or not
    for (int j = 0; j < 2; j++){
      newGoal.startState[3+6*i+4] = j;

      // and goal is in hand or not
      for (int k = 0; k < 2; k++){
        newGoal.goalState[3+6*i+5] = 1 - k;
        newGoal.startState[3+6*i+5] = k;

        if (GOALDEBUG) cout << "Adding goal " << goals.size() << ": Put object " << i << " in hand: " << newGoal.goalState[3+6*i+5] << " from reachable: " << newGoal.startState[3+6*i+4] << endl;
        goals.push_back(newGoal);
      }
    }
  }
  
  // Goal Type 5: Get object i to location x,y
  for (int i = 0; i < numobjects; i++){
    newGoal.goalMask.assign(featmax.size(), false);
    newGoal.startMask.assign(featmax.size(), false);
    // care about starting reachable or not
    newGoal.startMask[3+6*i+4] = true;
    // and care about final destination 
    newGoal.goalMask[3+6*i+0] = true;
    newGoal.goalMask[3+6*i+1] = true;    

    for (int x = 0; x < width; x++){
      for (int y = 0; y < width*2; y++){
        newGoal.goalState[3+6*i+0] = x;
        newGoal.goalState[3+6*i+1] = y;

        // start from reachable/unreachable
        for (int j = 0; j < 2; j++){
          newGoal.startState[3+6*i+4] = j;
          
          if (GOALDEBUG) cout << "Adding goal " << goals.size() << ": Put object " << i << " at loc: " << x << ", " << y << " from reachable: " << newGoal.startState[3+6*i+4] << endl;
          goals.push_back(newGoal);
        }
      }
    }
  }
}


int EffectSpaceGoalSelector::selectGoal(std::vector<float> currentState, goal* selectedGoal){
  int goalIndex = 0;
  if (goalSelect == RandomGoal) {
    goalIndex = selectRandomGoal(currentState, selectedGoal);
  } else if (goalSelect == SaggRiac) {
    goalIndex = selectCompetenceProgressGoal(currentState, selectedGoal);
  } else {
    cout << "ERROR: invalid goal select type: " << goalSelect << endl;
    exit(-1);
  }

  if (GOALDEBUG) cout << "Selected goal "<< goalIndex << endl;

  // now choose learner
  int learner = chooseLearner(goalIndex);

  // save some stuff so we know what we were doing at the end
  currentGoalIndex = goalIndex;
  startState = currentState;
  whichLearner = learner;

  return learner;
}


int EffectSpaceGoalSelector::selectRandomGoal(std::vector<float> currentState, goal* selectedGoal){
  // select a goal at random
  int goalIndex = -1;
  bool goalValid = false;

  // randomly select goals until we find one that is valid from the current state
  while (!goalValid){
    goalIndex = rng.uniformDiscrete(0, goals.size()-1);
    selectedGoal = &(goals[goalIndex]);
    goalValid = true;
    // current state must match start state
    // and be different from goal state
    for (unsigned i = 0; i < currentState.size(); i++){
      // check if start state matches
      if (selectedGoal->startMask[i] && selectedGoal->startState[i] != currentState[i]){
        if (GOALDEBUG) cout << "Goal " << goalIndex << " not valid because feat " << i << " does not match start state" << endl;
        goalValid = false;
        break;
      }
      // check if we're already at goal
      bool goalCheck = false;
      if (selectedGoal->goalMask[i] && selectedGoal->goalState[i] != currentState[i]){
        // it's ok, we do differ from goal state in at least this feature
        goalCheck = true;
        break;
      }
      if (!goalCheck){
        if (GOALDEBUG) cout << "Goal " << goalIndex << " not valid because current state already matches goal" << endl;
        goalValid = false;
        break;
      }
      if (!goalValid) break;
    } // loop over features
  }

  return goalIndex;
}
      

int EffectSpaceGoalSelector::selectCompetenceProgressGoal(std::vector<float> currentState, goal* selectedGoal){
  // TODO
  cout << endl << "ERROR: Competence Progress SAGG-RIAC goal selection not implemented yet!" << endl << endl;
  return selectRandomGoal(currentState, selectedGoal);
}


int EffectSpaceGoalSelector::chooseLearner(int goalIndex){
  int learner = 0;
  if (learnerSelect == RandomLearner){
    learner = rng.bernoulli(0.5);
  } else if (learnerSelect == TexploreOnly){
    learner = TexploreLearner;
  } else if (learnerSelect == Competence){
    learner = chooseHighCompetenceLearner(goalIndex);
  } else {
    cout << "ERROR: invalid learner selection method: " << learnerSelect << endl;
    exit(-1);
  }

  if (GOALDEBUG) cout << "Selected Learner " << learner << endl;
  return learner;
}

int EffectSpaceGoalSelector::chooseHighCompetenceLearner(int goalIndex){
  // TODO
  cout << endl << "ERROR: Competence-based learner selection not implemented yet!" << endl << endl;
  return rng.bernoulli(0.5);
}


void EffectSpaceGoalSelector::updateGoal(std::vector<float> finalState){
  // figure out distance to goal achieved
  float dist = calculateDistance(finalState);

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

}
 

float EffectSpaceGoalSelector::calculateDistance(std::vector<float> finalState){
  // lets scale this so distance of 1 if we're still at start state, and dist of 0, if we're at goal
  int nRelevantFeatures = 0;
  float totalDist = 0;
  for (unsigned i = 0; i < finalState.size(); i++){
    if (goals[currentGoalIndex].goalMask[i]){
      nRelevantFeatures++;
      // distance in this feature from start to goal
      float fullRange = fabs(startState[i] - goals[currentGoalIndex].goalState[i]);
      if (fullRange < 1e-2) fullRange = 1e-2; // in case we started at goal in this feature
      // distance in this feature final state is from goal
      float finalDist = fabs(finalState[i] - goals[currentGoalIndex].goalState[i]);
      // dist scaled to pct of range
      totalDist += (finalDist / fullRange);
    }
  }
  // normalize by number of features
  float normDist = totalDist / (float) nRelevantFeatures;

  if (GOALDEBUG) cout << "Normalized distance reached to goal " << currentGoalIndex << ": " << normDist << endl;

  return normDist;
}

