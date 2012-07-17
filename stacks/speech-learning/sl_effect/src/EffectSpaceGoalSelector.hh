/** \file EffectSpaceGoalSelector.hh
    Defines the EffectSpaceGoalSelector class
    \author Todd Hester
*/

#ifndef _EFFECTGOALS_HH_
#define _EFFECTGOALS_HH_

#include "Random.h"

#include <set>
#include <vector>
#include <deque>

#include <sl_msgs/SLGoal.h>


/** Code for to select goals in effect space. */
class EffectSpaceGoalSelector {
public:

  /** Standard constructor 
      \param numobjects The number of objects in the domain
      \param width Width of table in domain
      \param featmin min values of each feature
      \param featmax max values of each feature
      \param goalSelect how we're selecting goals (0: random, 1: sagg-riac)
      \param learnerSelect how we're choosing learner
      \param tau Compare with average tau steps ago for computing competence progress
      \param theta Average this many steps together for computing competence progress
      \param evalFreq Frequency to run evals on hand-picked goal set 
      \param debug debug prints on/off
      \param rng Initial state of the random number generator to use*/
  EffectSpaceGoalSelector(int numobjects, int width, const std::vector<float> &featmin, 
                          const std::vector<float> &featmax, int goalSelect,
                          int learnerSelect, int tau, int theta, bool debug,
                          int evalFreq,
                          Random rng = Random());
  
  /** Unimplemented copy constructor: internal state cannot be simply
      copied. */
  EffectSpaceGoalSelector(const EffectSpaceGoalSelector &);

  ~EffectSpaceGoalSelector();

  enum goalSelectTypes {
    RandomGoal,
    SaggRiac
  };

  enum learnerSelectTypes {
    RandomLearner,
    Competence,
    CompProgress,
    TexploreOnly,
    SpeechOnly
  };

  enum learners {
    TexploreLearner,
    SpeechLearner
  };

  struct goal {
    // goal message (start and goal states and mask of which features we're interested in)
    sl_msgs::SLGoal goalMsg;

    // stuff to track performance for this goal
    std::deque<float> allRewards;
    std::deque<float> texploreRewards;
    std::deque<float> speechRewards;

  };


  /** Init our set of possible goals. */
  void initGoals();

  /** Select a goal. */
  goal selectGoal(std::vector<float> currentState, int* learner);
  int selectRandomGoal(std::vector<float> currentState);
  int selectCompetenceProgressGoal(std::vector<float> currentState);
  bool isGoalValid(std::vector<float> currentState, int goalIndex);

  void printGoal(goal* selectedGoal);

  float updateGoal(std::vector<float>);
  float calculateDistance(std::vector<float>);
  float calcCompetenceProgress(std::deque<float> &results);

  /** Select a learner */
  int chooseLearner(int goalIndex);
  int chooseHighCompetenceLearner(int goalIndex);
  int chooseCompetenceProgressLearner(int goalIndex);

  /** For eval. */
  int evaluateGoals();
  void updateEval(float dist);

  int currentEvalGoal;
  std::vector<int> evalIndices;
  bool evalMode;
  int lastEvalGoal;

  bool GOALDEBUG;
  bool GOALSELECTDEBUG;

protected:

  // all possible goals
  std::vector<goal> goals;

  // goal we're on
  int currentGoalIndex;
  std::vector<float> startState;
  int whichLearner;
  int nGoalsTried;

private:

  const int numobjects;
  const int width;
  std::vector<float> featmin;
  std::vector<float> featmax;
  const int goalSelect;
  const int learnerSelect;
  const int tau;
  const int theta;
  const int evalFreq;

  Random rng;

};

#endif
