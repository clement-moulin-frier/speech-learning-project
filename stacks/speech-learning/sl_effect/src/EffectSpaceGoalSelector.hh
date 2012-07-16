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
      \param tau Parameter for computing competence progress
      \param theta Parameter for computing competence progress
      \param rng Initial state of the random number generator to use*/
  EffectSpaceGoalSelector(int numobjects, int width, const std::vector<float> &featmin, 
                          const std::vector<float> &featmax, int goalSelect,
                          int learnerSelect, int tau, int theta,
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
    TexploreOnly
  };

  enum learners {
    TexploreLearner,
    SpeechLearner
  };

  struct goal {
    std::vector<float> startState;
    std::vector<bool>  startMask;
    std::vector<float> goalState;
    std::vector<bool>  goalMask;

    std::deque<float> allRewards;
    std::deque<float> texploreRewards;
    std::deque<float> speechRewards;
  };


  /** Init our set of possible goals. */
  void initGoals();

  /** Select a goal. */
  int selectGoal(std::vector<float> currentState, goal* selectedGoal);
  int selectRandomGoal(std::vector<float> currentState, goal* selectedGoal);
  int selectCompetenceProgressGoal(std::vector<float> currentState, goal* selectedGoal);

  void updateGoal(std::vector<float>);
  float calculateDistance(std::vector<float>);

  /** Select a learner */
  int chooseLearner(int goalIndex);
  int chooseHighCompetenceLearner(int goalIndex);

  bool GOALDEBUG;

protected:

  // all possible goals
  std::vector<goal> goals;

  // goal we're on
  int currentGoalIndex;
  std::vector<float> startState;
  int whichLearner;

private:

  const int numobjects;
  const int width;
  std::vector<float> featmin;
  std::vector<float> featmax;
  const int goalSelect;
  const int learnerSelect;
  const int tau;
  const int theta;

  Random rng;

};

#endif
