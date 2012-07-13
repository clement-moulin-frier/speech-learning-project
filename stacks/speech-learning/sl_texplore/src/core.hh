#ifndef _RLCORE_H_
#define _RLCORE_H_

#include "Random.h"
#include <vector>
#include <map>


/** \file
    Fundamental declarations for the universal concepts in the
    reinforcement learning framework.
    \author Nick Jong
    \author Todd Hester
*/


// types of models

#define C45TREE     2

// types of model combos
#define AVERAGE        1
#define WEIGHTAVG      2
#define BEST           3
#define SEPARATE       4 // sep model for planning, and forest for uncertainty
#define SELECTED       5
#define BOSS_COMBO     6
#define BAYESDP_COMBO  7

const std::string comboNames[] = {
  "ZERO",
  "Average",
  "Weighted Average",
  "Best",
  "Separate",
  "Selected", 
  "BOSS Augmented Model",
  "Bayesian DP Combo"
};

// types of exploration
#define EXPLORE_UNKNOWN    0
#define TWO_MODE           1
#define TWO_MODE_PLUS_R    2
#define CONTINUOUS_BONUS   3
#define THRESHOLD_BONUS    4
#define CONTINUOUS_BONUS_R 5
#define THRESHOLD_BONUS_R  6
#define NO_EXPLORE         7
#define GREEDY             7
#define EPSILONGREEDY      8
#define VISITS_CONF        9
#define UNVISITED_BONUS    11
#define UNVISITED_ACT_BONUS 13
#define DIFF_AND_VISIT_BONUS 16
#define RANDOM_EXPLORE       17
#define NOVEL_STATE_BONUS    18
#define DIFF_AND_NOVEL_BONUS 19
#define NOVEL_FEAT_BONUS     20
#define DIFF_AND_NF_BONUS    21
#define NOISE_BONUS          22
#define ERROR_BONUS          23
#define ERROR_IMPROVE_BONUS  24
#define INPUT_ENTROPY_BONUS  25
#define LEAST_TAKEN_ACT      26
#define BOLTZMANN            27
#define RMAX_EXPLORATION     28
#define DIFF_NOVEL_UNVISITED 29

const std::string exploreNames[] = {
  "Explore Unknowns",
  "Two Modes",
  "Two Models +R",
  "Continuous Bonus",
  "Threshold Bonus",
  "Continuous Bonus +R",
  "Threshold Bonus +R",
  "Greedy",
  "Epsilon-Greedy",
  "Visits Confidence",
  "Type 10",
  "Unvisited State Bonus",
  "Type 12",
  "Unvisited Action Bonus",
  "Type 14",
  "Type 15",
  "Model Diff & Visit Bonus",
  "Random",
  "FeatDist Bonus",
  "Model Diff & FeatDist Bonus",
  "Novel Feature Bonus",
  "Model Diff & Novel Feature Bonus",
  "Noisy Prediction Bonus",
  "Prediction Error Bonus",
  "Improvement In Error Bonus",
  "Tree inputs entropy bonus",
  "Least taken action",
  "Boltzmann Softmax",
  "RMax Exploration",
  "Diff, Novel, and Unvisited Bonuses"
};


#define EPSILON   1e-5

/** Experience <s,a,s',r> struct */
struct experience {
  std::vector<float> s;
  int act;
  float reward;
  std::vector<float> next;
  bool terminal;
};


/** Training instances for prediction models */
struct modelPair {
  std::vector<float> in;
  std::vector<float> out;
};

/** Training instances for classification models */
struct classPair {
  std::vector<float> in;
  float out;
};



/** Interface for an agent.  Implementations of the Agent interface
    determine the choice of actions given previous sensations and
    rewards. */
class Agent {
public:
  /** Determines the first action that an agent takes in an
      environment.  This method implies that the environment is
      currently in an initial state.
      \param s The initial sensation from the environment.
      \return The action the agent wishes to take first. */
  virtual int first_action(const std::vector<float> &s) = 0;

  /** Determines the next action that an agent takes in an environment
      and gives feedback for the previous action.  This method may
      only be called if the last method called was first_action or
      next_action.
      \param r The one-step reward resulting from the previous action.
      \param s The current sensation from the environment.
      \return The action the agent wishes to take next. */
  virtual int next_action(float r, const std::vector<float> &s) = 0;

  /** Gives feedback for the last action taken.  This method may only
      be called if the last method called was first_action or
      next_action.  It implies that the task is episodic and has just
      terminated.  Note that terminal sensations (states) are not
      represented.
      \param r The one-step reward resulting from the previous action. */
  virtual void last_action(float r) = 0;

  /** Set some debug flags on/off */
  virtual void setDebug(bool d) = 0;

  /** Use the model seeds from the environment to initialize the agent or its model */
  virtual void seedExp(std::vector<experience> seeds) {};

  /** Save the current policy to a file */
  virtual void savePolicy(const char* filename) {};

  virtual ~Agent() {};
};

/** Interface for a model that predicts a vector of floats given a vector of floats as input. */
class Model {
public:
  /** Train the model on a vector of training instances */
  virtual bool trainInstances(std::vector<modelPair> &instances) = 0;

  /** Train the model on a single training instance */
  virtual bool trainInstance(modelPair &instance) = 0;

  /** Get the model's prediction for a given input */
  virtual std::vector<float> testInstance(const std::vector<float> &in) = 0;

  virtual ~Model() {};
};

/** Interface for a classification model that predicts a class given a vector of floats as input. */
class Classifier {
public:
  /** Train the model on a vector of training instances */
  virtual bool trainInstances(std::vector<classPair> &instances) = 0;

  /** Train the model on a single training instance */
  virtual bool trainInstance(classPair &instance) = 0;

  /** Get the model's prediction for a given input */
  virtual void testInstance(const std::vector<float> &in, std::map<float, float>* retval) = 0;

  /** Get the model's confidence in its predictions for a given input. */
  virtual float getConf(const std::vector<float> &in) = 0;

  /** Get a copy of the model */
  virtual Classifier* getCopy() = 0;

  virtual ~Classifier() {};
};

/** All the relevant information predicted by a model for a given state-action.
    This includes predicted reward, next state probabilities, probability of episod termination, and model confidence.
*/
struct StateActionInfo {
  float conf;
  bool known;
  float reward;
  float termProb;
  int frameUpdated;

  // map from outcome state to probability
  std::map< std::vector<float> , float> transitionProbs;

  StateActionInfo(){
    conf = 0.0;
    known = false;
    reward = 0.0;
    termProb = 0.0;
    frameUpdated = -1;
  };
};


/** Interface for a model of an MDP. */
class MDPModel {
public:
  /** Update the MDP model with a vector of experiences. */
  virtual bool updateWithExperiences(std::vector<experience> &instances) = 0;

  /** Update the MDP model with a single experience. */
  virtual bool updateWithExperience(experience &instance) = 0;

  /** Get the predictions of the MDP model for a given state action */
  virtual bool getStateActionInfo(const std::vector<float> &state, int action, StateActionInfo* retval) = 0;

  /** Get a copy of the MDP Model */
  virtual MDPModel* getCopy() = 0;
  virtual ~MDPModel() {};
};




/** Interface for planners */
class Planner {
public:
  /** Give the planner the model being used with the agent */
  virtual void setModel(MDPModel* model) = 0;

  /** Update the given model with an experience <s,a,s',r>. */
  virtual bool updateModelWithExperience(const std::vector<float>& last,
                                         int act,
                                         const std::vector<float>& curr,
                                         float reward, bool terminal) = 0;

  /** Plan a new policy suing the current model. */
  virtual void planOnNewModel() = 0;

  /** Return the best action for a given state. */
  virtual int getBestAction(const std::vector<float> &s) = 0;

  /** Save the policy to a file. */
  virtual void savePolicy(const char* filename) {};

  /** Set whether the next experiences are seeds or actual experiences from the agent. */
  virtual void setSeeding(bool seeding) {};

  /** Set if this is the first experience of the agent. */
  virtual void setFirst() {};

  /** A method to return at random one of the maximum values in the vector. 
      Such that when more than one actions are optimal, we select one of them at random.
  */
  std::vector<float>::iterator
  random_max_element(std::vector<float>::iterator start,
		     std::vector<float>::iterator end) {
    const float Q_EPSILON = 2e-4;
    
    std::vector<float>::iterator max =
    std::max_element(start, end);

    // find # within epsilon of max
    int nfound = 0;
    for (std::vector<float>::iterator it = start; it != end; it++){
      if (fabs(*it - *max) < Q_EPSILON){
        nfound++;
      }
    }
    
    // only 1: take it
    if (nfound == 1)
      return max;

    // take one of close to max at random
    for (std::vector<float>::iterator it = start; it != end; it++){
      if (fabs(*it - *max) < Q_EPSILON){
        if (rng.uniform() < (1.0 / (float)nfound)){
          return it;
        }
        nfound--;
      }
    }
    
    return max;
  };

  virtual ~Planner() {};
  
  MDPModel* model;
  Random rng;

};


  
#endif
