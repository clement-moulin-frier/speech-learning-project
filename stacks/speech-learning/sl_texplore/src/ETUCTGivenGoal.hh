/** \file ETUCTGivenGoal.hh
    Defines the ETUCTGivenGoal class. 
    UCT with eligiblity traces. A modified version of UCT 
    as presented in:
    L. Kocsis and C. Szepesv´ari, "Bandit based monte-carlo planning," in
    ECML-06. Number 4212 in LNCS. Springer, 2006, pp. 282-293.
    \author Todd Hester
*/

#ifndef _ETUCTGivenGoal_HH_
#define _ETUCTGivenGoal_HH_

#include "Random.h"
#include "core.hh"


#include <set>
#include <vector>
#include <map>
#include <deque>

/** This class defines a modified version of UCT, which plans on a model using Monte Carlo rollouts. Unlike the original UCT, it does not separate values by tree depth, and it incorporates eligibility traces. */
class ETUCTGivenGoal: public GoalPlanner {
public:


  /** Standard constructor
      \param numactions, numactions in the domain
      \param gamma discount factor
      \param rrange range of one-step rewards in the domain
      \param lambda for use with eligibility traces
      \param MAX_ITER maximum number of MC rollouts to perform
      \param MAX_TIME maximum amount of time to run Monte Carlo rollouts
      \param MAX_DEPTH maximum depth to perform rollout to
      \param modelType specifies model type
      \param featmax maximum value of each feature
      \param featmin minimum value of each feature
      \param statesPerDim # of values to discretize each feature into
      \param trackActual track actual real-valued states (or just discrete states)
      \param rng random number generator
  */
  ETUCTGivenGoal(int numactions, float gamma, float rrange, float lambda,
        int MAX_ITER, float MAX_TIME, int MAX_DEPTH,  int modelType,
        const std::vector<float> &featmax, const std::vector<float> &featmin,
	const std::vector<int> &statesPerDim, bool trackActual, 
	Random rng = Random());
  
  /** Unimplemented copy constructor: internal state cannot be simply
      copied. */
  ETUCTGivenGoal(const ETUCTGivenGoal &);

  virtual ~ETUCTGivenGoal();

  virtual void setModel(MDPModel* model);
  virtual bool updateModelWithExperience(const std::vector<float> &last, 
                                         int act, 
                                         const std::vector<float> &curr, 
                                         float reward, bool term);
  virtual void planOnNewModel();
  virtual int getBestAction(const std::vector<float> &s);

  virtual void setSeeding(bool seed);
  virtual void setFirst();

  std::vector<float> getQForState(const std::vector<float> &state);
  void planFromState(const std::vector<float> &state);

  /** Output value function to a file */
  void logValues(ofstream *of, int xmin, int xmax, int ymin, int ymax);

  /** Initialize the states for this domain (based on featmin and featmax) */
  void initStates();
  
  /** Fill in a state based on featmin and featmax */
  void fillInState(std::vector<float>s, int depth);
  
  /** Return a discretized version of the input state. */
  std::vector<float> discretizeState(const std::vector<float> &s);

  bool PLANNERDEBUG;
  bool MODELDEBUG;
  bool ACTDEBUG;
  bool UCTDEBUG;
  bool REALSTATEDEBUG;

  /** The implementation maps all sensations to a set of canonical
      pointers, which serve as the internal representation of
      environment state. */
  typedef const std::vector<float> *state_t;

  // fill intrinsicRewards vector with these values
  void setIntrinsicRewards(std::vector<float> im);

  // goal stuff
  std::vector<float> goalState;
  std::vector<bool> goalMask;
  virtual void setGoal(std::vector<float> goal, std::vector<bool> mask);

  // variable # of actions stuff
  int numUsableActions;
  virtual void setUsableActions(int n);

  // for evaluation
  bool evaluationMode;
  virtual void setEvaluationMode(bool eval);
  
protected:


  struct state_info;
  struct model_info;

  /** A struct that contains a vector of possible next state samples, weighted by their probabilities. */
  struct state_samples {
    std::vector<state_t> samples;
  };

  /** State info struct. Maintains visit counts, models, and q-values for state-actions. */
  struct state_info {

    // data filled in from models
    StateActionInfo* model;

    // q values from policy creation
    std::vector<float> Q;

    // uct experience data
    int uctVisits;
    std::vector<int> uctActions;
    unsigned short int visited;
    unsigned short int id;

    // needs update
    bool needsUpdate;

  };



  /** Initialize state info struct */
  void initStateInfo(state_t s, state_info* info);
  
  /** Produces a canonical representation of the given sensation.
      \param s The current sensation from the environment.
      \return A pointer to an equivalent state in statespace. */
  state_t canonicalize(const std::vector<float> &s);

  /** Delete a state_info struct */
  void deleteInfo(state_info* info);
  
  /** Initialize a new state */
  void initNewState(state_t s);
  
  /** Compuate a policy from a model */
  void createPolicy();
  
  /** Print information for each state. */
  void printStates();
  
  /** Calculate which states are reachable from states the agent has actually visited. */
  void calculateReachableStates();
  
  /** Remove states from set that were deemed unreachable. */
  void removeUnreachableStates();

  /** Update the state_info copy of the model for the given state-action from the MDPModel */
  void updateStateActionFromModel(state_t s, int a, state_info* info);
  
  /** Get the current time in seconds */
  double getSeconds();

  /** Reset UCT visit counts to some baseline level (to decrease our confidence in q-values because model has changed. */
  void resetUCTCounts();

  /** Perform UCT/Monte Carlo rollout from the given state.
      If terminal or at depth, return some reward.
      Otherwise, select an action based on UCB.
      Simulate action to get reward and next state.
      Call search on next state at depth+1 to get reward return from there on.
      Update q value towards new value: reward + gamma * searchReturn
      Update visit counts for confidence bounds
      Return q
      
      From "Bandit Based Monte Carlo Planning" by Kocsis and Szepesv´ari.
  */
  float uctSearch(const std::vector<float> &actualS, state_t state, int depth);

  /** Return a sampled state from the next state distribution of the model. 
      Simulate the next state from the given state, action, and possibly history of past actions. */
  std::vector<float> simulateNextState(const std::vector<float> &actualState, state_t discState, state_info* info, int action, float* reward, bool* term);

  /** Select UCT action based on UCB1 algorithm. */
  int selectUCTAction(state_info* info);

  /** Canonicalize all the next states predicted by this model. */
  void canonNextStates(StateActionInfo* modelInfo);

  virtual void savePolicy(const char* filename);

  /** Add two vectors together. */
  std::vector<float> addVec(const std::vector<float> &a, const std::vector<float> &b);

  /** Subtract two vectors. */
  std::vector<float> subVec(const std::vector<float> &a, const std::vector<float> &b);

private:

  /** Set of all distinct sensations seen.  Pointers to elements of
      this set serve as the internal representation of the environment
      state. */
  std::set<std::vector<float> > statespace;

  /** Hashmap mapping state vectors to their state_info structs. */
  std::map<state_t, state_info> statedata;

  std::vector<float> featmax;
  std::vector<float> featmin;
  std::vector<float> intrinsicRewards;

  state_t prevstate;
  int prevact;

  double planTime;

  bool seedMode;

  int nstates;
  int nactions; 
  int lastUpdate;
  bool timingType;

  const int numactions;
  const float gamma;
  const float rrange;
  const float lambda;

  const int MAX_ITER;
  const float MAX_TIME;
  const int MAX_DEPTH;
  const int modelType;
  const std::vector<int> &statesPerDim;
  const bool trackActual;

};

#endif
