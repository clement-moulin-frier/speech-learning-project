#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include "Random.h"
#include <vector>
#include <map>


/** \file
    Fundamental declarations for the universal concepts in the
    reinforcement learning framework.
    \author Nick Jong
    \author Todd Hester
*/


/** Interface for an environment, whose states can be represented as
    vectors of floats and whose actions can be represented as ints.
    Implementations of the Environment interface determine how actions
    influence sensations.  Note that this design assumes only one
    agent: it would be more accurate to name this interface
    EnvironmentAsPerceivedByOneParticularAgent. */
class Environment {
public:
  /** Provides access to the current sensation that the environment
      gives to the agent.
      \return The current sensation. */
  virtual const std::vector<float> &sensation() const = 0;

  /** Allows an agent to affect its environment.
      \param action The action the agent wishes to apply.
      \return The immediate one-step reward caused by the action. */
  virtual float apply(int action) = 0;

  /** Allows an agent to affect its environment with speech.
      \param f1 
      \param f2
      \return The immediate one-step reward caused by the speech action. */
  virtual float applySpeechAction(float f1, float f2) = 0;

  /** Determines whether the environment has reached a terminal state.
      \return true iff the task is episodic and the present episode
      has ended.  Nonepisodic tasks should simply always
      return false. */
  virtual bool terminal() const = 0;

  /** Resets the internal state of the environment according to some
      initial state distribution.  Typically the user calls this only
      for episodic tasks that have reached terminal states, but this
      usage is not required. */
  virtual void reset() = 0;

  /** Returns the number of actions available in this environment.
      \return The number of actions available */
  virtual int getNumActions() = 0;

  /** Gets the minimum and maximum of the features in the environment.
   */
  virtual void getMinMaxFeatures(std::vector<float> *minFeat,
                                 std::vector<float> *maxFeat) = 0;

  /** Gets the minimum and maximum one-step reward in the domain. */
  virtual void getMinMaxReward(float *minR, float *maxR) = 0;

  /** Returns if the domain is episodic (true by default). */
  virtual bool isEpisodic(){ return true; };

  /** Set the current state for testing purposes. */
  virtual void setSensation(std::vector<float> s){};

  virtual ~Environment() {};

};


#endif
