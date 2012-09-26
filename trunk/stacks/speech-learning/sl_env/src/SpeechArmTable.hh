/**
 The SpeechArmTable domain
*/

#ifndef _SPEECHARMTABLE_H_
#define _SPEECHARMTABLE_H_

#include <set>
#include "Random.h"
#include "Environment.hh"


class SpeechArmTable: public Environment {
public:
  /** Creates a SpeechArmTable domain with the specified size and number of objects.
      \param rand Random number generator to use.
      \param stochastic Whether to use nondeterministic actions 
      \param width How many states wide is the table? (its twice as long)
      \param nobjects How many objects are there (0-5)
      \param speech_radius How far from true speech will it work?
  */
  SpeechArmTable(Random &rand, bool stochastic, int width, int nobjects, float speech_radius);

  virtual ~SpeechArmTable();

  // interact with learning agents
  virtual const std::vector<float> &sensation() const;
  virtual float apply(int action);
  float applySpeechAction(float a1, float a2);
  virtual bool terminal() const;
  virtual void reset();

  // provide some info about task
  virtual int getNumActions();
  virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
  virtual bool isEpisodic() { return false; };
  virtual void getMinMaxReward(float* minR, float* maxR);

  // stuff for speech debug
  void addSpeechAction(float a1, float a2);
  void setOptimalSpeeches();
  void setDefaultSpeeches();

  // things that help with computing next state
  void updateRelativeDistances();
  int applyNoise(int action);
  void initObjects();
  void initSpeeches();
  float applyObjectAction(int object);

  // for debug
  void print_state() const;
  void print_map() const;
  bool SPDEBUG;

  // speech info
  struct speech {
    float f1;
    float f2;
  };

  // keep track of speech actions we want to use for the domain
  std::vector<speech> savedSpeeches;
  int numactions;
  int numspeeches;

  // domain parameters
  const bool noisy;
  const int width;
  const int height;
  const int nobjects;
  const float speech_radius;
  Random &rng;

  // domain state features
  std::vector<float> s;
  float& hand_x;
  float& hand_y;
  float& hand_obj;

  struct object_info {
    float* x;
    float* y;
    float* rel_x;
    float* rel_y;
    float* reachable;
    speech sound;
  };

  std::vector<object_info> objects;


  // domain actions
  enum speecharmtable_action_t {NORTH, EAST, WEST, SOUTH, PICKUP, PUTDOWN, INIT_NUMACTIONS};


};

#endif
