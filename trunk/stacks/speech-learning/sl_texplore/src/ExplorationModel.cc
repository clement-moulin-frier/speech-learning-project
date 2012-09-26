/** \file ExplorationModel.cc
    Implements the ExplorationModel class.
    Reward bonuses based on the variance in model predictions are described in: Hester and Stone, "Real Time Targeted Exploration in Large Domains", ICDL 2010.
    \author Todd Hester
*/

#include "ExplorationModel.hh"




ExplorationModel::ExplorationModel(MDPModel* innermodel, int modelType, int exploreType,
                                   int predType, int nModels,
                                   float m, int numactions,
                                   float rmax, float qmax, float rrange,
                                   int nfactors, float v, float n,
                                   const std::vector<float> &fmax,
                                   const std::vector<float> &fmin, Random rng):
  modelType(modelType), exploreType(exploreType), predType(predType),
  nModels(nModels),
  M(m), numactions(numactions), rmax(rmax), qmax(qmax), rrange(rrange),
  nfactors(nfactors), taskCoeff(1), varianceCoeff(v), noveltyCoeff(n), unvisitedCoeff(n), rng(rng)
{

  model = innermodel;

  MODEL_DEBUG = false; //true;
  NOVELTYDEBUG = false; //true;

  cout << "Exploration Model " << exploreType << ", t: " << taskCoeff << ", v: " << varianceCoeff << ", n: " << noveltyCoeff << ", u: " << unvisitedCoeff << endl;

  featmax = fmax;
  featmin = fmin;

}

ExplorationModel::ExplorationModel(const ExplorationModel &em):
  modelType(em.modelType), exploreType(em.exploreType), predType(em.predType),
  nModels(em.nModels),
  M(em.M), numactions(em.numactions), rmax(em.rmax), qmax(em.qmax), rrange(em.rrange),
  nfactors(em.nfactors), taskCoeff(em.taskCoeff), varianceCoeff(em.varianceCoeff), noveltyCoeff(em.noveltyCoeff), unvisitedCoeff(em.unvisitedCoeff), rng(em.rng)
{
  model = em.model->getCopy();

  MODEL_DEBUG = em.MODEL_DEBUG;
  NOVELTYDEBUG = em.NOVELTYDEBUG;
  featmax = em.featmax;
  featmin = em.featmin;
  statespace = em.statespace;
  novelfeats = em.novelfeats;
}

ExplorationModel* ExplorationModel::getCopy(){
  ExplorationModel* copy = new ExplorationModel(*this);
  return copy;
}


ExplorationModel::~ExplorationModel() {
  delete model;
}



bool ExplorationModel::updateWithExperiences(std::vector<experience> &instances){
  bool changed = model->updateWithExperiences(instances);
  bool visitChange = false;

  // keep track of which states we've been to for this mode
  for (unsigned i = 0; i < instances.size(); i++){
    if (exploreType == UNVISITED_BONUS){
      bool retval = addStateToSet(instances[i].s);
      visitChange = visitChange || retval;
    }

    if (exploreType == UNVISITED_ACT_BONUS || exploreType == DIFF_AND_VISIT_BONUS || exploreType == NOVEL_STATE_BONUS || exploreType == DIFF_AND_NOVEL_BONUS || exploreType == DIFF_NOVEL_UNVISITED){
      std::vector<float> last2 = instances[i].s;
      last2.push_back(instances[i].act);
      bool retval = addStateToSet(last2);
      visitChange = visitChange || retval;

    }

    if (exploreType == DIFF_AND_NF_BONUS || exploreType == NOVEL_FEAT_BONUS){
      bool retval = addFeatsToSet(instances[i].s, instances[i].act);
      visitChange = visitChange || retval;
    }
  }

  return (changed || visitChange);
}

// update all the counts, check if model has changed
// stop counting at M
bool ExplorationModel::updateWithExperience(experience &e){
  //if (MODEL_DEBUG) cout << "updateWithExperience " << &last << ", " << act
  //        << ", " << &curr << ", " << reward << endl;

  bool changed = model->updateWithExperience(e);
  bool visitChange = false;

  // keep track of which states we've been to for this mode
  if (exploreType == UNVISITED_BONUS){
    bool retval = addStateToSet(e.s);
    visitChange = visitChange || retval;
  }

  if (exploreType == UNVISITED_ACT_BONUS || exploreType == DIFF_AND_VISIT_BONUS || exploreType == NOVEL_STATE_BONUS || exploreType == DIFF_AND_NOVEL_BONUS || exploreType == DIFF_NOVEL_UNVISITED){
    if (NOVELTYDEBUG){
      cout << "Add experience from state " << e.s[0];
      for (unsigned i = 1; i < e.s.size(); i++){
        cout << ", " << e.s[i];
      }
      cout << ", and action : "<< e.act << endl;
    }
    std::vector<float> last2 = e.s;
    last2.push_back(e.act);
    bool retval = addStateToSet(last2);
    visitChange = visitChange || retval;
  }

  if (exploreType == DIFF_AND_NF_BONUS || exploreType == NOVEL_FEAT_BONUS){
    bool retval = addFeatsToSet(e.s, e.act);
    visitChange = visitChange || retval;
  }


  // anything that got past the 'return false' above is a change in conf or predictions
  return (changed || visitChange);

}



// calculate state info such as transition probs, known/unknown, reward prediction
float ExplorationModel::getStateActionInfo(const std::vector<float> &state, int act, StateActionInfo* retval){
  //if (MODEL_DEBUG) cout << "getStateActionInfo, " << &state <<  ", " << act << endl;

  retval->transitionProbs.clear();

  float conf = model->getStateActionInfo(state, act, retval);

  // possibly scale task reward relative to others
  retval->reward *= taskCoeff;

  //cout << "state: " << state[0] << " act: " << act;

  if (MODEL_DEBUG)// || (retval->conf > 0.0 && retval->conf < 1.0))
    cout << "reward: " << retval->reward << " conf: " << conf << endl;

  // check exploration bonuses

  if (exploreType == NOISE_BONUS){
    float entropy = calcEntropy(retval->transitionProbs);
    retval->reward += (entropy/3.0);
    if (MODEL_DEBUG) {
      cout << "State-action prediction had noise: " << entropy
           << ", added bonus: " << (entropy/3.0) << endl;
    }
  }

  // use qmax if state is unknown
  if (exploreType == EXPLORE_UNKNOWN){
    if (!retval->known){
      if (MODEL_DEBUG){
        cout << "State-Action Unknown in model: conf: " << conf << " ";
        for (unsigned si = 0; si < state.size(); si++){
          cout << (state)[si] << ",";
        }
        cout << " Action: " << act << endl;
      }
      retval->reward = qmax;
      retval->termProb = 1.0;
      if (MODEL_DEBUG || MODEL_DEBUG)
        cout << "   State-Action Unknown in model, using qmax "
             << qmax << endl;
    }
  }

  // small bonus for unvisited states
  if (exploreType == UNVISITED_BONUS){
    float bonus = -unvisitedCoeff;
    if (!checkForState(state)){
      // modify reward with a bonus of u
      bonus += unvisitedCoeff;
      if (MODEL_DEBUG){
        cout << "   State unvisited bonus, orig R: "
             << retval->reward
             << " adding u: " << unvisitedCoeff
             << endl;
      }
    }
    retval->reward += bonus;
  }

  // small bonus for unvisited state-actions
  if (exploreType == UNVISITED_ACT_BONUS || exploreType == DIFF_AND_VISIT_BONUS || exploreType == DIFF_NOVEL_UNVISITED){
    std::vector<float> state2 = state;
    state2.push_back(act);
    float bonus = -unvisitedCoeff;
    if (!checkForState(state2)){
      // modify reward with a bonus of u
      bonus += unvisitedCoeff;
      if (MODEL_DEBUG){
        cout << "   State-Action unvisited bonus, orig R: "
             << retval->reward
             << " adding u: " << unvisitedCoeff
             << endl;
      }
    }
    retval->reward += bonus;
  }

  // small bonus for states far from visited states with same action
  if (exploreType == NOVEL_STATE_BONUS || exploreType == DIFF_AND_NOVEL_BONUS || exploreType == DIFF_NOVEL_UNVISITED){
    std::vector<float> state2 = state;
    state2.push_back(act);
    float featDist = getFeatDistToVisitedSA(state2);
    float bonus = -noveltyCoeff;
    if (featDist > 0){
      // modify reward with proportional bonus of n
      bonus += featDist * noveltyCoeff;
      if (MODEL_DEBUG){
        cout << "   State-Action novel state bonus, dist: " << featDist
             << " n: " << noveltyCoeff << ", bonus, " << bonus << endl;
      }
    }
    retval->reward += bonus;
  }

  // small bonus for states with feature values that haven't been seen before
  if (exploreType == DIFF_AND_NF_BONUS || exploreType == NOVEL_FEAT_BONUS){
    int numNovelFeats = getNumNovelFeats(state, act);
    float novelFeatRatio = (float)numNovelFeats/(float)state.size();
    float bonus = -noveltyCoeff;
    if (numNovelFeats > 0){
      // modify reward with proportional bonus of n
      bonus += novelFeatRatio * noveltyCoeff;
      if (MODEL_DEBUG){
        cout << "   State-Action novel feat bonus, num novel: " << numNovelFeats
             << " ratio: " << novelFeatRatio
             << " n: " << noveltyCoeff << ", bonus, " << bonus << endl;
      }
    }
    retval->reward += bonus;
  }

  // use some % of v if we're doing continuous terminal bonus
  if (exploreType == CONTINUOUS_BONUS){
    if (conf < 1.0){
      // percent of conf
      float bonus = (1.0-conf)*varianceCoeff;
      if (MODEL_DEBUG){
        cout << "   State-Action continuous bonus conf: "
             << conf
             << ", using v*(1-conf): "
             << bonus << endl;
      }
      retval->reward = bonus;
      retval->termProb = 1.0;
    }
  }

  // use some % of v if we're doing continuous bonus
  if (exploreType == CONTINUOUS_BONUS_R || exploreType == DIFF_AND_VISIT_BONUS || exploreType == DIFF_AND_NOVEL_BONUS || exploreType == DIFF_AND_NF_BONUS || exploreType == DIFF_NOVEL_UNVISITED){
    float bonus = -varianceCoeff;
    if (conf < 1.0){
      // percent of conf
      bonus += (1.0-conf)*varianceCoeff;
      if (MODEL_DEBUG){
        cout << "   State-Action continuous bonus conf: "
             << conf
             << ", using v*(1-conf): "
             << bonus << endl;
      }
    }
    retval->reward += bonus;
  }

  // use qmax if we're doing threshold terminal bonus and conf under threshold
  if (exploreType == THRESHOLD_BONUS){
    if (conf < 0.5){
      float bonus = varianceCoeff;
      if (MODEL_DEBUG){
        cout << "   State-Action conf< thresh: "
             << conf
             << " M: " << M
             << ", using v "
             << varianceCoeff << endl;
      }
      retval->reward = bonus;
      retval->termProb = 1.0;
    }
  }

  // use rmax for additional thresh bonus and conf under thresh
  if (exploreType == THRESHOLD_BONUS_R){
    float bonus = -varianceCoeff;
    if (conf < 0.9){
      bonus += varianceCoeff;
      if (MODEL_DEBUG){
        cout << "   State-Action conf< thresh: "
             << conf
             << " M: " << M
             << ", using v "
             << varianceCoeff << endl;
      }
    }
    retval->reward += bonus;
  }

  // visits conf
  if (exploreType == VISITS_CONF){
    if (conf < 0.5){
      float bonus = qmax;
      retval->reward += bonus;
      if (MODEL_DEBUG){
        cout << "   State-Action conf< thresh or 0 visits: "
             << conf
             << " M: " << M
             << ", using qmax "
             << qmax << endl;
      }
      retval->reward = bonus;
      retval->termProb = 1.0;
    }
  }



  if (MODEL_DEBUG)
    cout << "   Conf: " << conf << "   Avg reward: " << retval->reward << endl;
  if (isnan(retval->reward))
    cout << "ERROR: Model returned reward of NaN" << endl;

  return conf;
}

// add state to set (if its not already in it)
bool ExplorationModel::addStateToSet(const std::vector<float> &s){
  std::pair<std::set<std::vector<float> >::iterator, bool> retval;
  retval = statespace.insert(s);
  return retval.second;
}

// check if state is in set (so we know if we've visited it)
bool ExplorationModel::checkForState(const std::vector<float> &s){
  return (statespace.count(s) == 1);
}

// get distance in feature space from this state to one we've visited
float ExplorationModel::getFeatDistToVisitedSA(const std::vector<float> &s){

  if (NOVELTYDEBUG){
    cout << "Get feat dist for state: ";
    for (unsigned j = 0; j < s.size(); j++){
      cout << s[j] << ", ";
    }
    cout << endl;
  }

  // if we've visited this exact s,a then dist is 0
  if (checkForState(s)){
    if (NOVELTYDEBUG) cout << "S-A was visited. Dist 0" << endl;
    return 0;
  }

  // otherwise go through all states and find minimum distance
  float maxDist = 0;
  unsigned nfeats = s.size()-1;
  std::vector<float> featRange(nfeats, 0);
  for (unsigned i = 0; i < nfeats; i++){
    featRange[i] = featmax[i] - featmin[i];
    maxDist += 1.0;//featmax[i] - featmin[i];

    //cout << "feat " << i << " diff: " << (featmax[i] - featmin[i]) << " max: " << maxDist << endl;
  }

  float minDist = maxDist;//nfeats;
  unsigned actionIndex = nfeats;

  for (std::set<std::vector<float> >::iterator i = statespace.begin(); i != statespace.end(); i++){
    // ignore if not the same action
    if (s[actionIndex] != (*i)[actionIndex]) continue;

    if (NOVELTYDEBUG){
      cout << "State is ";
      for (unsigned j = 0; j < (*i).size(); j++){
        cout << (*i)[j] << ", ";
      }
      cout << endl;
    }

    // otherwise, sum all features that are different
    float count = 0;
    for (unsigned j = 0; j < nfeats; j++){
      // distance is just different/ not different
      //if (s[j] != (*i)[j]) count++;
      // distance based on magnitude of feature difference
      // normalize by feature range
      //cout << "  Feat " << j << " diff is " << fabs(s[j] - (*i)[j]) << ", range: " << featRange[j] << ", normalized: " << fabs(s[j] - (*i)[j]) / featRange[j] << endl;
      count += fabs(s[j] - (*i)[j]) / featRange[j];
    }
    if (count < minDist) minDist = count;


    if (NOVELTYDEBUG){
      cout << "dist: " << count << " minDist: " << minDist << endl;
    }

    // we can't get any lower than a dist of 1
    /*
      if (minDist == 1){
      if (NOVELTYDEBUG) cout << "found dist 1, returning " << (1.0/(float)maxDist) << endl;
      return 1.0/(float)nfeats;
      }
    */
  }

  if (NOVELTYDEBUG) cout << "return " << ((float)minDist/(float)maxDist) << endl;
  return (float)minDist/(float)nfeats;

}


bool ExplorationModel::addFeatsToSet(const std::vector<float> &s, int act){

  if (NOVELTYDEBUG){
    cout << "Insert novel feats for state: ";
    for (unsigned j = 0; j < s.size(); j++){
      cout << s[j] << ", ";
    }
    cout << "action: " << act << endl;
    cout << endl;
  }

  std::pair<std::set<std::vector<float> >::iterator, bool> retval;
  bool change = false;
  std::vector<float> featinfo(3);
  for (unsigned i = 0; i < s.size(); i++){
    featinfo[0] = i;
    featinfo[1] = s[i];
    featinfo[2] = act;
    retval = novelfeats.insert(featinfo);
    change = change || retval.second;
  }

  if (NOVELTYDEBUG) cout << "novelfeats size: "<< novelfeats.size() << endl;

  return change;
}

int ExplorationModel::getNumNovelFeats(const std::vector<float> &s, int act){

  if (NOVELTYDEBUG){
    cout << "Get # novel feats for state: ";
    for (unsigned j = 0; j < s.size(); j++){
      cout << s[j] << ", ";
    }
    cout << "action: " << act << endl;
    cout << endl;
  }


  int count = 0;
  std::vector<float> featinfo(3);
  for (unsigned i = 0; i < s.size(); i++){
    featinfo[0] = i;
    featinfo[1] = s[i];
    featinfo[2] = act;
    bool novelfeat = (novelfeats.count(featinfo) == 0);
    if (novelfeat) count++;
    if (NOVELTYDEBUG){
      cout << "Feat " << i << " val " << s[i] << " novelty: " << novelfeat << endl;
    }
  }

  if (NOVELTYDEBUG) cout << "novelfeat count; " << count << endl;

  return count;
}


float ExplorationModel::calcEntropy(std::map< std::vector<float> , float> &transitionProbs){
  float sum = 0;
  for (std::map< std::vector<float> , float>::iterator i = transitionProbs.begin(); i != transitionProbs.end(); i++){
    float prob = i->second;
    sum += prob * log2(1/prob);
  }
  return sum;
}


void ExplorationModel::setExploration(int type, float newT, float newV, float newN, float newU){
  exploreType = type;
  taskCoeff = newT;
  varianceCoeff = newV;
  noveltyCoeff = newN;
  unvisitedCoeff = newU;
}
