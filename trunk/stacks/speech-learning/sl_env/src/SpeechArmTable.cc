#include "SpeechArmTable.hh"
/**
   The SpeechArmTable domain.
   Here there is a discrete table with a number of objects on it. The agent can move its hand N,S,E,W to the objects and then pick them up and put them down. However, it can only reach halfway across the table. For objects on the other side of the table, it has to ask for a caregiver to pass them to it by making the right speech.

*/

SpeechArmTable::SpeechArmTable(Random &rand, bool stochastic, int width, int nobjects, float speech_radius):
  noisy(stochastic),
  width(width),
  height(width*2),
  nobjects(nobjects),
  speech_radius(speech_radius),
  rng(rand),
  s(3+5*nobjects),
  hand_x(s[0]),
  hand_y(s[1]),
  hand_obj(s[2]),
  objects(nobjects)
{

  if (nobjects > 5 || nobjects < 0){
    cout << "ERROR: nobjects must be between 0-5" << endl;
    exit(-1);
  }

  SPDEBUG = false;
  numspeeches = 0;
  numactions = INIT_NUMACTIONS;

  initObjects();
  initSpeeches();

  reset();
}



SpeechArmTable::~SpeechArmTable() {  }

void SpeechArmTable::initObjects(){
  for (int i = 0; i < nobjects; i++){
    if (SPDEBUG) cout << "Init object " << i << endl << flush;
    int startIndex = 3+i*5;
    objects[i].x = &(s[startIndex]);
    objects[i].y = &(s[startIndex+1]);
    objects[i].rel_x = &(s[startIndex+2]);
    objects[i].rel_y = &(s[startIndex+3]);
    objects[i].reachable = &(s[startIndex+4]);
  }
  if (SPDEBUG) cout << "Objects init'd" << endl << flush;
}

void SpeechArmTable::initSpeeches(){

  if (SPDEBUG) cout << "init speeches" << endl << flush;

  if (nobjects == 0) return;

  objects[0].sound.f1 = 6.4;
  objects[0].sound.f2 = 3;

  if (nobjects == 1) return;

  objects[1].sound.f1 = 3.2;
  objects[1].sound.f2 = 3.95;

  if (nobjects == 2) return;

  objects[2].sound.f1 = 3.3;
  objects[2].sound.f2 = 2.5;

  if (nobjects == 3) return;

  objects[3].sound.f1 = 4.75;
  objects[3].sound.f2 = 3.75;

  if (nobjects == 4) return;

  objects[4].sound.f1 = 4.8;
  objects[4].sound.f2 = 2.6;

  if (SPDEBUG) cout << "Speeches for each object init'd" << endl << flush;

}


const std::vector<float> &SpeechArmTable::sensation() const {
  if (SPDEBUG) print_map();
  return s;
}


int SpeechArmTable::applyNoise(int action){
  switch(action) {
  case NORTH:
  case SOUTH:
    return rng.bernoulli(0.9) ? action : (rng.bernoulli(0.5) ? EAST : WEST);
  case EAST:
  case WEST:
    return rng.bernoulli(0.9) ? action : (rng.bernoulli(0.5) ? NORTH : SOUTH);
  case PICKUP:
  case PUTDOWN:
    return rng.bernoulli(0.9) ? action : -1;
  default:
    return action;
  }
}


void SpeechArmTable::addSpeechAction(float a1, float a2){
  if (SPDEBUG) cout << "Add speech: " << a1 << ", " << a2 << " as speech " << numspeeches << endl;
  speech newSpeech;
  newSpeech.f1 = a1;
  newSpeech.f2 = a2;
  savedSpeeches.push_back(newSpeech);
  numspeeches = savedSpeeches.size();
  numactions++;
}


float SpeechArmTable::applySpeechAction(float a1, float a2){

  // see about applying this speech action
  if (SPDEBUG) cout << "Speech is " << a1 << ", " << a2 << endl;

  // compare speech distance to each true speech
  for (int i = 0; i < nobjects; i++){
    float dist = sqrtf((a1-objects[i].sound.f1)*(a1-objects[i].sound.f1)+(a2-objects[i].sound.f2)*(a2-objects[i].sound.f2));
    if (SPDEBUG)
      cout << "Speech dist to true speech " << i << ": " << objects[i].sound.f1 << ", " << objects[i].sound.f2 << " dist: " << dist << endl;
    if (dist < speech_radius){
      // check probability if noisy, or it just works if deterministic
      if (noisy){
        float prob = 1.0 - dist / speech_radius;
        if (SPDEBUG) cout << "Speech probability: " << prob << endl;
        if (rng.bernoulli(prob)) return applyObjectAction(i);
      } else {
        return applyObjectAction(i);
      }
    } // within speech_radius
  } // all speeches

  // no reward
  return 0;

}


float SpeechArmTable::applyObjectAction(int object){

  // if reachable, other agent takes it back
  if (*objects[object].reachable){
    *objects[object].y += width;
  }
  // if not reachable, other agent pushes it towards us
  else {
    *objects[object].y -= width;
  }

  // update relative distances to objects, and their reachability
  updateRelativeDistances();

  // no reward
  return 0;

}

// apply this discrete action to the domain
float SpeechArmTable::apply(int origAction) {

  // add noise to this action
  int action = origAction;
  if (noisy)
    action = applyNoise(origAction);

  // north, south, east, west actions
  if (action == NORTH && hand_y < width-1)
    hand_y++;
  if (action == SOUTH && hand_y > 0)
    hand_y--;
  if (action == EAST && hand_x < width-1)
    hand_x++;
  if (action == WEST && hand_x > 0)
    hand_x--;

  // pickup action
  if (action == PICKUP && hand_obj == -1){
    for (int i = 0; i < nobjects; i++){
      if (*objects[i].rel_x == 0 && *objects[i].rel_y == 0 && hand_obj == -1){
        hand_obj = i;
        break;
      }
    }
  }

  // putdown action
  if (action == PUTDOWN && hand_obj != -1){
    for (int i = 0; i < nobjects; i++){
      if (hand_obj == i){
        hand_obj = -1;
        break;
      }
    }
  }

  // if we called speech action this way (just for testing)
  if (action >= INIT_NUMACTIONS){
    int sa = action - INIT_NUMACTIONS;
    if (SPDEBUG) cout << "Chose speech action " << savedSpeeches[sa].f1 << ", " << savedSpeeches[sa].f2 << endl;
    applySpeechAction(savedSpeeches[sa].f1, savedSpeeches[sa].f2);
  }

  updateRelativeDistances();

  // no reward
  return 0;

}

/** Update relative distances of objects to hand, and whether objects are in reachable space or not. */
void SpeechArmTable::updateRelativeDistances() {

  for (int i = 0; i < nobjects; i++){
    // update object location if its in our hand
    if (hand_obj == i){
      *objects[i].x = hand_x;
      *objects[i].y = hand_y;
    }

    // update relative distances of objects
    *objects[i].rel_x = *objects[i].x - hand_x;
    *objects[i].rel_y = *objects[i].y - hand_y;

    // update if they're in reachable space or not
    if (*objects[i].y < width)
      *objects[i].reachable = true;
    else
      *objects[i].reachable = false;
  }
}

bool SpeechArmTable::terminal() const {
  return false;
}

void SpeechArmTable::reset() {
  if (SPDEBUG) cout << "reset" << endl << flush;

  // start objects and hand in random locations
  hand_x = rng.uniformDiscrete(0, width-1);
  hand_y = rng.uniformDiscrete(0, width-1);
  hand_obj = -1;

  for (int i = 0; i < nobjects; i++){
    if (SPDEBUG) cout << "Set position of obj " << i << endl << flush;
    // select random position
    // and make sure it doesnt conflict with others
    bool positionConflict = true;

    while (positionConflict){
      // select new random position
      *objects[i].x = rng.uniformDiscrete(0, width-1);
      *objects[i].y = rng.uniformDiscrete(0, height-1);
      positionConflict = false;

      // and check with other objects
      if (hand_x == *objects[i].x && hand_y == *objects[i].y){
        positionConflict = true;
        if (SPDEBUG) cout << "conflict with position of hand" << endl << flush;
      }
      for (int j = 0; j < i && !positionConflict; j++){
        if (*objects[j].x == *objects[i].x && *objects[j].y == *objects[i].y){
          positionConflict = true;
          if (SPDEBUG) 
            cout << "conflict with position of obj " << j << endl << flush;
          break;
        }
      }
    } // conflict loop
  } // object loop

  if (SPDEBUG) cout << "Object locations complete" << endl << flush;

  updateRelativeDistances();

  if (SPDEBUG) print_map();

}

void SpeechArmTable::setDefaultSpeeches(){
  // try random speeches in the 2 to 7 range in each dimension
  for (int i = 0; i < 20; i++){
    addSpeechAction(rng.uniform(2,7), rng.uniform(2,7));
  }
}

void SpeechArmTable::setOptimalSpeeches(){
  for (unsigned i = 0; i < objects.size(); i++){
    addSpeechAction(objects[i].sound.f1, objects[i].sound.f2);
  }
}

int SpeechArmTable::getNumActions() {
  if (SPDEBUG) cout << "Return number of actions: " << numactions << endl;
  return numactions; //num_actions;
}


void SpeechArmTable::print_map() const{
  // print map in rows, showing agent hand and all objects

  cout << "\nSpeechArmTable" << endl;

  // for each row
  for (int j = height-1; j >= 0; --j){
    // for each column
    for (int i = 0; i < width; i++){
      bool foundObj = false;
      if (hand_y == j && hand_x == i){
        foundObj = true;
        cout << "H";
      }
      for (int k = 0; k < nobjects && !foundObj; k++){
        if (*objects[k].x == i && *objects[k].y == j){
          cout << k;
          foundObj = true;
          break;
        }
      }
      if (!foundObj) cout << ".";
    } // last col of row
    cout << endl;
  } // last row

  cout << "At " << hand_x << ", " << hand_y << ", hand_obj? " << hand_obj << endl;
  for (int k = 0; k < nobjects; k++){
    cout << "Object " << k << " at " << *objects[k].x << ", " << *objects[k].y << endl;
    cout << " Relative location " << *objects[k].rel_x << ", " << *objects[k].rel_y << endl;
    cout << " Reachable: " << *objects[k].reachable << endl;
  }
}



void SpeechArmTable::getMinMaxFeatures(std::vector<float> *minFeat,
                                       std::vector<float> *maxFeat){

  minFeat->resize(s.size(), 0.0);
  maxFeat->resize(s.size(), 1.0);

  (*maxFeat)[0] = width-1; // x
  (*maxFeat)[1] = height-1; // y: not really, but we dont know its not reachable
  (*maxFeat)[2] = nobjects-1; // hand_obj
  (*minFeat)[2] = -1;

  for (int i = 0; i < nobjects; i++){
    int startIndex = 3+i*5;
    (*maxFeat)[startIndex] = width-1; // x
    (*maxFeat)[startIndex+1] = height-1; // y
    (*minFeat)[startIndex+2] = -(width-1); // rel x
    (*minFeat)[startIndex+3] = -(height-1); // rel y
    (*maxFeat)[startIndex+2] = width-1; // rel x
    (*maxFeat)[startIndex+3] = height-1; // rel y
    (*maxFeat)[startIndex+4] = 1.0; // reachable
  }

}

void SpeechArmTable::getMinMaxReward(float *minR, float *maxR){

  // return a reward range
  // I think if range is 0, I'd get a divide by 0 error somewhere
  *minR = -1.0;
  *maxR = 10.0;

}

