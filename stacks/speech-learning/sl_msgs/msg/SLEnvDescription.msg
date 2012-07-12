# Message that contains details about an RL enviroment/task
int32 num_actions
int32 num_states
int32 num_speeches
int32 num_state_features

#Optional information to describe the range of each state feature
float32[] min_state_range
float32[] max_state_range

#Info needed for r-max some other methods
float32 max_reward
float32 reward_range

int32 width
int32 num_objects
bool stochastic
bool episodic
string title
