#include "taskplanner.h"


TaskPlanner::TaskPlanner(vector<int>& objects_info, int table_index) {
	this->objects_ = objects_info;
	this->table_index_ = table_index; 
}

void TaskPlanner::printStateInfo(State& state) {
	for (auto it = state.on.begin(); it != state.on.end(); ++it) {
		cout << it->first << " is on " << it->second << ";  ";
	}
	for (auto it = state.clear.begin(); it != state.clear.end(); ++it) {
		cout << *it << " is clear. "; 
	}
	cout << endl;
}

int TaskPlanner::computeHeuristic(State& state) {

	int num_mismatch = 0;

	for (auto it = this->goal_state_.on.begin(); it != this->goal_state_.on.end(); ++it) {
		if (state.on[it->first] != it->second) num_mismatch++;
	}

	for (auto it = this->goal_state_.clear.begin(); it != this->goal_state_.clear.end(); ++it) {
		if (state.clear.find(*it) == state.clear.end()) num_mismatch++;
	}
	return (this->h_weight_) * num_mismatch;
}

void TaskPlanner::setGoalState(int** goal_on, int goal_on_length,
						int* goal_clear, int goal_clear_length) {

	// insert new element into map
    for (int i = 0; i < goal_on_length; i++) 
    	this->goal_state_.on[goal_on[i][0]] = goal_on[i][1];
    // insert new element into set
    for (int i = 0; i < goal_clear_length; i++) 
    	this->goal_state_.clear.insert(goal_clear[i]);

    // cout << "Goal state info: " << endl;
    // printStateInfo(this->goal_state_);
}

void TaskPlanner::setInitState(int** start_on, int start_on_length, 
	        			int* start_clear, int start_clear_length) {

	// insert new element into map
    for (int i = 0; i < start_on_length; i++) 
    	this->start_state_.on[start_on[i][0]] = start_on[i][1];
    // insert new element into set
    for (int i = 0; i < start_clear_length; i++) 
    	this->start_state_.clear.insert(start_clear[i]);   

    // -1 as INVALID
    this->start_state_.prior_action = vector<int>(3,-1); 
    this->start_state_.cost = 0;
    this->start_state_.value = computeHeuristic(this->start_state_);

    this->start_state_.id = 0;
    this->start_state_.parent_id = -1;

    // cout << "Start state info: " << endl;
    // printStateInfo(this->start_state_);
}

// void TaskPlanner::setAction(vector<vector<State>>& actions);

void TaskPlanner::setAction(int move_action_ind, int move_to_table_action_ind) {
	this->move_action_ind_ = move_action_ind;
	this->move_to_table_action_ind_ = move_to_table_action_ind;
}

bool TaskPlanner::sameState(State& state1, State& state2) {
	return state1.on == state2.on && state1.clear == state2.clear;
}


// check whether x can be moved from y to z
bool TaskPlanner::canMoveXFromYToZ(State& curr_state, int x_ind, int y_ind, int z_ind) {
	return  x_ind != this->table_index_ &&
			z_ind != this->table_index_ &&
			curr_state.on[x_ind] == y_ind && 
			(curr_state.clear.find(x_ind) != curr_state.clear.end()) &&
			(curr_state.clear.find(z_ind) != curr_state.clear.end()) &&
			//this->objects_[x_ind] == 0 &&
			this->objects_[z_ind] == 0 &&
			x_ind != z_ind;
}

// move x from y to z
State TaskPlanner::moveXFromYToZ(State& curr_state, int x_ind, int y_ind, int z_ind) {
	// assert (canMoveXFromYToZ);

	// copy over the current state first
	State next_state = curr_state;
	next_state.on[x_ind] = z_ind;
	if (y_ind != this->table_index_) next_state.clear.insert(y_ind);
	// assert(next_state.on[x_ind] != y_ind);
	next_state.clear.erase(z_ind);

	// add prior action to state 
	next_state.prior_action = {x_ind, y_ind, z_ind};

	// specify cost and value
	next_state.cost = curr_state.cost + 1;
	next_state.value = next_state.cost + computeHeuristic(next_state);

	return next_state;
}

bool TaskPlanner::canMoveXFromYToTable(State& curr_state, int x_ind, int y_ind) {
	return  x_ind != this->table_index_ &&
			y_ind != this->table_index_ &&
			curr_state.on[x_ind] == y_ind &&
			curr_state.clear.find(x_ind) != curr_state.clear.end() &&
			//this->objects_[x_ind] == 0 && 
			this->objects_[y_ind] == 0;
}

State TaskPlanner::moveXFromYToTable(State& curr_state, int x_ind, int y_ind) {
	// assert (canMoveXFromYToTable)

	// copy over the current state first
	State next_state = curr_state;
	next_state.on[x_ind] = this->table_index_;
	next_state.clear.insert(y_ind);
	// assert(next_state.on[x_ind] != y_ind);

	// add prior action to state
	next_state.prior_action = {x_ind, y_ind, this->table_index_};

	// specify cost and value
	next_state.cost = curr_state.cost + 1;
	next_state.value = next_state.cost + computeHeuristic(next_state);

	return next_state;
}

// expend open list with successor states
void TaskPlanner::setSuccessorStates(State& curr_state) {

	State new_state;

	// cout << "Current state info: " << endl;
	// printStateInfo(curr_state);

	// explore possible new states
	for (auto i = curr_state.clear.begin(); i != curr_state.clear.end(); ++i) {
		for (auto j = curr_state.clear.begin(); j != curr_state.clear.end(); ++j) {

			if (*i == *j) continue;

			//try move action 
			if (canMoveXFromYToZ(curr_state, *i, curr_state.on[*i], *j)) {

				//cout << *i << " " << *j << endl;

				new_state = moveXFromYToZ(curr_state, *i, curr_state.on[*i], *j);

				bool visited = false; 
				// make sure new_state has not been visited (i.e. not in close list)
				for (auto it = this->close_list_.begin(); it != this->close_list_.end(); ++it) {
					if (sameState(new_state, *it)) {
						visited = true;
						break;
					}
				}
				// insert new states into open list
				if (!visited) {
					new_state.parent_id = curr_state.id;
					new_state.id = this->visited_states_.size();
					open_list_.push(new_state);
					this->visited_states_.push_back(new_state);

					//cout << "move action: " << endl;
					//printStateInfo(new_state);
				}				
			}
			
		}

		//try move_to_table action
		if (canMoveXFromYToTable(curr_state, *i, curr_state.on[*i])) {

			new_state = moveXFromYToTable(curr_state, *i, curr_state.on[*i]);

			bool visited = false; 
			// make sure new_state has not been visited (i.e. not in close list)
			for (auto it = this->close_list_.begin(); it != this->close_list_.end(); ++it) {
				if (sameState(new_state, *it)) {
					visited = true;
					break;
				}
			}
			// insert new states into open list
			if (!visited) {
				new_state.parent_id = curr_state.id;
				new_state.id = this->visited_states_.size();
				open_list_.push(new_state);
				this->visited_states_.push_back(new_state);

				//cout << "move to table action: " << endl;
				//printStateInfo(new_state);
			}
		}
	}

}

bool TaskPlanner::checkGoalReached(State& curr_state) {

	bool goal_reached = true;

	// check if all on() attributes are met 
	for (auto it = this->goal_state_.on.begin(); it != this->goal_state_.on.end(); ++it) {

		// if attribute specified in goal state does not match with that in current state...
		if (curr_state.on[it->first] != it->second) {
			goal_reached = false;
			break;
		}
	}

	// check if all clear() attributes are met
	for (auto it = this->goal_state_.clear.begin(); it != this->goal_state_.clear.end(); ++it) {

		// if attribute specified in goal state is missing in current state...
		if (curr_state.clear.find(*it) == curr_state.clear.end()) {
			goal_reached = false;
			break;
		}
	}

	return goal_reached;
}

// generate and return plan (using weighted A*)
void TaskPlanner::generatePlan(vector<vector<int>>& plan) {

	State curr_state = this->start_state_;
	this->open_list_.push(curr_state);
	this->visited_states_.push_back(curr_state);

	clock_t tStart = clock();
	cout << "generating plan..." << endl;

	while (!(this->open_list_.empty())) {
		// remove state with smallest value from open list 
		curr_state = this->open_list_.top();
		this->open_list_.pop();
		this->close_list_.push_back(curr_state);
		if (checkGoalReached(curr_state)) {
			//printStateInfo(curr_state);
			break;
		}

		setSuccessorStates(curr_state); 
	}

	if (this->open_list_.empty()) 
		cout << "WARNING: OPEN LIST EMPTY!!!" << endl;
	else 
		std::cout << "Time taken for planning : " << (double)(clock() - tStart)/CLOCKS_PER_SEC << " sec" << std::endl;

	while (curr_state.id > 0) {
		if (curr_state.prior_action[2] == this->table_index_)
			plan.push_back({this->move_to_table_action_ind_, 
				            curr_state.prior_action[0],
				        	curr_state.prior_action[1],
				        	curr_state.prior_action[2]});
		else 
			plan.push_back({this->move_action_ind_, 
							curr_state.prior_action[0],
							curr_state.prior_action[1],
							curr_state.prior_action[2]});

		curr_state = (this->visited_states_)[curr_state.parent_id];
	}

	assert(sameState(curr_state, this->start_state_));

}
