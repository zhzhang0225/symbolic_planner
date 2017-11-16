#ifndef TASKPLANNER_H
#define TASKPLANNER_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <time.h>

#include <assert.h>

using namespace std;

struct State {
	// key = top, val = bottom
	unordered_map<int, int> on;
	unordered_set<int> clear;
	// vector with size of 3 (x,y,z)
	vector<int> prior_action;
	int cost;
	// value = cost + heuristic
	int value;
	// id of the state 
	int id;
	// id of parent state
	int parent_id;
};

struct compare
{
    bool operator()(const State& s1, const State& s2)
    {
        return s1.value > s2.value;
    } 
};


class TaskPlanner{
private:

	// vector size denotes # of objects
	// value denotes id of each type of object (block id = 0, triangle id = 1)
	// "objects" not including table !!
	vector<int> objects_;

	int table_index_;

	int move_action_ind_;

	int move_to_table_action_ind_;

	// open list for A* search
	priority_queue<State, vector<State>, compare> open_list_;
	// stored states in close list
	vector<State> close_list_;

	// full state information
	State start_state_;

	// NOTE: goal_state may not represent full state!
	State goal_state_;

	vector<State> visited_states_;

	// weight for Weighted A* algorithm
	int h_weight_ = 2;

public:

	TaskPlanner(vector<int>& objects_info, int table_index);

	void printStateInfo(State& state);

	int computeHeuristic(State& state);

	// convert to State and store as private variable  
	void setGoalState(int** goal_on, int goal_on_length,
					int* goal_clear, int goal_clear_length);

	// convert to State and add to open list 
	void setInitState(int** start_on, int start_on_length, 
		    	    int* start_clear, int start_clear_length);

	// void setAction(vector<vector<State>>& actions);

	void setAction(int move_action_ind, int move_to_table_action_ind);

	bool sameState(State& state1, State& state2);

	bool canMoveXFromYToZ(State& curr_state, int x_ind, int y_ind, int z_ind);

	State moveXFromYToZ(State& curr_state, int x_ind, int y_ind, int z_ind);

	bool canMoveXFromYToTable(State& curr_state, int x_ind, int y_ind);

	State moveXFromYToTable(State& curr_state, int x_ind, int y_ind);

	// expend open list with successor states
	void setSuccessorStates(State& curr_state);

	bool checkGoalReached(State& curr_state);

	void generatePlan(vector<vector<int>>& plan);

};

#endif