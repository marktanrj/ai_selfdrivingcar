import random
import math
from environment import Agent, Environment
from simulator import Simulator
import sys
from searchUtils import searchUtils
import os

class SearchAgent(Agent):
    """ An agent that drives in the Smartcab world.
        This is the object you will be modifying. """

    def __init__(self, env, location=None):
        # Set the agent in the evironment
        super(SearchAgent, self).__init__(env)
        self.valid_actions = self.env.valid_actions  # The set of valid actions
        self.action_sequence = []
        self.searchutil = searchUtils(env)

    def choose_action(self):
        """ The choose_action function is called when the agent is asked to choose
            which action to take next"""

        # Set the agent state and default action
        action = None
        if len(self.action_sequence) >= 1:
            action = self.action_sequence[0]
        if len(self.action_sequence) >= 2:
            self.action_sequence = self.action_sequence[1:]
        else:
            self.action_sequence = []
        return action

    def drive(self, goalstates, inputs):
        """Write your algorithm for self driving car"""

        results = []
        startState = self.state
        for goalState in goalstates:
            goalReached, path = self.Best_First_Search(startState, goalState)
            results.append({"goalReached": goalReached, "path": path})

        onlyGoalReachedResults = []
        for result in results:
            if result["goalReached"]:
                onlyGoalReachedResults.append(result)

        best = {}
        if len(onlyGoalReachedResults) > 0:
            best = min(onlyGoalReachedResults, key=lambda x: len(x["path"]))
        else:
            best = min(results, key=lambda x: len(x["path"]))

        return best["path"]

    def Best_First_Search(self, start, goal):
        closedSet = []
        openSet = [start]
        start["fScore"] = self.heuristic_cost_estimate(start, goal)
        current = {}

        while len(openSet) > 0:
            # get node with lowest fScore
            current = min(openSet, key=lambda x: x["fScore"])

            # return if goal reached
            if current["location"] == goal["location"]:
                return True, self.reconstruct_path(current)

            openSet.remove(current)
            closedSet.append(current)

            # loop all actions and get next possible nodes
            for action in self.valid_actions:
                neighbor = self.env.applyAction(self, current, action)
                if neighbor == current:
                    continue
                
                # when vertically nearer to goal, don't go opposite direction of goal
                startX = neighbor["location"][0]
                goalX = goal["location"][0]
                startY = neighbor["location"][1]
                if startY > 15:
                    if action == "right" and goalX - startX < 0:
                        continue
                    elif action == "left" and goalX - startX > 0:
                        continue

                # add neighbour to list if not explored
                if not self.searchutil.isPresentStateInList(neighbor, openSet) and \
                    not self.searchutil.isPresentStateInList(neighbor, closedSet):
                    openSet.append(neighbor)

                # calculate heuristic cost of neigbour
                neighbor["fScore"] = self.heuristic_cost_estimate(neighbor, goal)

        if current["location"] == goal["location"]:
            return True, self.reconstruct_path(current)
        else:
            return False, self.reconstruct_path(current)

    def heuristic_cost_estimate(self, start, goal):
        startX = start["location"][0]
        goalX = goal["location"][0]
        startY = start["location"][1]
        goalY = goal["location"][1]

        # when vertically nearer to goal, increase cost for nodes horizontally further from goal
        if startY > 15:
            return abs(goalY - startY) * 2 + abs(goalX - startX) * max(1, startX)
        else:
            return abs(goalY - startY) * 2 + abs(goalX - startX)

    def reconstruct_path(self, state):
        total_path = []
        if "previous" in state:
            total_path = self.searchutil.retrieveActionSequenceFromState(state)
        return total_path

    def update(self):
        """ The update function is called when a time step is completed in the
            environment for a given trial. This function will build the agent
            state, choose an action, receive a reward, and learn if enabled. """
        startstate = self.state
        goalstates = self.env.getGoalStates()
        inputs = self.env.sense(self)
        self.action_sequence = self.drive(goalstates, inputs)
        action = self.choose_action()  # Choose an action
        self.state = self.env.act(self, action)
        return


def run(filename):
    """ Driving function for running the simulation.
        Press ESC to close the simulation, or [SPACE] to pause the simulation. """

    env = Environment(config_file=filename, fixmovement=False)

    agent = env.create_agent(SearchAgent)
    env.set_primary_agent(agent)

    ##############
    # Create the simulation
    # Flags:
    #   update_delay - continuous time (in seconds) between actions, default is 2.0 seconds
    #   display      - set to False to disable the GUI if PyGame is enabled
    sim = Simulator(env, update_delay=0.2)

    ##############
    # Run the simulator
    ##############
    sim.run()    

if __name__ == '__main__':
    run(sys.argv[1])
