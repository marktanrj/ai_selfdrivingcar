import random
import math
from environment import Agent, Environment
from simulator import Simulator
import sys
from searchUtils import searchUtils


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
            goalReached, path = self.A_Star(startState, goalState)
            results.append({"goalReached": goalReached, "path": path})
            # print("LOC:",  startState["location"],  "GOAL:", goalState["location"])

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

    def A_Star(self, start, goal):
        closedSet = []
        openSet = [start]
        start["gScore"] = 0
        start["fScore"] = self.heuristic_cost_estimate(start, goal)
        current = {}

        while len(openSet) > 0:
            current = min(openSet, key=lambda x: x["fScore"])

            if current["location"] == goal["location"]:
                return True, self.reconstruct_path(current)

            openSet.remove(current)
            closedSet.append(current)

            for action in self.valid_actions:
                neighbor = self.env.applyAction(self, current, action)
                if neighbor == current:
                    continue

                tentative_gScore = current["gScore"] + self.distance_current_to_neighbor(current, neighbor)

                if not self.searchutil.isPresentStateInList(neighbor, openSet) and not self.searchutil.isPresentStateInList(neighbor, closedSet):
                    openSet.append(neighbor)

                neighbor["gScore"] = tentative_gScore
                neighbor["fScore"] = neighbor["gScore"] + self.heuristic_cost_estimate(neighbor, goal)

                # print(neighbor)

        if current["location"] == goal["location"]:
            return True, self.reconstruct_path(current)
        else:
            return False, self.reconstruct_path(current)

    def heuristic_cost_estimate(self, start, goal):
        startX = start["location"][0]
        goalX = goal["location"][0]
        startY = start["location"][1]
        goalY = goal["location"][1]
        return abs(goalY - startY) + abs(goalX - startX)

    def reconstruct_path(self, state):
        total_path = []
        if "previous" in state:
            total_path = self.searchutil.retrieveActionSequenceFromState(state)
        return total_path

    def distance_current_to_neighbor(self, current, neighbor):
        cX = current["location"][0]
        cY = current["location"][1]
        nX = neighbor["location"][0]
        nY = neighbor["location"][1]
        result = math.sqrt(((cX-nX)**2)+((cY-nY)**2))/4
        return result

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
    sim = Simulator(env, update_delay=2)

    ##############
    # Run the simulator
    ##############
    sim.run()


if __name__ == '__main__':
    run(sys.argv[1])
