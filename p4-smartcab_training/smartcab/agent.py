import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator

n_trials = 100 

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
        self.state = None
        self.q_table = {}
        self.frame = 0
        self.last_state_tuple = None
        self.last_action = None
        self.alpha = .5

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required
        self.state = None
        self.last_state_tuple = None
        self.last_action = None


    def update(self, t):

        self.frame += 1

        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  #e.g. left, forward, right
        deadline = self.env.get_deadline(self)

        # TODO: Update state
        env_state = self.env.sense(self) 
        state_tuple = (env_state['light'], self.next_waypoint)
        self.state = "Light: %s, Waypoint: %s" % state_tuple


    # TODO: Select action according to your policy
    # TODO: Learn policy based on state, action, reward

        #default to random action 
        action = random.choice([None, 'forward', 'left', 'right'])

        #look for action prescribed by q table 
        if state_tuple in self.q_table:

            #calculate epsilon 
            epsilon = 1-(((1./(n_trials*30.))*self.frame)+.5)
            if epsilon >= 1.:
                epsilon = .99
            print "EP: ", epsilon

            #randomly decide whether to act randomly, based on epsilon
            roll = random.random()
            if roll < epsilon:
                reward = self.env.act(self, action)
                self.q_table[state_tuple].append((action, reward))
            else:
                #act according to highest recorded q value for the state
                q_temp = 0
                for i in self.q_table[state_tuple]:
                    val = i[1]
                    if val > q_temp:
                        q_temp = val
                        action = i[0]
                reward = self.env.act(self, action)

        #if action not in q table:
        else:
            reward = self.env.act(self, action)

            #q value for the state is "initialized" during the simulation (i.e. when the state/action pair is first encountered).
            #It is initialized to the immediate reward. 
            self.q_table[state_tuple] = [(action, reward)]


        #update q value for last state 
        if self.last_state_tuple != None:
            c = 0
            for i in self.q_table[self.last_state_tuple]:
                if i[0] == self.last_action:
                    last_act = self.q_table[self.last_state_tuple][c][0]
                    last_val = self.q_table[self.last_state_tuple][c][1]
                    updated_val = (last_val * (1 - self.alpha)) + (self.alpha * reward)

                    self.q_table[self.last_state_tuple][c] = (last_act, updated_val)

        #save last state_tuple, so it can be updated during the next iteration
        self.last_state_tuple = state_tuple
        self.last_action = action 

        print "FRAME: ", self.frame
        print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, env_state, action, reward)  # [debug]


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # set agent to track

    # Now simulate it
    sim = Simulator(e, update_delay=0.00001)  # reduce update_delay to speed up simulation
    sim.run(n_trials=n_trials)  # press Esc or close pygame window to quit


if __name__ == '__main__':
    run()
