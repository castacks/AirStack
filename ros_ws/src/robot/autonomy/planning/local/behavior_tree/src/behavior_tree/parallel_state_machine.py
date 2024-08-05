#!/usr/bin/python
import rospy
from std_msgs.msg import String
import yaml
import sys
import collections

# Comments:
# 1. State names must be unique across all state machines
#    - Would be confusing to interpret if you were reading what state the robot is in
#    - Graphviz doesn't seem to support having the same name in different subgraphs
# 2. Can transition from state -> state (within a state machine), from state -> state machine and from state machine -> state machine, but not from a state machine -> state

class State:
    def __init__(self, name, state_machine):
        self.name = name
        self.state_machine = state_machine
        self.transitions = collections.OrderedDict()

    def add_transition(self, transition, state):
        self.transitions[transition] = state

    def transition(self, trans):
        if trans in self.transitions.keys():
            print(self.name + ': transitioning to ' + self.transitions[trans].name)
            return self.transitions[trans]
        else:
            print(self.name + ': invalid transition ' + trans)
            return None

    def print_structure(self):
        print('\t' + self.name)
        for transition, state_or_state_machine in self.transitions.iteritems():
            print('\t\t' + transition + ': this -> ' + state_or_state_machine.name)

class StateMachine:
    def __init__(self, name):
        self.name = name
        self.states = collections.OrderedDict()
        self.transitions = collections.OrderedDict()
        self.current_state = None

    def add_state(self, state):
        self.states[state.name] = state
        if self.current_state == None:
            self.current_state = state

    def add_transition(self, transition, state_machine):
        self.transitions[transition] = state_machine

    def transition(self, trans):
        if trans in self.transitions.keys():
            print(self.name + ': transitioning to new state machine ' + self.transitions[trans].name)
            return self.transitions[trans]
        else:
            new_state_or_state_machine = self.current_state.transition(trans)
            print(new_state_or_state_machine.name, isinstance(new_state_or_state_machine, State))
            if isinstance(new_state_or_state_machine, State):
                self.current_state = new_state_or_state_machine
                print(self.current_state.name)
            return new_state_or_state_machine

    def print_structure(self):
        print(self.name)
        for transition, state_machine in self.transitions.iteritems():
            print('\t' + transition + ': this -> ' + state_machine.name)
        for state_name, state in self.states.iteritems():
            state.print_structure()
    
class ParallelStateMachine:
    def __init__(self, config_filename):
        self.states = collections.OrderedDict()
        self.state_machines = collections.OrderedDict()
        self.current_state_machine = None
        
        # parse the config file
        config = yaml.load(open(config_filename, 'r').read())

        # Do a first pass to initialize all of the state machines and states
        for state_machine_dict in config['state_machines']:
            name = state_machine_dict.keys()[0]
            state_machine = StateMachine(name)
            self.state_machines[name] = state_machine

            for state_name in state_machine_dict[name]['states']: # TODO: check that len(states) > 0
                state = State(state_name, state_machine) # TODO: check that name is unique
                self.states[state_name] = state
                state_machine.add_state(state)
        # TODO: check that state names and state machines names are unique

        # Do a second pass to build the structure of the state machines
        # TODO: check that transitions are valid according to rules above
        for state_machine_dict in config['state_machines']:
            name = state_machine_dict.keys()[0]
            state_machine = self.state_machines[name]
            
            for transition in state_machine_dict[name]['transitions']:
                transition_dict = state_machine_dict[name]['transitions'][transition]
                to_name = transition_dict['to']
                
                if to_name in self.states.keys():
                    to = self.states[to_name]
                elif to_name in self.state_machines.keys():
                    to = self.state_machines[to_name]
                
                if 'from' in transition_dict.keys():
                    from_states = transition_dict['from']
                    if type(from_states) == str:
                        from_states = [from_states]
                    for from_state in from_states:
                        self.states[from_state].add_transition(transition, to)
                else:
                    state_machine.add_transition(transition, to)

        self.current_state_machine = self.state_machines[self.state_machines.keys()[0]]
        
        self.print_structure()
        
    def print_structure(self):
        for state_machine_name, state_machine in self.state_machines.iteritems():
            state_machine.print_structure()
        

    def transition(self, trans):
        state_or_state_machine = self.current_state_machine.transition(trans)

        if isinstance(state_or_state_machine, StateMachine):
            self.current_state_machine = state_or_state_machine


def transition_callback(msg):
    parallel_state_machine.transition(msg.data)

def timer_callback(x):
    state_pub.publish(parallel_state_machine.current_state_machine.current_state.name)

if __name__ == '__main__':
    #parallel_state_machine = ParallelStateMachine(sys.argv[1])
    #exit()
    rospy.init_node('state_machine', anonymous=True)
    
    config_filename = rospy.get_param('~config', '')
    state_pub_rate = 1./rospy.get_param('~state_pub_rate', 10.)
    
    parallel_state_machine = ParallelStateMachine(config_filename)
    
    
    
    transition_sub = rospy.Subscriber('/transition', String, transition_callback)
    state_pub = rospy.Publisher('/state', String, queue_size=10)
    state_pub_timer = rospy.Timer(rospy.Duration(state_pub_rate), timer_callback)
    
    rospy.spin()
