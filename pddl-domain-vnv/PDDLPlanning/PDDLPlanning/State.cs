using System;
using System.Collections.Generic;

namespace PDDLPlanning
{

    //An object of State class represents the nodes in the tree

    internal class State
    {

        public struct Transition
        {
            public Action a;
            public State s;
        }
        public List<Action> Actions { get; set; }
        public List<Transition> Transitions;
        public List<Predicate> StateInfo; //Represents all true predicates in this state
        public int StateID { get; set; } // Unique ID for each node/state in the state tree
      
        
        public State(int id,List<Predicate> stateInfo)
        {
            StateID = id;
            Actions = new List<Action>();
            Transitions = new List<Transition>();
            StateInfo = stateInfo;
        }
    }
}