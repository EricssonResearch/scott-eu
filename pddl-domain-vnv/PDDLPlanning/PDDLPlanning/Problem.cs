using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PDDLPlanning
{

    //Represents the PDDL problem file
    class Problem
    {

        
        public Domain domain;
        public List<string> Objects;
        public List<Predicate> initialState;
        public List<Predicate> goalState;
        public Problem(Domain d)
        {
            domain = d;
            Objects = new List<string>();
            initialState = new List<Predicate>();
            goalState = new List<Predicate>();
        }

        //Adding objects i.e (:objects rooma roomb ball1 ball2 left right)
        public void AddObj(string name)
        {
            Objects.Add(name);
        }
        //Creates the first initial node in the state tree
        public void AddInitState(Predicate p)
        {
            initialState.Add(p);
        }
        //Creates the first initial node in the state tree
        public void AddGoalState(Predicate p)
        {
            goalState.Add(p);
        }
    }
}
