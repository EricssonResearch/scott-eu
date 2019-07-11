using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace PDDLPlanning
{
    /*TDOD: Update current state and state tree after action (CHECK)
            Call action (CHECK)
            Calculate current possible actions (CHECK)
            Create new states (CHECK)

    */
    class StateManager
    {
        public State rootState;
        Problem p;
        List<State> stateList;
        List<State> stateHistory;
        State currentState;

        public StateManager(Problem p)
        {
            this.p = p;
            stateList = new List<State>();
            stateHistory = new List<State>();
            currentState = new State(0, p.initialState);
            stateList.Add(currentState);
            CalculatePossibleActions();
            rootState = currentState;
        }

        public List<Action> GetActions()
        {
            return currentState.Actions;
        }
        public List<State> GetStates()
        {
            return stateList;
        }
        public State GetCurrentState()
        {
            return currentState;
        }

        public void PerformAction(int index)
        {
            List<Predicate> newStateInfo = new List<Predicate>();
            newStateInfo = currentState.Actions[index].PerformAction(currentState.StateInfo);
            bool stateFound = true;
            foreach (State s in stateList)
            {
                foreach (Predicate p in newStateInfo)
                {
                    if (!s.StateInfo.Exists(pred => pred.IsEqual(p)))
                    {
                        stateFound = false;
                        break;
                    }
                }
                if (stateFound)
                {
                    //The new state should be the same as State s
                    stateHistory.Add(currentState);
                    currentState = s;
                    break;
                }
            }
            //IF STATE is not found create a new state and set current to that one
            if (!stateFound)
            {
                stateHistory.Add(currentState);
                currentState = new State(stateList.Count, newStateInfo);
                stateList.Add(currentState);
                CalculatePossibleActions();
            }
        }
        public void PerformAction(Action a)
        {
            List<Predicate> newStateInfo = new List<Predicate>();
            newStateInfo = a.PerformAction(currentState.StateInfo);
            bool stateFound = true;
            foreach (State s in stateList)
            {
                stateFound = (s.StateInfo.Count == newStateInfo.Count) ? true : false;
                foreach (Predicate p in newStateInfo)
                {
                    if (!s.StateInfo.Exists(pred => pred.IsEqual(p)))
                    {
                        stateFound = false;
                        break;
                    }
                }
                if (stateFound)
                {
                    stateHistory.Add(currentState);
                    //The new state should be the same as State s
                    currentState = s;
                    break;
                }
            }
            //IF STATE is not found create a new state and set current to that one
            if (!stateFound)
            {
                stateHistory.Add(currentState);
                currentState = new State(stateList.Count, newStateInfo);
                stateList.Add(currentState);
                CalculatePossibleActions();
            }
        }
        public void CalculatePossibleActions()
        {

            //TODO: Send arguments to action with a.SendArguments(), need either try all objects or know which to send
            foreach (Action a in p.domain.Actions)
            {
                Action tmpAction = new Action(a);
                List<Predicate> predicates = new List<Predicate>();
                List<string> previousPreds = new List<string>();
                Dictionary<string, List<string>> argDict = new Dictionary<string, List<string>>();

                for (int i = 0; i < a.nrOfParams; i++)
                    argDict.Add(a.ActionParameters[i], new List<string>());
                foreach (Predicate p in tmpAction.precondition)
                {
                    predicates = (currentState.StateInfo.FindAll(pred => p.Name == pred.Name));
                    previousPreds.Add(p.Name);
                    for (int i = 0; i < predicates.Count; i++)
                        for (int j = 0; j < p.nrOfArgs; j++)
                            if (!argDict[p.args[j]].Contains(predicates[i].args[j]))
                                argDict[p.args[j]].Add(predicates[i].args[j]);
                }
                //argList contains all possible arguments for the action for each paramater
                List<List<string>> argList = new List<List<string>>(a.nrOfParams);
                for (int i = 0; i < a.nrOfParams; i++)
                    argList.Add(new List<string>(argDict[a.ActionParameters[i]]));
                List<string> current = new List<string>(a.nrOfParams);
                for (int i = 0; i < a.nrOfParams; i++)
                    current.Add("");
                recCalculateParams(argList, 0, current, a);
            }
        }
        private void recCalculateParams(List<List<string>> lists, int level, List<string> current, Action a)
        {
            if (level == lists.Count)
            {
                Action tmp = new Action(a);
                tmp.SendArguments(current);
                if (tmp.EvaluatePrecondition(currentState.StateInfo))
                {
                    //TODO: add transition containing the action and the new state
                    currentState.Actions.Add(tmp);
                    AddTransition(currentState, tmp);
                }
            }
            else
            {
                foreach (string s in lists[level])
                {
                    current[level] = s;
                    recCalculateParams(lists, level + 1, current, a);
                }
            }
        }

        private void AddTransition(State currentNode, Action tmp)
        {
            State.Transition transition = new State.Transition();
            transition.a = tmp;
            List<Predicate> newStateInfo = new List<Predicate>();
            newStateInfo = tmp.PerformAction(currentNode.StateInfo);
            bool stateFound = true;
            foreach (State s in stateList)
            {
                stateFound = (s.StateInfo.Count == newStateInfo.Count) ? true : false;
                foreach (Predicate p in newStateInfo)
                {
                    if (!s.StateInfo.Exists(pred => pred.IsEqual(p)))
                    {
                        stateFound = false;
                        break;
                    }
                }
                if (stateFound)
                {
                    //The new state should be the same as State s
                    transition.s = s;
                    currentNode.Transitions.Add(transition);
                    break;
                }
            }
            //IF STATE is not found create a new state
            if (!stateFound)
            {
                transition.s = new State(stateList.Count, newStateInfo);
                stateList.Add(transition.s);
                currentNode.Transitions.Add(transition);
                currentState = transition.s;
                CalculatePossibleActions();
                currentState = currentNode;
            }
        }

        public void stepBack()
        {
            if (stateHistory.Count > 0)
            {
                currentState = stateHistory.Last();
                stateHistory.RemoveAt(stateHistory.Count - 1);
            }
        }
        public string GenerateGraphString(State s,Action selectedAction, List<State> previousStates = null)
        {
            previousStates = (previousStates == null) ? new List<State>() : previousStates;
            if (previousStates.Contains(s))
                return "";
            previousStates.Add(s);
            StringBuilder builder = new StringBuilder("");
                bool isGoalState = true;
                foreach (var pred in p.goalState)
                    foreach (var pred2 in s.StateInfo)
                    {
                        if (pred.IsEqual(pred2))
                        {
                            isGoalState = true;
                            break;
                        }
                        else
                            isGoalState = false;
                    }
                        
                if (isGoalState)
                {
                    builder.Append(s.StateID + "[color=Green];\n  ");
                }
            foreach (State.Transition t in s.Transitions)
            {
                builder.Append(s.StateID);
                builder.Append("->");
                builder.Append(t.s.StateID);
                if (s == currentState)
                {
                    if (t.a == selectedAction)
                        builder.Append("[color = blue style=bold]");
                }
                builder.Append("\n  ");
            }
            foreach (State.Transition t in s.Transitions)
            {
                builder.Append(GenerateGraphString(t.s, selectedAction, previousStates));
            }
            return builder.ToString();

        }
    }
}
