using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PDDLPlanning
{
    //Test comment #2

    //Represents the actions in the domain file
    class Action
    {
        public string Name { get; set; }
        public int nrOfParams;
        public List<string> ActionParameters;
        public List<Predicate> positiveEffects; //Represents the AND Objects -- and (carry ?obj ?gripper)
        public List<Predicate> negativeEffects; //Represents the NOT Objects -- (not (at ?obj ?room))
        public List<Predicate> precondition; //Preconditions for Action 
        public List<string> actualParameters;

        public Action(string name,int nrOfParams)
        {
            Name = name;
            this.nrOfParams = nrOfParams;
            positiveEffects = new List<Predicate>();
            negativeEffects = new List<Predicate>();
            precondition = new List<Predicate>();
            ActionParameters = new List<string>();
            actualParameters = new List<string>();
        }
        public Action(Action a)
        {
            Name = a.Name;
            nrOfParams = a.nrOfParams;
            positiveEffects = new List<Predicate>();
            negativeEffects = new List<Predicate>();
            foreach (Predicate p in a.positiveEffects)
                positiveEffects.Add(p);
            foreach (Predicate p in a.negativeEffects)
                negativeEffects.Add(p);
            precondition = new List<Predicate>();
            foreach (Predicate p in a.precondition)
                this.AddPrecondition(p);
            ActionParameters = new List<string>(a.ActionParameters);
            actualParameters = new List<string>(a.actualParameters);
        }
        public void SendArguments(List<string> args)
        {
            actualParameters.Clear();
            actualParameters = new List<string>(args);
        }
        public void SendArgument(string key,string value)
        {
            for (int i = 0; i < nrOfParams; i++)
                if (ActionParameters[i] == key)
                    actualParameters[i] = value;
        }
        //This function evaluates if preconditions are matched
        public bool EvaluatePrecondition(List<Predicate> stateInfo)
        {
            List<Predicate> tmpList = new List<Predicate>();
            foreach (Predicate p in precondition)
                tmpList.Add(new Predicate(p));
            foreach (Predicate p in tmpList)
                for (int i = 0; i < p.args.Count; i++)
                {
                    int index = ActionParameters.FindIndex(str => p.args[i] == str);
                    p.args[i] = actualParameters[index];
                }
             
            foreach (Predicate p in tmpList)
            {
                bool exists = false;
                foreach (Predicate si in stateInfo)
                {
                    if (p.IsEqual(si))
                    {
                        exists = true;
                        break;
                    }
                }
                if (exists == false)
                    return false;
            }
            return true;
        }
        public void AddPrecondition(Predicate p)
        {
            precondition.Add(new Predicate(p));
        }
        //Performs the action by adding or removing predicates
        //The argument stateInfo is a list containing all current true predicates
        //The return value of this method is a new list of true predicates
      
        public List<Predicate> PerformAction(List<Predicate> stateInfo)
        {
            List<Predicate> retList = new List<Predicate>(stateInfo);
            List<Predicate> tmpList = negativeEffects.ConvertAll(pred => new Predicate(pred));
            //TODO: Make this into a function #1
            foreach (Predicate p in tmpList)
            {
                for (int i = 0; i < p.args.Count; i++)
                {
                    int index = ActionParameters.FindIndex(str => str == p.args[i]);
                    p.args[i] = actualParameters[index];
                }
            }

            //Removes predicates, see negativeffects above:
            foreach (Predicate p  in tmpList)
            {
                retList.RemoveAll(Pred => p.IsEqual(Pred));
            }
            //TODO: Make this into a function #1
            tmpList = positiveEffects.ConvertAll(pred => new Predicate(pred));
            
            foreach (Predicate p in tmpList)
            {
                for (int i = 0; i < p.args.Count; i++)
                {
                    int index = ActionParameters.FindIndex(str => str == p.args[i]);
                    p.args[i] = actualParameters[index];
                }
            }
            //Add predicates, see positiveeffects above:
            foreach (Predicate p in tmpList)
            {
                retList.RemoveAll(Pred => p.IsEqual(Pred));
                retList.Add(p);
            }
            return retList;
        }
    }
}
