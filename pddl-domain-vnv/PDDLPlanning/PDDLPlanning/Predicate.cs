using System.Collections.Generic;
namespace PDDLPlanning
{
    //Represents the PDDL predicates
    public class Predicate
    {

        public string Name;
        public int nrOfArgs;
        //Represents the arguments of the predicates. i.e -- (at ?b ?r)
        //This does not have to be set in for example the domain file
        public List<string> args;
        public Predicate(string name,int nrOfArgs)
        {
            Name = name;
            this.nrOfArgs = nrOfArgs;
            args = new List<string>();
        }
        public Predicate(Predicate p)
        {
            Name = p.Name;
            nrOfArgs = p.nrOfArgs;
            args = new List<string>();
            foreach (string s in p.args)
                args.Add(s);
        }
        public Predicate(string name,int nrOfArgs, List<string> args)
        {
            Name = name;
            this.nrOfArgs = nrOfArgs;
            this.args = args;
        }
        public void AddArg(string s)
        {
            args.Add(s);
        }
        public void ClearArgs()
        {
            args.Clear();
        }
        //This is the method for determining if to predicates are the same. 
        //This is used when checking preconditions, which is
        //represented as a list of predicates
        public bool IsEqual(Predicate p)
        {
            if (this.args.Count < nrOfArgs || p.args.Count < nrOfArgs)
                return false;
            if(this.Name == p.Name && this.nrOfArgs == p.nrOfArgs)
            {
                for (int i = 0; i < this.nrOfArgs; i++)
                    if (this.args[i] != p.args[i])
                        return false;
                return true;
            }
            return false;
        }
    }
}