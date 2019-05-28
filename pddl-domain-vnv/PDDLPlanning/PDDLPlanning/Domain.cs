using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PDDLPlanning
{

    //Defines domain definition; Actions and predicats. 
    //Effects and actions instructions are defined in Actions.cs
    class Domain
    {
        public List<Action> Actions;
        public List<Predicate> Predicates;
        public Domain(List<Action> acts, List<Predicate> preds)
        {
            Actions = acts;
            Predicates = preds;
        }
    }
}
