# Author: Thomas Wagenaar (t.wagenaar@student.tue.nl)
#
# Implementation of the algorithms desribed in the paper "Improving Robot
# Controller Transparency Through Autonomous Policy Explanation" by B. Hayes and
# J.A. Shah. The individual algorithms are named as follows in the class below:
#
# Algorithm 1: getLanguage()
# Algorithm 2: getStateRegion()
# Algorithm 3: getBehavioralDivergences()
# Algorithm 4: getSituationalBehaviour()

from qm import QuineMcCluskey
import json

class Explainer():
    def __init__(self, states, actions, predicates):
        self.states = states;
        self.actions = actions;
        self.predicates = predicates;

        self.numStates = len(states);

    # Algorithm 1: Convert State Region to Language
    def getLanguage(self, targetStates, nonTargetStates):
        """Describes the difference between the target and non-target states"""
        self._validateInput(targetStates, nonTargetStates)

        # Find the minterms
        minterms = self._getMinTerms(targetStates, nonTargetStates);

        # Find the clauses
        clauses = self._findClauses(minterms);

        return '     OR     '.join(clauses)

    def _validateInput(self, targetStates, nonTargetStates):
        """Checks if target and non-target state set are mutually exclusive"""
        hashedTargetStates = [];
        hashedNonTargetStates = [];

        # Since the states can be nested dictionaries, we hash each of the
        # states by converting them to JSONs. Note: this is inefficient. Better
        # would be to associate an id per state.
        for state in targetStates:
            hashedTargetStates.append(json.dumps(state))

        for state in nonTargetStates:
            hashedNonTargetStates.append(json.dumps(state))

        # Assert targetStates intersection nonTargetStates is null
        intersection = list(set(hashedTargetStates).intersection(hashedNonTargetStates))
        if intersection != []:
            raise Exception('There must be no intersection between targetStates and nonTargetStates');

    def _findValidPredicatesInSet(self, stateSet):
        """String encodes the states based on boolean predicates"""
        targetList = []
        for s in stateSet:
            state_val = '';
            for c in self.predicates:
                state_val += str(int(c['verify'](s)))

            targetList.append(state_val)
        return targetList

    def _generateBitStrings(self, n):
        """Generates all possible n-bit strings, e.g. if n=2: 00,01,10,11"""
        return [bin(x)[2:].rjust(n, '0') for x in range(2**n)];

    def _getMinTerms(self, targetStates, nonTargetStates):
        """Applies the Quine-McCluskey algorithm to get the minterms"""
        ones = self._findValidPredicatesInSet(targetStates)
        zeros = self._findValidPredicatesInSet(nonTargetStates)

        # We ignore states that are not in the target or non-target state set,
        # i.e. the ones and zeros
        n_bits = len(self.predicates)
        all_states = self._generateBitStrings(n_bits)
        dont_cares = list(set(all_states) - set(ones) - set(zeros))

        qm = QuineMcCluskey();
        return qm.simplify_los(ones, dont_cares)

    def _findClauses (self, minterms):
        """Converts the minterms to real clauses based on the given predicates"""
        clauses = []

        for min_term in minterms:
            str_term = '';
            for i in range(len(min_term)):
                predicate = self.predicates[i];
                if (min_term[i] == '0'):
                    str_term += predicate['false'];
                    if (i < len(min_term) - 1 and (min_term.find("0", i+1) != -1 or min_term.find("1", i+1) != -1)):
                        str_term += ' and '
                elif (min_term[i] == '1'):
                    str_term += predicate['true'];
                    if (i < len(min_term) - 1 and (min_term.find("0", i + 1) != -1 or min_term.find("1", i + 1) != -1)):
                        str_term += ' and '

            clauses.append(str_term);

        return clauses

    # Algorithm 2: Identify Dominant-action State Region
    def getStateRegion (self, targetAction):
        """Finds the states in which a certain action is performed"""
        # Define output sets
        targetStates = [];
        nonTargetStates = [];

        # Loop over states
        for i in range(0, self.numStates):
            if (len(self.actions[i]) > 0):
                mostFrequentAction = max(self.actions[i], key=self.actions[i].get)
                if (mostFrequentAction == targetAction):
                    targetStates.append(self.states[i])
                    continue;
            nonTargetStates.append(self.states[i])

        # TWO MOST FREQUENT ACTIONS
        # if len(targetStates)!= 0:
        #     return [targetStates, nonTargetStates]
        # else:
        #     for i in range(0, self.numStates):
        #         if (len(self.actions[i]) > 0):
        #             mostFrequentAction = max(self.actions[i], key=self.actions[i].get)
        #             if (mostFrequentAction == targetAction):
        #                 targetStates.append(self.states[i])
        #             else:
        #                 act = self.actions
        #                 del act[i][mostFrequentAction]
        #                 mostFrequentAction = max(act[i], key=self.actions[i].get)
        #                 if (mostFrequentAction == targetAction):
        #                     targetStates.append(self.states[i])
        #         nonTargetStates.append(self.states[i])

        return [targetStates, nonTargetStates]


    # Algorithm 3: Identify Behavioral Divergences
    def getBehavioralDivergences (self, state, targetAction):
        """Determines why a certain action is not performed in a given state"""
        # Define output sets
        targetStates = [];
        nonTargetStates = [];

        # Loop over states
        for i in range(0, self.numStates):
            if (len(self.actions[i]) > 0):
                mostFrequentAction = max(self.actions[i], key=self.actions[i].get)
                if (mostFrequentAction == targetAction):
                    targetStates.append(self.states[i])
                    continue;


            nonTargetStates.append(self.states[i])


        # When does the robot go straight?
        expected = self.getLanguage(targetStates, nonTargetStates);
        current = self.getLanguage([state], targetStates)

        print("I didn't {} because {}. I {} when {}.".format(targetAction, current, targetAction, expected))

        return 0;

    # Algorithm 4: Characterize Situational Behaviour
    def getSituationalBehaviour (self, description, maxActions):
        """Determines what action is performed based on a predicate description"""
        # Define output sets
        newActions = {};
        descriptions = [];

        # Determine the predicates that are set in the description
        set_terms = []
        for i in range(0, len(description)):
            if description[i] == '0' or description[i] == '1':
                set_terms.append(i)

        # Loop over states
        for i in range(0, self.numStates):
            # Checks if the state matches the description
            meetsRequirements = True
            for j in set_terms:
                if description[j] == '1' and self.predicates[j]['verify'](self.states[i]) != True:
                    meetsRequirements = False
                elif description[j] == '0' and self.predicates[j]['verify'](self.states[i]) != False:
                    meetsRequirements = False

            # Go over all actions in that state if it matches the description
            if meetsRequirements:
                if len(newActions) < maxActions:
                    action = max(self.actions[i], key=self.actions[i].get)
                    if action in newActions:
                        newActions[action].append(self.states[i])
                    else:
                        newActions[action] = [self.states[i]]
                else:
                    return "There are too many actions for me to describe."

        # We do the same as in _findClauses(), but not we must discard the
        # conditions that are already given by "description"
        if newActions:
            clauses = []
            for action in newActions:
                minterms = self._getMinTerms(newActions[action], self._complement(self.states, newActions[action]))

                for min_term in minterms:
                    str_term = '';
                    for i in range(len(min_term)):
                        if i in set_terms: continue;
                        predicate = self.predicates[i];
                        if (min_term[i] == '0'):
                            if str_term:
                                str_term += ' and '
                            str_term += predicate['false']
                        elif (min_term[i] == '1'):
                            if str_term:
                                str_term += ' and '
                            str_term += predicate['true'];

                    clauses.append(str_term);

                if (clauses[0] != ''):
                    descriptions.append('I will ' + action + ' if ' + ' or '.join(clauses) + '.')
                else:
                    descriptions.append('I will ' + action + '.')
                clauses = []
        else:
            return "I perform no action under those conditions";

        return ' '.join(descriptions);

    def _complement (self, list1, list2):
        """Returns a list of dictionaries that are in list1 but not in list2"""
        newList = [];

        list2Hash = [];

        for state in list2:
            list2Hash.append(json.dumps(state))

        for state in list1:
            if json.dumps(state) not in list2Hash:
                newList.append(state)

        return newList
