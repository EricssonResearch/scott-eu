:- module(planner_reasoner, [
    generate_pddl/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(pp, 'http://ontology.cf.ericsson.net/planning_ontology#').
:- rdf_register_prefix(wd, 'http://ontology.cf.ericsson.net/warehouse_domain#').
:- rdf_register_prefix(wp, 'http://ontology.cf.ericsson.net/warehouse_problem#').
:- rdf_register_prefix(ppos, 'http://ontology.cf.ericsson.net/planning_ontology_oslc_shapes#').

:- rdf_attach_library(planner_reasoner(rdf)).
:- rdf_load_library(pp).
:- rdf_load_library(wd).
:- rdf_load_library(ppos).
:- rdf_load_library(wp).

:- rdf_meta generate_pddl(r, -).
:- rdf_meta generate_pddl(r, r, -, -).

generate_pddl(Resource, Out) :-
  rdf(Resource, rdf:type, Type),
  generate_pddl(Resource, Type, 2, Out).

generate_pddl(Resource, pp:'Action', Indent, Out) :-
  rdf(Resource, rdfs:label, ^^(Label, _)),
  format(Out, '~*|(:action ~w~n', [Indent,Label]),
  rdf(Resource, pp:hasVariables, Variables),
  rdf(Resource, pp:hasTypes, Types),
  rdf(Resource, pp:hasPrecondition, TopPredPrecondition),
  rdf(Resource, pp:hasEffect, TopPredEffect),
  SubIndent is Indent + 2,

  generate_parameters(Variables, Types, SubIndent, Out),

  format(Out, '~*|:precondition (', [SubIndent]),
  generate_precondition(TopPredPrecondition, SubIndent, Out),
  format(Out, '~n~*|)~n', [SubIndent]), %closed bracket from the precondition

  format(Out, '~*|:effect (', [SubIndent]),
  generate_effect(TopPredEffect, SubIndent, Out),
  format(Out, '~*|)~n', [SubIndent]), %closed bracket from the effect

  format(Out, '~*|)~n', [Indent]). %closed bracket from the action

generate_parameters(Variables, Types, Indent, Out) :-
  rdf_list(Variables, VarList),
  rdf_list(Types, TypeList),
  format(Out, '~*|:parameters (', [Indent]),
  generate_parameters0(VarList, TypeList, Out),
  format(Out, ')~n', []).

generate_parameters0([], [], _) :- !.
generate_parameters0([H1|T1], [H2|T2], Out) :-
  rdf(H1, rdf:type, pp:'Variable'),
  rdf(H1, rdfs:label, ^^(Var, _)),
  rdf(H2, rdf:type, pp:'VariableType'),
  rdf(H2, rdfs:label, ^^(Type, _)),
  format(Out, '?~w - ~w', [Var,Type]),
  (T1=[]->true;format(Out, ' ', [])),
  generate_parameters0(T1, T2, Out).

generate_precondition(TopPredPrecondition, SubIndent, Out):-
  (rdf(TopPredPrecondition, rdf:type, pp:'Predicate') ->
                                                    rdf(TopPredPrecondition, rdfs:label, ^^(PredName, _)),
                                                    rdf(TopPredPrecondition, pp:hasArguments, SubPredicates),
                                                    rdf_list(SubPredicates, PredicateList),
                                                    format(Out, '~w ', [PredName, SubIndent]),
                                                    generate_precondition0(PredicateList, SubIndent, Out));
    (rdf(TopPredPrecondition, rdf:type, pp:'And') ->
                                                      rdf(pp:'And', rdfs:label, ^^(PredName, _)),
                                                      rdf(TopPredPrecondition, pp:hasArguments, SubPredicates),
                                                      rdf_list(SubPredicates, PredicateList),
                                                      format(Out, '~w ~n', [PredName, SubIndent]),
                                                      generate_precondition0(PredicateList, SubIndent, Out));
    (rdf(TopPredPrecondition, rdf:type, pp:'Not') ->
                                                     rdf(pp:'Not', rdfs:label, ^^(PredName, _)),
                                                     rdf(TopPredPrecondition, pp:hasArguments, SubPredicates),
                                                     rdf_list(SubPredicates, PredicateList),
                                                     format(Out, '~w  ~n', [PredName, SubIndent]),
                                                     generate_precondition0(PredicateList, SubIndent, Out)).


generate_precondition0([], SubIndent, _) :- !.
generate_precondition0([H1|T1], SubIndent, Out) :-
  %if it is a predicate
  (rdf(H1, rdf:type, pp:'Not') ->
                                        rdf(pp:'Not', rdfs:label, ^^(PredName1, _)),
                                        rdf(H1, pp:hasArguments, SubPredicates),
                                        rdf_list(SubPredicates, SubPredicateList),
                                        SubSubIndent is SubIndent+10,
                                        format(Out, '~*|(~w', [SubSubIndent,PredName1]),
                                        generate_precondition0(SubPredicateList, SubIndent, Out));
  (rdf(H1, rdf:type, pp:'And') ->
                                        rdf(pp:'And', rdfs:label, ^^(PredName1, _)),
                                        rdf(H1, pp:hasArguments, SubPredicates),
                                        rdf_list(SubPredicates, SubPredicateList),
                                        SubSubIndent is SubIndent+10,
                                        format(Out, '~*|(~w', [SubSubIndent,PredName1]),
                                        generate_precondition0(SubPredicateList, SubIndent, Out));
  (rdf(H1, rdf:type, pp:'Variable') ->
                                      rdf(H1, rdf:type, pp:'Variable'),
                                      rdf(H1, rdfs:label, ^^(VarName, _)),
                                      format(Out, ' ?~w) ~n', [VarName]),
                                      generate_precondition0(SubPredicateList, SubIndent, Out));
  (rdf(H1, rdf:type, pp:'Predicate') ->
                                      rdf(H1, rdfs:label, ^^(PredName1, _)),
                                      rdf(H1, pp:hasArguments, SubPredicates),
                                      rdf_list(SubPredicates, SubPredicateList),
                                      SubSubIndent is SubIndent+10,
                                      format(Out, '~*|(~w', [SubSubIndent,PredName1]),
                                      generate_precondition0(SubPredicateList, SubIndent, Out));
  generate_precondition0(T1, SubIndent, Out),
  (T1=[]->true;format(Out, ' ~n', [])).



/*generate_precondition1([], SubIndent, _) :- !.
generate_precondition1([H1|T1], SubIndent, Out):-
  rdf(H1, rdf:type, pp:'Variable'),
  rdf(H1, rdfs:label, ^^(VarName, _)),
  format(Out, ' ?~w', [VarName]),
  generate_precondition1(T1, SubIndent, Out),
  (T1=[]->true;format(Out, ')~n', [])).*/


generate_effect(TopPredEffect, SubIndent, Out):-
  rdf(TopPredEffect, rdf:type, pp:'And'),
  rdf(pp:'And', rdfs:label, ^^(PredName2, _)),
  format(Out, '~w ~n', [PredName2, SubIndent]).



%(a -> b ;c)  if statements
