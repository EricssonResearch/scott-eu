:- module(planner_reasoner, [
    generate_pddl/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(pp, 'http://ontology.cf.ericsson.net/planning_problem#').

:- rdf_attach_library(planner_reasoner(rdf)).
:- rdf_load_library(pp).
:- rdf_load_library(wd).

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
  SubIndent is Indent + 2,
  generate_parameters(Variables, Types, SubIndent, Out),
  format(Out, '~*|)', [Indent]).

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
