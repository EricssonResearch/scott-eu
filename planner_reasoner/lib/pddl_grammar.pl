:- module(pddl_grammar, [
  generate_pddl/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).
:- use_module(library(indent_processor)).

generate_pddl(Resource, String) :-
  once((
    phrase(domain, [Resource], O1)
  %; problem(Resource, 0)
  )),
  insert_indents(O1, O2),
  atomics_to_string(O2, "", String).

domain, ["(define (domain ", Label, ")",
            indent(2, Types),
            indent(2, Predicates),
         "\n)"] --> [Resource],
  {
    label([Resource], Label),
    types([Resource], Types),
    predicates([Resource], Predicates)
  }.

label, [Label] --> [Resource],
  {
    once((
      rdf(Resource, rdfs:label, Label^^xsd:string)
    ; rdfs_individual_of(Resource, Class),
      rdf(Class, rdfs:label, Label^^xsd:string)
    ))
  }.

types, ["(:types", Types, ")"] --> [Resource],
  {
    collect([Resource, pddl:type, type], Types)
  }.

type, [" ", Type] --> [Resource],
  {
    rdf(Resource, rdf:type, pddl:'PrimitiveType'),
    label([Resource], Type)
  }.

type, [" (either", Types, ")"] --> [Resource],
  {
    rdf(Resource, rdf:type, pddl:'EitherType'),
    collect([Resource, rdfs:member, type], Types)
  }.

predicates, ["(:predicates ",
                indent(13, Predicates),
             "\n)"] --> [Resource],
  {
    collect([Resource, pddl:predicate, predicate], Predicates)
  }.

predicate, ["(", Predicate, Parameters, ")\n"] --> [Resource],
  {
    label([Resource], Predicate),
    parameters([Resource], Parameters)
  }.

parameters, [Parameters] --> [Resource],
  {
    findall(ParamResource,
      rdf(Resource, pddl:parameter, ParamResource),
    ParamResources),
    predsort(order_compare, ParamResources, SortedParameters),
    group_typed_things(SortedParameters, GrouppedParameters),
    groupped_typed_things([GrouppedParameters], Parameters)
  }.

order_compare(Ineq, Thing1, Thing2) :-
  rdf(Thing1, sh:order, Order1^^xsd:integer),
  rdf(Thing2, sh:order, Order2^^xsd:integer),
  ( Order1 =< Order2
  -> Ineq = <
  ; Ineq = >
  ).

group_typed_things(Parameters, GrouppedParameters) :-
  group_typed_things0(_, Parameters, [], GrouppedParameters).

group_typed_things0(_, [], A, [A]) :- !.

group_typed_things0(Type, [H|T], A, O) :-
  rdf(H, pddl:type, HType),
  group_typed_things1(Type, HType, [H|T], A, O).

group_typed_things1(Type, Type, [H|T], A, O) :- !,
  append(A, [H], NA),
  group_typed_things0(Type, T, NA, O).

group_typed_things1(_, Type, [H|T], A, [A|T2]) :-
  group_typed_things0(Type, T, [H], T2).


groupped_typed_things, [] --> [[]].
groupped_typed_things, [H2|T2] --> [[H|T]],
  {
    typed_thing_group([H], H2),
    groupped_typed_things([T], T2)
  }.

typed_thing_group, [] --> [[]].
typed_thing_group, [H2|T2] --> [[H|T]],
  {
    typed_thing([H], TypedThing),
    ( T == []
    -> rdf(H, pddl:type, TypeResource),
       type([TypeResource], Type),
       H2 = [" ", TypedThing, " -", Type],
       T2 = []
    ; H2 = [" ", TypedThing],
      typed_thing_group([T], T2)
    )
  }.

typed_thing, Thing --> [Resource],
  {
    label([Resource], Label),
    ( rdf(Resource, rdf:type, pddl:'Parameter')
    -> Thing = ["?", Label]
    ; ( rdf(Thing, rdf:type, pddl:'Object')
      -> Thing = [Label]
      ; fail
      )
    )
  }.

collect, [Collection] --> [Resource, Property, Rule],
  {
    findall(Element, (
      rdf(Resource, Property, ElementResource),
      T =.. [Rule, [ElementResource], Element],
      T
    ), Collection)
  }.

test2 :-
  rdf_global_id(pddle:'adl-blocksworld', Domain),
  generate_pddl(Domain, String),
  writeln(String).
  %nl,
  %rdf_global_id(pddle:'adl-blocksworld-problem', Problem),
  %generate_pddl(Problem, Out).
