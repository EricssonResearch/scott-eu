/*
Copyright 2017 Ericsson AB

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

:- module(pddl_generator, [
  generate_pddl/3
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).
:- use_module(library(indent_processor)).

:- thread_local graph/1.

:- rdf_meta rdf0(r, r, t).
:- rdf_meta generate_pddl(r, -, -).

rdf0(S, P, O) :-
  graph(G),
  rdf(S, P, O, G).

generate_pddl(Resource, Graph, String) :-
  setup_call_cleanup(assertz(graph(Graph)), (
    once((
      phrase(domain, [Resource], O1)
    ; phrase(problem, [Resource], O1)
    )),
    insert_indents(O1, O2),
    atomics_to_string(O2, "", String)
  ), retractall(graph(Graph))).

domain, ["(define (domain ", Label, ")",
            indent(2, "(:requirements :adl :equality)"),
            indent(2, Types),
            indent(2, Constants),
            indent(2, Predicates),
            indent(2, Functions),
            indent(2, Actions),
         ")"] --> [Resource],
  {
    rdf0(Resource, rdf:type, pddl:'Domain'),
    label([Resource], Label),
    types([Resource], Types),
    constants([Resource], Constants),
    predicates([Resource], Predicates),
    functions([Resource], Functions),
    actions([Resource], Actions)
  }.

label, [Label] --> [Resource],
  {
    once((
      rdf0(Resource, rdfs:label, Label^^xsd:string)
    ; rdfs_individual_of(Resource, Class),
      rdf0(Class, rdfs:label, Label^^xsd:string)
    ; rdf(Resource, rdfs:label, Label^^xsd:string)
    ; rdfs_individual_of(Resource, Class),
      rdf(Class, rdfs:label, Label^^xsd:string)
    ))
  }.

types, ["(:types", Types, ")"] --> [Resource],
  {
    collect([Resource, pddl:type, type], Types)
  }.

collect, Collection --> [Resource, Property, Rule],
  {
    findall(Element, (
      rdf0(Resource, Property, ElementResource),
      T =.. [Rule, [ElementResource], Element],
      T
    ), Collection)
  }.

type, [" "|Label] --> [Resource],
  {
    rdfs_subclass_of(Resource, pddl:'PrimitiveType'),
    label([Resource], Label)
  }.

type, [" (either", Types, ")"] --> [Resource],
  {
    rdfs_subclass_of(Resource, pddl:'EitherType'),
    collect([Resource, rdfs:member, type], Types)
  }.

constants, ["(:constants ", Constants, ")"] --> [Resource],
  {
    findall(ConstantResource,
      rdf0(Resource, pddl:constant, ConstantResource),
    ConstantResources),
    group_typed_things(ConstantResources, GrouppedConstants, rdf:type),
    groupped_typed_things(object, rdf:type, [GrouppedConstants], Constants)
  }.

predicates, ["(:predicates ",
                indent(2, Predicates),
             ")"] --> [Resource],
  {
    collect([Resource, pddl:predicate, predicate], Predicates)
  }.

predicate, ["(", Label, " ", Parameters, ")\n"] --> [Resource],
  {
    rdfs_subclass_of(Resource, pddl:'Predicate'),
    label([Resource], Label),
    parameters([Resource], Parameters)
  }.

parameters, Parameters --> [Resource],
  {
    findall(ParamResource,
      rdf0(Resource, pddl:parameter, ParamResource),
    ParamResources),
    predsort(order_compare, ParamResources, SortedParameters),
    group_typed_things(SortedParameters, GrouppedParameters, pddl:type),
    groupped_typed_things(parameter, pddl:type, [GrouppedParameters], Parameters)
  }.

order_compare(Ineq, Thing1, Thing2) :-
  once((
    rdf0(Thing1, sh:order, Order1^^xsd:integer),
    rdf0(Thing2, sh:order, Order2^^xsd:integer)
  ; rdf(Thing1, sh:order, Order1^^xsd:integer),
    rdf(Thing2, sh:order, Order2^^xsd:integer)
  )),
  ( Order1 =< Order2
  -> Ineq = <
  ; Ineq = >
  ).

group_typed_things(Parameters, GrouppedParameters, TypeProperty) :-
  group_typed_things0(_, Parameters, [], GrouppedParameters, TypeProperty).

group_typed_things0(_, [], A, [A], _) :- !.

group_typed_things0(Type, [H|T], A, O, TypeProperty) :-
  rdf0(H, TypeProperty, HType),
  group_typed_things1(Type, HType, [H|T], A, O, TypeProperty).

group_typed_things1(Type, Type, [H|T], A, O, TypeProperty) :- !,
  append(A, [H], NA),
  group_typed_things0(Type, T, NA, O, TypeProperty).

group_typed_things1(_, Type, [H|T], A, [A|T2], TypeProperty) :-
  group_typed_things0(Type, T, [H], T2, TypeProperty).

groupped_typed_things(_,  _) --> [[]]. % TODO: test this and similar [[]]!
groupped_typed_things(Rule, TypeProperty), [H2|T2] --> [[H|T]],
  {
    typed_thing_group(Rule, TypeProperty, [H], TTG),
    ( T == []
    -> H2 = TTG
    ; append(TTG, [" "], H2)
    ),
    groupped_typed_things(Rule, TypeProperty, [T], T2)
  }.

typed_thing_group(Rule, TypeProperty), [H2|T2] --> [[H|T]],
  {
    Term =.. [Rule, [H], TypedThing],
    call(Term),
    ( T == []
    -> rdf0(H, TypeProperty, TypeResource),
       type([TypeResource], Type),
       H2 = [TypedThing, " -", Type],
       T2 = []
    ; append(TypedThing, [" "], H2),
      typed_thing_group(Rule, TypeProperty, [T], T2)
    )
  }.

parameter, ["?"|Label] --> [Resource],
  {
    rdf0(Resource, rdf:type, pddl:'Parameter'),
    label([Resource], Label)
  }.

functions, ["(:functions ",
                indent(2, Functions),
             ")"] --> [Resource],
  {
    collect([Resource, pddl:function, function], Functions)
  }.

function, ["(", Label, Parameters, ")\n"] --> [Resource],
  {
    rdfs_subclass_of(Resource, pddl:'Function'),
    label([Resource], Label),
    parameters([Resource], Params),
    ( flatten(Params, [])
    -> Parameters = []
    ; Parameters = [" "|Params]
    )
  }.

actions, Actions --> [Resource],
  {
    collect([Resource, pddl:action, action], Actions)
  }.

action, ["(:action ",
            Label,
            indent(2, [":parameters", indent(2, ["(", Parameters, ")\n"]),
                       ":precondition", indent(2, Precondition),
                       ":effect", indent(2, Effect)]),
         ")\n"] --> [Resource],
  {
    rdfs_subclass_of(Resource, pddl:'Action'),
    label([Resource], Label),
    parameters([Resource], Parameters),
    rdf0(Resource, pddl:precondition, PreconditionResource),
    goal_description([PreconditionResource], Precondition),
    rdf0(Resource, pddl:effect, EffectResource),
    effect([EffectResource], Effect)
  }.

goal_description, GD --> [Resource],
  {
    once((
      literal([parameter, object], [Resource], GD)
    ; logical_operator([Resource], GD)
    ; quantifier([Resource], GD)
    ; binary_comparator([Resource], GD)
    ))
  }.

object, Label --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'PrimitiveType'),
    label([Resource], Label)
  }.

literal(P), Literal --> [Resource],
  {
    once((
      atomic_formula(P, [Resource], Literal)
    ; not(atomic_formula, P, [Resource], Literal)
    ))
  }.

not(Rule, P), ["(", Label, indent(2, Argument), ")\n"] --> [Resource],
  {
    rdf0(Resource, rdf:type, pddl:'Not'),
    label([Resource], Label),
    rdf0(Resource, pddl:argument, NotArgument),
    Term =.. [Rule, P, [NotArgument], Argument],
    call(Term)
  }.

atomic_formula(P), ["(", Label, Arguments, ")\n"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'Predicate'),
    label([Resource], Label),
    rdf(Resource, rdf:type, PredicateType),
    findall(Parameter,
      rdf(PredicateType, pddl:parameter, Parameter),
    Parameters),
    predsort(order_compare, Parameters, SortedParameters),
    predicate_arguments(P, Resource, [SortedParameters], Arguments)
  }.

predicate_arguments(_, _) --> [[]].
predicate_arguments(P, Resource), [" ", H2|T2] --> [[Parameter|T]],
  {
    rdf0(Resource, Parameter, Argument),
    member(Rule, P),
    Term =.. [Rule, [Argument], H2],
    call(Term),
    predicate_arguments(P, Resource, [T], T2)
  }.

logical_operator, ["(", Label, indent(2, Arguments), ")\n"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'LogicalOperator'),
    label([Resource], Label),
    operator_arguments(goal_description, [Resource], Arguments)
  }.

operator_arguments(Rule), Arguments --> [Resource],
  {
    findall([Parameter, Argument], (
      rdf0(Resource, Parameter, Argument),
      rdf(Parameter, rdf:type, pddl:'Parameter')
    ), ParamArgs),
    predsort(arg_compare, ParamArgs, SortedParamArgs),
    operator_arguments0(Rule, [SortedParamArgs], Arguments)
  }.

arg_compare(Ineq, [Param1, _], [Param2, _]) :-
  once((
    rdf0(Param1, sh:order, P1^^xsd:integer),
    rdf0(Param2, sh:order, P2^^xsd:integer)
  ; rdf(Param1, sh:order, P1^^xsd:integer),
    rdf(Param2, sh:order, P2^^xsd:integer)
  )),
  ( P1 =< P2
  -> Ineq = <
  ; Ineq = >
  ).

operator_arguments0(_) --> [[]].
operator_arguments0(Rule), [H2|T2] --> [[[_, Argument]|T]],
  {
    Term =.. [Rule, [Argument], H2],
    call(Term),
    operator_arguments0(Rule, [T], T2)
  }.

quantifier, ["(", Label, " (", Parameters, ")", indent(2, Arguments), ")\n"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'Quantifier'),
    label([Resource], Label),
    parameters([Resource], Parameters),
    operator_arguments(goal_description, [Resource], Arguments)
  }.

binary_comparator, ["(", Label, Arguments, ")\n"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'BinaryComparator'),
    label([Resource], Label),
    operator_arguments(f_exp, [Resource], Arguments)
  }.

f_exp, [" "|FExp] --> [Resource],
  {
    once((
      number([Resource], FExp)
    ; binary_operator(f_exp, [Resource], FExp)
    ; f_head([Resource], FExp)
    ))
  }.

number, [Number] --> [Resource],
  {
    once((
      rdf_equal(^^(Number, xsd:integer), Resource)
    ; rdf_equal(^^(Number, xsd:decimal), Resource)
    ))
  }.

binary_operator(Rule), ["(", Label, Arguments, ")"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'BinaryOperator'),
    label([Resource], Label),
    operator_arguments(Rule, [Resource], Arguments)
  }.

f_head, Label --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'Function'),
    label([Resource], Label),
    rdf(Resource, rdf:type, FunctionType),
    \+ rdf(FunctionType, pddl:parameter, _), !
  }.

f_head, ["(", Label, Arguments, ")"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'Function'),
    label([Resource], Label),
    rdf(Resource, rdf:type, FunctionType),
    findall(Parameter,
      rdf(FunctionType, pddl:parameter, Parameter),
    Parameters),
    predsort(order_compare, Parameters, SortedParameters),
    predicate_arguments([parameter, object], Resource, [SortedParameters], Arguments)
  }.

effect, Effect --> [Resource],
  {
    (
      and(c_effect, [Resource], Effect)
    ; c_effect([Resource], Effect)
    )
  }.

and(Rule), ["(", Label, indent(2, Arguments), ")\n"] --> [Resource],
  {
    rdf0(Resource, rdf:type, pddl:'And'),
    label([Resource], Label),
    operator_arguments(Rule, [Resource], Arguments)
  }.

c_effect, CEffect --> [Resource],
  {
    once((
      for_all([Resource], CEffect)
    ; when_effect([Resource], CEffect)
    ; p_effect([Resource], CEffect)
    ))
  }.

for_all, ["(", Label, "(", Variables, ")", indent(2, Arguments), ")\n"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'ForAll'),
    label([Resource], Label),
    variables([Resource], Variables),
    rdf0(Resource, pddl:argument, Argument),
    effect([Argument], Arguments)
  }.

variables --> [[]].
variables, [H2|T2] --> [[H|T]],
  {
    parameter([H], V),
    ( T == []
    -> H2 = V
    ; append(V, [" "], H2)
    ),
    variables([T], T2)
  }.

when_effect, ["(", Label, " ", indent(2, [GD, CondEffect]), ")\n"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'When'),
    label([Resource], Label),
    rdf0(Resource, pddl:parameter, GDResource),
    goal_description([GDResource], GD),
    rdf0(Resource, pddl:argument, CondEffectResource),
    cond_effect([CondEffectResource], CondEffect)
  }.

cond_effect, CondEffect --> [Resource],
  {
    once((
      and(p_effect, [Resource], CondEffect)
    ; p_effect([Resource], CondEffect)
    ))
  }.

p_effect, PEffect --> [Resource],
  {
    once((
      assignment_operator([Resource], PEffect)
    ; literal([parameter, object], [Resource], PEffect)
    ))
  }.

assignment_operator, ["(", Label, " ", FHead, FExp, ")\n"] --> [Resource],
  {
    rdfs_individual_of(Resource, pddl:'AssignmentOperator'),
    label([Resource], Label),
    rdf0(Resource, pddl:parameter, FHeadResource),
    f_head([FHeadResource], FHead),
    rdf0(Resource, pddl:argument, FExpResource),
    f_exp([FExpResource], FExp)
  }.

problem, ["(define (problem ", Label, ")",
            indent(2, ["(:domain ", Domain, ")"]),
            indent(2, Objects),
            indent(2, Init),
            indent(2, Goal),
            indent(2, Metric),
         "\n", ")"] --> [Resource],
  {
    rdf0(Resource, rdf:type, pddl:'Problem'),
    label([Resource], Label),
    rdf0(Resource, pddl:domain, DomainResource),
    label([DomainResource], Domain),
    objects([Resource], Objects),
    init([Resource], Init),
    goal([Resource], Goal),
    metric([Resource], Metric)
  }.

objects, ["(:objects ", Objects, ")"] --> [Resource],
  {
    findall(ObjectResource,
      rdf0(Resource, pddl:object, ObjectResource),
    ObjectResources),
    group_typed_things(ObjectResources, GrouppedObjects, rdf:type),
    groupped_typed_things(object, rdf:type, [GrouppedObjects], Objects)
  }.

init, ["(:init", indent(2, Init), ")"] --> [Resource],
  {
    collect([Resource, pddl:init, init_el], Init)
  }.

init_el, InitEl --> [Resource],
  {
    once((
      literal([object], [Resource], InitEl)
    ; eq([Resource], InitEl)
    ))
  }.

eq, ["(", Label, " ", FHead, " ", Number, ")\n"] --> [Resource],
  {
    rdf0(Resource, rdf:type, pddl:'EQ'),
    label([Resource], Label),
    rdf0(Resource, pddl:left, FHeadResource),
    f_head([FHeadResource], FHead),
    rdf0(Resource, pddl:right, NumberResource),
    number([NumberResource], Number)
  }.

goal, ["(:goal", indent(2, GD), ")"] --> [Resource],
  {
    rdf0(Resource, pddl:goal, GoalResource),
    goal_description([GoalResource], GD)
  }.

metric, ["(:metric ", Metric, " ", GroundFExp, ")"] --> [Resource],
  {
    once((
      rdf0(Resource, pddl:minimize, Criteria),
      Metric = "minimize"
    ; rdf0(Resource, pddl:maximize, Criteria),
      Metric = "maximize"
    )),
    ground_f_exp([Criteria], GroundFExp)
  }.

ground_f_exp, GroundFExp --> [Resource],
  {
    once((
      number([Resource], GroundFExp)
    ; binary_operator(ground_f_exp, [Resource], GroundFExp)
    ; f_head([Resource], GroundFExp)
    ))
  }.

test :-
  generate_pddl(pddle:'adl-blocksworld', 'http://ontology.cf.ericsson.net/pddl_example', String1),
  writeln(String1),
  nl,
  generate_pddl(pddle:'adl-blocksworld-problem', 'http://ontology.cf.ericsson.net/pddl_example', String2),
  writeln(String2).
