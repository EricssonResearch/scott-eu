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

generate_pddl(Resource, Graph, String) :-
  Context = _{r:Resource, g:Graph},
  once((
    phrase(domain, [Context], O1)
  ; phrase(problem, [Context], O1)
  )),
  insert_indents(O1, O2),
  atomics_to_string(O2, "", String).

domain, ["(define (domain ", Label, ")",
            indent(2, Types),
            indent(2, Predicates),
            indent(2, Functions),
            indent(2, Actions),
         ")"] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'Domain', Context.g),
    label([Context], Label),
    types([Context], Types),
    predicates([Context], Predicates),
    functions([Context], Functions),
    actions([Context], Actions)
  }.

label, [Label] --> [Context],
  {
    once((
      rdf(Context.r, rdfs:label, Label^^xsd:string, Context.g)
    ; rdf(Context.r, rdfs:label, Label^^xsd:string)
    ; rdfs_individual_of(Context.r, Class),
      (
        rdf(Class, rdfs:label, Label^^xsd:string, Context.g)
      ; rdf(Class, rdfs:label, Label^^xsd:string)
      )
    ))
  }.

types, ["(:types", Types, ")"] --> [Context],
  {
    collect([Context, pddl:type, type], Types)
  }.

collect, Collection --> [Context, Property, Rule],
  {
    findall(Element, (
      rdf(Context.r, Property, ElementResource, Context.g),
      T =.. [Rule, [Context.put(r, ElementResource)], Element],
      T
    ), Collection)
  }.

type, [" "|Label] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'PrimitiveType', Context.g),
    label([Context], Label)
  }.

type, [" (either", Types, ")"] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'EitherType', Context.g),
    collect([Context, rdfs:member, type], Types)
  }.

predicates, ["(:predicates ",
                indent(2, Predicates),
             ")"] --> [Context],
  {
    collect([Context, pddl:predicate, predicate], Predicates)
  }.

predicate, ["(", Label, " ", Parameters, ")\n"] --> [Context],
  {
    rdfs_subclass_of(Context.r, pddl:'Predicate'),
    label([Context], Label),
    parameters([Context], Parameters)
  }.

parameters, Parameters --> [Context],
  {
    findall(Context.put(r, ParamResource),
      rdf(Context.r, pddl:parameter, ParamResource, Context.g),
    ParamResources),
    predsort(order_compare, ParamResources, SortedParameters),
    group_typed_things(SortedParameters, GrouppedParameters),
    groupped_typed_things(parameter, [GrouppedParameters], Parameters)
  }.

order_compare(Ineq, Thing1, Thing2) :-
  once((
    rdf(Thing1.r, sh:order, Order1^^xsd:integer, Thing1.g)
  ; rdf(Thing1.r, sh:order, Order1^^xsd:integer)
  )),
  once((
    rdf(Thing2.r, sh:order, Order2^^xsd:integer, Thing2.g)
  ; rdf(Thing2.r, sh:order, Order2^^xsd:integer)
  )),
  ( Order1 =< Order2
  -> Ineq = <
  ; Ineq = >
  ).

group_typed_things(Parameters, GrouppedParameters) :-
  group_typed_things0(_, Parameters, [], GrouppedParameters).

group_typed_things0(_, [], A, [A]) :- !.

group_typed_things0(Type, [H|T], A, O) :-
  rdf(H.r, pddl:type, HType, H.g),
  group_typed_things1(Type, HType, [H|T], A, O).

group_typed_things1(Type, Type, [H|T], A, O) :- !,
  append(A, [H], NA),
  group_typed_things0(Type, T, NA, O).

group_typed_things1(_, Type, [H|T], A, [A|T2]) :-
  group_typed_things0(Type, T, [H], T2).

groupped_typed_things(_) --> [[]].
groupped_typed_things(Rule), [H2|T2] --> [[H|T]],
  {
    typed_thing_group(Rule, [H], TTG),
    ( T == []
    -> H2 = TTG
    ; append(TTG, [" "], H2)
    ),
    groupped_typed_things(Rule, [T], T2)
  }.

typed_thing_group(Rule), [H2|T2] --> [[H|T]],
  {
    Term =.. [Rule, [H], TypedThing],
    call(Term),
    ( T == []
    -> rdf(H.r, pddl:type, TypeResource, H.g),
       type([H.put(r, TypeResource)], Type),
       H2 = [TypedThing, " -", Type],
       T2 = []
    ; append(TypedThing, [" "], H2),
      typed_thing_group(Rule, [T], T2)
    )
  }.

parameter, ["?"|Label] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'Parameter', Context.g),
    label([Context], Label)
  }.

functions, ["(:functions ",
                indent(2, Functions),
             ")"] --> [Context],
  {
    collect([Context, pddl:function, function], Functions)
  }.

function, ["(", Label, Parameters, ")\n"] --> [Context],
  {
    rdfs_subclass_of(Context.r, pddl:'Function'),
    label([Context], Label),
    parameters([Context], Params),
    ( flatten(Params, [])
    -> Parameters = []
    ; Parameters = [" "|Params]
    )
  }.

actions, Actions --> [Context],
  {
    collect([Context, pddl:action, action], Actions)
  }.

action, ["(:action ",
            Label,
            indent(2, [":parameters", indent(2, ["(", Parameters, ")\n"]),
                       ":precondition", indent(2, Precondition),
                       ":effect", indent(2, Effect)]),
         ")\n"] --> [Context],
  {
    rdfs_subclass_of(Context.r, pddl:'Action'),
    label([Context], Label),
    parameters([Context], Parameters),
    rdf(Context.r, pddl:precondition, PreconditionResource, Context.g),
    goal_description([Context.put(r, PreconditionResource)], Precondition),
    rdf(Context.r, pddl:effect, EffectResource, Context.g),
    effect([Context.put(r, EffectResource)], Effect)
  }.

goal_description, GD --> [Context],
  {
    once((
      literal([parameter, object], [Context], GD)
    ; logical_operator([Context], GD)
    ; quantifier([Context], GD)
    ; binary_comparator([Context], GD)
    ))
  }.

object, Label --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'Object', Context.g),
    label([Context], Label)
  }.

literal(P), Literal --> [Context],
  {
    once((
      atomic_formula(P, [Context], Literal)
    ; not(atomic_formula, P, [Context], Literal)
    ))
  }.

not(Rule, P), ["(", Label, indent(2, Argument), ")\n"] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'Not', Context.g),
    label([Context], Label),
    rdf(Context.r, Parameter, NotArgument),
    rdf(Parameter, rdf:type, pddl:'Parameter'),
    Term =.. [Rule, P, [Context.put(r, NotArgument)], Argument],
    call(Term)
  }.

atomic_formula(P), ["(", Label, Arguments, ")\n"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'Predicate'),
    label([Context], Label),
    rdf(Context.r, rdf:type, PredicateType, Context.g),
    findall(Context.put(r, Parameter),
      rdf(PredicateType, pddl:parameter, Parameter),
    Parameters),
    predsort(order_compare, Parameters, SortedParameters),
    predicate_arguments(P, Context, [SortedParameters], Arguments)
  }.

predicate_arguments(_, _) --> [[]].
predicate_arguments(P, Context), [" ", H2|T2] --> [[Parameter|T]],
  {
    rdf(Context.r, Parameter.r, Argument, Context.g),
    member(Rule, P),
    Term =.. [Rule, [Context.put(r, Argument)], H2],
    call(Term),
    predicate_arguments(P, Context, [T], T2)
  }.

logical_operator, ["(", Label, indent(2, Arguments), ")\n"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'LogicalOperator'),
    label([Context], Label),
    operator_arguments(goal_description, [Context], Arguments)
  }.

operator_arguments(Rule), Arguments --> [Context],
  {
    findall([Context.put(r, Parameter), Context.put(r, Argument)], (
      rdf(Context.r, Parameter, Argument, Context.g),
      rdf(Parameter, rdf:type, pddl:'Parameter')
    ), ParamArgs),
    predsort(arg_compare, ParamArgs, SortedParamArgs),
    operator_arguments0(Rule, [SortedParamArgs], Arguments)
  }.

arg_compare(Ineq, [Param1,_], [Param2,_]) :-
  once((
    rdf(Param1.r, sh:order, P1^^xsd:integer, Param1.g)
  ; rdf(Param1.r, sh:order, P1^^xsd:integer)
  )),
  once((
    rdf(Param2.r, sh:order, P2^^xsd:integer, Param2.g)
  ; rdf(Param2.r, sh:order, P2^^xsd:integer)
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

quantifier, ["(", Label, " (", Parameters, ")", indent(2, Arguments), ")\n"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'Quantifier'),
    label([Context], Label),
    parameters([Context], Parameters),
    operator_arguments(goal_description, [Context], Arguments)
  }.

binary_comparator, ["(", Label, Arguments, ")\n"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'BinaryComparator'),
    label([Context], Label),
    operator_arguments(f_exp, [Context], Arguments)
  }.

f_exp, [" "|FExp] --> [Context],
  {
    once((
      number([Context], FExp)
    ; binary_operator(f_exp, [Context], FExp)
    ; f_head([Context], FExp)
    ))
  }.

number, [Number] --> [Context],
  {
    once((
      rdf_equal(^^(Number, xsd:integer), Context.r)
    ; rdf_equal(^^(Number, xsd:decimal), Context.r)
    ))
  }.

binary_operator(Rule), ["(", Label, Arguments, ")"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'BinaryOperator'),
    label([Context], Label),
    operator_arguments(Rule, [Context], Arguments)
  }.

f_head, Label --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'Function'),
    label([Context], Label),
    rdf(Context.r, rdf:type, FunctionType, Context.g),
    \+ rdf(FunctionType, pddl:parameter, _, Context.g), !
  }.

f_head, ["(", Label, Arguments, ")"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'Function'),
    label([Context], Label),
    rdf(Context.r, rdf:type, FunctionType, Context.g),
    findall(Context.put(r, Parameter),
      rdf(FunctionType, pddl:parameter, Parameter, Context.g),
    Parameters),
    predsort(order_compare, Parameters, SortedParameters),
    predicate_arguments([parameter, object], Context, [SortedParameters], Arguments)
  }.

effect, Effect --> [Context],
  {
    (
      and(c_effect, [Context], Effect)
    ; c_effect([Context], Effect)
    )
  }.

and(Rule), ["(", Label, indent(2, Arguments), ")\n"] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'And', Context.g),
    label([Context], Label),
    operator_arguments(Rule, [Context], Arguments)
  }.

c_effect, CEffect --> [Context],
  {
    once((
      for_all([Context], CEffect)
    ; when_effect([Context], CEffect)
    ; p_effect([Context], CEffect)
    ))
  }.

for_all, ["(", Label, "(", Variables, ")", indent(2, Arguments), ")\n"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'ForAll'),
    label([Context], Label),
    variables([Context], Variables),
    rdf(Context.r, pddl:argument, Argument, Context.g),
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

when_effect, ["(", Label, " ", indent(2, [GD, CondEffect]), ")\n"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'When'),
    label([Context], Label),
    rdf(Context.r, pddl:parameter, GDResource, Context.g),
    goal_description([Context.put(r, GDResource)], GD),
    rdf(Context.r, pddl:argument, CondEffectResource, Context.g),
    cond_effect([Context.put(r, CondEffectResource)], CondEffect)
  }.

cond_effect, CondEffect --> [Context],
  {
    once((
      and(p_effect, [Context], CondEffect)
    ; p_effect([Context], CondEffect)
    ))
  }.

p_effect, PEffect --> [Context],
  {
    once((
      assignment_operator([Context], PEffect)
    ; literal([parameter, object], [Context], PEffect)
    ))
  }.

assignment_operator, ["(", Label, " ", FHead, FExp, ")\n"] --> [Context],
  {
    rdfs_individual_of(Context.r, pddl:'AssignmentOperator'),
    label([Context], Label),
    rdf(Context.r, pddl:parameter, FHeadResource, Context.g),
    f_head([Context.put(r, FHeadResource)], FHead),
    rdf(Context.r, pddl:argument, FExpResource, Context.g),
    f_exp([Context.put(r, FExpResource)], FExp)
  }.

problem, ["(define (problem ", Label, ")",
            indent(2, ["(:domain ", Domain, ")"]),
            indent(2, Objects),
            indent(2, Init),
            indent(2, Goal),
            indent(2, Metric),
         "\n", ")"] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'Problem', Context.g),
    label([Context], Label),
    rdf(Context.r, pddl:domain, DomainResource, Context.g),
    label([Context.put(r, DomainResource)], Domain),
    objects([Context], Objects),
    init([Context], Init),
    goal([Context], Goal),
    metric([Context], Metric)
  }.

objects, ["(:objects ", Objects, ")"] --> [Context],
  {
    findall(Context.put(r, ObjectResource),
      rdf(Context.r, pddl:object, ObjectResource, Context.g),
    ObjectResources),
    group_typed_things(ObjectResources, GrouppedObjects),
    groupped_typed_things(object, [GrouppedObjects], Objects)
  }.

init, ["(:init", indent(2, Init), ")"] --> [Context],
  {
    collect([Context, pddl:init, init_el], Init)
  }.

init_el, InitEl --> [Context],
  {
    once((
      literal([object], [Context], InitEl)
    ; eq([Context], InitEl)
    ))
  }.

eq, ["(", Label, " ", FHead, " ", Number, ")\n"] --> [Context],
  {
    rdf(Context.r, rdf:type, pddl:'EQ', Context.g),
    label([Context], Label),
    rdf(Context.r, pddl:left, FHeadResource, Context.g),
    f_head([FHeadResource], FHead),
    rdf(Context.r, pddl:right, NumberResource, Context.g),
    number([NumberResource], Number)
  }.

goal, ["(:goal", indent(2, GD), ")"] --> [Context],
  {
    rdf(Context.r, pddl:goal, GoalResource, Context.g),
    goal_description([Context.put(r, GoalResource)], GD)
  }.

metric, ["(:metric ", Metric, " ", GroundFExp, ")"] --> [Context],
  {
    once((
      rdf(Context.r, pddl:minimize, Criteria, Context.g),
      Metric = "minimize"
    ; rdf(Context.r, pddl:maximize, Criteria, Context.g),
      Metric = "maximize"
    )),
    ground_f_exp([Context.put(r, Criteria)], GroundFExp)
  }.

ground_f_exp, GroundFExp --> [Context],
  {
    once((
      number([Context], GroundFExp)
    ; binary_operator(ground_f_exp, [Context], GroundFExp)
    ; f_head([Context], GroundFExp)
    ))
  }.

test :-
  rdf_global_id(pddle:'adl-blocksworld', Domain),
  generate_pddl(Domain, 'http://ontology.cf.ericsson.net/pddl_example', String1),
  writeln(String1),
  nl,
  rdf_global_id(pddle:'adl-blocksworld-problem', Problem),
  generate_pddl(Problem, 'http://ontology.cf.ericsson.net/pddl_example', String2),
  writeln(String2).
