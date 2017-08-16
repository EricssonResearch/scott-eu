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
:- use_module(library(grammar_utils)).
:- use_module(library(indent_processor)).

:- rdf_meta generate_pddl(r, -, -).
:- rdf_meta group_typed_things(-, -, r).
:- rdf_meta groupped_typed_things(-, r, -, -).

generate_pddl(Resource, Graph, String) :-
  setup_call_cleanup(
    rdf0_set_graph(Graph), (
      pddl(PDDL, [Resource], [Resource]),
      insert_indents(PDDL, Indented),
      atomics_to_string(Indented, "", String)
    ),
    rdf0_unset_graph(Graph)
  ).

pddl(PDDL) -->
  domain(PDDL), !.

pddl(PDDL) -->
  problem(PDDL).

domain(["(define (domain ", Label, ")",
          indent(2, ["(:requirements :adl :equality)"]),
          indent(2, Types),
          indent(2, Constants),
          indent(2, Predicates),
          indent(2, Functions),
          indent(2, Actions),
        "\n", ")\n"]) -->
  of_type(pddl:'Domain'),
  label(Label),
  types(Types),
  constants(Constants),
  predicates(Predicates),
  functions(Functions),
  actions(Actions).

label(Label) -->
  lookup(Resource),
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

types(["(:types", Types, ")"]) -->
  apply_to_properties(pddl:type, type, Types),
  { nonempty(Types), ! }.

types([]) --> [].

type([" ", Label]) -->
  subclass_of(pddl:'PrimitiveType'), !,
  label(Label).

type([" (either", Types, ")"]) -->
  subclass_of(pddl:'EitherType'),
  apply_to_properties(rdfs:member, type, Types).

constants(["(:constants ", Constants, ")"]) -->
  all_properties(pddl:constant, ConstantResources),
  {
    nonempty(ConstantResources), !,
    group_typed_things(ConstantResources, GrouppedConstants, rdf:type),
    groupped_typed_things(object, rdf:type, GrouppedConstants, Constants)
  }.

constants([]) --> [].

predicates(["(:predicates ", indent(2, Predicates), ")"]) -->
  apply_to_properties(pddl:predicate, predicate, Predicates),
  { nonempty(Predicates), ! }.

predicates([]) --> [].

predicate(["(", Label, Parameters, ")\n"]) -->
  subclass_of(pddl:'Predicate'),
  label(Label),
  parameters_w_space(Parameters).

parameters_w_space([" ", Parameters]) -->
  parameters(Parameters),
  { nonempty(Parameters), ! }.

parameters_w_space([]) --> [].

parameters(Parameters) -->
  all_properties(pddl:parameter, ParameterResources),
  {
    nonempty(ParameterResources), !,
    predsort(order_compare, ParameterResources, SortedParameters),
    group_typed_things(SortedParameters, GrouppedParameters, pddl:type),
    groupped_typed_things(parameter, pddl:type, GrouppedParameters, Parameters)
  }.

parameters([]) --> [].

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

group_typed_things0(_, [], [], [], _) :- !.

group_typed_things0(_, [], A, [A], _) :- !.

group_typed_things0(Type, [H|T], A, O, TypeProperty) :-
  rdf0(H, TypeProperty, HType),
  group_typed_things1(Type, HType, [H|T], A, O, TypeProperty).

group_typed_things1(Type, Type, [H|T], A, O, TypeProperty) :- !,
  append(A, [H], NA),
  group_typed_things0(Type, T, NA, O, TypeProperty).

group_typed_things1(_, Type, [H|T], A, [A|T2], TypeProperty) :-
  group_typed_things0(Type, T, [H], T2, TypeProperty).

groupped_typed_things(_,  _, [], []).
groupped_typed_things(Rule, TypeProperty, [H|T], [H2|T2]) :-
  typed_thing_group(Rule, TypeProperty, H, TTG),
  ( T == []
  -> H2 = TTG
  ; append(TTG, [" "], H2)
  ),
  groupped_typed_things(Rule, TypeProperty, T, T2).

typed_thing_group(Rule, TypeProperty, [H|T], [H2|T2]) :-
  Term =.. [Rule, TypedThing, [H], [H]],
  call(Term),
  ( T == []
  -> rdf0(H, TypeProperty, TypeResource),
     type(Type, [TypeResource], [TypeResource]),
     H2 = [TypedThing, " -", Type],
     T2 = []
  ; H2 = [TypedThing, " "],
    typed_thing_group(Rule, TypeProperty, T, T2)
  ).

parameter(["?", Label]) -->
  of_type(pddl:'Parameter'),
  label(Label).

functions(["(:functions ", indent(2, Functions), ")"]) -->
  apply_to_properties(pddl:function, function, Functions),
  { nonempty(Functions), ! }.

functions([]) --> [].

function(["(", Label, Parameters, ")\n"]) -->
  subclass_of(pddl:'Function'),
  label(Label),
  parameters_w_space(Parameters).

actions(Actions) -->
  apply_to_properties(pddl:action, action, Actions),
  { nonempty(Actions), ! }.

actions([]) --> [].

action(["(:action ", Label,
        indent(2, [ ":parameters", indent(2, ["(", Parameters, ")\n"]),
                    ":precondition", indent(2, Precondition),
                    ":effect", indent(2, Effect)
                  ]),
        ")"]) -->
  subclass_of(pddl:'Action'),
  label(Label),
  parameters(Parameters),
  apply_to_property(pddl:precondition, goal_description, Precondition),
  apply_to_property(pddl:effect, effect, Effect).

goal_description(GD) -->
  literal([parameter, object], GD), !.

goal_description(GD) -->
  logical_operator(GD), !.

goal_description(GD) -->
  quantifier(GD), !.

goal_description(GD) -->
  binary_comparator(GD).

object(Label) -->
  individual_of(pddl:'PrimitiveType'),
  label(Label).

literal(P, Literal) -->
  atomic_formula(P, Literal), !.

literal(P, Literal) -->
  not(atomic_formula, P, Literal).

not(Rule, P, ["(", Label, indent(2, Argument), ")\n"]) -->
  of_type(pddl:'Not'),
  label(Label),
  lookup(Resource),
  {
    rdf0(Resource, pddl:argument, ArgumentResource),
    T =.. [Rule, P, Argument, [ArgumentResource], [ArgumentResource]],
    T
  }.

atomic_formula(P, ["(", Label, Arguments, ")\n"]) -->
  individual_of(pddl:'Predicate'),
  label(Label),
  lookup(Resource),
  {
    rdf0(Resource, rdf:type, PredicateType),
    findall(Parameter,
      rdf(PredicateType, pddl:parameter, Parameter),
    Parameters),
    predsort(order_compare, Parameters, SortedParameters),
    predicate_arguments(P, Resource, SortedParameters, Arguments)
  }.

predicate_arguments(_, _, [], []).
predicate_arguments(P, Resource, [Parameter|T], [" ", H2|T2]) :-
  rdf0(Resource, Parameter, Argument),
  member(Rule, P),
  Term =.. [Rule, H2, [Argument], [Argument]],
  call(Term),
  predicate_arguments(P, Resource, T, T2).

logical_operator(["(", Label, indent(2, Arguments), ")\n"]) -->
  individual_of(pddl:'LogicalOperator'),
  label(Label),
  operator_arguments(goal_description, Arguments).

operator_arguments(Rule, Arguments) -->
  lookup(Resource),
  {
    findall([Parameter, Argument], (
      rdf0(Resource, Parameter, Argument),
      rdf(Parameter, rdf:type, pddl:'Parameter')
    ), ParamArgs),
    predsort(arg_compare, ParamArgs, SortedParamArgs),
    operator_arguments0(Rule, SortedParamArgs, Arguments)
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

operator_arguments0(_, [], []).
operator_arguments0(Rule, [[_, Argument]|T], [H2|T2]) :-
  Term =.. [Rule, H2, [Argument], [Argument]],
  call(Term),
  operator_arguments0(Rule, T, T2).

quantifier(["(", Label, " (", Parameters, ")", indent(2, Arguments), ")\n"]) -->
  individual_of(pddl:'Quantifier'),
  label(Label),
  parameters(Parameters),
  operator_arguments(goal_description, Arguments).

binary_comparator(["(", Label, " ", Arguments, ")\n"]) -->
  individual_of(pddl:'BinaryComparator'),
  label(Label),
  operator_arguments(f_exp, Arguments).

f_exp(FExp) -->
  number(FExp), !.

f_exp(FExp) -->
  binary_operator(f_exp, FExp), !.

f_exp(FExp) -->
  f_head(FExp).

number(Number) -->
  lookup(Resource),
  {
    once((
      rdf_equal(^^(Number, xsd:integer), Resource)
    ; rdf_equal(^^(Number, xsd:decimal), Resource)
    ))
  }.

binary_operator(Rule, ["(", Label, " ", Arguments, ")"]) -->
  individual_of(pddl:'BinaryOperator'),
  label(Label),
  operator_arguments(Rule, Arguments).

f_head(Label) -->
  individual_of(pddl:'Function'),
  label(Label),
  lookup(Resource),
  {
    rdf0(Resource, rdf:type, FunctionType),
    \+ rdf(FunctionType, pddl:parameter, _), !
  }.

f_head(["(", Label, Arguments, ")"]) -->
  individual_of(pddl:'Function'),
  label(Label),
  lookup(Resource),
  {
    rdf0(Resource, rdf:type, FunctionType),
    findall(Parameter,
      rdf(FunctionType, pddl:parameter, Parameter),
    Parameters),
    predsort(order_compare, Parameters, SortedParameters),
    predicate_arguments([parameter, object], Resource, SortedParameters, Arguments)
  }.

effect(Effect) -->
  and(c_effect, Effect), !.

effect(Effect) -->
  c_effect(Effect).

and(Rule, ["(", Label, indent(2, Arguments), ")\n"]) -->
  of_type(pddl:'And'),
  label(Label),
  operator_arguments(Rule, Arguments).

c_effect(CEffect) -->
  for_all(CEffect), !.

c_effect(CEffect) -->
  when_effect(CEffect), !.

c_effect(CEffect) -->
  p_effect(CEffect).

for_all(["(", Label, " (", Variables, ")", indent(2, Arguments), ")\n"]) -->
  individual_of(pddl:'ForAll'),
  label(Label),
  apply_to_properties(pddl:parameter, variable, VariableList),
  {
    ( nonempty(VariableList)
    -> flatten(VariableList, [_|Variables])
    ; Variables = []
    )
  },
  apply_to_property(pddl:argument, effect, Arguments).

variable([" "|Variable]) -->
  parameter(Variable).

when_effect(["(", Label, indent(2, [GD, CondEffect]), ")\n"]) -->
  individual_of(pddl:'When'),
  label(Label),
  apply_to_property(pddl:parameter, goal_description, GD),
  apply_to_property(pddl:argument, cond_effect, CondEffect).

cond_effect(CondEffect) -->
  and(p_effect, CondEffect), !.

cond_effect(CondEffect) -->
  p_effect(CondEffect).

p_effect(PEffect) -->
  assignment_operator(PEffect), !.

p_effect(PEffect) -->
  literal([parameter, object], PEffect).

assignment_operator(["(", Label, " ", FHead, " ", FExp, ")\n"]) -->
  individual_of(pddl:'AssignmentOperator'),
  label(Label),
  apply_to_property(pddl:parameter, f_head, FHead),
  apply_to_property(pddl:argument, f_exp, FExp).

problem(["(define (problem ", Label, ")",
           indent(2, ["(:domain ", Domain, ")"]),
           indent(2, Objects),
           indent(2, Init),
           indent(2, Goal),
           indent(2, Metric),
         "\n", ")\n"]) -->
  of_type(pddl:'Problem'),
  label(Label),
  apply_to_property(pddl:domain, label, Domain),
  objects(Objects),
  init(Init),
  goal(Goal),
  metric(Metric).

objects(["(:objects ", Objects, ")"]) -->
  all_properties(pddl:object, ObjectResources),
  {
    nonempty(ObjectResources), !,
    group_typed_things(ObjectResources, GrouppedObjects, rdf:type),
    groupped_typed_things(object, rdf:type, GrouppedObjects, Objects)
  }.

objects([]) --> [].

init(["(:init", indent(2, Init), ")"]) -->
  apply_to_properties(pddl:init, init_el, Init).

init_el(InitEl) -->
  literal([object], InitEl), !.

init_el(InitEl) -->
  eq(InitEl).

eq(["(", Label, " ", FHead, " ", Number, ")\n"]) -->
  of_type(pddl:'EQ'),
  label(Label),
  apply_to_property(pddl:left, f_head, FHead),
  apply_to_property(pddl:right, number, Number).

goal(["(:goal", indent(2, GD), ")"]) -->
  apply_to_property(pddl:goal, goal_description, GD).

metric(["(:metric ", Metric, " ", GroundFExp, ")"]) -->
  lookup(Resource),
  {
    once((
      rdf0(Resource, pddl:minimize, Criteria),
      Metric = "minimize"
    ; rdf0(Resource, pddl:maximize, Criteria),
      Metric = "maximize"
    )), !,
    ground_f_exp(GroundFExp, [Criteria], [Criteria])
  }.

metric([]) --> [].

ground_f_exp(GroundFExp) -->
  f_head(GroundFExp), !.

ground_f_exp(GroundFExp) -->
  binary_operator(ground_f_exp, GroundFExp), !.

ground_f_exp(GroundFExp) -->
  number(GroundFExp).

test :-
  generate_pddl(pddle:'adl-blocksworld', 'http://ontology.cf.ericsson.net/pddl_example', String1),
  writeln(String1),
  generate_pddl(pddle:'adl-blocksworld-problem', 'http://ontology.cf.ericsson.net/pddl_example', String2),
  writeln(String2).
