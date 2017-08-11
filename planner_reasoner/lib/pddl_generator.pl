:- module(pddl_generator, [
    generate_pddl/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).

generate_pddl(Resource, Stream) :-
  with_output_to(
    Stream, (
    once((
      domain(Resource, 0)
    ; problem(Resource, 0)
    ))
  )).

domain(Resource, Indent) :-
  rdf(Resource, rdf:type, pddl:'Domain'),
  find_label(Resource, Label),
  format('~*|(define (domain ~w)~n', [Indent, Label]),
  NewIndent is Indent + 2,
  types(Resource, NewIndent),
  predicates(Resource, NewIndent),
  functions(Resource, NewIndent),
  actions(Resource, NewIndent),
  format('~*|)~n', [Indent]).

types(Resource, Indent) :-
  format('~*|(:types', [Indent]),
  forall(
    rdf(Resource, pddl:type, Type), (
    type(Type)
  )),
  format('~*|)~n', [Indent]).

type(Type) :-
  rdf(Type, rdf:type, pddl:'PrimitiveType'),
  find_label(Type, Label),
  format(' ~w', [Label]).

type(Type) :-
  rdf(Type, rdf:type, pddl:'EitherType'),
  format(' (either', []),
  forall(
    rdf(Type, rdfs:member, Member),
    type(Member)
  ),
  format(')').

predicates(Resource, Indent) :-
  format('~*|(:predicates~n', [Indent]),
  NewIndent is Indent + 2,
  forall(
    rdf(Resource, pddl:predicate, Predicate),
    predicate(Predicate, NewIndent)
  ),
  format('~*|)~n', [Indent]).

predicate(Predicate, Indent) :-
  find_label(Predicate, Label),
  format('~*|(~w', [Indent, Label]),
  findall(Parameter,
    rdf(Predicate, pddl:parameter, Parameter),
    Parameters
  ),
  ( Parameters \== []
  -> format(' ')
  ; true
  ),
  parameters(Parameters),
  format(')~n').

parameters(Parameters) :-
  predsort(order_compare, Parameters, SortedParameters),
  group_typed_things(SortedParameters, GrouppedParameters),
  groupped_typed_things(GrouppedParameters).

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

order_compare(Ineq, Thing1, Thing2) :-
  rdf(Thing1, sh:order, Order1^^xsd:integer),
  rdf(Thing2, sh:order, Order2^^xsd:integer),
  ( Order1 =< Order2
  -> Ineq = <
  ; Ineq = >
  ).

groupped_typed_things([]) :- !.
groupped_typed_things([H|T]) :-
  typed_thing_group(H),
  ( T \== []
  -> format(' ')
  ; true
  ),
  groupped_typed_things(T).

typed_thing_group([]) :- !.
typed_thing_group([Thing]) :- !,
  typed_thing(Thing),
  rdf(Thing, pddl:type, Type),
  format(' -'),
  type(Type).

typed_thing_group([Thing|T]) :-
  typed_thing(Thing),
  ( T \== []
  -> format(' ')
  ; true
  ),
  typed_thing_group(T).

typed_thing(Thing) :-
  find_label(Thing, Label),
  ( rdf(Thing, rdf:type, pddl:'Parameter')
  -> format('?~w', [Label])
  ; ( rdf(Thing, rdf:type, pddl:'Object')
    -> format('~w', [Label])
    ; fail
    )
  ).

functions(Resource, Indent) :-
  format('~*|(:functions~n', [Indent]),
  NewIndent is Indent + 2,
  forall(
    rdf(Resource, pddl:function, Function),
    function(Function, NewIndent)
  ),
  format('~*|)~n', [Indent]).

function(Function, Indent) :-
  find_label(Function, Label),
  format('~*|(~w', [Indent, Label]),
  findall(Parameter,
    rdf(Function, pddl:parameter, Parameter),
    Parameters
  ),
  ( Parameters \== []
  -> format(' ')
  ; true
  ),
  parameters(Parameters),
  format(')~n').

actions(Resource, Indent) :-
  forall(
    rdf(Resource, pddl:action, Action),
    action(Action, Indent)
  ).

action(Action, Indent) :-
  find_label(Action, Label),
  format('~*|(:action ~w~n', [Indent, Label]),
  NewIndent is Indent + 2,
  format('~*|:parameters (', [NewIndent, Label]),
  findall(Parameter,
    rdf(Action, pddl:parameter, Parameter),
    Parameters
  ),
  parameters(Parameters),
  NewIndent2 is NewIndent + 2,
  format(')~n~*|:precondition', [NewIndent, Label]),
  rdf(Action, pddl:precondition, Precondition),
  precondition(Precondition, NewIndent2),
  format('~n~*|:effect', [NewIndent, Label]),
  rdf(Action, pddl:effect, Effect),
  effect(Effect, NewIndent2),
  format('~n~*|)~n', [Indent]).

precondition(Operator, Indent) :-
  rdfs_individual_of(Operator, pddl:'LogicalOperator'),
  format('~n'),
  operator(Operator, Indent, precondition).

precondition(Comparator, Indent) :-
  rdfs_individual_of(Comparator, pddl:'BinaryComparator'),
  format('~n'),
  operator(Comparator, Indent, fexp).

precondition(Predicate, Indent) :-
  rdfs_individual_of(Predicate, pddl:'Predicate'),
  format('~n'),
  predicate_call(Predicate, Indent).

precondition(Quantifier, Indent) :-
  rdfs_individual_of(Quantifier, pddl:'Quantifier'),
  format('~n'),
  precondition_quantifier(Quantifier, Indent).

operator(Operator, Indent, Continuation) :-
  find_label(Operator, Label),
  format('~*|(~w', [Indent, Label]),
  NewIndent is Indent + 2,
  findall([Parameter,Argument], (
    rdf(Operator, Parameter, Argument),
    rdf(Parameter, rdf:type, pddl:'Parameter')
  ), ParamArgs),
  predsort(arg_compare, ParamArgs, SortedParamArgs),
  operator0(Operator, SortedParamArgs, NewIndent, Continuation),
  format('~*|)', [Indent]).

operator0(_, [], _, _) :- !.
operator0(Operator, [[_,Argument]|T], Indent, Continuation) :-
  Term =.. [Continuation, Argument, Indent],
  call(Term),
  operator0(Operator, T, Indent, Continuation).

arg_compare(Ineq, [Param1,_], [Param2,_]) :-
  rdf(Param1, sh:order, P1^^xsd:integer),
  rdf(Param2, sh:order, P2^^xsd:integer),
  ( P1 =< P2
  -> Ineq = <
  ; Ineq = >
  ).

num(Number) :-
  (
    rdf_equal(^^(Value, xsd:integer), Number)
  ; rdf_equal(^^(Value, xsd:decimal), Number)
  ),
  format(' ~w', [Value]).

fexp(Number, _) :-
  num(Number).

fexp(Operator, Indent) :-
  rdfs_individual_of(Operator, pddl:'BinaryOperator'),
  format(' '),
  operator(Operator, Indent, fexp).

fexp(Function, Indent) :-
  rdfs_individual_of(Function, pddl:'Function'),
  format(' '),
  predicate_call(Function, Indent).

predicate_call(Predicate, Indent) :-
  find_label(Predicate, Label),
  format('~*|(~w', [Indent, Label]),
  rdf(Predicate, rdf:type, PredicateType),
  findall(Parameter, rdf(PredicateType, pddl:parameter, Parameter), Parameters),
  predsort(order_compare, Parameters, SortedParameters),
  predicate_arguments(Predicate, SortedParameters),
  format(')').

predicate_arguments(_, []) :- !.
predicate_arguments(Predicate, [Parameter|T]) :-
  rdf(Predicate, Parameter, Argument),
  format(' '),
  typed_thing(Argument),
  predicate_arguments(Predicate, T).

precondition_quantifier(Quantifier, Indent) :-
  find_label(Quantifier, Label),
  format('~*|(~w (', [Indent, Label]),
  findall(Parameter,
    rdf(Quantifier, pddl:parameter, Parameter),
    Parameters
  ),
  parameters(Parameters),
  format(')'),
  NewIndent is Indent + 8,
  rdf(Quantifier, pddl:argument, Argument),
  precondition(Argument, NewIndent),
  format('~*|)', [Indent]).

effect(Effect, Indent) :-
  rdfs_individual_of(Effect, pddl:'And'),
  format('~n'),
  operator(Effect, Indent, ceffect).

effect(Effect, Indent) :-
  ceffect(Effect, Indent).

ceffect(CEffect, Indent) :-
  rdfs_individual_of(CEffect, pddl:'ForAll'),
  format('~n'),
  forall_effect(CEffect, Indent).

ceffect(CEffect, Indent) :-
  rdfs_individual_of(CEffect, pddl:'When'),
  format('~n'),
  when_effect(CEffect, Indent).

ceffect(CEffect, Indent) :-
  peffect(CEffect, Indent).

forall_effect(ForAllEffect, Indent) :-
  find_label(ForAllEffect, Label),
  format('~*|(~w (', [Indent, Label]),
  findall(Parameter,
    rdf(ForAllEffect, pddl:parameter, Parameter),
    Parameters
  ),
  variables(Parameters),
  format(')'),
  NewIndent is Indent + 8,
  rdf(ForAllEffect, pddl:argument, Argument),
  effect(Argument, NewIndent),
  format('~*|)', [Indent]).

variables([]) :- !.
variables([Variable|T]) :-
  find_label(Variable, Label),
  format('?~w', [Label]),
  ( T \== []
  -> format(' ')
  ; true
  ),
  variables(T).

when_effect(WhenEffect, Indent) :-
  find_label(WhenEffect, Label),
  format('~*|(~w ', [Indent, Label]),
  rdf(WhenEffect, pddl:parameter, Parameter),
  NewIndent is Indent + 6,
  precondition(Parameter, NewIndent),
  rdf(WhenEffect, pddl:argument, Argument),
  cond_effect(Argument, NewIndent),
  format('~*|)', [Indent]).

cond_effect(CondEffect, Indent) :-
  rdfs_individual_of(CondEffect, pddl:'And'),
  format('~n'),
  operator(CondEffect, Indent, peffect).

cond_effect(CondEffect, Indent) :-
  peffect(CondEffect, Indent).

peffect(PEffect, Indent) :-
  rdfs_individual_of(PEffect, pddl:'AssignmentOperator'),
  format('~n'),
  assignment_operator(PEffect, Indent).

peffect(PEffect, Indent) :-
  rdfs_individual_of(PEffect, pddl:'Predicate'),
  format('~n'),
  predicate_call(PEffect, Indent).

peffect(PEffect, Indent) :-
  rdfs_individual_of(PEffect, pddl:'Not'),
  format('~n'),
  operator(PEffect, Indent, not_predicate_call).

not_predicate_call(Predicate, Indent) :-
  format(' '),
  predicate_call(Predicate, Indent).

assignment_operator(AssignmentOperator, Indent) :-
  find_label(AssignmentOperator, Label),
  format('~*|(~w ', [Indent, Label]),
  rdf(AssignmentOperator, pddl:parameter, Parameter),
  rdfs_individual_of(Parameter, pddl:'Function'),
  predicate_call(Parameter, Indent),
  NewIndent is Indent + 8,
  rdf(AssignmentOperator, pddl:argument, Argument),
  fexp(Argument, NewIndent),
  format('~*|)', [Indent]).

problem(Resource, Indent) :-
  rdf(Resource, rdf:type, pddl:'Problem'),
  find_label(Resource, Label),
  format('~*|(define (problem ~w)~n', [Indent, Label]),
  NewIndent is Indent + 2,
  domain_call(Resource, NewIndent),
  objects(Resource, NewIndent),
  init(Resource, NewIndent),
  goal(Resource, NewIndent),
  metric(Resource, NewIndent),
  format('~n~*|)~n', [Indent]).

domain_call(Problem, Indent) :-
  rdf(Problem, pddl:domain, Domain),
  find_label(Domain, Label),
  format('~*|(:domain ~w)~n', [Indent, Label]).

objects(Problem, Indent) :-
  findall(Object,
    rdf(Problem, pddl:object, Object),
  Objects),
  group_typed_things(Objects, GrouppedObjects),
  format('~*|(:objects ', [Indent]),
  groupped_typed_things(GrouppedObjects),
  format(')').

init(Problem, Indent) :-
  format('~n~*|(:init', [Indent]),
  NewIndent is Indent + 2,
  forall(
    rdf(Problem, pddl:init, Init),
    init_el(Init, NewIndent)
  ),
  format('~n~*|)', [Indent]).

init_el(Init, Indent) :-
  rdfs_individual_of(Init, pddl:'Predicate'),
  format('~n'),
  predicate_call(Init, Indent).

init_el(Init, Indent) :-
  rdfs_individual_of(Init, pddl:'Not'),
  format('~n'),
  operator(Init, Indent, not_predicate_call).

init_el(Init, Indent) :-
  rdfs_individual_of(Init, pddl:'EQ'),
  find_label(Init, Label),
  format('~n~*|(~w', [Indent, Label]),
  rdf(Init, pddl:left, Function),
  rdfs_individual_of(Function, pddl:'Function'),
  format(' '),
  predicate_call(Function, Indent),
  rdf(Init, pddl:right, Number),
  num(Number),
  format(')').

goal(Problem, Indent) :-
  rdf(Problem, pddl:goal, Goal),
  format('~n~*|(:goal', [Indent]),
  NewIndent is Indent + 2,
  precondition(Goal, NewIndent),
  format('~n~*|)', [Indent]).

metric(Problem, Indent) :-
  format('~n~*|(:metric ', [Indent]),
  once((
    rdf(Problem, pddl:minimize, Criteria),
    format('minimize')
  ; rdf(Problem, pddl:maximize, Criteria),
    format('maximize')
  )),
  NewIndent is Indent + 2,
  ground_fexp(Criteria, NewIndent),
  format(')').

ground_fexp(Number, _) :-
  num(Number).

ground_fexp(TotalTime, _) :-
  rdf_equal(pddl:'total-time', TotalTime),
  format(' total-time').

ground_fexp(Operator, Indent) :-
  rdfs_individual_of(Operator, pddl:'BinaryOperator'),
  format(' '),
  operator(Operator, Indent, ground_fexp).

ground_fexp(Function, Indent) :-
  rdfs_individual_of(Function, pddl:'Function'),
  format(' '),
  predicate_call(Function, Indent).

find_label(Resource, Label) :-
  once((
    rdf(Resource, rdfs:label, Label^^xsd:string)
  ; rdfs_individual_of(Resource, Class),
    rdf(Class, rdfs:label, Label^^xsd:string)
  )).

test :-
  current_output(Out),
  rdf_global_id(pddld:'adl-blocksworld', Domain),
  generate_pddl(Domain, Out),
  nl,
  rdf_global_id(pddlp:'adl-blocksworld-problem', Problem),
  generate_pddl(Problem, Out).
