:- module(pddl_generator, [
    generate_pddl/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).

:- rdf_meta type(r, -).
:- rdf_meta predicate_argument(r, -).
:- rdf_meta fexp(t, -).

:- rdf_register_prefix(sh, 'http://www.w3.org/ns/shacl#').

generate_pddl(Resource, Stream) :-
  with_output_to(
    Stream, (
    domain(Resource, 0)
  )).
  %generate_problem(Resource, 0).

domain(Resource, Indent) :-
  rdf(Resource, rdf:type, pddl:'Domain'),
  rdf(Resource, rdfs:label, Label^^xsd:string),
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
    rdf(Type, rdf:type, TypeType),
    type(TypeType, Type)
  )),
  format('~*|)~n', [Indent]).

type(pddl:'PrimitiveType', Type) :-
  rdf(Type, rdfs:label, Label^^xsd:string),
  format(' ~w', [Label]).

type(pddl:'EitherType', Type) :-
  format(' (either', []),
  forall(
    rdf(Type, rdfs:member, Member), (
    rdf(Member, rdf:type, MemberType),
    type(MemberType, Member)
  )),
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
  rdf(Predicate, rdfs:label, Label^^xsd:string),
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
  predsort(param_compare, Parameters, SortedParameters),
  group_parameters(SortedParameters, GrouppedParameters),
  groupped_parameters(GrouppedParameters).

group_parameters(Parameters, GrouppedParameters) :-
  group_parameters0(_, Parameters, [], GrouppedParameters).

group_parameters0(_, [], A, [A]) :- !.

group_parameters0(Type, [H|T], A, O) :-
  rdf(H, pddl:type, HType),
  group_parameters1(Type, HType, [H|T], A, O).

group_parameters1(Type, Type, [H|T], A, O) :- !,
  append(A, [H], NA),
  group_parameters0(Type, T, NA, O).

group_parameters1(_, Type, [H|T], A, [A|T2]) :-
  group_parameters0(Type, T, [H], T2).

param_compare(Ineq, Param1, Param2) :-
  rdf(Param1, sh:order, P1^^xsd:integer),
  rdf(Param2, sh:order, P2^^xsd:integer),
  ( P1 =< P2
  -> Ineq = <
  ; Ineq = >
  ).

groupped_parameters([]) :- !.
groupped_parameters([H|T]) :-
  parameter_group(H),
  ( T \== []
  -> format(' ')
  ; true
  ),
  groupped_parameters(T).

parameter_group([]) :- !.
parameter_group([Parameter]) :- !,
  parameter(Parameter),
  rdf(Parameter, pddl:type, Type),
  rdf(Type, rdf:type, TypeType),
  format(' -'),
  type(TypeType, Type).

parameter_group([Parameter|T]) :-
  parameter(Parameter),
  ( T \== []
  -> format(' ')
  ; true
  ),
  parameter_group(T).

parameter(Parameter) :-
  rdf(Parameter, rdfs:label, Label^^xsd:string),
  format('?~w', [Label]).

functions(Resource, Indent) :-
  format('~*|(:functions~n', [Indent]),
  NewIndent is Indent + 2,
  forall(
    rdf(Resource, pddl:function, Function),
    function(Function, NewIndent)
  ),
  format('~*|)~n', [Indent]).

function(Function, Indent) :-
  rdf(Function, rdfs:label, Label^^xsd:string),
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
  rdf(Action, rdfs:label, Label^^xsd:string),
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

fexp(Integer^^xsd:integer, _) :-
  format(' ~w', [Integer]).

fexp(Decimal^^xsd:decimal, _) :-
  format(' ~w', [Decimal]).

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
  predsort(param_compare, Parameters, SortedParameters),
  predicate_arguments(Predicate, SortedParameters),
  format(')').

predicate_arguments(_, []) :- !.
predicate_arguments(Predicate, [Parameter|T]) :-
  rdf(Predicate, Parameter, Argument),
  rdf(Argument, rdf:type, ArgumentType),
  find_label(Argument, Label),
  predicate_argument(ArgumentType, Label),
  predicate_arguments(Predicate, T).

predicate_argument(pddl:'Parameter', Label) :-
  format(' ?~w', [Label]).

predicate_argument(pddl:'Object', Label) :-
  format(' ~w', [Label]).

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
  rdf(Variable, rdfs:label, Label^^xsd:string),
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

find_label(Resource, Label) :-
  once((
    rdf(Resource, rdfs:label, Label^^xsd:string)
  ; rdfs_individual_of(Resource, Class),
    rdf(Class, rdfs:label, Label^^xsd:string)
  )).

test :-
  current_output(Out),
  rdf_global_id(pddld:'adl-blocksworld', Resource),
  generate_pddl(Resource, Out).
