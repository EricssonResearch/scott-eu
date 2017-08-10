:- module(pddl_generator, [
    generate_pddl/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).

:- rdf_meta type(-, r, -).

:- rdf_register_prefix(sh, 'http://www.w3.org/ns/shacl#').

generate_pddl(Resource, Stream) :-
  domain(Resource, Stream, 0).
  %generate_problem(Resource, Stream).

domain(Resource, Stream, Indent) :-
  rdf(Resource, rdf:type, pddl:'Domain'),
  rdf(Resource, rdfs:label, Label^^xsd:string),
  format(Stream, '~*|(define (domain ~w)~n', [Indent, Label]),
  NewIndent is Indent + 2,
  types(Resource, Stream, NewIndent),
  predicates(Resource, Stream, NewIndent),
  functions(Resource, Stream, NewIndent),
  actions(Resource, Stream, NewIndent),
  format(Stream, '~*|)~n', [Indent]).

types(Resource, Stream, Indent) :-
  format(Stream, '~*|(:types', [Indent]),
  forall(
    rdf(Resource, pddl:type, Type), (
    rdf(Type, rdf:type, TypeType),
    type(Type, TypeType, Stream)
  )),
  format(Stream, '~*|)~n', [Indent]).

type(Type, pddl:'PrimitiveType', Stream) :- !,
  rdf(Type, rdfs:label, Label^^xsd:string),
  format(Stream, ' ~w', [Label]).

type(Type, pddl:'EitherType', Stream) :- !,
  format(Stream, ' (either', []),
  forall(
    rdf(Type, rdfs:member, Member), (
    rdf(Member, rdf:type, MemberType),
    type(Member, MemberType, Stream)
  )),
  format(Stream, ')', []).

predicates(Resource, Stream, Indent) :-
  format(Stream, '~*|(:predicates~n', [Indent]),
  NewIndent is Indent + 2,
  forall(
    rdf(Resource, pddl:predicate, Predicate),
    predicate(Predicate, Stream, NewIndent)
  ),
  format(Stream, '~*|)~n', [Indent]).

predicate(Predicate, Stream, Indent) :-
  rdf(Predicate, rdfs:label, Label^^xsd:string),
  format(Stream, '~*|(~w', [Indent, Label]),
  findall(Parameter,
    rdf(Predicate, pddl:parameter, Parameter),
    Parameters
  ),
  parameters(Parameters, Stream),
  format(Stream, ')~n', []).

parameters(Parameters, Stream) :-
  predsort(param_compare, Parameters, SortedParameters),
  group_parameters(SortedParameters, GrouppedParameters),
  parameters0(GrouppedParameters, Stream).

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

parameters0([], _) :- !.
parameters0([Parameter|T], Stream) :-
  parameter_group(Parameter, Stream),
  parameters0(T, Stream).

param_compare(Ineq, Param1, Param2) :-
  rdf(Param1, sh:order, P1^^xsd:integer),
  rdf(Param2, sh:order, P2^^xsd:integer),
  ( P1 < P2
  -> Ineq = <
  ; Ineq = >
  ).

parameter_group([], _) :- !.

parameter_group([Parameter], Stream) :- !,
  parameter(Parameter, Stream),
  rdf(Parameter, pddl:type, Type),
  rdf(Type, rdf:type, TypeType),
  format(Stream, ' -', []),
  type(Type, TypeType, Stream).

parameter_group([Parameter|T], Stream) :-
  parameter(Parameter, Stream),
  parameter_group(T, Stream).

parameter(Parameter, Stream) :-
  rdf(Parameter, rdfs:label, Label^^xsd:string),
  format(Stream, ' ?~w', [Label]).

functions(Resource, Stream, Indent) :-
  format(Stream, '~*|(:functions~n', [Indent]),
  NewIndent is Indent + 2,
  forall(
    rdf(Resource, pddl:function, Function),
    function(Function, Stream, NewIndent)
  ),
  format(Stream, '~*|)~n', [Indent]).

function(Function, Stream, Indent) :-
  rdf(Function, rdfs:label, Label^^xsd:string),
  format(Stream, '~*|(~w', [Indent, Label]),
  findall(Parameter,
    rdf(Function, pddl:parameter, Parameter),
    Parameters
  ),
  parameters(Parameters, Stream),
  format(Stream, ')~n', []).

actions(Resource, Stream, Indent) :-
  forall(
    rdf(Resource, pddl:action, Action),
    action(Action, Stream, Indent)
  ).

action(Action, Stream, Indent) :-
  rdf(Action, rdfs:label, Label^^xsd:string),
  format(Stream, '~*|(:action ~w~n', [Indent, Label]),
  NewIndent is Indent + 2,
  format(Stream, '~*|:parameters (', [NewIndent, Label]),
  findall(Parameter,
    rdf(Action, pddl:parameter, Parameter),
    Parameters
  ),
  parameters(Parameters, Stream),
  format(Stream, ')~n', []),
  NewIndent2 is NewIndent + 2,
  format(Stream, '~*|:precondition~n', [NewIndent, Label]),
  rdf(Action, pddl:precondition, Precondition),
  precondition(Precondition, Stream, NewIndent2),
  format(Stream, '~*|:effect~n', [NewIndent, Label]),
  %rdf(Action, pddl:effect, Effect),
  %exp(Effect, Stream, NewIndent2),
  format(Stream, '~*|)~n', [Indent]).

precondition(Precondition, Stream, Indent) :-
  rdfs_individual_of(Precondition, pddl:'LogicalOperator'),
  operator(Precondition, Stream, Indent, precondition).

precondition(Precondition, Stream, Indent) :-
  rdfs_individual_of(Precondition, pddl:'BinaryComparator'),
  operator(Precondition, Stream, Indent, fexp).

precondition(Precondition, Stream, Indent) :-
  rdfs_individual_of(Precondition, pddl:'Predicate'),
  predicate_call(Precondition, Stream, Indent).

operator(Operator, Stream, Indent, Continuation) :-
  find_label(Operator, Label),
  format(Stream, '~*|(~w~n', [Indent, Label]),
  NewIndent is Indent + 2,
  findall(Parameter, (
    rdf(Operator, Parameter, _),
    rdf(Parameter, rdf:type, pddl:'Parameter')
  ), Parameters),
  predsort(param_compare, Parameters, SortedParameters),
  operator0(Operator, SortedParameters, Stream, NewIndent, Continuation),
  format(Stream, '~*|)~n', [Indent]).

operator0(_, [], _, _, _) :- !.
operator0(Operator, [Parameter|T], Stream, Indent, Continuation) :-
  rdf(Operator, Parameter, Argument),
  Term =.. [Continuation, Argument, Stream, Indent],
  call(Term),
  operator0(Operator, T, Stream, Indent, Continuation).

fexp(Fexp, Stream, Indent).

predicate_call(Predicate, Stream, Indent) :-
  find_label(Predicate, Label),
  format(Stream, '~*|(~w', [Indent, Label]),
  rdf(Predicate, rdf:type, PredicateType),
  findall(Parameter, rdf(PredicateType, pddl:parameter, Parameter), Parameters),
  predsort(param_compare, Parameters, SortedParameters),
  predicate_arguments(Predicate, SortedParameters, Stream),
  format(Stream, '~*|)~n', [Indent]).

predicate_arguments(_, [], _) :- !.
predicate_arguments(Predicate, [Parameter|T], Stream) :-
  rdf(Predicate, Parameter, Argument),
  rdf(Argument, rdf:type, ArgumentType),
  find_label(Argument, Label),
  predicate_argument(ArgumentType, Label, Stream),
  predicate_arguments(Predicate, T, Stream).

predicate_argument(pddl:'Parameter', Label, Stream) :-
  format(Stream, ' ?~w', [Label]).

predicate_argument(pddl:'Object', Label, Stream) :-
  format(Stream, ' ~w', [Label]).

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
