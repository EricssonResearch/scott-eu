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

:- module(validate, [
  validate_plan/8
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).
:- use_module(library(dcg/basics)).
:- use_module(library(oslc)).
:- use_module(library(pddl_generator)).

:- rdf_meta validate_plan(r, -, -, r, -, -, r, -).
:- rdf_meta callable_bnode(-, r, -, -).

:- thread_local context/1.

validate_plan(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, Plan, PlanGraph) :-
  generate_pddl(Plan, PlanGraph, PlanString),
  setup_call_cleanup(tmp_file_stream(text, PlanFile, PlanStream), (
    write(PlanStream, PlanString),
    close(PlanStream),
    setup_call_cleanup(true, (
      process_create('validate', ['-v', DomainFile, ProblemFile, PlanFile], [stdout(pipe(ValidPlanStream))]),
      findall(StepResource,
        rdf(Plan, pddl:step, StepResource, PlanGraph),
      StepResources),
      predsort(order_compare, StepResources, SortedSteps),
      Context = _{
             domain : Domain,
       domain_graph : DomainGraph,
            problem : Problem,
      problem_graph : ProblemGraph,
         plan_graph : PlanGraph },
      assertz(context(Context)),
      read_lines(ValidPlanStream, [_|SortedSteps])
    ), (retractall(context(_)), close(ValidPlanStream)))
  ), delete_file(PlanFile)).

read_lines(Out, SortedSteps) :-
  read_line_to_codes(Out, Line),
  read_lines(Line, Out, SortedSteps).

read_lines(end_of_file, _, _) :- !.
read_lines(Codes, Out, SortedSteps) :-
  (phrase(parse(SortedSteps, NewSortedSteps), Codes, [])
  -> true
  ; NewSortedSteps = SortedSteps
  ),
  read_line_to_codes(Out, Line),
  read_lines(Line, Out, NewSortedSteps).

order_compare(Ineq, Thing1, Thing2) :-
  rdf(Thing1, sh:order, Order1^^xsd:integer),
  rdf(Thing2, sh:order, Order2^^xsd:integer),
  ( Order1 =< Order2
  -> Ineq = <
  ; Ineq = >
  ).

parse([_|T], T) -->
  checking, !.

parse([Step|T], [Step|T]) -->
  adding(Step), !.

parse([Step|T], [Step|T]) -->
  deleting(Step), !.

parse([Step|T], [Step|T]) -->
  updating(Step).

checking -->
  "Checking next happening (time ", string(_), eos.

adding(Step) -->
  ("Adding (", callable_bnode(PBN, pddl:'Predicate'), ")"),
  {
    context(Context),
    rdf_assert(Step, pddl:adding, PBN, Context.plan_graph)
  }.

deleting(Step) -->
  ("Deleting (", callable_bnode(PBN, pddl:'Predicate'), ")"),
  {
    context(Context),
    rdf_assert(Step, pddl:deleting, PBN, Context.plan_graph)
  }.

updating(Step) -->
  "Updating (", callable_bnode(FBN, pddl:'Function'), ") (",
  integer(Current), ") by ", integer(Argument), " ", label(AssignmentOperator),
  {
    context(Context),
    rdf_create_bnode(UBN),
    find_assignment_operator(AssignmentOperator, AssignmentOperatorType),
    create_resource(UBN, [AssignmentOperatorType], [pddl:'AssignmentOperatorShape'], [
      pddl:current = '^^'(Current, xsd:integer),
      pddl:parameter = FBN,
      pddl:argument = '^^'(Argument, xsd:integer)
    ], rdf(Context.plan_graph)),
    rdf_assert(Step, pddl:updating, UBN, Context.plan_graph)
  }.

callable_bnode(CBN, Class) -->
  label(Callable),
  arguments(Arguments),
  {
    length(Arguments, Arity),
    find_callable(Callable, Arity, CallableResource, Class),
    rdf_create_bnode(CBN),
    context(Context),
    create_resource(CBN, [CallableResource], [], [], rdf(Context.plan_graph)),
    findall(ParameterResource,
      rdf(CallableResource, pddl:parameter, ParameterResource, Context.domain_graph),
    ParameterResources),
    predsort(order_compare, ParameterResources, SortedParameters),
    callable_arguments(SortedParameters, Arguments, CBN, Context)
  }.

label(Label) -->
  string_without(" )\n", Codes),
  { atom_codes(Label, Codes) }.

arguments([H|T]) -->
  " ",
  string_without(" )", Codes),
  { atom_codes(H, Codes) },
  arguments(T).

arguments([]) --> [], !.

find_callable(CallablePattern, Arity, CallableResource, Class) :-
  context(Context),
  { icase(CallableLabel, CallablePattern) },
  rdf(CallableResource, rdfs:label, CallableLabel^^xsd:string, Context.domain_graph),
  rdfs_subclass_of(CallableResource, Class),
  aggregate_all(count, rdf(CallableResource, pddl:parameter, _), Arity).

callable_arguments([], [], _, _) :- !.
callable_arguments([SortedParameter|T], [Argument|T2], PBN, Context) :-
  find_object(Argument, ArgumentResource),
  oslc_resource(PBN, [SortedParameter=ArgumentResource], rdf(Context.plan_graph)),
  callable_arguments(T, T2, PBN, Context).

find_object(ObjectPattern, ObjectResource) :-
  context(Context),
  { icase(ObjectLabel, ObjectPattern) },
  ( rdf(ObjectResource, rdfs:label, ObjectLabel^^xsd:string, Context.domain_graph)
  ; rdf(ObjectResource, rdfs:label, ObjectLabel^^xsd:string, Context.problem_graph)
  ),
  rdfs_individual_of(ObjectResource, pddl:'PrimitiveType').

find_assignment_operator(AssignmentOperatorPattern, AssignmentOperatorType) :-
  { icase(ObjectLabel, AssignmentOperatorPattern) },
  rdf(AssignmentOperatorType, rdfs:label, ObjectLabel^^xsd:string),
  rdfs_subclass_of(AssignmentOperatorType, pddl:'AssignmentOperator').
