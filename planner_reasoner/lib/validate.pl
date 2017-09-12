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

:- thread_local context/1.

validate_plan(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, Plan, PlanGraph) :-
  generate_pddl(Plan, PlanGraph, PlanString),
  setup_call_cleanup(tmp_file_stream(text, PlanFile, PlanStream), (
    write(PlanStream, PlanString),
    close(PlanStream),
    setup_call_cleanup(true, (
      tmp(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, Plan, PlanGraph, PlanFile)
    ), (retractall(context(_)), close(ValidPlanStream)))
  ), delete_file(PlanFile)).

tmp(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, Plan, PlanGraph, PlanFile) :-
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
  read_lines(ValidPlanStream, SortedSteps).

read_lines(Out, SortedSteps) :-
  read_line_to_codes(Out, Line),
  read_lines(Line, Out, SortedSteps).

read_lines(end_of_file, _, _) :- !.
read_lines(Codes, Out, SortedSteps) :-
  atom_codes(A, Codes),
  atom_concat(A, '\n', AA),
  print(AA),
  ignore(phrase(parse(SortedSteps, NewSortedSteps), Codes, [])),
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
  {true}. %deleting(Step), !.

parse([Step|T], [Step|T]) -->
  {true}. %updating(Step).

checking -->
  ("Checking next happening (time ", float(_), ")").

adding(Step) -->
  ("Adding (", Predicate, arguments(Arguments)),
  {
    length(Arguments, Arity),
    find_predicate(Predicate, Arity, PredicateResource),
    rdf_create_bnode(PBN),
    create_resource(PBN, [PredicateResource], [], [], rdf(Context.plan_graph)),
    findall(ParameterResource,
      rdf(PredicateResource, pddl:parameter, ParameterResource, Context.domain_graph),
    ParameterResources),
    predsort(order_compare, ParameterResources, SortedParameters),
    predicate_arguments(SortedParameters, Arguments, PBN, Context),
    rdf_assert(Step, pddl:adding, PBN, Context.plan_graph)
  }.

arguments([H|T]) -->
  (" " ; ")"),
  string_without(" )", Codes),
  { atom_codes(H, Codes) },
  arguments(T).

arguments([]) --> [].

find_predicate(PredicatePattern, Arity, PredicateResource) :-
  context(Context),
  { icase(PredicateLabel, PredicatePattern) },
  rdf(PredicateResource, rdfs:label, PredicateLabel^^xsd:string, Context.domain_graph),
  rdfs_subclass_of(PredicateResource, pddl:'Predicate'),
  aggregate_all(count, rdf(PredicateResource, pddl:parameter, _), Arity).

predicate_arguments([], [], _, _) :- !.
predicate_arguments([SortedParameter|T], [Argument|T2], PBN, Context) :-
  find_object(Argument, ArgumentResource),
  oslc_resource(PBN, [SortedParameter=ArgumentResource], rdf(Context.plan_graph)),
  predicate_arguments(T, T2, PBN, Context).

find_object(ObjectPattern, ObjectResource) :-
  context(Context),
  { icase(ObjectLabel, ObjectPattern) },
  ( rdf(ObjectResource, rdfs:label, ObjectLabel^^xsd:string, Context.domain_graph)
  ; rdf(ObjectResource, rdfs:label, ObjectLabel^^xsd:string, Context.problem_graph)
  ),
  rdfs_individual_of(ObjectResource, pddl:'PrimitiveType').
