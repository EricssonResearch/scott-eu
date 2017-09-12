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

:- module(metric_ff, [
  generate_plan/6,
  generate_plan/8
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).
:- use_module(library(dcg/basics)).
:- use_module(library(oslc)).
:- use_module(library(pddl_generator)).

:- rdf_meta generate_plan(r, -, r, -, r, -).
:- rdf_meta generate_plan(r, -, -, r, -, -, r, -).

:- thread_local context/1.

generate_plan(Domain, DomainGraph, Problem, ProblemGraph, Plan, PlanGraph) :-
  generate_plan0(Domain, DomainGraph, _, Problem, ProblemGraph, _, Plan, PlanGraph, false).

generate_plan(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, Plan, PlanGraph) :-
  generate_plan0(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, Plan, PlanGraph, true).

generate_plan0(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, Plan, PlanGraph, KeepFiles) :-
  generate_pddl(Domain, DomainGraph, DomainString),
  generate_pddl(Problem, ProblemGraph, ProblemString),
  setup_call_cleanup(tmp_file_stream(text, DomainFile, DomainStream), (
    write(DomainStream, DomainString),
    close(DomainStream),
    setup_call_cleanup(tmp_file_stream(text, ProblemFile, ProblemStream), (
      write(ProblemStream, ProblemString),
      close(ProblemStream),
      setup_call_cleanup(true, (
        process_create('ff', ['-o', DomainFile, '-f', ProblemFile], [stdout(pipe(PlanStream))]),
        create_resource(Plan, [pddl:'Plan'], [pddl:'PlanShape'], [], rdf(PlanGraph)),
        Context = _{ plan : Plan,
               plan_graph : PlanGraph,
             domain_graph : DomainGraph,
            problem_graph : ProblemGraph },
        assertz(context(Context)),
        read_lines(PlanStream)
      ), (retractall(context(_)), close(PlanStream)))
    ), (KeepFiles ; delete_file(ProblemFile)))
  ), (KeepFiles ; delete_file(DomainFile))).

read_lines(Out) :-
  read_line_to_codes(Out, Line),
  read_lines(Line, Out).

read_lines(end_of_file, _) :- !.
read_lines(Codes, Out) :-
  ignore(phrase(parse, Codes, [])),
  read_line_to_codes(Out, Line),
  read_lines(Line, Out).

parse -->
  action, !.

parse -->
  cost, !.

parse -->
  time.

action -->
  ("step", ! ; []),
  step(Action, Parameters, Order),
  {
    context(Context),
    length(Parameters, Arity),
    find_action(Action, Arity, ActionResource),
    rdf_create_bnode(ABN),
    create_resource(ABN, [ActionResource], [], [], rdf(Context.plan_graph)),
    findall(ParameterResource,
      rdf(ActionResource, pddl:parameter, ParameterResource, Context.domain_graph),
    ParameterResources),
    predsort(order_compare, ParameterResources, SortedParameters),
    action_parameters(SortedParameters, Parameters, ABN, Context),
    rdf_create_bnode(SBN),
    create_resource(SBN,
      [pddl:'Step'],
      [pddl:'StepShape'],
      [pddl:action=ABN, sh:order='^^'(Order, xsd:integer)],
      rdf(Context.plan_graph)
    ),
    rdf_assert(Context.plan, pddl:step, SBN, Context.plan_graph)
  }.

find_action(ActionPattern, Arity, ActionResource) :-
  context(Context),
  { icase(ActionLabel, ActionPattern) },
  rdf(ActionResource, rdfs:label, ActionLabel^^xsd:string, Context.domain_graph),
  rdfs_subclass_of(ActionResource, pddl:'Action'),
  aggregate_all(count, rdf(ActionResource, pddl:parameter, _), Arity).

order_compare(Ineq, Thing1, Thing2) :-
  rdf(Thing1, sh:order, Order1^^xsd:integer),
  rdf(Thing2, sh:order, Order2^^xsd:integer),
  ( Order1 =< Order2
  -> Ineq = <
  ; Ineq = >
  ).

action_parameters([], [], _, _) :- !.
action_parameters([SortedParameter|T], [Parameter|T2], ABN, Context) :-
  find_object(Parameter, ObjectResource),
  oslc_resource(ABN, [SortedParameter=ObjectResource], rdf(Context.plan_graph)),
  action_parameters(T, T2, ABN, Context).

find_object(ObjectPattern, ObjectResource) :-
  context(Context),
  { icase(ObjectLabel, ObjectPattern) },
  ( rdf(ObjectResource, rdfs:label, ObjectLabel^^xsd:string, Context.domain_graph)
  ; rdf(ObjectResource, rdfs:label, ObjectLabel^^xsd:string, Context.problem_graph)
  ),
  rdfs_individual_of(ObjectResource, pddl:'PrimitiveType').

step(Action, Parameters, Order) -->
  whites,
  integer(OrderM1), { Order is OrderM1 + 1 },
  ":", whites,
  string_without(" \n", Codes),
  { atom_codes(Action, Codes) },
  parameters(Parameters).

parameters([H|T]) -->
  (" " ; "\n"),
  string_without(" \n", Codes),
  { atom_codes(H, Codes) },
  parameters(T).

parameters([]) --> [].

cost -->
  "plan cost:", whites, float(Cost),
  {
    context(Context),
    oslc_resource(Context.plan, [pddl:cost='^^'(Cost, xsd:decimal)], rdf(Context.plan_graph))
  }.

time -->
  whites, float(Time), whites, "seconds total time",
  {
    context(Context),
    oslc_resource(Context.plan, [pddl:time='^^'(Time, xsd:decimal)], rdf(Context.plan_graph))
  }.
