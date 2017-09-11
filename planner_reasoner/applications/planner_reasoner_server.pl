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

:- module(planner_reasoner_server, []).

:- use_module(library(semweb/rdf11)).
:- use_module(library(oslc_dispatch)).
:- use_module(library(oslc_rdf)).
:- use_module(library(pddl_generator)).
:- use_module(library(metric_ff)).
:- use_module(library(validate)).

:- oslc_post(planner:pddlCreationFactory, pddl_creation_factory).
:- oslc_post(planner:planCreationFactory, plan_creation_factory).
:- oslc_post(planner:validatedPlanCreationFactory, validated_plan_creation_factory).

oslc_dispatch:serializer(text/'x-pddl', turtle).

pddl_creation_factory(Context) :-
  Context.content_type == text/'x-pddl',
  Graph = Context.graph_in,
  findall(String, (
    ( rdf(Resource, rdf:type, pddl:'Domain', Graph)
    ; rdf(Resource, rdf:type, pddl:'Problem', Graph)
    ; rdf(Resource, rdf:type, pddl:'Plan', Graph)
    ),
    once((
      generate_pddl(Resource, Graph, String)
    ; throw(response(400))
    ))
  ), Strings),
  response(200),
  forall(
    member(String, Strings),
    write(String)
  ).

plan_creation_factory(Context) :-
  Graph = Context.graph_in,
  make_temp_graph(Context.graph_out),
  once((
    rdf(Domain, rdf:type, pddl:'Domain', Graph),
    rdf(Problem, rdf:type, pddl:'Problem', Graph),
    once((
      generate_plan(Domain, Graph, Problem, Graph, plan, Context.graph_out)
    ; throw(response(400))
    ))
  )).

validated_plan_creation_factory(Context) :-
  Graph = Context.graph_in,
  make_temp_graph(Context.graph_out),
  once((
    rdf(Domain, rdf:type, pddl:'Domain', Graph),
    rdf(Problem, rdf:type, pddl:'Problem', Graph),
    once((
      generate_plan(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, plan, Context.graph_out, true),
      validate_plan(Domain, DomainGraph, DomainFile, Problem, ProblemGraph, ProblemFile, plan, Context.graph_out)
    ; throw(response(400))
    ))
  )).
