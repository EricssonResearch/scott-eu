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
:- use_module(library(pddl_generator)).

:- oslc_post(planner:planCreationFactory, plan_creation_factory).

oslc_dispatch:serializer(text/'x-pddl', turtle).

plan_creation_factory(Context) :-
  once((
    Context.content_type == text/'x-pddl',
    once((
      rdf(Resource, rdf:type, pddl:'Domain', Context.graph_in)
    ; rdf(Resource, rdf:type, pddl:'Problem', Context.graph_in)
    )),
    generate_pddl(Resource, Context.graph_in, String),
    response(200),
    write(String)
  ; throw(response(400))
  )).
