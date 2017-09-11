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
  true.
