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

:- module(grammar_utils, [
  rdf0_set_graph/1,
  rdf0_unset_graph/1,
  rdf0/3,
  lookup//1,
  of_type//1,
  individual_of//1,
  subclass_of//1,
  all_properties//2,
  apply_to_property//3,
  apply_to_properties//3
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdfs)).

:- meta_predicate apply_to_property(+, 3, -, -, -).
:- meta_predicate apply_to_properties(+, 3, -, -, -).

:- rdf_meta rdf0(r, r, t).
:- rdf_meta of_type(r, -, -).
:- rdf_meta individual_of(r, -, -).
:- rdf_meta subclass_of(r, -, -).
:- rdf_meta all_properties(r, -, -, -).
:- rdf_meta apply_to_property(r, -, -, -, -).
:- rdf_meta apply_to_properties(r, -, -, -, -).

:- thread_local graph/1.

rdf0_set_graph(Graph) :-
  assertz(graph(Graph)).

rdf0_unset_graph(Graph) :-
  retractall(graph(Graph)).

rdf0(S, P, O) :-
  graph(G),
  rdf(S, P, O, G).

lookup(Resource), [Resource] --> [Resource].

of_type(Class) -->
  lookup(Resource),
  {
    rdf0(Resource, rdf:type, Class)
  }.

individual_of(Class) -->
lookup(Resource),
  {
    rdfs_individual_of(Resource, Class)
  }.

subclass_of(Class) -->
  lookup(Resource),
  {
    rdfs_subclass_of(Resource, Class)
  }.

all_properties(Property, List) -->
  lookup(Resource),
  {
    findall(Element,
      rdf0(Resource, Property, Element),
    List)
  }.

apply_to_property(Property, Module:Rule, Result) -->
  lookup(Resource),
  {
    rdf0(Resource, Property, Element),
    T =.. [Rule, Result, [Element], [Element]],
    call(Module:T)
  }.

apply_to_properties(Property, Module:Rule, Collection) -->
  lookup(Resource),
  {
    findall(Element, (
      rdf0(Resource, Property, ElementResource),
      T =.. [Rule, Element, [ElementResource], [ElementResource]],
      call(Module:T)
    ), Collection)
  }.
