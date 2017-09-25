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

:- begin_tests(oslc, [
  setup(rdf_unload_graph(oslc_test))
  %,cleanup(rdf_unload_graph(oslc_test))
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).
:- use_module(library(oslc)).

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(tests).

:- rdf_meta assertion(r,t).

assertion(Actual, Expected) :-
  assertion(Actual == Expected).

test(service_provider_catalog) :-
  rdf_create_bnode(Publisher),

  create_resource(Publisher, [oslc:'Publisher'],
                  [ title = "publisher",
                    label = "publabel",
                    identifier = "id11",
                    icon = pubicon
                  ], rdf(oslc_test)),

  rdf_create_bnode(Service1),
  create_resource(Service1, [oslc:'Service'],
                  [
                    domain = 'http://mydomain.com'
                  ], rdf(oslc_test)),

  create_resource(tests:sp1, [oslc:'ServiceProvider'],
                  [
                    service = [ Service1 ]
                  ], rdf(oslc_test)),

  rdf_create_bnode(Service2),
  create_resource(Service2, [oslc:'Service'],
                  [
                    domain = 'http://mydomain.com'
                  ], rdf(oslc_test)),

  create_resource(tests:sp2, [oslc:'ServiceProvider'],
                  [
                    service = [ Service2 ]
                  ], rdf(oslc_test)),

  create_resource(tests:s1, [oslc:'ServiceProviderCatalog'],
                  [ title = "service provider catalog",
                    description = "o3",
                    publisher = Publisher,
                    serviceProvider = [tests:sp1, tests:sp2]
                  ], rdf(oslc_test)),

  rdf(tests:s1, rdf:type, SPC),
  rdf_global_id(oslc:'ServiceProviderCatalog', OSPC),
  assertion(SPC, OSPC),

  oslc_resource(tests:s1, [ publisher = A,
                           description = B,
                           title = C,
                           serviceProvider = D
                         ], rdf(oslc_test)),

  assertion(C == "service provider catalog"),
  assertion(B == "o3"),
  assertion(A, Publisher),
  assertion(D, [tests:sp1, tests:sp2]).

:- end_tests(oslc).
