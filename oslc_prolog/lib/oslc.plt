:- begin_tests(oslc, [
  setup(rdf_unload_graph(oslc_test))
  %,cleanup(rdf_unload_graph(oslc_test))
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(oslc)).

:- rdf_meta assertion(r,t).

:- rdf_register_prefix(test, 'http://ontology.cf.ericsson.net/test#').
:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').

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

  create_resource(test:s1, [oslc:'ServiceProviderCatalog'],
                  [ title = "service provider catalog",
                    description = "o3",
                    publisher = Publisher,
                    serviceProvider = [sp1,sp2]
                  ], rdf(oslc_test)),

  rdf(test:s1, rdf:type, SPC),
  rdf_global_id(oslc:'ServiceProviderCatalog', OSPC),
  assertion(SPC, OSPC),

  oslc_resource(test:s1, [ publisher = A,
                           description = B,
                           title = C,
                           serviceProvider = D
                         ], rdf(oslc_test)),

  assertion(C == "service provider catalog"),
  assertion(B == "o3"),
  assertion(A, Publisher),
  assertion(D, [sp1, sp2]).

:- end_tests(oslc).
