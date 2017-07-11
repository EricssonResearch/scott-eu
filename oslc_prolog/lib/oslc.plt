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
  rdf_bnode(Publisher),
  service_provider_catalog(test:s1, [title("service provider catalog"), description="o3", publisher=Publisher], oslc_test),
  %[test:o5, test:o6], [test:o7, test:o8], [test:o9], [test:o10]
  rdf(test:s1, rdf:type, oslc:'ServiceProviderCatalog'),
  service_provider_catalog(test:s1, [publisher(A), description(B), title=C], oslc_test),
  assertion(C == "service provider catalog"),
  assertion(B == "o3"),
  assertion(A, Publisher).
  /*
  assertion(D, [test:o5, test:o6]),
  assertion(E, [test:o7, test:o8]),
  assertion(F, [test:o9]),
  assertion(G, [test:o10]).
  */

:- end_tests(oslc).
