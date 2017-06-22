:- module(oslc_api, [
    assert_resource/2,
    assert_resource/3,
    resource_print_rdf/1,
    resource_save_rdf/2
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(oslc, 'http://open-services.net/ns/core#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc).

:- rdf_meta assert_resource(r, -).
:- rdf_meta assert_resource(r, -, -).
:- rdf_meta resource_print_rdf(r).
:- rdf_meta resource_save_rdf(-, r).

assert_resource(URI, Properties) :-
  assert_resource(URI, Properties, user).

assert_resource(_, [], _) :- !.

assert_resource(URI, [H|T], Graph) :-
  H =.. [Predicate, Object],
  property(Predicate, P),
  rdf_assert(URI, P, Object, Graph),
  assert_resource(URI, T, Graph).

resource_print_rdf(URI) :-
  current_output(Out),
  resource_save_rdf(Out, URI).

resource_save_rdf(Out, URI) :-
  rdf_save_subject(Out, URI, [nsmap([])]).

property(type, rdf:'Type').
property(title, dcterms:title).
