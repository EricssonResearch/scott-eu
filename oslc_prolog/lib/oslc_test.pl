:- module(oslc_test, []).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).

:- rdf_register_prefix(oslct, 'http://ontology.cf.ericsson.net/oslc_test#').

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslct).

get_current_time(A) :-
  get_time(T),
  stamp_date_time(T, D, local),
  format_time(atom(A), '%T', D, posix).
