:- module(oslc_core, [
]).

:- use_module(library(semweb/rdf11)).
:- use_module(library(semweb/rdf_library)).

:- rdf_attach_library(oslc_prolog(rdf)).
:- rdf_load_library(oslc_test).

reload :-
  rdf_reset_db,
  rdf_load_library(oslc),
  rdf_load_library(oslc_shapes),
  rdf_load_library(oslc_test).
